#!/usr/bin/env python3
"""Apply a verified RotorSync BBB maintenance update package.

This helper is intended to run through a narrow sudoers rule. It never
downloads anything, and it only applies a package that the maintenance agent
has already staged and SHA-256 verified.
"""

from __future__ import annotations

import hashlib
import json
import os
import py_compile
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
from pathlib import Path


UPDATE_ROOT = Path(os.environ.get("BBB_MAINT_UPDATE_ROOT", "/home/pi/.rotorsync-maintenance-updates")).resolve()
REPO_ROOT = Path(os.environ.get("BBB_MAINT_REPO_ROOT", "/home/pi/Big-Beautiful-Box")).resolve()
BACKUP_ROOT = Path(os.environ.get("BBB_MAINT_BACKUP_ROOT", "/home/pi/bbb-maint-backups")).resolve()
OPT_AGENT_PATH = Path("/opt/bbb_maint_agent.py")
PI_PROTOCOL_PATH = Path("/home/pi/src/maintenance_protocol.py")
RESTART_SERVICES = (
    "iol_dashboard.service",
    "rotorsync.service",
    "bbb-maint-agent.service",
)
APPLY_SELF = Path(os.environ.get("BBB_MAINT_APPLY_SELF", "/opt/rotorsync-maint-apply"))
POSTCHECK_DELAY_SECONDS = int(os.environ.get("BBB_MAINT_POSTCHECK_DELAY_SECONDS", "8"))
SYSTEMD_RUN = os.environ.get("BBB_MAINT_SYSTEMD_RUN", "/usr/bin/systemd-run")


class ApplyError(RuntimeError):
    pass


def emit(payload: dict) -> None:
    print(json.dumps(payload, separators=(",", ":"), sort_keys=True), flush=True)


def path_is_relative_safe(name: str) -> bool:
    path = Path(name)
    return bool(name) and not path.is_absolute() and ".." not in path.parts


def archive_member_is_ignored(name: str) -> bool:
    parts = Path(name).parts
    return any(part == "__MACOSX" or part.startswith("._") for part in parts)


def read_manifest(path: Path) -> dict:
    manifest_path = path.resolve()
    if not str(manifest_path).startswith(str(UPDATE_ROOT) + os.sep):
        raise ApplyError("manifest outside update root")
    manifest = json.loads(manifest_path.read_text())
    if manifest.get("verified") is not True:
        raise ApplyError("update is not verified")
    artifact = Path(str(manifest.get("artifact", ""))).resolve()
    if not str(artifact).startswith(str(UPDATE_ROOT) + os.sep):
        raise ApplyError("artifact outside update root")
    if not artifact.exists():
        raise ApplyError("artifact is missing")
    expected_sha = str(manifest.get("sha256", "")).lower()
    if len(expected_sha) != 64:
        raise ApplyError("manifest sha256 is invalid")
    actual_sha = hashlib.sha256(artifact.read_bytes()).hexdigest()
    if actual_sha != expected_sha:
        raise ApplyError("artifact sha256 mismatch")
    manifest["artifact"] = str(artifact)
    manifest["sha256"] = actual_sha
    return manifest


def validate_tar(artifact: Path, extract_dir: Path) -> list[tarfile.TarInfo]:
    try:
        archive = tarfile.open(artifact, "r:*")
    except tarfile.TarError as exc:
        raise ApplyError("artifact is not a tar package") from exc

    with archive:
        members = archive.getmembers()
        if not members:
            raise ApplyError("update package is empty")
        for member in members:
            if not path_is_relative_safe(member.name):
                raise ApplyError(f"unsafe archive path {member.name!r}")
            if member.issym() or member.islnk() or member.isdev():
                raise ApplyError("links and device files are not allowed")
        safe_members = [
            member for member in members if not archive_member_is_ignored(member.name)
        ]
        if not safe_members:
            raise ApplyError("update package contains no usable files")
        try:
            archive.extractall(extract_dir, members=safe_members, filter="data")
        except TypeError:
            archive.extractall(extract_dir, members=safe_members)
    return safe_members


def package_root(extract_dir: Path) -> Path:
    children = [child for child in extract_dir.iterdir() if child.name != "__MACOSX"]
    if len(children) == 1 and children[0].is_dir():
        return children[0]
    return extract_dir


def destination_for(relative: Path) -> Path | None:
    parts = relative.parts
    if not parts:
        return None
    if parts[0] == "repo":
        return (REPO_ROOT / Path(*parts[1:])).resolve()
    if parts[0] == "opt" and len(parts) == 2 and parts[1] == "bbb_maint_agent.py":
        return OPT_AGENT_PATH
    if parts[0] == "home-pi-src" and len(parts) == 2 and parts[1] == "maintenance_protocol.py":
        return PI_PROTOCOL_PATH
    return (REPO_ROOT / relative).resolve()


def destination_is_allowed(dest: Path) -> bool:
    resolved = dest.resolve()
    return (
        str(resolved).startswith(str(REPO_ROOT) + os.sep)
        or resolved == OPT_AGENT_PATH
        or resolved == PI_PROTOCOL_PATH
    )


def validate_destinations(root: Path) -> list[tuple[Path, Path]]:
    file_pairs: list[tuple[Path, Path]] = []
    for source in root.rglob("*"):
        if not source.is_file():
            continue
        relative = source.relative_to(root)
        dest = destination_for(relative)
        if dest is None:
            continue
        if not destination_is_allowed(dest):
            raise ApplyError(f"destination outside allowed roots: {relative}")
        file_pairs.append((source, dest))
    if not file_pairs:
        raise ApplyError("update package contains no files")
    return file_pairs


def compile_python(files: list[tuple[Path, Path]]) -> None:
    for source, _dest in files:
        if source.suffix == ".py":
            py_compile.compile(str(source), doraise=True)


def backup_and_copy(update_id: str, files: list[tuple[Path, Path]]) -> Path:
    backup_dir = BACKUP_ROOT / f"update-{update_id}-{time.strftime('%Y%m%d-%H%M%S')}"
    backup_dir.mkdir(parents=True, exist_ok=False)
    manifest_rows = []
    for source, dest in files:
        backup_path = backup_dir / str(dest).lstrip("/")
        had_existing = dest.exists()
        if had_existing:
            backup_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(dest, backup_path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
        manifest_rows.append({
            "source": str(source),
            "dest": str(dest),
            "backup": str(backup_path) if had_existing else None,
        })

    protocol_in_repo = REPO_ROOT / "src" / "maintenance_protocol.py"
    if protocol_in_repo.exists() and not any(dest == PI_PROTOCOL_PATH for _src, dest in files):
        backup_path = backup_dir / str(PI_PROTOCOL_PATH).lstrip("/")
        had_existing = PI_PROTOCOL_PATH.exists()
        if had_existing:
            backup_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(PI_PROTOCOL_PATH, backup_path)
        PI_PROTOCOL_PATH.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(protocol_in_repo, PI_PROTOCOL_PATH)
        manifest_rows.append({
            "source": str(protocol_in_repo),
            "dest": str(PI_PROTOCOL_PATH),
            "backup": str(backup_path) if had_existing else None,
        })

    (backup_dir / "applied-files.json").write_text(
        json.dumps(manifest_rows, indent=2, sort_keys=True)
    )
    return backup_dir


def backup_dir_is_safe(backup_dir: Path) -> bool:
    resolved = backup_dir.resolve()
    return str(resolved).startswith(str(BACKUP_ROOT.resolve()) + os.sep)


def service_states() -> dict[str, str]:
    states: dict[str, str] = {}
    for service in RESTART_SERVICES:
        completed = subprocess.run(
            ["/bin/systemctl", "is-active", service],
            check=False,
            capture_output=True,
            text=True,
        )
        state = (completed.stdout or completed.stderr or "unknown").strip()
        states[service] = state or "unknown"
    return states


def services_are_healthy(states: dict[str, str]) -> bool:
    return all(states.get(service) == "active" for service in RESTART_SERVICES)


def restart_services() -> None:
    subprocess.run(["/bin/systemctl", "restart", *RESTART_SERVICES], check=False)


def restore_backup(backup_dir: Path) -> list[dict]:
    if not backup_dir_is_safe(backup_dir):
        raise ApplyError("backup directory outside backup root")
    applied_path = backup_dir / "applied-files.json"
    rows = json.loads(applied_path.read_text())
    restored = []
    for row in reversed(rows):
        dest = Path(str(row.get("dest", ""))).resolve()
        backup = row.get("backup")
        if not destination_is_allowed(dest):
            raise ApplyError(f"refusing rollback destination {dest}")
        if backup:
            backup_path = Path(str(backup)).resolve()
            if not str(backup_path).startswith(str(backup_dir.resolve()) + os.sep):
                raise ApplyError(f"backup path outside rollback directory: {backup_path}")
            if not backup_path.exists():
                raise ApplyError(f"backup file missing: {backup_path}")
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(backup_path, dest)
            restored.append({"dest": str(dest), "restored": True})
        elif dest.exists():
            dest.unlink()
            restored.append({"dest": str(dest), "removed": True})
        else:
            restored.append({"dest": str(dest), "already_absent": True})
    return restored


def postcheck(backup_dir: Path) -> dict:
    states = service_states()
    if services_are_healthy(states):
        result = {
            "postcheck": "healthy",
            "services": states,
            "rollback": False,
        }
    else:
        restored = restore_backup(backup_dir)
        restart_services()
        result = {
            "postcheck": "rollback",
            "services": states,
            "rollback": True,
            "restored": restored,
        }

    (backup_dir / "postcheck-result.json").write_text(
        json.dumps(result, separators=(",", ":"), sort_keys=True)
    )
    return result


def schedule_restarts(backup_dir: Path) -> str:
    command = (
        "sleep 2; /bin/systemctl restart "
        + " ".join(shlex.quote(service) for service in RESTART_SERVICES)
        + f"; sleep {POSTCHECK_DELAY_SECONDS}; "
        + shlex.quote(str(APPLY_SELF))
        + " --postcheck "
        + shlex.quote(str(backup_dir))
    )
    unit = f"rotorsync-maint-postcheck-{int(time.time())}"
    systemd_run = Path(SYSTEMD_RUN)
    if systemd_run.exists():
        completed = subprocess.run(
            [
                str(systemd_run),
                "--unit",
                unit,
                "--description",
                "RotorSync maintenance restart and postcheck",
                "/bin/sh",
                "-c",
                command,
            ],
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode == 0:
            return f"systemd-run:{unit}"

    subprocess.Popen(
        ["/bin/sh", "-c", command],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return "subprocess"


def apply(manifest_path: Path) -> dict:
    manifest = read_manifest(manifest_path)
    update_id = str(manifest.get("update_id") or manifest_path.parent.name)
    with tempfile.TemporaryDirectory(prefix="rotorsync-apply-", dir=str(UPDATE_ROOT)) as tmp:
        extract_dir = Path(tmp)
        validate_tar(Path(str(manifest["artifact"])), extract_dir)
        root = package_root(extract_dir)
        files = validate_destinations(root)
        compile_python(files)
        backup_dir = backup_and_copy(update_id, files)

    scheduler = schedule_restarts(backup_dir)
    result = {
        "applied": True,
        "update_id": update_id,
        "backup": str(backup_dir),
        "files": len(files),
        "restart_scheduled": True,
        "scheduler": scheduler,
    }
    result_path = manifest_path.parent / "apply-result.json"
    result_path.write_text(json.dumps(result, separators=(",", ":"), sort_keys=True))
    return result


def main(argv: list[str]) -> int:
    if len(argv) == 3 and argv[1] == "--postcheck":
        try:
            emit(postcheck(Path(argv[2])))
            return 0
        except Exception as exc:
            emit({"postcheck": "failed", "error": str(exc)})
            return 1
    if len(argv) != 2:
        emit({"applied": False, "error": "usage: rotorsync_maint_apply.py MANIFEST|--postcheck BACKUP_DIR"})
        return 2
    try:
        emit(apply(Path(argv[1])))
        return 0
    except Exception as exc:
        emit({"applied": False, "error": str(exc)})
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))

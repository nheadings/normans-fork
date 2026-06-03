import hashlib
import importlib.util
import json
import tarfile
import tempfile
import unittest
from pathlib import Path


HELPER_PATH = Path(__file__).resolve().parents[1] / "scripts" / "rotorsync_maint_apply.py"
spec = importlib.util.spec_from_file_location("rotorsync_maint_apply", HELPER_PATH)
apply_helper = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(apply_helper)


def write_tar(path: Path, files: dict[str, str]) -> None:
    with tarfile.open(path, "w:gz") as archive:
        for name, content in files.items():
            source = path.parent / name.replace("/", "_")
            source.write_text(content)
            archive.add(source, arcname=name)


class RotorSyncMaintApplyTests(unittest.TestCase):
    def test_apply_verified_package_backs_up_and_copies_repo_files(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            update_root = root / "updates"
            repo_root = root / "repo"
            backup_root = root / "backups"
            pi_src = root / "pi-src" / "maintenance_protocol.py"
            opt_agent = root / "opt" / "bbb_maint_agent.py"
            update_dir = update_root / "test-update"
            update_dir.mkdir(parents=True)
            repo_root.mkdir()
            (repo_root / "dashboard.py").write_text("print('old')\n")
            artifact = update_dir / "artifact.tar.gz"
            write_tar(artifact, {"repo/dashboard.py": "print('new')\n"})
            sha256 = hashlib.sha256(artifact.read_bytes()).hexdigest()
            manifest = {
                "artifact": str(artifact),
                "sha256": sha256,
                "size": artifact.stat().st_size,
                "update_id": "test-update",
                "verified": True,
            }
            manifest_path = update_dir / "manifest.json"
            manifest_path.write_text(json.dumps(manifest))

            original_values = (
                apply_helper.UPDATE_ROOT,
                apply_helper.REPO_ROOT,
                apply_helper.BACKUP_ROOT,
                apply_helper.PI_PROTOCOL_PATH,
                apply_helper.OPT_AGENT_PATH,
                apply_helper.schedule_restarts,
            )
            try:
                apply_helper.UPDATE_ROOT = update_root.resolve()
                apply_helper.REPO_ROOT = repo_root.resolve()
                apply_helper.BACKUP_ROOT = backup_root.resolve()
                apply_helper.PI_PROTOCOL_PATH = pi_src.resolve()
                apply_helper.OPT_AGENT_PATH = opt_agent.resolve()
                apply_helper.schedule_restarts = lambda _backup_dir: None

                result = apply_helper.apply(manifest_path)

                self.assertTrue(result["applied"])
                self.assertEqual((repo_root / "dashboard.py").read_text(), "print('new')\n")
                backup_file = Path(result["backup"]) / str((repo_root / "dashboard.py").resolve()).lstrip("/")
                self.assertEqual(backup_file.read_text(), "print('old')\n")
            finally:
                (
                    apply_helper.UPDATE_ROOT,
                    apply_helper.REPO_ROOT,
                    apply_helper.BACKUP_ROOT,
                    apply_helper.PI_PROTOCOL_PATH,
                    apply_helper.OPT_AGENT_PATH,
                    apply_helper.schedule_restarts,
                ) = original_values

    def test_apply_rejects_unsafe_tar_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            update_root = root / "updates"
            update_dir = update_root / "bad-update"
            update_dir.mkdir(parents=True)
            artifact = update_dir / "artifact.tar.gz"
            write_tar(artifact, {"../escape.py": "print('bad')\n"})
            sha256 = hashlib.sha256(artifact.read_bytes()).hexdigest()
            manifest_path = update_dir / "manifest.json"
            manifest_path.write_text(json.dumps({
                "artifact": str(artifact),
                "sha256": sha256,
                "update_id": "bad-update",
                "verified": True,
            }))

            original_update_root = apply_helper.UPDATE_ROOT
            try:
                apply_helper.UPDATE_ROOT = update_root.resolve()
                with self.assertRaises(apply_helper.ApplyError):
                    apply_helper.apply(manifest_path)
            finally:
                apply_helper.UPDATE_ROOT = original_update_root

    def test_apply_ignores_macos_metadata_members(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            update_root = root / "updates"
            repo_root = root / "repo"
            backup_root = root / "backups"
            update_dir = update_root / "metadata-update"
            update_dir.mkdir(parents=True)
            repo_root.mkdir()
            artifact = update_dir / "artifact.tar.gz"
            write_tar(artifact, {
                "repo/.probe": "real\n",
                "repo/._.probe": "metadata\n",
                "__MACOSX/._probe": "metadata\n",
            })
            sha256 = hashlib.sha256(artifact.read_bytes()).hexdigest()
            manifest_path = update_dir / "manifest.json"
            manifest_path.write_text(json.dumps({
                "artifact": str(artifact),
                "sha256": sha256,
                "update_id": "metadata-update",
                "verified": True,
            }))

            original_values = (
                apply_helper.UPDATE_ROOT,
                apply_helper.REPO_ROOT,
                apply_helper.BACKUP_ROOT,
                apply_helper.PI_PROTOCOL_PATH,
                apply_helper.OPT_AGENT_PATH,
                apply_helper.schedule_restarts,
            )
            try:
                apply_helper.UPDATE_ROOT = update_root.resolve()
                apply_helper.REPO_ROOT = repo_root.resolve()
                apply_helper.BACKUP_ROOT = backup_root.resolve()
                apply_helper.PI_PROTOCOL_PATH = (root / "pi-src" / "maintenance_protocol.py").resolve()
                apply_helper.OPT_AGENT_PATH = (root / "opt" / "bbb_maint_agent.py").resolve()
                apply_helper.schedule_restarts = lambda _backup_dir: None

                result = apply_helper.apply(manifest_path)

                self.assertTrue(result["applied"])
                self.assertEqual((repo_root / ".probe").read_text(), "real\n")
                self.assertFalse((repo_root / "._.probe").exists())
                self.assertFalse((repo_root / "__MACOSX").exists())
            finally:
                (
                    apply_helper.UPDATE_ROOT,
                    apply_helper.REPO_ROOT,
                    apply_helper.BACKUP_ROOT,
                    apply_helper.PI_PROTOCOL_PATH,
                    apply_helper.OPT_AGENT_PATH,
                    apply_helper.schedule_restarts,
                ) = original_values

    def test_postcheck_leaves_files_when_services_are_healthy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo_root = root / "repo"
            backup_root = root / "backups"
            backup_dir = backup_root / "update-good"
            live_file = repo_root / "dashboard.py"
            backup_file = backup_dir / str(live_file.resolve()).lstrip("/")
            live_file.parent.mkdir(parents=True)
            backup_file.parent.mkdir(parents=True)
            live_file.write_text("print('new')\n")
            backup_file.write_text("print('old')\n")
            (backup_dir / "applied-files.json").write_text(json.dumps([
                {
                    "dest": str(live_file.resolve()),
                    "backup": str(backup_file.resolve()),
                    "source": "/tmp/source",
                }
            ]))

            original_values = (
                apply_helper.REPO_ROOT,
                apply_helper.BACKUP_ROOT,
                apply_helper.PI_PROTOCOL_PATH,
                apply_helper.OPT_AGENT_PATH,
                apply_helper.service_states,
                apply_helper.restart_services,
            )
            restarted = []
            try:
                apply_helper.REPO_ROOT = repo_root.resolve()
                apply_helper.BACKUP_ROOT = backup_root.resolve()
                apply_helper.PI_PROTOCOL_PATH = (root / "pi-src" / "maintenance_protocol.py").resolve()
                apply_helper.OPT_AGENT_PATH = (root / "opt" / "bbb_maint_agent.py").resolve()
                apply_helper.service_states = lambda: {
                    service: "active" for service in apply_helper.RESTART_SERVICES
                }
                apply_helper.restart_services = lambda: restarted.append(True)

                result = apply_helper.postcheck(backup_dir)

                self.assertEqual(result["postcheck"], "healthy")
                self.assertFalse(result["rollback"])
                self.assertEqual(live_file.read_text(), "print('new')\n")
                self.assertEqual(restarted, [])
            finally:
                (
                    apply_helper.REPO_ROOT,
                    apply_helper.BACKUP_ROOT,
                    apply_helper.PI_PROTOCOL_PATH,
                    apply_helper.OPT_AGENT_PATH,
                    apply_helper.service_states,
                    apply_helper.restart_services,
                ) = original_values

    def test_postcheck_rolls_back_when_service_is_unhealthy(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            repo_root = root / "repo"
            backup_root = root / "backups"
            backup_dir = backup_root / "update-bad"
            live_file = repo_root / "dashboard.py"
            new_file = repo_root / "new_probe.py"
            backup_file = backup_dir / str(live_file.resolve()).lstrip("/")
            live_file.parent.mkdir(parents=True)
            backup_file.parent.mkdir(parents=True)
            live_file.write_text("print('bad')\n")
            new_file.write_text("print('new file')\n")
            backup_file.write_text("print('old')\n")
            (backup_dir / "applied-files.json").write_text(json.dumps([
                {
                    "dest": str(live_file.resolve()),
                    "backup": str(backup_file.resolve()),
                    "source": "/tmp/source",
                },
                {
                    "dest": str(new_file.resolve()),
                    "backup": None,
                    "source": "/tmp/source-new",
                },
            ]))

            original_values = (
                apply_helper.REPO_ROOT,
                apply_helper.BACKUP_ROOT,
                apply_helper.PI_PROTOCOL_PATH,
                apply_helper.OPT_AGENT_PATH,
                apply_helper.service_states,
                apply_helper.restart_services,
            )
            restarted = []
            try:
                apply_helper.REPO_ROOT = repo_root.resolve()
                apply_helper.BACKUP_ROOT = backup_root.resolve()
                apply_helper.PI_PROTOCOL_PATH = (root / "pi-src" / "maintenance_protocol.py").resolve()
                apply_helper.OPT_AGENT_PATH = (root / "opt" / "bbb_maint_agent.py").resolve()
                apply_helper.service_states = lambda: {
                    "iol_dashboard.service": "active",
                    "rotorsync.service": "failed",
                    "bbb-maint-agent.service": "active",
                }
                apply_helper.restart_services = lambda: restarted.append(True)

                result = apply_helper.postcheck(backup_dir)

                self.assertEqual(result["postcheck"], "rollback")
                self.assertTrue(result["rollback"])
                self.assertEqual(live_file.read_text(), "print('old')\n")
                self.assertFalse(new_file.exists())
                self.assertEqual(restarted, [True])
            finally:
                (
                    apply_helper.REPO_ROOT,
                    apply_helper.BACKUP_ROOT,
                    apply_helper.PI_PROTOCOL_PATH,
                    apply_helper.OPT_AGENT_PATH,
                    apply_helper.service_states,
                    apply_helper.restart_services,
                ) = original_values

    def test_schedule_restarts_uses_systemd_run_outside_agent_cgroup(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            backup_dir = root / "backup"
            backup_dir.mkdir()
            systemd_run = root / "systemd-run"
            systemd_run.write_text("#!/bin/sh\nexit 0\n")

            calls = []

            class Completed:
                returncode = 0
                stdout = ""
                stderr = ""

            def fake_run(args, **kwargs):
                calls.append((args, kwargs))
                return Completed()

            original_values = (
                apply_helper.SYSTEMD_RUN,
                apply_helper.subprocess.run,
            )
            try:
                apply_helper.SYSTEMD_RUN = str(systemd_run)
                apply_helper.subprocess.run = fake_run

                scheduler = apply_helper.schedule_restarts(backup_dir)

                self.assertTrue(scheduler.startswith("systemd-run:"))
                self.assertEqual(calls[0][0][0], str(systemd_run))
                self.assertIn("/bin/sh", calls[0][0])
                self.assertIn("bbb-maint-agent.service", calls[0][0][-1])
                self.assertIn("--postcheck", calls[0][0][-1])
            finally:
                (
                    apply_helper.SYSTEMD_RUN,
                    apply_helper.subprocess.run,
                ) = original_values


if __name__ == "__main__":
    unittest.main()

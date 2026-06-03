import os
import base64
import hashlib
import tempfile
import unittest
from pathlib import Path

from bbb_maint_agent import MaintenanceAgent, ShellSession, apply_helper_command
from src.maintenance_protocol import MaintenanceProtocolError


class ExitedProcess:
    def poll(self):
        return 0


class RunningProcess:
    def poll(self):
        return None


class MaintenanceAgentLifecycleTests(unittest.IsolatedAsyncioTestCase):
    async def test_update_apply_uses_python_helper_sudo_invocation(self):
        command = apply_helper_command(Path("/tmp/update/manifest.json"))

        self.assertEqual(command[0:4], [
            "/usr/bin/sudo",
            "-n",
            "/usr/bin/python3",
            "/opt/rotorsync-maint-apply",
        ])
        self.assertEqual(command[-1], "/tmp/update/manifest.json")

    async def test_same_session_open_is_idempotent(self):
        agent = MaintenanceAgent()
        current_session = ShellSession(
            session_id="session",
            nonce="nonce",
            created_at=1,
            last_activity=1,
            process=RunningProcess(),
        )
        broadcasts = []
        closed = []

        async def broadcast(payload):
            broadcasts.append(payload)

        async def close_session(reason, exit_status):
            closed.append((reason, exit_status))

        agent.session = current_session
        agent.broadcast = broadcast
        agent.close_session = close_session

        await agent.open_session({
            "session_id": "session",
            "nonce": "new-nonce-ignored",
            "seq": 1,
        })

        self.assertIs(agent.session, current_session)
        self.assertEqual(closed, [])
        self.assertEqual(broadcasts[-1]["type"], "opened")
        self.assertEqual(broadcasts[-1]["session_id"], "session")
        self.assertTrue(broadcasts[-1]["existing"])
        self.assertEqual(broadcasts[-1]["nonce"], "nonce")

    async def test_stale_stdout_pump_does_not_close_replacement_session(self):
        agent = MaintenanceAgent()
        old_session = ShellSession(
            session_id="old",
            nonce="old",
            created_at=1,
            last_activity=1,
            master_fd=os.open(os.devnull, os.O_RDONLY),
            process=ExitedProcess(),
        )
        new_session = ShellSession(
            session_id="new",
            nonce="new",
            created_at=2,
            last_activity=2,
            process=ExitedProcess(),
        )
        closed = []

        async def close_session(reason, exit_status):
            closed.append((reason, exit_status))

        agent.session = new_session
        agent.close_session = close_session

        await agent._pump_stdout(old_session)

        self.assertEqual(closed, [])
        self.assertIs(agent.session, new_session)

    async def test_update_stages_and_verifies_sha256(self):
        payload = b"rotorsync update bytes"
        update_id = "test-update"
        agent = MaintenanceAgent()
        agent.session = ShellSession(
            session_id="session",
            nonce="nonce",
            created_at=1,
            last_activity=1,
            process=RunningProcess(),
        )

        with tempfile.TemporaryDirectory() as tmp:
            agent.update_root = Path(tmp)
            await agent.update_begin({
                "session_id": "session",
                "update_id": update_id,
                "size": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            })
            await agent.update_chunk({
                "session_id": "session",
                "update_id": update_id,
                "offset": 0,
                "data_b64": base64.b64encode(payload).decode("ascii"),
            })
            await agent.update_finalize({
                "session_id": "session",
                "update_id": update_id,
            })

            artifact = agent.update_root / update_id / "artifact.bin"
            manifest = agent.update_root / update_id / "manifest.json"
            self.assertEqual(artifact.read_bytes(), payload)
            self.assertIn('"verified":true', manifest.read_text())
            self.assertIsNone(agent.active_update)

    async def test_update_rejects_out_of_order_chunks(self):
        payload = b"abcdefgh"
        agent = MaintenanceAgent()
        agent.session = ShellSession(
            session_id="session",
            nonce="nonce",
            created_at=1,
            last_activity=1,
            process=RunningProcess(),
        )

        with tempfile.TemporaryDirectory() as tmp:
            agent.update_root = Path(tmp)
            await agent.update_begin({
                "session_id": "session",
                "update_id": "ordered",
                "size": len(payload),
                "sha256": hashlib.sha256(payload).hexdigest(),
            })

            with self.assertRaises(MaintenanceProtocolError):
                await agent.update_chunk({
                    "session_id": "session",
                    "update_id": "ordered",
                    "offset": 4,
                    "data_b64": base64.b64encode(payload[4:]).decode("ascii"),
                })

    async def test_update_rejects_sha_mismatch(self):
        payload = b"good bytes"
        agent = MaintenanceAgent()
        agent.session = ShellSession(
            session_id="session",
            nonce="nonce",
            created_at=1,
            last_activity=1,
            process=RunningProcess(),
        )

        with tempfile.TemporaryDirectory() as tmp:
            agent.update_root = Path(tmp)
            await agent.update_begin({
                "session_id": "session",
                "update_id": "bad-sha",
                "size": len(payload),
                "sha256": "0" * 64,
            })
            await agent.update_chunk({
                "session_id": "session",
                "update_id": "bad-sha",
                "offset": 0,
                "data_b64": base64.b64encode(payload).decode("ascii"),
            })

            with self.assertRaises(MaintenanceProtocolError):
                await agent.update_finalize({
                    "session_id": "session",
                    "update_id": "bad-sha",
                })


if __name__ == "__main__":
    unittest.main()

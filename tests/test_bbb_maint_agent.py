import os
import unittest

from bbb_maint_agent import MaintenanceAgent, ShellSession


class ExitedProcess:
    def poll(self):
        return 0


class MaintenanceAgentLifecycleTests(unittest.IsolatedAsyncioTestCase):
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


if __name__ == "__main__":
    unittest.main()

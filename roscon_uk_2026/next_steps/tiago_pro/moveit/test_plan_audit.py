"""A rejected native path must never reach trajectory execution."""
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from moveit_msgs.msg import Constraints
from check import Check, FloorClearanceError


class PlanAuditTest(unittest.TestCase):
    def make_check(self, outcomes):
        check = object.__new__(Check)
        trajectory = SimpleNamespace(joint_names=["joint"], points=[SimpleNamespace(positions=[0.1])])
        result = SimpleNamespace(error_code=SimpleNamespace(val=1),
                                 planned_trajectory=SimpleNamespace(joint_trajectory=trajectory))
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: SimpleNamespace(result=result))
        check.move = SimpleNamespace(send_goal_async=Mock(return_value=handle))
        check.future = lambda value, *args: value
        check.audit = Mock(side_effect=outcomes)
        check.execute_trajectory = Mock()
        return check

    def test_three_unsafe_plans_fail_without_execution(self):
        check = self.make_check([FloorClearanceError("0.049 m")] * 3)
        with self.assertRaises(FloorClearanceError):
            check.execute("both_arms", Constraints())
        self.assertEqual(check.move.send_goal_async.call_count, 3)
        check.execute_trajectory.wait_for_server.assert_not_called()
        check.execute_trajectory.send_goal_async.assert_not_called()

    def test_fresh_safe_plan_is_audited_after_rejections(self):
        check = self.make_check([FloorClearanceError("0.049 m"),
                                 {"minimum_arm_hand_z_m": 0.12}])
        _, endpoint = check.execute("both_arms", Constraints(), plan_only=True)
        self.assertEqual(check.move.send_goal_async.call_count, 2)
        self.assertEqual(check.last_audit["planning_attempt"], 2)
        self.assertEqual(endpoint, {"joint": 0.1})
        check.execute_trajectory.send_goal_async.assert_not_called()


if __name__ == "__main__":
    unittest.main()

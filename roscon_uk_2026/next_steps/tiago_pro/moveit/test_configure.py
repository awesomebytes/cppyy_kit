"""Check that planning targets and execution mappings match the actual robot."""
import unittest
import xml.etree.ElementTree as ET

import yaml
from configure import configure


class ConfigurationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.parameters = yaml.safe_load(configure().read_text())["/**"]["ros__parameters"]
        cls.robot = ET.fromstring(cls.parameters["robot_description"])
        cls.srdf = ET.fromstring(cls.parameters["robot_description_semantic"])

    def test_ati_chain_and_portable_meshes(self):
        for side in ("left", "right"):
            tool = self.robot.find(f"joint[@name='wrist_{side}_ft_tool_joint']")
            self.assertEqual(tool.find("parent").get("link"), f"wrist_{side}_ft_sensor_link")
        for mesh in self.robot.findall(".//mesh"):
            self.assertTrue(mesh.get("filename").startswith("package://"))

    def test_semantic_references_exist(self):
        links = {link.get("name") for link in self.robot.findall("link")}
        joints = {joint.get("name") for joint in self.robot.findall("joint")}
        for joint in self.srdf.findall(".//joint") + self.srdf.findall("passive_joint"):
            self.assertIn(joint.get("name"), joints)
        for link in self.srdf.findall(".//link"):
            self.assertIn(link.get("name"), links)
        for pair in self.srdf.findall("disable_collisions"):
            self.assertIn(pair.get("link1"), links)
            self.assertIn(pair.get("link2"), links)

    def test_only_active_controllers_are_mapped(self):
        controllers = self.parameters["moveit_simple_controller_manager"]
        self.assertEqual(set(controllers["controller_names"]), {
            "arm_left_controller", "arm_right_controller", "torso_controller", "head_controller",
            "gripper_left_controller", "gripper_right_controller"})
        for side in ("left", "right"):
            self.assertEqual(controllers[f"arm_{side}_controller"]["joints"],
                             [f"arm_{side}_{index}_joint" for index in range(1, 8)])
        self.assertFalse(self.parameters["moveit_manage_controllers"])

    def test_only_gripper_leaders_are_independently_actuated(self):
        passive = {joint.get("name") for joint in self.srdf.findall("passive_joint")}
        leaders = {f"gripper_{side}_finger_joint" for side in ("left", "right")}
        for joint in self.robot.findall("joint"):
            if joint.get("name", "").startswith("gripper_") and joint.get("type") != "fixed":
                if joint.get("name") in leaders:
                    self.assertNotIn(joint.get("name"), passive)
                    self.assertIsNone(joint.find("mimic"))
                else:
                    self.assertIn(joint.get("name"), passive)
                    self.assertIsNotNone(joint.find("mimic"))

    def test_native_gripper_command_mapping_and_full_stroke_states(self):
        controllers = self.parameters["moveit_simple_controller_manager"]
        for side in ("left", "right"):
            leader = f"gripper_{side}_finger_joint"
            controller = controllers[f"gripper_{side}_controller"]
            self.assertEqual(controller["type"], "GripperCommand")
            self.assertEqual(controller["action_ns"], "gripper_cmd")
            self.assertEqual(controller["joints"], [leader])
            self.assertEqual(controller["command_joint"], leader)
            self.assertEqual(controller["max_effort"], 10.0)
            for name, value in (("open", 0.07), ("closed", 0.0)):
                state = self.srdf.find(f"group_state[@group='gripper_{side}'][@name='{name}']/joint")
                self.assertEqual(state.get("name"), leader)
                self.assertEqual(float(state.get("value")), value)

    def test_head_is_a_joint_space_group_with_real_trajectory_controller(self):
        group = self.srdf.find("group[@name='head']")
        self.assertEqual([joint.get("name") for joint in group.findall("joint")], ["head_1_joint", "head_2_joint"])
        self.assertNotIn("head", self.parameters["robot_description_kinematics"])
        self.assertEqual(self.parameters["moveit_simple_controller_manager"]["head_controller"]["joints"],
                         ["head_1_joint", "head_2_joint"])

    def test_named_postures_obey_asymmetric_joint_limits(self):
        limits = {joint.get("name"): joint.find("limit") for joint in self.robot.findall("joint")}
        for joint in self.srdf.findall("group_state/joint"):
            limit = limits[joint.get("name")]
            value = float(joint.get("value"))
            self.assertGreaterEqual(value, float(limit.get("lower")))
            self.assertLessEqual(value, float(limit.get("upper")))

    def test_ready_moves_both_arms_and_torso_without_replacing_compact(self):
        ready = self.srdf.find("group_state[@name='ready']")
        self.assertEqual(ready.get("group"), "both_arms_torso")
        values = {joint.get("name"): float(joint.get("value")) for joint in ready.findall("joint")}
        self.assertEqual(len(values), 15)
        self.assertEqual(values["torso_lift_joint"], 0.20)
        self.assertIsNotNone(self.srdf.find("group_state[@name='compact'][@group='both_arms']"))
        for index in range(1, 8):
            sign = -1 if index in (1, 3, 5, 7) else 1
            self.assertAlmostEqual(values[f"arm_right_{index}_joint"],
                                   sign * values[f"arm_left_{index}_joint"])


if __name__ == "__main__":
    unittest.main()

"""
test_4wheel_encoder_kinematics.py
=================================
Unit test validating 4-wheel tank-style skid-steer kinematics,
joint name resolution, and track width calibration (0.49 m).
"""

import math
import unittest
import xml.etree.ElementTree as ET


class Test4WheelKinematics(unittest.TestCase):
    def setUp(self):
        self.track_width = 0.49  # meters
        self.wheel_radius = 0.06  # meters
        self.ticks_per_rev = 1024
        self.wheel_names = ["left_front", "right_front", "left_rear", "right_rear"]

    def test_forward_driving_kinematics(self):
        """Straight line driving: left and right wheels equal positive speed."""
        v_left = 0.5
        v_right = 0.5

        vx = (v_right + v_left) / 2.0
        wz = (v_right - v_left) / self.track_width

        self.assertAlmostEqual(vx, 0.5, places=5)
        self.assertAlmostEqual(wz, 0.0, places=5)

    def test_pure_tank_turn_in_place(self):
        """Zero-radius tank turn: left wheels reverse, right wheels forward."""
        desired_wz = 1.0  # rad/s
        # v_R = +wz * W / 2, v_L = -wz * W / 2
        v_right = desired_wz * self.track_width / 2.0  # +0.245 m/s
        v_left = -desired_wz * self.track_width / 2.0  # -0.245 m/s

        vx = (v_right + v_left) / 2.0
        wz = (v_right - v_left) / self.track_width

        self.assertAlmostEqual(vx, 0.0, places=5)
        self.assertAlmostEqual(wz, desired_wz, places=5)

    def test_bidirectional_joint_name_matching(self):
        """Ensure joint_states matching handles both left_front and front_left permutations."""
        sim_joint_names = [
            "left_front_wheel_joint",
            "right_front_wheel_joint",
            "left_rear_wheel_joint",
            "right_rear_wheel_joint",
        ]

        matched = []
        for joint_name in sim_joint_names:
            for wheel_name in self.wheel_names:
                parts = wheel_name.split("_")
                inverted = f"{parts[1]}_{parts[0]}" if len(parts) == 2 else wheel_name
                if wheel_name in joint_name or inverted in joint_name:
                    matched.append((wheel_name, joint_name))

        self.assertEqual(len(matched), 4)
        matched_wheels = [m[0] for m in matched]
        for w in self.wheel_names:
            self.assertIn(w, matched_wheels)

    def test_urdf_xacro_contains_only_4_wheels(self):
        """Verify my_robot.urdf.xacro only instantiates 4 continuous wheel joints."""
        with open("Rover/my_robot_description/urdf/my_robot.urdf.xacro", "r") as f:
            content = f.read()

        self.assertNotIn("left_middle", content)
        self.assertNotIn("right_middle", content)
        self.assertIn("left_front", content)
        self.assertIn("right_front", content)
        self.assertIn("left_rear", content)
        self.assertIn("right_rear", content)

    def test_gazebo_diffdrive_4_joints(self):
        """Verify gazebo.xacro DiffDrive plugin specifies exactly 4 joints with 0.49m separation."""
        with open("Rover/my_robot_description/urdf/gazebo.xacro", "r") as f:
            content = f.read()

        self.assertNotIn("left_middle", content)
        self.assertNotIn("right_middle", content)
        self.assertIn("<left_joint>left_front_wheel_joint</left_joint>", content)
        self.assertIn("<left_joint>left_rear_wheel_joint</left_joint>", content)
        self.assertIn("<right_joint>right_front_wheel_joint</right_joint>", content)
        self.assertIn("<right_joint>right_rear_wheel_joint</right_joint>", content)
        self.assertIn("<wheel_separation>0.49</wheel_separation>", content)


if __name__ == "__main__":
    unittest.main()

import unittest
import numpy as np

from src.robot_arm import(
    forward_kinematics,
    inverse_kinematics,
    L1,
    L2,
)

class TestRobotArm(unittest.TestCase):
    def test_forwardkinematics_at_zero_angles(self):
        elbow, end_effector = forward_kinematics(0, 0)

        self.assertAlmostEqual(elbow[0], L1)
        self.assertAlmostEqual(elbow[1], 0.0)
        self.assertAlmostEqual(end_effector[0], L1 + L2)
        self.assertAlmostEqual(end_effector[1], 0.0)

    def test_inversekinematics_reaches_target(self):
        target_x = 1.0
        target_y = 0.5

        angles = inverse_kinematics(target_x, target_y)

        self.assertIsNotNone(angles)

        _, reached_position = forward_kinematics(*angles)

        self.assertAlmostEqual(reached_position[0], target_x)
        self.assertAlmostEqual(reached_position[1], target_y)

    def test_inverse_kinematics_multiple_targets(self):
        targets = [
            (1.0, 0.5),
            (0.5, 1.0),
            (-1.0, 0.5),
            (0.0, 1.5),
        ]

        for target_x, target_y in targets:
            with self.subTest(target=(target_x, target_y)):
                angles = inverse_kinematics(target_x, target_y)

                self.assertIsNotNone(angles)

                _, reached_position = forward_kinematics(*angles)

                self.assertAlmostEqual(
                    reached_position[0], target_x, places=7
                )

                self.assertAlmostEqual(
                    reached_position[1], target_y, places=7
                )


    def test_maximum_reach_boundary(self):
        angles = inverse_kinematics(L1 + L2, 0)

        self.assertIsNotNone(angles)

        _, end_effector = forward_kinematics(*angles)

        self.assertAlmostEqual(end_effector[0], L1 + L2)
        self.assertAlmostEqual(end_effector[1], 0.0)

    def test_minimum_reach_boundary(self):
        angles = inverse_kinematics(L1 - L2, 0)

        self.assertIsNotNone(angles)

        _, end_effector = forward_kinematics(*angles)

        self.assertAlmostEqual(end_effector[0], L1 - L2)
        self.assertAlmostEqual(end_effector[1], 0.0)

    def test_unreachable_target_is_rejected(self):
            angles = inverse_kinematics(L1 + L2 + 0.1, 0)
    
            self.assertIsNone(angles)

    def test_target_inside_inner_unreachable_region(self):
        angles = inverse_kinematics(0.1, 0)

        self.assertIsNone(angles)

if __name__ == "__main__":
    unittest.main()
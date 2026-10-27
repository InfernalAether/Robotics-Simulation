import numpy as np
import matplotlib.pyplot as plt

#Arm link lengths in meters
L1 = 1.0
L2 = 0.8

def forward_kinematics(theta1_deg, theta2_deg):
    """Calcuate the joint and end-effector positions of a 2-link robot arm."""

    #Convert joint angles from degrees to radians
    theta1 = np.deg2rad(theta1_deg)
    theta2 = np.deg2rad(theta2_deg)

    #Position of the first joint
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)

    #Position of the end effector
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)

    return (x1, y1), (x2, y2)

def inverse_kinematics(x, y):
    """Calculate joint angles needed to reach a target (x, y)"""

    distance_squared = x**2 + y**2

    cos_theta2 = (distance_squared - L1**2 - L2**2) / (2 * L1 * L2)

    #Reject targets outside the arm's reachable workspace
    if cos_theta2 < -1 or cos_theta2 > 1:
        return None

    theta2 = np.arccos(cos_theta2)
    theta1 = np.arctan2(y, x) - np.arctan2(
        L2 * np.sin(theta2),
        L1 + L2 * np.cos(theta2)
    )

    return np.rad2deg(theta1), np.rad2deg(theta2)

def plot_robot_arm(theta1_deg, theta2_deg):
    """Plots the two-link robot arm for the given joint angles"""

    elbow, end_effector = forward_kinematics(theta1_deg, theta2_deg)

    x_positions = [0, elbow[0], end_effector[0]]
    y_positions = [0, elbow[1], end_effector[1]]

    plt.figure(figsize=(7,7))
    plt.plot(x_positions[:2], y_positions[:2], "o-", linewidth=4, markersize=8, label="Link 1")
    plt.plot(x_positions[1:], y_positions[1:], "o-", linewidth=4, markersize=8, label="Link 2")
    plt.scatter(0, 0, color="black", s=100, label="Base")

    plt.xlim(-2,2)
    plt.ylim(-2,2)
    plt.xlabel("X position (m)")
    plt.ylabel("Y position (m)")
    plt.title("Two-Link Robot Arm")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()


#Test forward kinematics
theta1 = 45
theta2 = 30

elbow, end_effector = forward_kinematics(theta1, theta2)

print("Elbow position:", elbow)
print("End-effector position:", end_effector)

plot_robot_arm(theta1, theta2)

# Test inverse kinematics
target_x = 1.0
target_y = 0.5

angles = inverse_kinematics(target_x, target_y)

if angles is not None:
    print("Target joint angles:", angles)

    elbow, end_effector = forward_kinematics(*angles)
    print("Reached position:", end_effector)
else:
    print("Target is outside the robot's reachable workspace.")
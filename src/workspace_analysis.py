import numpy as np
import matplotlib.pyplot as plt

from robot_arm import forward_kinematics, L1, L2

# Workspace analysis settings
num_angles = 200

print("Robot arm link lengths:")
print(f"Link 1: {L1} m")
print(f"Link 2: {L2} m")

# Theoretical reachable workspace boundaries
min_reach = abs(L1 - L2)
max_reach = L1 + L2

print(f"Minimum reach: {min_reach:.2f} m")
print(f"Maximum reach: {max_reach:.2f} m")

# Generate join t angle combinations
theta1_values = np.linspace(0, 360, num_angles, endpoint=False)
theta2_values = np.linspace(-180, 180, num_angles)

# Calculate the end-effector position for each combination
workspace_x = []
workspace_y = []

for theta1 in theta1_values:
    for theta2 in theta2_values:
        _, end_effector, = forward_kinematics(theta1, theta2)

        workspace_x.append(end_effector[0])
        workspace_y.append(end_effector[1])

# Plot the reachable workspace
plt.figure(figsize = (8, 8))
plt.scatter(workspace_x, workspace_y, s=1)
plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Reachable Workspace of a Two-Link Robot Arm")
plt.xlim(-max_reach - 0.1, max_reach + 0.1)
plt.ylim(-max_reach - 0.1, max_reach + 0.1)
plt.grid(True)
plt.axis("equal")
plt.show()

# Validate the sampled workspace distances
workspace_distances = np.sqrt(
    np.array(workspace_x)**2 + np.array(workspace_y)**2
)

sampled_min_reach = np.min(workspace_distances)
sampled_max_reach = np.max(workspace_distances)

print(f"Sampled minimum reach: {sampled_min_reach:.4f} m")
print(f"Sampled maximum reach: {sampled_max_reach:.4f} m")

tolerance = 1e-6

assert sampled_min_reach >= min_reach - tolerance
assert sampled_max_reach <= max_reach + tolerance

print("Workspace boundary validation passed")
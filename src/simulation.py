import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from robot_arm import forward_kinematics, inverse_kinematics, L1, L2

# Simulation settings
num_steps = 100

# Joint 1 sweeps steadily while joint 2 bends and straightens
time_values = np.linspace(0, 2 * np.pi, num_steps)

theta1_values = 35 + 30 * np.sin(time_values)
theta2_values = 45 + 35 * np.sin(time_values - np.pi / 2)

print("Joint 1 angles:", theta1_values[:5])
print("Joint 2 angles:", theta2_values[:5])

# Store the end-effector position at each simulation step
end_effector_x = []
end_effector_y = []

for theta1, theta2 in zip(theta1_values, theta2_values):
    elbow, end_effector = forward_kinematics(theta1, theta2)

    end_effector_x.append(end_effector[0])
    end_effector_y.append(end_effector[1])

plt.figure(figsize=(8,6))
plt.plot(end_effector_x, end_effector_y, label="End-effector path")
plt.scatter(end_effector_x[0], end_effector_y[0], color="green", label="Start")
plt.scatter(end_effector_x[-1], end_effector_y[-1], color="red", label="End")

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Robot End-Effector Trajectory")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.show()

fig, ax = plt.subplots(figsize=(7,7))

arm_line, = ax.plot([], [], "o-", linewidth=4, markersize=8)
path_line, = ax.plot([], [], "--", alpha=0.6, label="End-effector path")

ax.set_xlim(-(L1+L2), L1+L2)
ax.set_ylim(-(L1+L2), L1+L2)
ax.set_xlabel("X position (m)")
ax.set_ylabel("Y position (m)")
ax.set_title("Two-Link Robot Arm Motion")
ax.grid(True)
ax.set_aspect("equal")
ax.legend()

def update(frame):
    elbow, end_effector = forward_kinematics (
        theta1_values[frame], theta2_values[frame]
    )

    arm_line.set_data(
        [0, elbow[0], end_effector[0]],
        [0, elbow[1], end_effector[1]]
    )

    path_line.set_data(
        end_effector_x[:frame + 1],
        end_effector_y[:frame + 1]
    )

    return arm_line, path_line

animation = FuncAnimation(
    fig, update, frames=num_steps, interval=50, blit=False
)

plt.show()

# Test moving the robot to a target position
target_x = 1.0
target_y = 0.5

target_angles = inverse_kinematics(target_x, target_y)

if target_angles is not None:
     # Animate the arm moving to the target
    start_theta1 = theta1_values[0]
    start_theta2 = theta2_values[0]

    target_theta1, target_theta2 = target_angles
    
    _, final_position = forward_kinematics(
        target_theta1, target_theta2
    )

    print("Target position:", (target_x, target_y))
    print("Reached position:", final_position)
    print(
        "Position error (m):",
        np.sqrt(
            (final_position[0] - target_x)**2
            + (final_position[1] - target_y)**2
        )
    )

    move_steps = 60
    move_theta1 = np.linspace(start_theta1, target_theta1, move_steps)
    move_theta2 = np.linspace(start_theta2, target_theta2, move_steps)

    fig_target, ax_target = plt.subplots(figsize=(7, 7))

    arm_line, = ax_target.plot([], [], "o-", linewidth=4, markersize=8)
    ax_target.scatter(
        target_x, target_y, color="red", marker="x",
        s=100, label="Target"
    )

    ax_target.set_xlim(-(L1 + L2), L1 + L2)
    ax_target.set_ylim(-(L1 + L2), L1 + L2)
    ax_target.set_xlabel("X position (m)")
    ax_target.set_ylabel("Y position (m)")
    ax_target.set_title("Robot Arm Moving to Target")
    ax_target.grid(True)
    ax_target.set_aspect("equal")
    ax_target.legend()

    def update_target(frame):
        elbow, end_effector = forward_kinematics(
            move_theta1[frame], move_theta2[frame]
        )

        arm_line.set_data(
            [0, elbow[0], end_effector[0]],
            [0, elbow[1], end_effector[1]]
        )

        return (arm_line,)

    target_animation = FuncAnimation(
    fig_target,
    update_target,
    frames=move_steps,
    interval=50,
    blit=False,
    repeat=False
    )

    plt.show()

else:
    print("Target is outside the robot's reachable workspace.")
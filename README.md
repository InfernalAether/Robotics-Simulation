# Robotics Simulation: Two-Link Robot Arm

This is a Python-based robotics projects that simulated a planar two-link robot arm. The project incorporates foward kinematics, inverse kinematics, end-effector motion, and reachable workspace analysis.

## Project Overview

This project uses mathematical modeling and Python visualization to study the motion and reachable positions of a two-link robotic arm. This includes kinematics calculations, target-reaching simulation, workspace vizualization, and automated tests.

## Engineering Model

The robot is modeled as a planar, two-linked manipulator with two rotational joints. Each link is treated as a rigid segmenet, and the joint angles determine the end-effector position in the XY plane.

### Robot Parameters

|   Parameter   | Symbol |   Value  |
|---------------|--------|----------|
| Link 1 length |   L1   |   1.0 m  |
| Link 2 length |   L2   |   0.8 m  |
| Joint 1 angle |   θ1   | Variable |
| Joint 2 angle |   θ2   | Variable |

### Forward Kinematics

We use forward kinematics to calculate the elbow and end effector positions from the two joint angles

The elbow position is:

x1 = L1 cos(θ1)
y1 = L1 sin(θ1)

The end effector position is:

x2 = L1 cos(θ1) + L2 cos(θ1 + θ2)
y2 = L1 sin(θ1) + L2 sin(θ1 + θ2)

### Inverse Kinematics

Inverse Kinematics calculates joint angles for a desired end-effector position (x, y)

Using the law of cosines:

cos(θ2) = (x² + y² - L1² - L2²) / (2 L1 L2)

For the implemented solution:

θ2 = arccos(cos(θ2))
θ1 = atan2(y, x) - atan2(L2 sin(θ2), L1 + L2 cos(θ2))

The implementation returns an unreachable-target result when the
calculated cosine falls outside the valid range [-1, 1].

### Workspace Boundaries

The theoretical minimum and maximum distances from the robot base
are:

r_min = |L1 - L2| = 0.2 m
r_max = L1 + L2 = 1.8 m

These boundaries describe the ideal reachable radial range for the
two-link arm, assuming unrestricted joint rotation.

## Project Features

### 1. Robot Arm Kinematics
File: `src/robot_arm.py`

- Calculates elbow and end-effecotr positions using forward kinematics
- Solves inverse kinematics for reachable target positions
- Rejects targets outside the supported reachable range
- Visualizes the two-link robot arm configuration

### 2. Motion Simulation
File: `src/simulation.py`

- Simulates robot arm movement using time-varying joint angles
- Visualizes end-effector trajectory
- Animates the robot arm moving towards a target position
- Reports the final end-effector position and target-reaching error.

### 3. WOrkspace Analysis
File: `src/workspace_analysis.py`

- Samples combinations of joint angles.
- Plots the resulting end-effector positions.
- Compares sampled reach distances against theoretical workspace boundaries.
- Checks that sampled positions remain within the expected radial limits.

### 4. Automated Testing
File: `tests/test_robot_arm.py`

The automated test suite checks:

- Forward kinematics at zero joint angles.
- Inverse kinematics for reachable targets.
- Multiple target positions.
- Rejection of unreachable targets.
- Maximum and minimum reach boundaries.
- Targets inside the inner unreachable region.


## Results and Validation

### Workspace Analysis

The workspace analysis was validated against the theoretical reach boundaries of the two-link robot arm.

| Measurement               | Result   | 
|---------------------------|----------|
| Theoretical minimum reach | 0.20 m   |
| Sampled minimum reach     | 0.2000 m |
| Theoretical maximum reach | 1.80 m   |
| Sampled maximum reach     | 1.7999 m |

The sampled workspace satisfied the theoretical radial limits.
The small difference between the sampled and theoretical maximum reach is due to finite joint-angle sampling.

### Inverse Kinematics Validation

A target position of (1.0, 0.5) m was tested using inverse kinematics.

- Target position: (1.0, 0.5) m
- Calculated end-effector position: approximately (1.0, 0.5) m
- Position error: approximately 1.11 × 10⁻¹⁶ m

The calculated position closely matched the target within floating-point precision.

### Automated Tests

The project includes seven automated unit tests covering forward kinematics, inverse kinematics, reachable and unreachable targets, and workspace boundary cases.

All seven tests passed during the latest verified test run.


## Assumptions and Limitations

- The robot arm is modeled as a planar, two-link mechanism operating in
  the XY plane.
- Both links are treated as rigid, with fixed lengths of 1.0 m and 0.8 m.
- Joint rotations are modeled without mechanical limits or collisions.
- The workspace analysis uses a finite set of sampled joint angles.
- Inverse kinematics currently returns one elbow configuration rather
  than enumerating both possible configurations.
- The model does not include link mass, joint torque, motor dynamics,
  friction, or structural deformation.
- The simulation is a kinematic visualization, not a physical dynamics
  simulation.

The results are intended to demonstrate robotic kinematics and workspace analysis. They do not represent a complete physical robot model.
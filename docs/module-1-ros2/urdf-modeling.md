# URDF and Robot Modeling

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the structure and components of URDF files
- Create basic robot models using URDF
- Define links, joints, and their properties
- Visualize robot models in RViz
- Apply transformations and coordinate frames

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid parts), joints (connections between links), and other properties like inertial parameters, visual meshes, and collision properties.

## URDF Structure

A URDF file contains:
- **Links**: Rigid parts of the robot (e.g., base, arms, wheels)
- **Joints**: Connections between links (e.g., rotational, prismatic)
- **Visual elements**: How the robot appears in simulation
- **Collision elements**: How the robot interacts with the environment
- **Inertial properties**: Mass, center of mass, and inertia tensor

## Links

Links represent rigid parts of the robot. Each link can have:
- **Visual**: How the link appears (shape, material, mesh)
- **Collision**: How the link interacts with the environment
- **Inertial**: Physical properties for simulation

### Common visual shapes:
- `<box>`: Rectangular prism
- `<cylinder>`: Cylindrical shape
- `<sphere>`: Spherical shape
- `<mesh>`: Complex shape from external file

## Joints

Joints define how links connect and move relative to each other. Joint types include:
- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Linear sliding movement
- **Planar**: Movement in a plane
- **Floating**: 6 DOF movement

## Example URDF Robot

Here's a simple differential drive robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Right wheel -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin rpy="1.570796 0 0"/>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
```

## Coordinate Frames and Transformations

ROS uses the tf2 (Transform Library) to keep track of coordinate frames in a tree structure over time. Each link in a URDF has its own coordinate frame, and tf2 manages the transformations between them.

### Key concepts:
- **Frame ID**: Unique identifier for each coordinate frame
- **Static transforms**: Fixed relationships between frames
- **Dynamic transforms**: Changing relationships over time

## Visualization in RViz

RViz is ROS's 3D visualization tool. To visualize a URDF robot:
1. Launch RViz
2. Add a RobotModel display
3. Set the Robot Description parameter to your URDF topic
4. Set Fixed Frame to the robot's base frame

## Xacro: URDF Macros

Xacro is an XML macro language that extends URDF with:
- Variables and constants
- Mathematical expressions
- Macros for repeated elements
- File inclusion

## Best Practices

- Use meaningful names for links and joints
- Ensure proper inertial properties for simulation
- Use consistent units (typically meters, kilograms, seconds)
- Validate URDF files using tools like `check_urdf`
- Separate complex URDFs into multiple files for modularity

## Common Issues and Troubleshooting

- **Invalid XML**: Check syntax and structure
- **Floating joints**: Ensure all links connect to the base
- **Simulation instability**: Verify inertial properties
- **Visualization problems**: Check coordinate frames and mesh paths

## Summary

URDF is essential for describing robot models in ROS. Understanding how to create and structure URDF files is crucial for robot simulation, visualization, and control. Proper URDF models enable accurate simulation and help with robot development and testing.

## Exercises

1. Create a simple URDF model of a robot arm with 3 joints
2. Visualize your robot model in RViz
3. Add a gripper to your robot arm model
4. Use Xacro to create a parameterized version of your robot
5. Validate your URDF file using the `check_urdf` command
6. Create a mobile robot model with differential drive
7. Add collision properties to your robot model and test in Gazebo

## Resources

- [URDF Documentation](https://wiki.ros.org/urdf)
- [Xacro Documentation](https://wiki.ros.org/xacro)
- [tf2 Documentation](https://docs.ros.org/en/humble/p/tf2/)
- [URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF.html)
# Chapter 3: Humanoid Robot Description with URDF and Controllers

## Learning Objectives
By the end of this chapter, you will be able to:
- Understand the fundamentals of URDF (Unified Robot Description Format)
- Define humanoid robot kinematic chains using URDF
- Create joint and link specifications for humanoid robots
- Integrate controllers with URDF models
- Visualize and validate humanoid robot models in ROS 2
- Use Xacro to simplify complex URDF definitions

## Prerequisites
Before reading this chapter, you should:
- Have completed Chapters 1 and 2 and understand basic ROS 2 concepts
- Be familiar with XML syntax
- Have ROS 2 Humble Hawksbill installed on your system
- Understand basic robotics kinematics concepts

## Understanding URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML format used in ROS to describe the physical and kinematic properties of robots. It defines the robot's structure, including links (rigid bodies), joints (connections between links), and their properties.

### URDF Structure

A basic URDF file consists of:
- `<robot>`: The root element that contains the entire robot description
- `<link>`: Represents a rigid body with physical properties
- `<joint>`: Defines the connection between two links
- `<material>`: Defines visual materials
- `<gazebo>`: Gazebo-specific extensions (optional)

### Basic URDF Example

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Child link connected by a joint -->
  <link name="child_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_child" type="fixed">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0 0 0.6"/>
  </joint>
</robot>
```

## Defining Humanoid Robot Kinematic Chains

Humanoid robots have complex kinematic structures that include:

- **Torso**: The main body
- **Head**: For perception and interaction
- **Arms**: For manipulation (shoulders, elbows, wrists, hands)
- **Legs**: For locomotion (hips, knees, ankles, feet)

### Kinematic Chain Principles

In URDF, kinematic chains are represented as tree structures where:
- Each link has at most one parent
- Joints define the connection and motion between parent and child links
- The base link (usually the torso) has no parent
- Multiple chains can originate from the same base

### Humanoid Skeleton Structure

```xml
<!-- Simplified humanoid skeleton -->
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

## Creating Joint and Link Specifications for Humanoids

### Link Specifications

Links represent rigid bodies in the robot. Each link should specify:

- **Visual properties**: How the link appears in visualization
- **Collision properties**: How the link interacts in physics simulation
- **Inertial properties**: Mass, center of mass, and inertia tensor

```xml
<link name="link_name">
  <!-- Visual representation -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Shape: box, cylinder, sphere, or mesh -->
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <material name="color">
      <color rgba="0.8 0.2 0.2 1.0"/>
    </material>
  </visual>

  <!-- Collision properties -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>
```

### Joint Specifications

Joints define how links connect and move relative to each other. Common joint types for humanoid robots:

- **revolute**: Rotational joint with limited range
- **continuous**: Rotational joint without limits
- **prismatic**: Linear sliding joint
- **fixed**: No movement between links

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

## Integrating Controllers with URDF Models

Controllers in ROS 2 are typically defined separately from the URDF but reference joint names defined in the URDF. The `ros2_control` framework provides a standardized way to integrate controllers.

### ros2_control Integration

```xml
<!-- Add ros2_control interface to URDF -->
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <!-- Define joints for control -->
  <joint name="left_shoulder_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="right_shoulder_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

### Controller Configuration

Controllers are typically configured in separate YAML files:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    left_arm_controller:
      type: position_controllers/JointTrajectoryController

left_arm_controller:
  ros__parameters:
    joints:
      - left_shoulder_joint
      - left_elbow_joint
      - left_wrist_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity
```

## Visualizing and Validating Humanoid Robot Models

### URDF Validation

Before using a URDF model, it's important to validate it:

```bash
# Check URDF syntax
check_urdf /path/to/robot.urdf

# View the robot model
urdf_to_graphiz /path/to/robot.urdf
```

### Visualization with RViz2

To visualize your humanoid robot in RViz2:

1. Launch your robot state publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat robot.urdf)'
```

2. Launch RViz2:
```bash
rviz2
```

3. Add a RobotModel display and set the topic to `/robot_description`

### Example: Complete Humanoid URDF

Here's a more complete example of a simple humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0" ixz="0" iyy="0.004" iyz="0" izz="0.004"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.35"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0.1 0.1" rpy="0 0 0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Right Arm (similar to left, mirrored) -->
  <link name="right_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.15 -0.1 0.1" rpy="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_leg"/>
    <origin xyz="-0.1 0.05 -0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.06"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 -0.05 -0.25"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="200" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.015" ixy="0" ixz="0" iyy="0.015" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="200" velocity="1"/>
  </joint>
</robot>
```

## Using Xacro for Complex URDF Models

Xacro (XML Macros) is a macro language for XML that allows you to create more maintainable and reusable URDF files by:

- Defining properties and reusing them
- Creating macros for repeated structures
- Including other xacro files

### Basic Xacro Example

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_humanoid">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.5" />
  <xacro:property name="torso_radius" value="0.15" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side reflect:=1">
    <link name="${side}_upper_arm">
      <visual>
        <geometry>
          <cylinder length="0.3" radius="0.05"/>
        </geometry>
      </visual>
    </link>

    <joint name="${side}_shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0.15 ${0.1*reflect} 0.1" rpy="0 0 ${0.5*reflect}"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:arm side="left" reflect="1"/>
  <xacro:arm side="right" reflect="-1"/>

</robot>
```

## Summary

This chapter covered the fundamentals of URDF for describing humanoid robots. We explored how to define kinematic chains, create proper link and joint specifications, integrate controllers, and visualize and validate robot models. These concepts are essential for creating realistic humanoid robot models that can be used in simulation and real-world applications.

## Exercises

### Exercise 1: Beginner - URDF Validation
Create a simple URDF file for a 3-link robot arm and validate it using ROS 2 tools. Visualize the robot in RViz2.

### Exercise 2: Intermediate - Humanoid Leg
Create a URDF model for a single humanoid leg with hip, knee, and ankle joints. Include proper inertial properties and visualize it.

### Exercise 3: Advanced - Xacro Humanoid
Create a complete humanoid robot using Xacro macros to reduce code duplication. Include at least 12 joints and proper controller interfaces.

## Further Reading
- URDF Documentation: http://wiki.ros.org/urdf
- Xacro Documentation: http://wiki.ros.org/xacro
- ros2_control Documentation: https://control.ros.org/
- Robot Modeling and Control by Spong, Hutchinson, and Vidyasagar
#!/bin/bash

echo "🤖 Creating Standalone UR5 Robot"
echo "================================"

# This creates a self-contained UR5 robot description
cat > /tmp/standalone_ur5.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="ur5">

  <!-- Arguments -->
  <xacro:arg name="limited" default="false"/>
  <xacro:arg name="transmission_hw_interface" default="hardware_interface/PositionJointInterface"/>

  <!-- World link -->
  <link name="world" />

  <joint name="world_joint" type="fixed">
    <parent link="world" />
    <child link="base_link" />
    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
  </joint>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
      <material name="base_material">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.1" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Shoulder link -->
  <link name="shoulder_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.15"/>
      </geometry>
      <material name="shoulder_material">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3.7"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0.0 0.0 0.089159" rpy="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="150.0" velocity="3.15"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm_link">
    <visual>
      <origin xyz="0 0 0.2125" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.425"/>
      </geometry>
      <material name="arm_material">
        <color rgba="0.9 0.9 0.9 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.2125" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.425"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="8.393"/>
      <origin xyz="0 0 0.2125" rpy="0 0 0"/>
      <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.02" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <origin xyz="0.0 0.13585 0.0" rpy="0.0 1.570796 0.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="150.0" velocity="3.15"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm_link">
    <visual>
      <origin xyz="0 0 0.196" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.392"/>
      </geometry>
      <material name="forearm_material">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.196" rpy="0 0 0"/>
      <geometry>
        <box size="0.08 0.08 0.392"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.275"/>
      <origin xyz="0 0 0.196" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <origin xyz="0.0 -0.1197 0.425" rpy="0.0 0.0 0.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="150.0" velocity="3.15"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Wrist 1 link -->
  <link name="wrist_1_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="wrist_material">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.219"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link"/>
    <child link="wrist_1_link"/>
    <origin xyz="0.0 0.0 0.39225" rpy="0.0 1.570796 0.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="28.0" velocity="3.2"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Wrist 2 link -->
  <link name="wrist_2_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
      <material name="wrist_material">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.219"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link"/>
    <child link="wrist_2_link"/>
    <origin xyz="0.0 0.093 0.0" rpy="0.0 0.0 0.0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="28.0" velocity="3.2"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Wrist 3 link -->
  <link name="wrist_3_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.08"/>
      </geometry>
      <material name="wrist_material">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.04" length="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1879"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <origin xyz="0.0 0.0 0.09465" rpy="0.0 0.0 0.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.14159" upper="3.14159" effort="28.0" velocity="3.2"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Gripper -->
  <link name="gripper_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
      <material name="gripper_material">
        <color rgba="0.8 0.8 0.8 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.05 0.05 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="gripper_joint" type="fixed">
    <parent link="wrist_3_link"/>
    <child link="gripper_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <!-- Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
      <material name="camera_material">
        <color rgba="0.2 0.2 0.2 1"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.02 0.02 0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.01"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="wrist_3_link"/>
    <child link="camera_link"/>
    <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  </joint>

</robot>
EOF

echo "✅ Standalone UR5 robot created at /tmp/standalone_ur5.urdf.xacro"
echo ""
echo "🚀 Test it with:"
echo "export ROBOT_DESCRIPTION=\$(xacro /tmp/standalone_ur5.urdf.xacro)"
# Module 1: The Robotic Nervous System (ROS 2) - Table of Contents

## Overview
This module introduces ROS 2 as the robotic nervous system, covering fundamental concepts, Python integration, and humanoid robot description with URDF. The module consists of three chapters designed to build knowledge progressively from basic concepts to advanced applications.

## Chapter List

### [Chapter 1: Introduction to ROS 2 and the Robotic Nervous System](./chapter-1-introduction-to-ros2.md)
- Learning Objectives
- Prerequisites
- Understanding ROS 2 Architecture and Its Role as a Robotic Nervous System
  - The Need for a Robotic Operating System
  - ROS 2 vs. ROS 1
- Core Concepts: Nodes, Topics, Services, Actions
  - Nodes
  - Topics and Publisher-Subscriber Pattern
  - Services
  - Actions
- Setting Up the ROS 2 Environment for Humanoid Robotics
  - Workspace Creation
  - Sourcing ROS 2
  - Building the Workspace
- Basic Publisher-Subscriber Patterns
  - Quality of Service (QoS) Settings
  - Running Publisher and Subscriber
- Introduction to ROS 2 Tools
  - ros2 topic
  - ros2 service
  - ros2 node
  - ros2 action
- Summary
- Exercises
  - Exercise 1: Beginner - Node Creation
  - Exercise 2: Intermediate - Publisher-Subscriber Pair
  - Exercise 3: Advanced - QoS Configuration
- Further Reading

### [Chapter 2: Python Agents and ROS 2 Integration (rclpy)](./chapter-2-python-agents-ros2.md)
- Learning Objectives
- Prerequisites
- Introduction to rclpy for Python-based ROS 2 Development
  - Installing rclpy
  - Basic rclpy Concepts
- Creating Python Nodes that Interact with ROS 2
  - Node Structure
  - Node Lifecycle Management
- Implementing Publishers and Subscribers in Python
  - Advanced Publisher with Error Handling
  - Advanced Subscriber with QoS Settings
- Working with ROS 2 Services and Clients in Python
  - Service Server with Error Handling
  - Service Client with Error Handling
- Best Practices for Integrating AI Agents with ROS 2 Controllers
  - AI Agent Integration Pattern
  - Example: AI Agent Node
- Summary
- Exercises
  - Exercise 1: Beginner - Parameter Server
  - Exercise 2: Intermediate - Custom Message Types
  - Exercise 3: Advanced - AI Agent with Learning
- Further Reading

### [Chapter 3: Humanoid Robot Description with URDF and Controllers](./chapter-3-urdf-humanoid-robots.md)
- Learning Objectives
- Prerequisites
- Understanding URDF (Unified Robot Description Format)
  - URDF Structure
  - Basic URDF Example
- Defining Humanoid Robot Kinematic Chains
  - Kinematic Chain Principles
  - Humanoid Skeleton Structure
- Creating Joint and Link Specifications for Humanoids
  - Link Specifications
  - Joint Specifications
- Integrating Controllers with URDF Models
  - ros2_control Integration
  - Controller Configuration
- Visualizing and Validating Humanoid Robot Models
  - URDF Validation
  - Visualization with RViz2
  - Example: Complete Humanoid URDF
- Using Xacro for Complex URDF Models
  - Basic Xacro Example
- Summary
- Exercises
  - Exercise 1: Beginner - URDF Validation
  - Exercise 2: Intermediate - Humanoid Leg
  - Exercise 3: Advanced - Xacro Humanoid
- Further Reading

## Learning Path

This module is designed to be completed in sequence:

1. **Start with Chapter 1** to understand the fundamentals of ROS 2 architecture and basic communication patterns.
2. **Proceed to Chapter 2** to learn how to implement ROS 2 concepts in Python using rclpy.
3. **Complete Chapter 3** to learn about robot description using URDF and controller integration.

Each chapter builds upon the previous one, with Chapter 2 requiring knowledge from Chapter 1, and Chapter 3 requiring understanding from both previous chapters.

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Documentation](http://wiki.ros.org/urdf)
- [ros2_control Documentation](https://control.ros.org/)

## Prerequisites for This Module

Before starting this module, you should:
- Have a basic understanding of robotics concepts
- Be familiar with Python programming
- Have ROS 2 Humble Hawksbill installed on your system
# Chapter 1: Introduction to ROS 2 and the Robotic Nervous System

## Learning Objectives
By the end of this chapter, you will be able to:
- Understand the core concepts of ROS 2 and its role as a robotic nervous system
- Explain the architecture of ROS 2 and its key components
- Describe the fundamental concepts: Nodes, Topics, Services, and Actions
- Set up the ROS 2 environment for humanoid robotics applications
- Implement basic publisher-subscriber patterns in ROS 2
- Use ROS 2 tools such as ros2 topic, ros2 service, and others

## Prerequisites
Before reading this chapter, you should:
- Have a basic understanding of robotics concepts
- Be familiar with Python programming
- Have ROS 2 Humble Hawksbill installed on your system
- Have completed the ROS 2 installation requirements as outlined in the textbook

## Understanding ROS 2 Architecture and Its Role as a Robotic Nervous System

ROS 2 (Robot Operating System 2) serves as the "nervous system" of robotic applications, providing a framework for developing robot software. Just as the nervous system connects different parts of the human body and allows them to communicate, ROS 2 connects different components of a robot and enables them to exchange information seamlessly.

### The Need for a Robotic Operating System

Traditional robotics development often involves tightly coupled components that are difficult to maintain, extend, and reuse. ROS 2 addresses these challenges by providing:

- **Modularity**: Components can be developed, tested, and maintained independently
- **Reusability**: Components can be shared and reused across different robotic platforms
- **Flexibility**: Communication between components can be reconfigured without code changes
- **Scalability**: Systems can be extended with new components without affecting existing ones

### ROS 2 vs. ROS 1

ROS 2 is the next generation of the Robot Operating System, addressing limitations of ROS 1:

- **Real-time support**: ROS 2 provides real-time capabilities critical for robot control
- **Multi-robot systems**: Better support for coordinating multiple robots
- **Security**: Built-in security features for protecting robotic systems
- **Quality of Service (QoS)**: Configurable reliability and performance settings
- **OS portability**: Support for multiple operating systems including Windows and macOS

## Core Concepts: Nodes, Topics, Services, Actions

ROS 2 is built around several core concepts that enable distributed robotics applications.

### Nodes

A node is an executable process that uses ROS 2 to communicate with other nodes. In the context of humanoid robotics, nodes might include:

- Joint controllers
- Sensor data processors
- Motion planners
- Perception systems
- User interfaces

#### Creating a Node

In Python, nodes are created by subclassing `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        # Node initialization code goes here
        self.get_logger().info('Minimal node created')

def main(args=None):
    rclpy.init(args=args)
    minimal_node = MinimalNode()

    # Keep the node alive
    rclpy.spin(minimal_node)

    # Clean up
    minimal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Publisher-Subscriber Pattern

Topics enable asynchronous communication between nodes using a publish-subscribe pattern. This is particularly useful in humanoid robotics where sensor data flows continuously from sensors to processing nodes.

- **Publisher**: A node that sends messages to a topic
- **Subscriber**: A node that receives messages from a topic
- **Message**: The data structure exchanged between nodes

#### Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Services

Services provide synchronous request-response communication between nodes. This is useful when a node needs to request specific information or perform an action and wait for the result.

- **Service Server**: A node that provides a service
- **Service Client**: A node that calls a service
- **Request/Response**: The data structures exchanged during a service call

### Actions

Actions provide goal-oriented communication with feedback, suitable for long-running tasks. In humanoid robotics, actions are often used for:

- Navigation goals
- Manipulation tasks
- Complex motion sequences

## Setting Up the ROS 2 Environment for Humanoid Robotics

### Workspace Creation

For humanoid robotics development, it's recommended to create a dedicated workspace:

```bash
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws
```

### Sourcing ROS 2

Before working with ROS 2, you need to source the setup script:

```bash
source /opt/ros/humble/setup.bash
```

To make this permanent, add it to your shell profile:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Building the Workspace

After adding packages to your workspace, build it:

```bash
colcon build
source install/setup.bash
```

## Basic Publisher-Subscriber Patterns

### Quality of Service (QoS) Settings

QoS settings control the reliability and performance of communication between nodes:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# Create a QoS profile for reliable communication
qos_profile = QoSProfile(
    depth=10,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE
)
```

### Running Publisher and Subscriber

To run the publisher and subscriber examples:

1. Terminal 1 - Run the publisher:
   ```bash
   ros2 run my_package publisher_node
   ```

2. Terminal 2 - Run the subscriber:
   ```bash
   ros2 run my_package subscriber_node
   ```

## Introduction to ROS 2 Tools

ROS 2 provides several command-line tools for inspecting and debugging robotic systems:

### ros2 topic

- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 topic info <topic_name>`: Get information about a topic

### ros2 service

- `ros2 service list`: List all active services
- `ros2 service call <service_name> <service_type> <arguments>`: Call a service

### ros2 node

- `ros2 node list`: List all active nodes
- `ros2 node info <node_name>`: Get information about a node

### ros2 action

- `ros2 action list`: List all active actions
- `ros2 action send_goal <action_name> <action_type> <goal>`: Send an action goal

## Summary

This chapter introduced the fundamental concepts of ROS 2 as the robotic nervous system. We explored the architecture of ROS 2, its core concepts including nodes, topics, services, and actions, and how to set up the environment for humanoid robotics. We also implemented basic publisher-subscriber patterns and introduced key ROS 2 tools for development and debugging.

## Exercises

### Exercise 1: Beginner - Node Creation
Create a simple ROS 2 node that prints "Hello from Chapter 1!" to the console every 2 seconds. Name the node "chapter1_hello".

### Exercise 2: Intermediate - Publisher-Subscriber Pair
Create a publisher-subscriber pair that exchanges a custom message containing a counter value. The publisher should increment the counter every second, and the subscriber should print the received value.

### Exercise 3: Advanced - QoS Configuration
Modify the publisher-subscriber example to use different QoS profiles (reliable vs best-effort) and observe the differences in behavior when network conditions change.

## Further Reading
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- Design ROS 2 Packages: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html
- Quality of Service: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
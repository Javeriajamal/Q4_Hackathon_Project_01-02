# Chapter 2: Python Agents and ROS 2 Integration (rclpy)

## Learning Objectives
By the end of this chapter, you will be able to:
- Use rclpy for Python-based ROS 2 development
- Create Python nodes that interact with ROS 2
- Implement publishers and subscribers in Python with proper error handling
- Work with ROS 2 services and clients in Python
- Apply best practices for integrating AI agents with ROS 2 controllers
- Use Quality of Service (QoS) settings appropriately in Python nodes

## Prerequisites
Before reading this chapter, you should:
- Have completed Chapter 1 and understand basic ROS 2 concepts
- Be familiar with Python programming
- Have ROS 2 Humble Hawksbill installed on your system
- Understand the basics of object-oriented programming in Python

## Introduction to rclpy for Python-based ROS 2 Development

rclpy is the Python client library for ROS 2, providing Python bindings for the ROS 2 client library (rcl). It allows Python developers to create ROS 2 nodes, publish and subscribe to topics, provide and use services, and interact with actions.

### Installing rclpy

rclpy is included with the ROS 2 installation. To ensure it's available in your Python environment:

```bash
pip3 install rclpy
```

Or ensure ROS 2 is sourced in your environment:
```bash
source /opt/ros/humble/setup.bash
```

### Basic rclpy Concepts

rclpy follows the same architectural concepts as other ROS 2 client libraries:

- **Node**: The basic execution unit that can perform computation
- **Publisher**: Sends messages to topics
- **Subscriber**: Receives messages from topics
- **Service Server**: Provides a service that can be called
- **Service Client**: Calls a service provided by a server
- **Action Server**: Provides a goal-oriented service with feedback
- **Action Client**: Uses an action server

## Creating Python Nodes that Interact with ROS 2

### Node Structure

A basic ROS 2 node in Python follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle Management

Proper lifecycle management is crucial for robust ROS 2 nodes:

```python
import rclpy
from rclpy.node import Node

class LifecycleNode(Node):
    def __init__(self):
        super().__init__('lifecycle_node')

        # Initialize components
        self.get_logger().info('Lifecycle node initialized')

        # Create any necessary publishers, subscribers, etc.
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Timer callback executed')

    def destroy_node(self):
        # Clean up resources before destroying the node
        self.get_logger().info('Cleaning up before node destruction')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing Publishers and Subscribers in Python

### Advanced Publisher with Error Handling

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import json


class AdvancedPublisher(Node):
    def __init__(self):
        super().__init__('advanced_publisher')

        # Create a QoS profile for the publisher
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create publisher with the QoS profile
        self.publisher_ = self.create_publisher(String, 'robot_data', qos_profile)

        # Timer to publish data periodically
        self.timer = self.create_timer(0.5, self.publish_data)
        self.counter = 0

    def publish_data(self):
        try:
            # Prepare message data
            data = {
                'timestamp': self.get_clock().now().nanoseconds,
                'counter': self.counter,
                'status': 'active'
            }

            # Create and publish message
            msg = String()
            msg.data = json.dumps(data)
            self.publisher_.publish(msg)

            self.get_logger().info(f'Published: {msg.data}')
            self.counter += 1

        except Exception as e:
            self.get_logger().error(f'Error in publisher: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    publisher = AdvancedPublisher()

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down publisher')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Advanced Subscriber with QoS Settings

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import json


class AdvancedSubscriber(Node):
    def __init__(self):
        super().__init__('advanced_subscriber')

        # Create a QoS profile for the subscriber
        qos_profile = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Create subscription with the QoS profile
        self.subscription = self.create_subscription(
            String,
            'robot_data',
            self.data_callback,
            qos_profile
        )

        # Prevent unused variable warning
        self.subscription  # type: ignore

    def data_callback(self, msg):
        try:
            # Parse the received data
            data = json.loads(msg.data)

            # Process the data
            self.get_logger().info(
                f'Received data - Counter: {data["counter"]}, '
                f'Timestamp: {data["timestamp"]}, Status: {data["status"]}'
            )

        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to decode JSON: {msg.data}')
        except KeyError as e:
            self.get_logger().error(f'Missing key in data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in subscriber: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    subscriber = AdvancedSubscriber()

    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        subscriber.get_logger().info('Shutting down subscriber')
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Working with ROS 2 Services and Clients in Python

### Service Server with Error Handling

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class RobustServiceServer(Node):
    def __init__(self):
        super().__init__('robust_service_server')

        # Create a service server
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints_safe',
            self.add_two_ints_callback
        )

        self.get_logger().info('Robust service server started')

    def add_two_ints_callback(self, request, response):
        try:
            # Validate input
            if not isinstance(request.a, int) or not isinstance(request.b, int):
                self.get_logger().error('Invalid input types')
                response.sum = 0
                return response

            # Perform the calculation
            result = request.a + request.b
            response.sum = result

            self.get_logger().info(
                f'Calculated {request.a} + {request.b} = {response.sum}'
            )

        except Exception as e:
            self.get_logger().error(f'Service error: {str(e)}')
            response.sum = 0  # Return a safe default value

        return response


def main(args=None):
    rclpy.init(args=args)
    server = RobustServiceServer()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down service server')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client with Error Handling

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys
import time


class RobustServiceClient(Node):
    def __init__(self):
        super().__init__('robust_service_client')

        # Create a client for the service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints_safe')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def call_service(self, a, b):
        # Create the request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call the service asynchronously
        future = self.cli.call_async(request)

        # Wait for the result with timeout
        start_time = time.time()
        timeout = 5.0  # 5 seconds timeout

        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('Service call timed out')
                return None

        try:
            response = future.result()
            return response.sum
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return None


def main(args=None):
    rclpy.init(args=args)
    client = RobustServiceClient()

    # Parse command line arguments
    if len(sys.argv) != 3:
        print('Usage: python3 robust_client.py <int1> <int2>')
        print('Using default values: a=10, b=20')
        a, b = 10, 20
    else:
        a, b = int(sys.argv[1]), int(sys.argv[2])

    try:
        result = client.call_service(a, b)
        if result is not None:
            print(f'Result: {a} + {b} = {result}')
        else:
            print('Service call failed')
    except KeyboardInterrupt:
        print('Service call interrupted')
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Best Practices for Integrating AI Agents with ROS 2 Controllers

### AI Agent Integration Pattern

When integrating AI agents with ROS 2 controllers, consider the following patterns:

1. **Separation of Concerns**: Keep AI logic separate from ROS 2 communication
2. **Asynchronous Processing**: Use async/await for non-blocking AI processing
3. **State Management**: Properly manage AI agent state across ROS 2 callbacks
4. **Error Handling**: Implement robust error handling for AI processing failures

### Example: AI Agent Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
import numpy as np
import time


class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')

        # AI agent state
        self.joint_positions = {}
        self.target_positions = {}
        self.ai_model_initialized = False

        # Create subscribers for sensor data
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Create publishers for control commands
        self.command_pub = self.create_publisher(
            JointTrajectoryControllerState,
            'joint_commands',
            10
        )

        # Create timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        # Initialize AI model (simplified example)
        self.initialize_ai_model()

        self.get_logger().info('AI Agent Node initialized')

    def initialize_ai_model(self):
        """Initialize the AI model (placeholder implementation)"""
        try:
            # In a real implementation, this would load a trained model
            # For this example, we'll just mark initialization as complete
            self.ai_model = "dummy_model"  # Placeholder
            self.ai_model_initialized = True
            self.get_logger().info('AI model initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize AI model: {str(e)}')
            self.ai_model_initialized = False

    def joint_state_callback(self, msg):
        """Callback for joint state messages"""
        try:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self.joint_positions[name] = msg.position[i]
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {str(e)}')

    def ai_processing_callback(self):
        """Main AI processing loop"""
        if not self.ai_model_initialized:
            return

        try:
            # Get current state (simplified)
            current_state = self.get_current_state()

            # Apply AI logic to determine next action (simplified)
            action = self.compute_action(current_state)

            # Publish the action
            self.publish_action(action)

        except Exception as e:
            self.get_logger().error(f'Error in AI processing: {str(e)}')

    def get_current_state(self):
        """Get the current state of the robot"""
        # Simplified state representation
        return {
            'joint_positions': self.joint_positions.copy(),
            'timestamp': self.get_clock().now().nanoseconds
        }

    def compute_action(self, state):
        """Compute the next action based on current state (simplified)"""
        # This is where the AI logic would go
        # For this example, we'll just return a dummy action
        return {
            'joint_targets': {name: pos + 0.1 for name, pos in state['joint_positions'].items()}
        }

    def publish_action(self, action):
        """Publish the computed action"""
        try:
            # Create and publish a command message
            cmd_msg = JointTrajectoryControllerState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()

            # Set joint names and target positions
            for joint_name, target_pos in action['joint_targets'].items():
                cmd_msg.joint_names.append(joint_name)
                # In a real implementation, this would be part of a trajectory
                # For simplicity, we're just showing the concept

            self.command_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f'Error publishing action: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgentNode()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        ai_agent.get_logger().info('Shutting down AI Agent Node')
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Summary

This chapter covered advanced topics in Python-based ROS 2 development using rclpy. We explored how to create robust nodes with proper error handling, implement publishers and subscribers with Quality of Service settings, work with services and clients safely, and integrate AI agents with ROS 2 controllers. These concepts are essential for building reliable and maintainable robotic systems.

## Exercises

### Exercise 1: Beginner - Parameter Server
Create a node that uses ROS 2 parameters to configure its behavior. The node should accept parameters for publishing frequency and message content, and update its behavior when parameters change.

### Exercise 2: Intermediate - Custom Message Types
Define a custom message type for humanoid robot joint commands and implement a publisher-subscriber pair that uses this custom message type.

### Exercise 3: Advanced - AI Agent with Learning
Extend the AI agent example to include a simple learning component that adapts its behavior based on feedback from the environment.

## Further Reading
- rclpy API Documentation: https://docs.ros.org/en/humble/p/rclpy/
- Quality of Service: https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- ROS 2 Parameters: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html
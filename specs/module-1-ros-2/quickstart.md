# Quickstart Guide: Module 1 — The Robotic Nervous System (ROS 2)

## Prerequisites

Before starting with Module 1, ensure you have:

1. **ROS 2 Installation**:
   - Install ROS 2 (recommended: Humble Hawksbill or later)
   - Follow the official installation guide for your OS: https://docs.ros.org/en/humble/Installation.html
   - Source the ROS 2 setup script: `source /opt/ros/humble/setup.bash` (Linux) or equivalent for your platform

2. **Python Environment**:
   - Python 3.8 or higher
   - pip package manager
   - Virtual environment recommended (optional but good practice)

3. **Development Tools**:
   - Text editor or IDE (VS Code with ROS extension recommended)
   - Terminal/shell access

## Setting Up the Development Environment

### 1. Create a ROS 2 Workspace
```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
```

### 2. Install Required Python Packages
```bash
pip install rclpy
pip install ros2cli  # For ROS 2 command-line tools
```

### 3. Verify Installation
```bash
python3 -c "import rclpy; print('rclpy imported successfully')"
ros2 --version
```

## Running the Examples

### 1. Navigate to the Workspace
```bash
cd ~/ros2_workspace
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. Test Basic ROS 2 Functionality
```bash
# Check available ROS 2 commands
ros2 --help

# Check available topics (should show /parameter_events if no other nodes running)
ros2 topic list
```

## Creating Your First ROS 2 Node with rclpy

### 1. Create a Simple Publisher Node
Create a file named `simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
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
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2. Run the Publisher Node
```bash
python3 simple_publisher.py
```

### 3. In a new terminal, create and run a Subscriber Node
Create `simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
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
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run the subscriber in a new terminal:
```bash
python3 simple_subscriber.py
```

## Docusaurus Setup for the Textbook

### 1. Install Docusaurus
```bash
npm install -g create-docusaurus
npx create-docusaurus@latest my-textbook classic
cd my-textbook
```

### 2. Add Module 1 Content
Create the module directory and add chapter files:
```bash
mkdir -p docs/module-1-ros-2
# Add your chapter markdown files to this directory
```

### 3. Update Sidebars
Edit `sidebars.js` to include Module 1:
```javascript
module.exports = {
  docs: [
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros-2/chapter-1-introduction-to-ros2',
        'module-1-ros-2/chapter-2-python-agents-ros2',
        'module-1-ros-2/chapter-3-urdf-humanoid-robots',
      ],
    },
  ],
};
```

### 4. Start the Development Server
```bash
npm start
```

## Testing Your Code Examples

### 1. Validate Python Code
Before including code in the textbook, test it:
```bash
python3 -m py_compile your_example.py  # Check for syntax errors
```

### 2. Test ROS 2 Communication
Verify that publisher-subscriber pairs work correctly:
```bash
# Terminal 1: Run publisher
ros2 run your_package publisher_node

# Terminal 2: Check topics
ros2 topic list

# Terminal 3: Echo messages
ros2 topic echo /topic std_msgs/msg/String
```

## Troubleshooting Common Issues

### 1. Import Errors
If you get "ModuleNotFoundError" for rclpy:
```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash
```

### 2. Permission Issues
If you encounter permission errors:
```bash
# Ensure you have the right permissions
sudo chmod +x your_script.py
```

### 3. Port Conflicts
If multiple ROS 2 nodes conflict:
```bash
# Kill any running ROS 2 processes
pkill -f ros
```
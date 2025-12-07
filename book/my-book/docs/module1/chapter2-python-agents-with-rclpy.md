# Python Agents with rclpy

Integrating Python agents with ROS 2 controllers is crucial for developing sophisticated robotic behaviors, especially when leveraging AI/ML algorithms or complex decision-making logic. `rclpy`, the Python client library for ROS 2, provides the necessary tools to bridge the gap between Python-based control logic and the underlying ROS 2 communication infrastructure.

### Why `rclpy` for Python Agents?

`rclpy` allows Python programs to create ROS 2 nodes, publish/subscribe to topics, offer/call services, and manage parameters, just like their C++ counterparts (`rclcpp`). This enables:

*   **Rapid Prototyping**: Python's ease of use and rich ecosystem of libraries make it ideal for quickly developing and testing agent behaviors.
*   **AI/ML Integration**: Seamlessly integrate popular Python AI/ML frameworks (e.g., TensorFlow, PyTorch, scikit-learn) with ROS 2 robotic systems.
*   **High-level Control**: Implement high-level decision-making processes, path planning, or perception algorithms that then send commands to lower-level ROS 2 controllers.

### Bridging Python Agents to ROS 2 Controllers

The typical workflow involves:

1.  **Agent Logic**: Your Python agent implements the core decision-making or AI algorithm.
2.  **ROS 2 Interface**: The agent uses `rclpy` to create a ROS 2 node.
3.  **Command Publication**: The agent's node publishes commands (e.g., `geometry_msgs/Twist` for velocity control, custom messages for specific actions) to topics that the robot's hardware controllers subscribe to.
4.  **Sensor Subscription**: The agent's node subscribes to sensor data topics (e.g., `sensor_msgs/LaserScan`, `sensor_msgs/Image`, custom state messages) to perceive the environment.
5.  **Service Calls**: The agent might call ROS 2 services for one-shot actions (e.g., resetting a joint, triggering a specific routine).

### Example: Simple Python Agent for Velocity Control

Let's imagine a simple agent that makes a robot move forward based on a command. This agent doesn't use complex AI, but demonstrates the `rclpy` interface.

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist # For sending velocity commands

class SimpleVelocityAgent(Node):
    def __init__(self):
        super().__init__('simple_velocity_agent')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Simple Velocity Agent started.')

    def timer_callback(self):
        # Create a Twist message to command forward velocity
        msg = Twist()
        msg.linear.x = 0.2  # Move forward at 0.2 m/s
        msg.angular.z = 0.0 # No angular velocity

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel: linear.x=%.2f' % msg.linear.x)

def main(args=None):
    rclpy.init(args=args)
    agent = SimpleVelocityAgent()
    rclpy.spin(agent)
    agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This `SimpleVelocityAgent` node uses `rclpy` to publish `Twist` messages to the `cmd_vel` topic, which is a common way to send velocity commands to a mobile robot.

---
**Citation**: ROS 2 Documentation. (n.d.). _Python client library (rclpy)_. Retrieved from [https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html#python-client-library-rclpy](https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html#python-client-library-rclpy) (Example code adapted from official ROS 2 tutorials).

## Python Agent Flow with `rclpy` Diagram

```mermaid
graph TD
    subgraph Python Agent (rclpy Node)
        Agent_Logic[Agent Logic]
        Publisher_Cmd[Publishes Commands]
        Subscriber_Sens[Subscribes to Sensor Data]
        ServiceClient[Calls Services]
    end

    subgraph ROS 2 Network
        Topic_Cmd[Command Topic]
        Topic_Sens[Sensor Topic]
        Service_Server[Service Server]
    end

    Agent_Logic --> Publisher_Cmd
    Publisher_Cmd --> Topic_Cmd
    Topic_Cmd --> Robot_Controller[Robot Controller (Node)]

    Sensor_Hardware[Sensor Hardware] --> Topic_Sens
    Topic_Sens --> Subscriber_Sens
    Subscriber_Sens --> Agent_Logic

    Agent_Logic --> ServiceClient
    ServiceClient --> Service_Server
    Service_Server --> Agent_Logic
```

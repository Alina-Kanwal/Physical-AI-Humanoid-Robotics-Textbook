# Nodes, Topics, and Services

## Learning Objectives

By the end of this chapter, you will be able to:
- Create and manage ROS 2 nodes using Python
- Implement publisher and subscriber communication patterns
- Design and use service-based communication
- Understand Quality of Service (QoS) settings
- Debug communication issues in ROS 2

## Nodes: The Basic Computational Units

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 program. Each node is designed to perform a specific task and communicates with other nodes through topics, services, or actions.

### Creating a Node in Python

To create a node in Python, you need to:
1. Import the `rclpy` library
2. Initialize the ROS 2 client library
3. Create a node instance
4. Spin the node to process callbacks
5. Shutdown when done

## Topics: Publisher-Subscriber Communication

Topics enable asynchronous, many-to-many communication between nodes using a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from a topic.

### Key Concepts:
- **Message types**: Define the structure of data exchanged
- **Topic names**: Unique identifiers for communication channels
- **Queues**: Buffer messages when processing rates differ
- **QoS policies**: Configure reliability, durability, etc.

## Services: Request-Response Communication

Services provide synchronous, one-to-one communication using a request-response pattern. A client sends a request to a service, and the service sends back a response.

### Key Concepts:
- **Service types**: Define the request and response message structure
- **Service names**: Unique identifiers for services
- **Synchronous**: Client waits for response before continuing

## Quality of Service (QoS) Settings

QoS settings allow you to configure communication behavior:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **History**: Keep last N messages vs. keep all messages
- **Depth**: Size of message queue

## Practical Example: Simple Publisher-Subscriber

Let's create a simple publisher that sends "Hello World" messages and a subscriber that receives and prints them.

### Publisher Code:
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
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
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

### Subscriber Code:
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
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Example

1. Save the publisher code as `talker.py`
2. Save the subscriber code as `listener.py`
3. Make sure ROS 2 environment is sourced
4. Run the publisher: `python3 talker.py`
5. In another terminal, run the subscriber: `python3 listener.py`

## Common ROS 2 Commands

- `ros2 node list`: List all active nodes
- `ros2 node info <node_name>`: Get detailed information about a node
- `ros2 topic list`: List all active topics
- `ros2 topic echo <topic_name>`: Print messages from a topic
- `ros2 topic info <topic_name>`: Get information about a topic
- `ros2 service list`: List all available services

## Summary

Nodes, topics, and services form the core communication infrastructure of ROS 2. Understanding these concepts is essential for building distributed robotic applications. Topics provide asynchronous communication for streaming data, while services provide synchronous request-response communication for specific tasks.

## Exercises

1. Create a publisher that publishes sensor data (e.g., temperature readings)
2. Create a subscriber that processes and displays the sensor data
3. Implement a service that performs a simple calculation based on request
4. Experiment with different QoS settings and observe the effects
5. Create a node that acts as both a publisher and subscriber
6. Use `ros2 topic hz` to measure the frequency of published messages
7. Implement a custom message type and use it in a publisher/subscriber pair

## Resources

- [ROS 2 Node Concepts](https://docs.ros.org/en/humble/Concepts/About-Nodes.html)
- [ROS 2 Topic Concepts](https://docs.ros.org/en/humble/Concepts/About-Topics.html)
- [ROS 2 Service Concepts](https://docs.ros.org/en/humble/Concepts/About-Services.html)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
# ROS 2 Python Examples

## Learning Objectives

By the end of this chapter, you will be able to:
- Implement complex ROS 2 nodes in Python
- Use parameters for node configuration
- Create custom message and service types
- Handle asynchronous operations in ROS 2
- Implement action servers and clients

## Advanced Node Patterns

### Node with Parameters

Parameters allow nodes to be configured externally without recompilation:

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with default values
        self.declare_parameter('frequency', 1.0)
        self.declare_parameter('robot_name', 'my_robot')

        # Access parameter values
        self.frequency = self.get_parameter('frequency').value
        self.robot_name = self.get_parameter('robot_name').value

        # Create a timer that uses the parameter
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info(f'Robot {self.robot_name} at {self.frequency} Hz')

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Parameter Callbacks

You can also set up callbacks to handle parameter changes:

```python
from rcl_interfaces.msg import ParameterDescriptor

class ParameterCallbackNode(Node):
    def __init__(self):
        super().__init__('parameter_callback_node')

        # Declare parameter with descriptor
        descriptor = ParameterDescriptor(
            description='Update rate in Hz',
            integer_range=[ParameterIntegerDescriptor(from_value=1, to_value=100)]
        )
        self.declare_parameter('update_rate', 10, descriptor)

        # Set up parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'update_rate' and param.type_ == Parameter.Type.INTEGER:
                self.get_logger().info(f'Update rate changed to {param.value}')
                # Update timer or other components as needed
        return SetParametersResult(successful=True)
```

## Custom Messages and Services

### Creating Custom Messages

Create a `msg` directory in your package and define messages in `.msg` files:

```
# Point2D.msg
float64 x
float64 y
```

### Using Custom Messages

```python
from my_robot_msgs.msg import Point2D

class CustomMessageNode(Node):
    def __init__(self):
        super().__init__('custom_message_node')
        self.publisher = self.create_publisher(Point2D, 'waypoints', 10)

    def publish_waypoint(self, x, y):
        msg = Point2D()
        msg.x = x
        msg.y = y
        self.publisher.publish(msg)
```

## Actions

Actions are used for long-running tasks with feedback:

```python
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            goal_handle.publish_feedback(feedback_msg)
            await asyncio.sleep(1)  # Simulate work

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info('Goal succeeded')
        return result
```

## Multi-threading in ROS 2

ROS 2 supports multi-threading for handling multiple callbacks:

```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class MultiThreadedNode(Node):
    def __init__(self):
        super().__init__('multi_threaded_node')

        # Create separate callback groups
        cb_group1 = MutuallyExclusiveCallbackGroup()
        cb_group2 = MutuallyExclusiveCallbackGroup()

        # Subscribers with different callback groups
        self.sub1 = self.create_subscription(
            String, 'topic1', self.callback1, 10, callback_group=cb_group1)
        self.sub2 = self.create_subscription(
            String, 'topic2', self.callback2, 10, callback_group=cb_group2)

    def callback1(self, msg):
        # This runs in its own thread
        self.get_logger().info(f'Callback 1: {msg.data}')

    def callback2(self, msg):
        # This runs in its own thread
        self.get_logger().info(f'Callback 2: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = MultiThreadedNode()

    # Use multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    rclpy.shutdown()
```

## Client-Server Communication

### Service Client

```python
from example_interfaces.srv import AddTwoInts

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        future = self.cli.call_async(self.req)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = ServiceClientNode()

    future = client.send_request(1, 2)
    rclpy.spin_until_future_complete(client, future)

    response = future.result()
    client.get_logger().info(f'Result: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()
```

## Error Handling and Best Practices

### Exception Handling

```python
class RobustNode(Node):
    def __init__(self):
        super().__init__('robust_node')
        self.subscription = self.create_subscription(
            String, 'topic', self.safe_callback, 10)

    def safe_callback(self, msg):
        try:
            # Process message
            processed_data = self.process_message(msg)
            self.publish_result(processed_data)
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
            # Optionally publish error status or take corrective action
```

### Node Lifecycle

```python
from rclpy.lifecycle import LifecycleNode, LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn

class LifecycleExample(LifecycleNode):
    def __init__(self):
        super().__init__('lifecycle_example')
        self.timer = None

    def on_configure(self, state: LifecycleState):
        self.get_logger().info('Configuring')
        # Initialize resources but don't start operations
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.get_logger().info('Activating')
        # Start operations
        self.timer = self.create_timer(1.0, self.timer_callback)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info('Deactivating')
        # Stop operations but keep resources
        if self.timer:
            self.timer.cancel()
            self.timer = None
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info('Cleaning up')
        # Clean up resources
        return TransitionCallbackReturn.SUCCESS
```

## Summary

This chapter covered advanced ROS 2 Python programming patterns including parameters, custom messages, actions, multi-threading, and lifecycle nodes. These patterns enable the development of robust, configurable, and maintainable robotic applications.

## Exercises

1. Create a node that uses parameters to configure its behavior
2. Implement a custom message type and use it in a publisher/subscriber pair
3. Create an action server that performs a long-running task with feedback
4. Implement a multi-threaded node that handles multiple subscriptions simultaneously
5. Create a lifecycle node and test its different states
6. Implement a client that calls multiple services asynchronously
7. Use the examples in the docs/module-1-ros2/examples/ directory and run them

## Resources

- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [ROS 2 Parameters Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Parameters-In-A-Class-Python.html)
- [ROS 2 Actions Guide](https://docs.ros.org/en/humble/Tutorials/Actions.html)
- [ROS 2 Threading Guide](https://docs.ros.org/en/humble/How-To-Guides/Using-Executors.html)
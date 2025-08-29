import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CommandPublisherNode(Node): #extends Node class, or inherits from Node class, or is a subclass of Node class
    """
    A ROS 2 node to publish hand commands based on user input.
    Uses a timer to periodically check for user input and publish commands.
    """
    def __init__(self):
        super().__init__("command_publisher") #call parent class and give node name
        self.publisher = self.create_publisher(String, "/hand_command", 10) #create publisher for hand command, this is an attribute, not method
        self.timer = self.create_timer(0.1, self.timer_callback) # Run loop quickly
        self.get_logger().info('Command Publisher Node has been started.')

    def timer_callback(self):
        """
        Called every 0.1 seconds by ROS 2 timer.
        Gets user input and publishes commands.
        """
        action = input("Enter 'o' for open, 'g' for grasp, 'q' to quit: ").lower()
        msg = String() #ROS2 String message have a data field (this is a message object)

        if action == 'q':
            raise KeyboardInterrupt  # Clean way to trigger shutdown
            
        if action == 'o':
            msg.data = "OPEN"
            self.get_logger().info("OPEN command sent")
            self.publisher.publish(msg) #publish the message to the topic
        elif action == 'g':
            msg.data = "GRASP"
            self.get_logger().info("GRASP command sent")
            self.publisher.publish(msg) #publish the message to the topic
        else:
            self.get_logger().warn(f"Invalid input: '{action}'. Please use 'o', 'g', or 'q'.") # dont' need to do the continue thing here because timer calls again in 0.1 seconds!


def main(args=None):
    rclpy.init(args=args) #initialize ROS2
    node = CommandPublisherNode() #create node object (fixed class name)
    try:
        rclpy.spin(node) #let ROS 2 manage the node's execution
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node() #destroy node object
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__": #only run if this file is the main file run directly, not if it is imported as a module
    main()
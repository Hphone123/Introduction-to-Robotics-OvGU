import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class WatchdogNode(Node):

    shouldTurn = True
    shouldMove = True

    def __init__(self):
        super().__init__('watchdog')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')

    def cmd_callback(self, msg):

        # Message contains all Information from the User input (linear -> Transformation, angular -> Rotation)
        msg.linear.x = 1 * msg.linear.x

        # Don't Turn if you shouldn't (Turing is only representat on z-Axis)
        if (self.shouldTurn == False): 
            msg.angular.z = 0.0

            # Don't move if you shouldn't (Moving is x and y-Axis)
        if (self.shouldMove == False): 
            msg.linear._x = 0.0
            msg.linear._y = 0.0
        
        # Plublishing Message Gives it to designated (next) Node -> cmd_vel-Node
        self.publisher.publish(msg)
        
    def controller_callback(self, msg):
        if (msg.data == "start"): self.shouldTurn = False
        elif (msg.data == "stop"): self.shouldMove = False
        self.get_logger().warn(f'The controller says I should {msg.data} the turtle ...')



def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

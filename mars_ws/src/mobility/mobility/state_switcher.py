# Created by Nelson Durrant, Feb 2025
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class StateSwitcher(Node):
    """
    Class for switching between drive states

    :author: Nelson Durrant
    :date: Mar 2025

    States:
    - Teleop (cmd_vel_teleop, blue LED)
    - Autonomous (cmd_vel_nav, flashing green LED)
    - Arrival (cmd_vel_nav, red LED)

    Subscribers:
    - cmd_vel_nav (geometry_msgs/Twist)
    - cmd_vel_teleop (geometry_msgs/Twist)
    Publishers:
    - cmd_vel_switch (geometry_msgs/Twist)
    - cmd_led (std_msgs/Int8)
    Services:
    - trigger_teleop (std_srvs/Trigger)
    - trigger_autonomous (std_srvs/Trigger)
    - trigger_arrival (std_srvs/Trigger)
    """

    def __init__(self):
        super().__init__('state_switcher')

        # Autonomy cmd_vel subscription (Nav2)
        self.nav_sub = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.nav_callback,
            10)
        self.nav_sub # prevent unused variable warning

        # Teleop cmd_vel subscription
        self.teleop_sub = self.create_subscription(
            Twist,
            'cmd_vel_teleop',
            self.teleop_callback,
            10)
        self.teleop_sub # prevent unused variable warning

        # Service to trigger teleop state
        self.teleop_service = self.create_service(
            Trigger,
            'trigger_teleop',
            self.teleop_service_callback)
        
        # Service to trigger autonomy state
        self.nav_service = self.create_service(
            Trigger,
            'trigger_autonomous',
            self.nav_service_callback)
        
        # Service to trigger arrival state
        self.arrival_service = self.create_service(
            Trigger,
            'trigger_arrival',
            self.arrival_service_callback)
        
        # Publisher for cmd_vel_switch
        self.cmd_vel_switch_pub = self.create_publisher(
            Twist,
            'cmd_vel_switch',
            10)
        
        # Publisher for cmd_led
        self.cmd_led_pub = self.create_publisher(
            Int8,
            'cmd_led',
            10)
        
        # Timer for publishing to cmd_led
        self.create_timer(0.5, self.led_timer_callback)

        self.state = 'teleop'

        self.get_logger().info("StateSwitcher started")

    def nav_callback(self, msg):
        """
        Callback for cmd_vel_nav subscription
        """

        if self.state == 'autonomous':
            self.cmd_vel_switch_pub.publish(msg)
        else:
            self.get_logger().warn("Received cmd_vel_nav while not in autonomous state")

    def teleop_callback(self, msg):
        """
        Callback for cmd_vel_teleop subscription
        """

        if self.state == 'teleop':
            self.cmd_vel_switch_pub.publish(msg)
        else:
            self.get_logger().warn("Received cmd_vel_teleop while not in teleop state")

    def teleop_service_callback(self, request, response):
        """
        Callback for trigger_teleop service
        """

        self.state = 'teleop'
        self.get_logger().info("Switched to teleop state")
        return response
    
    def nav_service_callback(self, request, response):
        """
        Callback for trigger_autonomous service
        """

        self.state = 'autonomous'
        self.get_logger().info("Switched to autonomous state")
        return response
    
    def arrival_service_callback(self, request, response):
        """
        Callback for trigger_arrival service
        """

        self.state = 'arrival'
        self.get_logger().info("Switched to arrival state")
        return response
    
    def led_timer_callback(self):
        """
        Publish to cmd_led based on current state
        """

        if self.state == 'teleop':
            led_state = 1
        elif self.state == 'autonomous':
            led_state = 2
        elif self.state == 'arrival':
            led_state = 3
        else:
            led_state = 0
        self.cmd_led_pub.publish(led_state)


def main(args=None):
    rclpy.init(args=args)

    state_switcher = StateSwitcher()

    rclpy.spin(state_switcher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_switcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
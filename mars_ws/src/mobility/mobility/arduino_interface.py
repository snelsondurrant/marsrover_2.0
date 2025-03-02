# Created by Nelson Durrant, Feb 2025
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import pyserial


class ArduinoInterface(Node):
    '''
    Interface between the ROS2 system and the Arduino
    
    :author: Nelson Durrant
    :date: Feb 2025

    Serial Messages:
    - "drive <left_wheel_velocity> <right_wheel_velocity>": Set the left and right wheel velocities
    - "stop": Stop the robot

    Subscribers:
    - cmd_vel (geometry_msgs/Twist): Set the robot velocity
    '''

    def __init__(self):

        super().__init__('arduino_interface')

        # Simple timer for timeout stop
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timeout = 5 # seconds
        self.last_msg_time = self.get_clock().now()

        # Nav2 and joystick teleop will publish Twist messages to this topic
        self.drive_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.drive_sub_callback,
            10)
        self.drive_sub  # prevent unused variable warning

        self.wheel_radius = 0.1
        self.wheel_base = 0.5 # distance between the two wheels

        # Serial connection parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        port = self.get_parameter('port').value
        self.declare_parameter('baudrate', 9600)
        baudrate = self.get_parameter('baudrate').value

        # Start a pyserial connection to the Arduino
        self.serial = pyserial.Serial(port, baudrate)

    def timer_callback(self):
        '''
        Stop the robot if no messages have been received in the timeout limit
        '''

        # If the last message was more than the timeout limit ago, stop the robot
        if self.get_clock().now() - self.last_msg_time > self.timeout:
            pyserial.write("stop")

    def drive_sub_callback(self, msg):
        '''
        Convert the Twist message to diff drive motor speeds and send to the Arduino
        '''

        self.last_msg_time = self.get_clock().now()

        # Kinematic model for differential drive robot
        left_wheel_velocity = (msg.linear.x - (msg.angular.z * self.wheel_base / 2)) / self.wheel_radius
        right_wheel_velocity = (msg.linear.x + (msg.angular.z * self.wheel_base / 2)) / self.wheel_radius

        # Send the motor speeds to the Arduino
        pyserial.write(f"drive {left_wheel_velocity} {right_wheel_velocity}")


def main(args=None):
    rclpy.init(args=args)

    arduino_interface = ArduinoInterface()

    rclpy.spin(arduino_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arduino_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

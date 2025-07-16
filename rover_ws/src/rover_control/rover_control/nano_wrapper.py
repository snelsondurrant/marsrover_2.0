import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int8
# from rover_msgs.msg import NavState, Battery, Gripper, RawBattery, Laser, Clicker
import time
import threading
import queue


class NanoWrapper(Node):
    """
    This node acts as a middleman between the Orin and the Arduino Nano.

    :author: ADD HERE
    :date: ADD HERE

    TODO: I think this could be simplified and cleaned up A LOT.

    NOTE: It might be worth considering using a more robust control system based on the 
    'ros2_control' and 'ros2_controllers' packages, which are designed for robust performance
    and provide easy hookups to Gazebo. See these links for more information:
    https://docs.nav2.org/setup_guides/odom/setup_odom_gz_classic.html#setting-up-odometry-on-your-robot
    https://articulatedrobotics.xyz/tutorials/mobile-robot/applications/ros2_control-concepts
    https://control.ros.org/humble/index.html
    https://github.com/ros-controls/ros2_controllers/tree/humble/diff_drive_controller

    Subscribers:
    - /nav_state (std_msgs.Int8): Navigation state to control LEDs
    """

    def __init__(self):
        super().__init__('nano_wrapper')
        
        self.q = queue.Queue()
        # Publishers
        # self.battery_pub = self.create_publisher(RawBattery, '/raw_battery_info', 10)
        
        # Subscribers
        self.create_subscription(Int8, '/nav_state', self.led_callback, 10)
        # self.create_subscription(Gripper, '/gripper', self.gripper_callback, 10)
        # self.create_subscription(Laser, '/laser_state', self.laser_callback, 10)
        # self.create_subscription(Clicker, '/click', self.click_callback, 10)

        # Arduino serial setup
        try:
            self.serial_port = serial.Serial("/dev/rover/peripheralsBoard", 9600)
            self.get_logger().info("Serial port initialized")
        except Exception as e:
            self.get_logger().error("Error: peripheralsBoard not yet ready")
            self.get_logger().error(str(e))
            rclpy.shutdown()
            exit(0)

        # Start threads
        self.queue_handler_thread = threading.Thread(target=self.queue_handler)
        self.queue_handler_thread.start()
        # self.arduino_listener_thread = threading.Thread(target=self.arduino_listener)
        # self.arduino_listener_thread.start()

        # Default no-operation values
        # self.gripper = 0
        # self.navigation_state = -1
        

    def led_callback(self, data):
        # Update LED based on the rover state
        # int8 AUTONOMOUS_STATE = 0
        # int8 TELEOPERATION_STATE = 1
        # int8 ARRIVAL_STATE = 2

        data_array = f"L{data.data};"
        self.q.put(data_array)

    # def gripper_callback(self, data):
    #     data_array = f"G{data.gripper}:0;"
    #     self.get_logger().info(f"Gripper command: {data_array}")
    #     self.q.put(data_array)

    # def laser_callback(self, data):
    #     data_array = "S+;" if data.laser_state else "S-;"
    #     self.get_logger().info("Laser call to Arduino")
    #     self.q.put(data_array)

    # def click_callback(self, data):
    #     data_array = "S!;"
    #     self.q.put(data_array)

    # def arduino_listener(self):
    #     self.serial_port.flush()

    #     while rclpy.ok():
    #         if self.serial_port.in_waiting:
    #             data = self.serial_port.readline().strip()
    #             decoded_data = data.decode('utf-8', 'ignore')  # Ignore any decoding errors
    #             clean_data = ''.join(c for c in decoded_data if c.isdigit())

    #             Convert to integer
    #             voltage = int(clean_data)
    #             voltage = int(data)

    #             bat_voltage_msg = RawBattery()
    #             bat_voltage_msg.voltage = voltage
    #             self.battery_pub.publish(bat_voltage_msg)
    #             self.serial_port.flush()
    #         time.sleep(0.1)

    def queue_handler(self):
        while rclpy.ok():
            if not self.q.empty():
                try:
                    data = self.q.get(timeout=1).encode("utf-8")
                    self.serial_port.write(data)
                except Exception as e:
                    self.get_logger().warn(f"Failed to write to serial: {e}")
            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = NanoWrapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.queue_handler_thread.join()
        node.arduino_listener_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
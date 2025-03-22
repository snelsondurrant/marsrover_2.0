import math

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu

import numpy as np

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from std_srvs.srv import Trigger

from tf_transformations import euler_from_quaternion, quaternion_from_euler

class FramePublisher(Node):

    def __init__(self):
        super().__init__('turtle_tf2_frame_publisher')

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.new_zed_calibration = True
        self.new_lidar_calibration = True

        self.zed_transform = TransformStamped()
        self.lidar_transform = TransformStamped()
        self.lidar_imu_transform = TransformStamped()

        self.subscription = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.zed_imu_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.lidar_subscription = self.create_subscription(
            Imu,
            '/unilidar/imu',
            self.lidar_imu_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create service to start the calibration
        self.calibrate_service = self.create_service(
            Trigger,
            'calibrate/zed_lidar',    # service name
            self.calibrate_callback)  # callback function
        


    def lidar_imu_callback(self, msg):
        # TODO: only run this callback when the service is called        
        
        # Read message content and assign it to
        # corresponding tf variables
        
        if self.new_lidar_calibration:

            yaw = 0.0
            # Read the linear acceleration values
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z

            # Check that az (acceleration in the z-direction) is less than 1 to assume we are not tilting or moving
            if abs(az) < 1.0:  # This ensures we are on a level plane
                # Construct the gravity vector (we ignore az)
                gravity_vector = np.array([-ax, -ay, 0.0])  # Negative acceleration for gravity vector (downward)

                # Normalize the gravity vector
                gravity_magnitude = np.linalg.norm(gravity_vector)
                if gravity_magnitude > 0.1:  # Ensure valid gravity reading (non-zero magnitude)
                    gravity_unit_vector = gravity_vector / gravity_magnitude
                    
                    # Assuming the mount's X-axis is aligned with gravity (mount frame X points downward)
                    # The yaw is the angle between the gravity vector and the X-axis of the mount frame
                    
                    # Mount frame X-axis is assumed to be along [1, 0, 0] (pointing in the gravity direction)
                    mount_x_axis = np.array([1.0, 0.0, 0.0])

                    # Compute the dot product to find the cosine of the angle
                    dot_product = np.dot(gravity_unit_vector, mount_x_axis)

                    # Clamping the dot product to avoid errors due to precision issues
                    dot_product = np.clip(dot_product, -1.0, 1.0)

                    # Calculate the yaw (angle between gravity vector and mount frame X-axis)
                    yaw = math.acos(dot_product)

                    # Check the sign of the yaw angle (to determine the direction of rotation)
                    if gravity_unit_vector[1] > 0:  # If gravity vector points in positive Y direction
                        yaw = -yaw  # Reverse the yaw direction

                    self.get_logger().info(f"Calculated Yaw: {yaw} radians")
                else:
                    self.get_logger().error("Invalid gravity vector magnitude. Calibration failed.")
                    return
            else:
                self.get_logger().error("Invalid acceleration in the Z-direction. Calibration failed.")
                return
            
            # DID NOT WORK
            # # Convert quaternion to Euler (roll, pitch, yaw)
            # q = msg.orientation
            # roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Set the transformation between the mount and the lidar
            # Only the yaw (Z-axis rotation) will be considered for calibration
            self.lidar_transform.header.frame_id = 'unilidar_mount'
            self.lidar_transform.child_frame_id = 'unilidar_lidar'

            self.lidar_transform.transform.translation.x = 0.0  
            self.lidar_transform.transform.translation.y = 0.0
            self.lidar_transform.transform.translation.z = 0.0

            # Use the yaw angle to set the quaternion (rotation about Z-axis)
            q_calibrated = quaternion_from_euler(0.0, 0.0, yaw)

            self.lidar_transform.transform.rotation.x = q_calibrated[0]
            self.lidar_transform.transform.rotation.y = q_calibrated[1]
            self.lidar_transform.transform.rotation.z = q_calibrated[2]
            self.lidar_transform.transform.rotation.w = q_calibrated[3]

            self.new_lidar_calibration = False
            self.get_logger().info("LiDAR calibration complete (Z-axis rotation only).")
            # print yaw for debugging
            self.get_logger().info(f"Yaw: {yaw}")
            

        # Ensure timestamps are updated before publishing
        now = self.get_clock().now().to_msg()
        self.lidar_transform.header.stamp = now
        self.lidar_imu_transform.header.stamp = now

        self.tf_broadcaster.sendTransform(self.lidar_transform)
        self.tf_broadcaster.sendTransform(self.lidar_imu_transform)

        # Publish real-time LiDAR movement (adjusted by IMU)
        t_imu = TransformStamped()
        t_imu.header.frame_id = 'unilidar_lidar'  # Relative to calibrated frame
        t_imu.child_frame_id = 'unilidar_moving'
        t_imu.transform.rotation = msg.orientation  # Real-time movement
        t_imu.header.stamp = now

        self.tf_broadcaster.sendTransform(t_imu)

        # self.get_logger().info('setting zed transform')

    def zed_imu_callback(self, msg):
        # TODO: only run this callback when the service is called        
        
        # Read message content and assign it to
        # corresponding tf variables
        # TODO: get rid of yaw and just use pitch and roll
        # roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        
        if self.new_zed_calibration:
            self.zed_transform.header.stamp = self.get_clock().now().to_msg()
            self.zed_transform.header.frame_id = 'zed_camera_mount'
            self.zed_transform.child_frame_id = 'zed_camera_link'

            self.zed_transform.transform.translation.x = 0.0
            self.zed_transform.transform.translation.y = 0.0
            self.zed_transform.transform.translation.z = 0.0

            # Convert quaternion to Euler (roll, pitch, yaw)
            q = msg.orientation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Use the pitch angle to set the quaternion (rotation about Y-axis)
            q_calibrated = quaternion_from_euler(0.0, pitch, 0.0)

            self.zed_transform.transform.rotation.x = q_calibrated[0]
            self.zed_transform.transform.rotation.y = q_calibrated[1]
            self.zed_transform.transform.rotation.z = q_calibrated[2]
            self.zed_transform.transform.rotation.w = q_calibrated[3]

            self.new_zed_calibration = False

        # self.tf_broadcaster.sendTransform(self.zed_transform)

        # self.get_logger().info('setting zed transform')
        self.zed_transform.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.zed_transform)

    def calibrate_callback(self, request, response):
        # Send the transformation
        self.get_logger().info('Sending zed transform')
        self.new_zed_calibration = True
        self.new_lidar_calibration = True

        self.get_logger().info('Calibrating ZED to LIDAR')

        # Return the response
        response.success = True
        response.message = 'Calibration completed'
        return response


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
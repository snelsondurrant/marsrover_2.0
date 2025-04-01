import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class CovarianceTuner(Node):
    """
    Tunes the sensor covariance matrices before sending to the EKF

    :author: Nelson Durrant
    :date: Apr 2025

    Subscribers:
    - odometry/gps (nav_msgs/Odometry)
    - zed/zed_node/odom (nav_msgs/Odometry)
    - imu/data (sensor_msgs/Imu)
    Publishers:
    - odometry/gps_tuned (nav_msgs/Odometry)
    - zed/zed_node/odom_tuned (nav_msgs/Odometry)
    - imu/data_tuned (sensor_msgs/Imu)
    """

    def __init__(self):
        super().__init__('covariance_tuner')

        self.gps_sub = self.create_subscription(Odometry, 'odometry/gps', self.gps_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'zed/zed_node/odom', self.odom_callback, 10)

        self.gps_pub = self.create_publisher(Odometry, 'odometry/gps_tuned', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_tuned', 10)
        self.odom_pub = self.create_publisher(Odometry, 'zed/zed_node/odom_tuned', 10)

        self.declare_parameter('gps_covariance_factor', 1.0)
        self.declare_parameter('odom_covariance_factor', 1.0)
        self.declare_parameter('imu_covariance_factor', 1.0)

    def gps_callback(self, msg):
        gps_covariance_factor = self.get_parameter('gps_covariance_factor').value
        msg.pose.covariance = [cov * gps_covariance_factor for cov in msg.pose.covariance]
        self.gps_pub.publish(msg)

    def imu_callback(self, msg):
        imu_covariance_factor = self.get_parameter('imu_covariance_factor').value
        msg.orientation_covariance = [cov * imu_covariance_factor for cov in msg.orientation_covariance]
        self.imu_pub.publish(msg)

    def odom_callback(self, msg):
        odom_covariance_factor = self.get_parameter('odom_covariance_factor').value
        msg.pose.covariance = [cov * odom_covariance_factor for cov in msg.pose.covariance]
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    covariance_tuner = CovarianceTuner()

    rclpy.spin(covariance_tuner)

    covariance_tuner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
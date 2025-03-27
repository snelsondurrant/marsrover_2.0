import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoseStamped


class GEO2NSF(Node):
    """
    Converts GeoPoseStamped messages from the ZED GNSS localization to NavSatFix messages

    :author: Nelson Durrant
    :date: Mar 2025

    Subscribers:
    - zed/zed_node/geo_pose (geographic_msgs/GeoPoseStamped)
    Publishers:
    - gps/filtered (sensor_msgs/NavSatFix)
    """

    def __init__(self):
        super().__init__('geo_to_nsf')

        self.geo_sub = self.create_subscription(GeoPoseStamped, 'zed/zed_node/geo_pose', self.geo_callback, 10)
        self.nsf_pub = self.create_publisher(NavSatFix, 'gps/filtered', 10)

    def geo_callback(self, msg):

        nsf_msg = NavSatFix(
            header=msg.header,
            latitude=msg.pose.position.latitude,
            longitude=msg.pose.position.longitude,
            altitude=msg.pose.position.altitude,
            position_covariance=[0, 0, 0, 0, 0, 0, 0, 0, 0],
            position_covariance_type=NavSatFix.COVARIANCE_TYPE_UNKNOWN,
        )

        self.nsf_pub.publish(nsf_msg)


def main(args=None):
    rclpy.init(args=args)

    geo_to_nsf = GEO2NSF()

    rclpy.spin(geo_to_nsf)

    geo_to_nsf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

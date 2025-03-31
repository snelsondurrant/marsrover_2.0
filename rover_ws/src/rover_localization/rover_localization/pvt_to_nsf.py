import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_read_2.msg import PositionVelocityTime


class PVT2NSF(Node):
    """
    Converts PositionVelocityTime messages from the ublox RTK GPS to NavSatFix messages

    :author: Nelson Durrant
    :date: Mar 2025

    Subscribers:
    - rover/PosVelTime (ublox_read_2/PositionVelocityTime)
    Publishers:
    - gps/fix (sensor_msgs/NavSatFix)
    """

    def __init__(self):
        super().__init__('pvt_to_nsf')

        self.pvt_sub = self.create_subscription(PositionVelocityTime, 'rover/PosVelTime', self.pvt_callback, 10)
        self.nsf_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)

    def pvt_callback(self, msg):

        # IMPORTANT! The covariance defines how closely our EFK output follows the GPS data
        # Since we have reliable RTK GPS data, we may want to trust it more than the GPS reports
        # Thus we can use a tuning factor to adjust the covariance (smaller means more trust)
        tuning_factor = 0.1
        h_var = (msg.h_acc)**2 * tuning_factor  # horizontal covariance
        v_var = (msg.v_acc)**2 * tuning_factor  # vertical covariance

        # Create covariance matrix
        pos_covariance = [
            h_var, 0, 0,
            0, h_var, 0,
            0, 0, v_var,
        ]

        # Creates navsat message
        nsf_msg = NavSatFix(
            header=msg.header,
            latitude=msg.lla[0],
            longitude=msg.lla[1],
            altitude=msg.lla[2],
            position_covariance=pos_covariance,
            position_covariance_type=NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN,
        )
        # TODO: Double check the incoming timestamps from GPS, they could be throwing us off
        # I think we test it without it first - Nelson
        # nsf_msg.header.stamp = self.get_clock().now().to_msg()
        nsf_msg.header.frame_id = 'gps_link'  # set frame id (urdf)
        self.nsf_pub.publish(nsf_msg)


def main(args=None):
    rclpy.init(args=args)

    pvt_to_nsf = PVT2NSF()

    rclpy.spin(pvt_to_nsf)

    pvt_to_nsf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from robot_localization.srv import SetDatum


class SyncOrigin(Node):
    """
    Syncs the origin of mapviz with the origin of navsat_transform on the first GPS fix

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Apr 2025

    Subscribers:
    - /gps/fix (sensor_msgs/NavSatFix)
    Publishers:
    - /mapviz/origin (sensor_msgs/NavSatFix)
    Clients:
    - /datum (robot_localization/SetDatum)
    """

    def __init__(self):
        super().__init__("sync_origin")

        self.origin_msg: NavSatFix = None
        self.origin_set_attempted = False

        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )

        self.gps_pub = self.create_publisher(NavSatFix, "/mapviz/origin", 10)

        self.srv_client = self.create_client(SetDatum, "/datum")
        while not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for SetDatum service...")

        self.publish_timer = self.create_timer(1.0, self.publish_origin_callback)

        self.get_logger().info(f"Sync Origin started. Waiting for first GPS fix...")

    def gps_callback(self, msg: NavSatFix):
        """
        Stores the first GPS fix, calls SetDatum, and destroys the subscriber.
        """

        if self.origin_set_attempted:
            return  # Only process the first message

        self.origin_set_attempted = True
        self.get_logger().info(
            f"First GPS fix received: Lat {msg.latitude}, Lon {msg.longitude}, Alt {msg.altitude}. Storing origin and calling SetDatum."
        )
        self.origin_msg = msg

        request = SetDatum.Request()
        request.geo_pose.position.latitude = msg.latitude
        request.geo_pose.position.longitude = msg.longitude
        request.geo_pose.position.altitude = msg.altitude
        request.geo_pose.orientation.x = 0.0
        request.geo_pose.orientation.y = 0.0
        request.geo_pose.orientation.z = 0.0
        request.geo_pose.orientation.w = 1.0

        future = self.srv_client.call_async(request)
        future.add_done_callback(self.datum_service_callback)

        self.destroy_subscription(self.gps_sub)
        self.gps_sub = None
        self.get_logger().info("SetDatum call initiated. GPS subscriber destroyed.")

    def datum_service_callback(self, future):
        """
        Callback for the SetDatum service response.
        """

        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully called SetDatum service.")
            else:
                self.get_logger().error(f"SetDatum service call failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"SetDatum service call failed: {e!r}")

    def publish_origin_callback(self):
        """
        Called by the timer to periodically publish the stored origin.
        """

        if self.origin_msg is not None:
            self.origin_msg.header.stamp = self.get_clock().now().to_msg()
            self.gps_pub.publish(self.origin_msg)


def main(args=None):
    rclpy.init(args=args)
    sync_origin_node = SyncOrigin()
    try:
        rclpy.spin(sync_origin_node)
    except KeyboardInterrupt:
        pass
    finally:
        sync_origin_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

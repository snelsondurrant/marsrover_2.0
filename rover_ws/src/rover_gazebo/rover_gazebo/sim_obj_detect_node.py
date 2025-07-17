import rclpy
import time
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from zed_msgs.msg import ObjectsStamped, Object
from std_srvs.srv import SetBool


class SimObjDetect(Node):
    """
    Simulates ZED object detection in Gazebo

    :author: Nelson Durrant
    :date: Mar 2025

    Publishers:
    - zed/zed_node/obj_det/objects (zed_msgs/ObjectsStamped)
    Clients:
    - get_entity_state (gazebo_msgs/GetEntityState)
    Services:
    - zed/zed_node/enable_obj_det (std_srvs/SetBool)
    """

    def __init__(self):
        super().__init__("sim_obj_detect")

        self.enable_flag = False

        self.declare_parameter("enable_mallet", True)
        self.declare_parameter("enable_bottle", False)
        self.enable_mallet = (
            self.get_parameter("enable_mallet").get_parameter_value().bool_value
        )
        self.get_logger().info(f"enable_mallet: {self.enable_mallet}")
        self.enable_bottle = (
            self.get_parameter("enable_bottle").get_parameter_value().bool_value
        )
        self.get_logger().info(f"enable_bottle: {self.enable_bottle}")

        if not self.enable_mallet and not self.enable_bottle:
            self.get_logger().warn("No objects enabled for detection")

        # Callback groups (for callback in a callback)
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        state_callback_group = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            0.5, self.timer_callback, callback_group=timer_callback_group
        )

        self.get_entity_state = self.create_client(
            GetEntityState, "get_entity_state", callback_group=state_callback_group
        )
        while not self.get_entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Gazebo position service not available, waiting again..."
            )
        self.get_logger().info("Gazebo position service is available")

        if self.enable_mallet:
            self.get_logger().info("Mallet detection enabled")

            # Request to get the state of the mallet
            self.mallet_request = GetEntityState.Request()
            self.mallet_request.name = "hammer"
            self.mallet_request.reference_frame = "zed_camera_link"

        if self.enable_bottle:
            self.get_logger().info("Bottle detection enabled")

            # Request to get the state of the bottle
            self.bottle_request = GetEntityState.Request()
            self.bottle_request.name = "coke_can"
            self.bottle_request.reference_frame = "zed_camera_link"

        self.zed_pub = self.create_publisher(
            ObjectsStamped, "zed/zed_node/obj_det/objects", 10
        )

        self.enable_service = self.create_service(
            SetBool, "zed/zed_node/enable_obj_det", self.enable_service_callback
        )

        self.get_logger().info("SimObjDetect node started")

    def enable_service_callback(self, request, response):
        self.enable_flag = request.data
        response.success = True
        return response

    def timer_callback(self):

        if self.enable_flag:

            if self.enable_mallet:
                future = self.get_entity_state.call_async(self.mallet_request)
                rclpy.spin_until_future_complete(self, future)
                mallet_gz_pos = future.result()

            if self.enable_bottle:
                future = self.get_entity_state.call_async(self.bottle_request)
                rclpy.spin_until_future_complete(self, future)
                bottle_gz_pos = future.result()

            zed_msg = ObjectsStamped()
            zed_msg.header.stamp = self.get_clock().now().to_msg()
            zed_msg.header.frame_id = "zed_camera_link"

            if self.enable_mallet:
                if (
                    mallet_gz_pos.state.pose.position.x < 5.0  # up to 5m away
                    and mallet_gz_pos.state.pose.position.x > 0
                    and abs(mallet_gz_pos.state.pose.position.y)
                    < mallet_gz_pos.state.pose.position.x  # cone of vision
                ):
                    mallet_obj = Object()
                    mallet_obj.label = "Class ID: 0"
                    mallet_obj.label_id = 1  # from testing
                    mallet_obj.confidence = 99.0
                    mallet_obj.position = [
                        mallet_gz_pos.state.pose.position.x,
                        mallet_gz_pos.state.pose.position.y,
                        mallet_gz_pos.state.pose.position.z,
                    ]
                    zed_msg.objects.append(mallet_obj)

            if self.enable_bottle:
                if (
                    bottle_gz_pos.state.pose.position.x < 5.0  # up to 5m away
                    and bottle_gz_pos.state.pose.position.x > 0
                    and abs(bottle_gz_pos.state.pose.position.y)
                    < bottle_gz_pos.state.pose.position.x  # cone of vision
                ):
                    bottle_obj = Object()
                    bottle_obj.label = "Class ID: 1"
                    bottle_obj.label_id = 5  # from testing
                    bottle_obj.confidence = 99.0
                    bottle_obj.position = [
                        bottle_gz_pos.state.pose.position.x,
                        bottle_gz_pos.state.pose.position.y,
                        bottle_gz_pos.state.pose.position.z,
                    ]
                    zed_msg.objects.append(bottle_obj)

            time.sleep(1)  # we're getting data faster than in real life
            self.zed_pub.publish(zed_msg)


def main(args=None):
    rclpy.init(args=args)
    sim_obj_detect = SimObjDetect()
    rclpy.spin(sim_obj_detect)
    sim_obj_detect.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

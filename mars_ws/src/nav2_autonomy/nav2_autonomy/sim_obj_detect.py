# Created by Nelson Durrant, Mar 2025
import rclpy
import time
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from vision_msgs.msg import (
    Detection3DArray,
    Detection3D,
    ObjectHypothesisWithPose,
)


class SimObjDetect(Node):
    """
    Simulates ZED object detection in Gazebo

    :author: Nelson Durrant
    :date: Mar 2025

    Publishers:
    - zed/detections (vision_msgs/Detection3DArray)
    Clients:
    - get_entity_state (gazebo_msgs/GetEntityState)
    """

    def __init__(self):

        super().__init__("sim_obj_detect")

        # Callback groups (for service call in a callback)
        timer_callback_group = MutuallyExclusiveCallbackGroup()
        state_callback_group = MutuallyExclusiveCallbackGroup()

        # Timer to call the get_entity_state service
        self.timer = self.create_timer(
            0.5, self.timer_callback, callback_group=timer_callback_group
        )

        # Client to get Gazebo entity states
        self.get_entity_state = self.create_client(
            GetEntityState, "get_entity_state", callback_group=state_callback_group
        )
        while not self.get_entity_state.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Gazebo position service not available, waiting again..."
            )
        self.get_logger().info("Gazebo position service is available")

        # Request to get the state of the mallet
        self.mallet_request = GetEntityState.Request()
        self.mallet_request.name = "hammer"
        self.mallet_request.reference_frame = "base_link"

        # Publisher for the ZED detections
        self.zed_pub = self.create_publisher(Detection3DArray, "zed/detections", 10)

        self.get_logger().info("SimObjDetect node started")

    def timer_callback(self):

        # Call the service to get the position of the mallet
        future = self.get_entity_state.call_async(self.mallet_request)
        rclpy.spin_until_future_complete(self, future)
        gazebo_position = future.result()

        # Check to see if the mallet is in view
        if (
            gazebo_position.state.pose.position.x < 3.5
            and gazebo_position.state.pose.position.x > 0
            and abs(gazebo_position.state.pose.position.y)
            < gazebo_position.state.pose.position.x  # cone of vision
        ):

            # Create a Detection3Darray
            zed_msg = Detection3DArray()
            zed_msg.header.stamp = self.get_clock().now().to_msg()
            zed_msg.header.frame_id = "base_link"
            # Create a Detection3D
            detection3d = Detection3D()
            detection3d.header.stamp = self.get_clock().now().to_msg()
            detection3d.header.frame_id = "base_link"
            detection3d.id = "mallet"
            # Add a BoundingBox3D to the Detection3D
            detection3d.bbox.center.position.x = gazebo_position.state.pose.position.x
            detection3d.bbox.center.position.y = gazebo_position.state.pose.position.y
            detection3d.bbox.center.position.z = gazebo_position.state.pose.position.z
            detection3d.bbox.size.x = 0.3  # arbitrary size
            detection3d.bbox.size.y = 0.3  # arbitrary size
            detection3d.bbox.size.z = 0.3  # arbitrary size
            # Create a ObjectHypothesisWithPose
            obj_hyp_pose = ObjectHypothesisWithPose()
            obj_hyp_pose.hypothesis.class_id = "mallet"
            obj_hyp_pose.hypothesis.score = 1.0  # 100% confidence
            # Add the pose to the ObjectHypothesisWithPose
            obj_hyp_pose.pose.pose.position.x = gazebo_position.state.pose.position.x
            obj_hyp_pose.pose.pose.position.y = gazebo_position.state.pose.position.y
            obj_hyp_pose.pose.pose.position.z = gazebo_position.state.pose.position.z
            obj_hyp_pose.pose.pose.orientation.x = (
                gazebo_position.state.pose.orientation.x
            )
            obj_hyp_pose.pose.pose.orientation.y = (
                gazebo_position.state.pose.orientation.y
            )
            obj_hyp_pose.pose.pose.orientation.z = (
                gazebo_position.state.pose.orientation.z
            )
            obj_hyp_pose.pose.pose.orientation.w = (
                gazebo_position.state.pose.orientation.w
            )
            # Add the ObjectHypothesisWithPose to the Detection3D
            detection3d.results.append(obj_hyp_pose)
            # Add the Detection3D to the Detection3DArray
            zed_msg.detections.append(detection3d)

            # Publish the Detection3DArray
            time.sleep(1)  # we're getting data faster than in real life
            self.zed_pub.publish(zed_msg)

        else:
            # Publish an empty Detection3DArray
            zed_msg = Detection3DArray()
            zed_msg.header.stamp = self.get_clock().now().to_msg()
            zed_msg.header.frame_id = "base_link"
            time.sleep(0.1)  # we're getting data faster than in real life
            self.zed_pub.publish(zed_msg)


def main(args=None):
    rclpy.init(args=args)

    sim_obj_detect = SimObjDetect()

    rclpy.spin(sim_obj_detect)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_obj_detect.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

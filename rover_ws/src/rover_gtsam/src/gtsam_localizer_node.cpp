// Created by Nelson Durrant, July 2025
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

// TODO: Include your necessary GTSAM headers here
#include <gtsam/geometry/Pose3.h> // Example include

class GtsamLocalizerNode : public rclcpp::Node
{
    /**
     * @brief Experimental GTSAM-based global localizer for the Mars Rover simulation.
     *
     * @author Nelson Durrant
     * @date July 2025
     *
     * NOTE: The below topics and frames can be configured via parameters.
     *
     * Subscribers:
     * - /imu (sensor_msgs::msg::Imu): IMU data from the onboard sensor.
     * - /odometry/gps (nav_msgs::msg::Odometry): GPS odometry from 'navsat_transform_node'
     * - odom->base_link (tf2_ros::Buffer): Odom coordinate transform from the EKF.
     * Publishers:
     * - /odometry/global (nav_msgs::msg::Odometry): Global odometry estimate.
     * - map->odom (tf2_ros::TransformBroadcaster): Calculated map coordinate transform.
     */
public:
    GtsamLocalizerNode() : Node("gtsam_localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "GTSAM Localizer starting up...");

        loadParameters();

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        global_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(global_odom_topic_, 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 200, std::bind(&GtsamLocalizerNode::imuCallback, this, std::placeholders::_1));
        gps_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            gps_odom_topic_, 20, std::bind(&GtsamLocalizerNode::gpsOdomCallback, this, std::placeholders::_1));

        factor_graph_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / factor_graph_update_rate_)),
            std::bind(&GtsamLocalizerNode::factorGraphTimerCallback, this));

        odom_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / odom_publish_rate_)),
            std::bind(&GtsamLocalizerNode::odomPublishTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Start up complete. Waiting for first IMU and GPS messages...");
    }

private:
    void loadParameters()
    {
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("gps_odom_topic", "/odometry/gps");
        this->declare_parameter<std::string>("global_odom_topic", "/odometry/global");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("base_link_frame", "base_link");
        this->declare_parameter<double>("factor_graph_update_rate", 1.0); // Hz
        this->declare_parameter<double>("odom_publish_rate", 10.0);       // Hz
        this->declare_parameter<bool>("publish_global_tf", true);       // true or false

        imu_topic_ = this->get_parameter("imu_topic").as_string();
        gps_odom_topic_ = this->get_parameter("gps_odom_topic").as_string();
        global_odom_topic_ = this->get_parameter("global_odom_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_link_frame_ = this->get_parameter("base_link_frame").as_string();
        factor_graph_update_rate_ = this->get_parameter("factor_graph_update_rate").as_double();
        odom_publish_rate_ = this->get_parameter("odom_publish_rate").as_double();
        publish_global_tf_ = this->get_parameter("publish_global_tf").as_bool();

        RCLCPP_INFO(this->get_logger(), "IMU topic: %s", imu_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "GPS odometry topic: %s", gps_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Global odometry topic: %s", global_odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Odometry frame: %s", odom_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Base link frame: %s", base_link_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Factor graph update rate: %.2f Hz", factor_graph_update_rate_);
        RCLCPP_INFO(this->get_logger(), "Odometry publish rate: %.2f Hz", odom_publish_rate_);
        RCLCPP_INFO(this->get_logger(), "Publish global TF: %s", publish_global_tf_ ? "true" : "false");
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_queue_.push_back(msg);
    }

    void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        gps_queue_.push_back(msg);
    }

    void factorGraphTimerCallback()
    {
        // This function is called at a fixed rate (e.g., 1 Hz) to update the factor graph.

        // Step 1: Get all queued measurements since the last update.
        std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_measurements;
        imu_measurements.assign(imu_queue_.begin(), imu_queue_.end());
        imu_queue_.clear();

        nav_msgs::msg::Odometry::SharedPtr latest_gps;
        if (!gps_queue_.empty())
        {
            latest_gps = gps_queue_.back();
            gps_queue_.clear();
        }

        // Step 2: Handle initialization on the first valid messages.
        if (!system_initialized_)
        {
            if (!imu_measurements.empty() && latest_gps)
            {
                RCLCPP_INFO(this->get_logger(), "First IMU and GPS messages received. Initializing system.");
                // TODO: Initialize your GTSAM smoother (e.g., ISAM2), preintegrator,
                // and initial state variables (pose, velocity, bias) here.
                // Add a prior factor to the graph to anchor the first pose.

                // Example:
                // prev_pose_ = gtsam::Pose3(initial_rotation, initial_position);
                // ... add prior factors to graph ...
                // ... update smoother ...

                system_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "System initialized and is now live.");
            }
            else
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for first IMU and GPS messages to initialize...");
            }
            return;
        }

        // Step 3: If initialized, process the measurements.
        if (imu_measurements.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No IMU measurements for this update.");
            return;
        }

        // TODO: Integrate the IMU measurements in the `imu_measurements` vector
        // into your IMU preintegrator.

        // TODO: Create a new factor graph and values for this update.

        // TODO: Add an IMU factor using the preintegrated measurements. This will
        // create a constraint between the previous state and the new state.

        // TODO: Add a bias evolution factor (BetweenFactor on the bias).

        // TODO: If a GPS measurement is available (`latest_gps`), add a GPS factor
        // to constrain the new pose.

        // TODO: Add initial estimates for the new state variables (Pose, Velocity, Bias)
        // to the `Values` object. A good initial guess is to predict from the previous state.

        // TODO: Update your GTSAM smoother (e.g., `isam2_->update(graph, initial_values)`).

        // TODO: Extract the latest state estimate from the smoother's result.
        // This will update your `prev_pose_`, `prev_vel_`, `prev_bias_` variables.

        // TODO: Reset your IMU preintegrator.

        RCLCPP_INFO(this->get_logger(), "Factor graph update complete.");
    }

    void odomPublishTimerCallback()
    {
        if (!system_initialized_)
            return;
        publishTransformsAndOdometry();
    }

    void publishTransformsAndOdometry()
    {
        if (publish_global_tf_) // Only publish TF if enabled
        {
            geometry_msgs::msg::TransformStamped odom_to_base_transform;
            try
            {
                odom_to_base_transform = tf_buffer_->lookupTransform(
                    base_link_frame_, odom_frame_, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                return; // Don't publish if odom->base_link TF is not available
            }

            gtsam::Pose3 odom_to_base_gtsam(
                gtsam::Rot3::Quaternion(odom_to_base_transform.transform.rotation.w,
                                        odom_to_base_transform.transform.rotation.x,
                                        odom_to_base_transform.transform.rotation.y,
                                        odom_to_base_transform.transform.rotation.z),
                gtsam::Point3(odom_to_base_transform.transform.translation.x,
                              odom_to_base_transform.transform.translation.y,
                              odom_to_base_transform.transform.translation.z));

            gtsam::Pose3 map_to_odom_gtsam = prev_pose_ * odom_to_base_gtsam.inverse();

            geometry_msgs::msg::TransformStamped map_to_odom_tf;
            map_to_odom_tf.header.stamp = this->get_clock()->now();
            map_to_odom_tf.header.frame_id = map_frame_;
            map_to_odom_tf.child_frame_id = odom_frame_;
            map_to_odom_tf.transform.translation.x = map_to_odom_gtsam.x();
            map_to_odom_tf.transform.translation.y = map_to_odom_gtsam.y();
            map_to_odom_tf.transform.translation.z = map_to_odom_gtsam.z();
            map_to_odom_tf.transform.rotation.w = map_to_odom_gtsam.rotation().toQuaternion().w();
            map_to_odom_tf.transform.rotation.x = map_to_odom_gtsam.rotation().toQuaternion().x();
            map_to_odom_tf.transform.rotation.y = map_to_odom_gtsam.rotation().toQuaternion().y();
            map_to_odom_tf.transform.rotation.z = map_to_odom_gtsam.rotation().toQuaternion().z();
            tf_broadcaster_->sendTransform(map_to_odom_tf);
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_link_frame_;
        odom_msg.pose.pose.position.x = prev_pose_.x();
        odom_msg.pose.pose.position.y = prev_pose_.y();
        odom_msg.pose.pose.position.z = prev_pose_.z();
        odom_msg.pose.pose.orientation.w = prev_pose_.rotation().toQuaternion().w();
        odom_msg.pose.pose.orientation.x = prev_pose_.rotation().toQuaternion().x();
        odom_msg.pose.pose.orientation.y = prev_pose_.rotation().toQuaternion().y();
        odom_msg.pose.pose.orientation.z = prev_pose_.rotation().toQuaternion().z();

        global_odom_pub_->publish(odom_msg);
    }

    // ROS 2
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr factor_graph_timer_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    // Queues for incoming messages (?)
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;

    // State
    bool system_initialized_ = false;

    // TODO: Define your state variables here
    // Example:
    gtsam::Pose3 prev_pose_;
    // gtsam::Vector3 prev_vel_;
    // gtsam::imuBias::ConstantBias prev_bias_;

    // Parameters
    std::string imu_topic_, gps_odom_topic_, global_odom_topic_;
    std::string map_frame_, odom_frame_, base_link_frame_;
    double factor_graph_update_rate_, odom_publish_rate_;
    bool publish_global_tf_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}

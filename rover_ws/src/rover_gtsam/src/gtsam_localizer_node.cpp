// Created by Nelson Durrant, July 2025
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (x,y,z)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class GtsamLocalizerNode : public rclcpp::Node
{
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
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<double>("factor_graph_update_rate", 1.0); // Hz
        this->declare_parameter<double>("odom_publish_rate", 10.0);      // Hz
        this->declare_parameter<bool>("publish_global_tf", true);        // true or false
        this->declare_parameter<double>("smoother_lag", 2.0);            // seconds

        // Measurement noise parameters
        this->declare_parameter<double>("accel_noise_sigma", 0.1);
        this->declare_parameter<double>("gyro_noise_sigma", 0.01);
        this->declare_parameter<double>("accel_bias_rw_sigma", 0.001);
        this->declare_parameter<double>("gyro_bias_rw_sigma", 0.0001);
        this->declare_parameter<double>("gps_noise_sigma", 0.05);

        // Initial state prior noise parameters
        this->declare_parameter<double>("prior_pose_rot_sigma", 0.01);
        this->declare_parameter<double>("prior_pose_pos_sigma", 0.01);
        this->declare_parameter<double>("prior_vel_sigma", 0.1);
        this->declare_parameter<double>("prior_bias_sigma", 1e-3);

        imu_topic_ = this->get_parameter("imu_topic").as_string();
        gps_odom_topic_ = this->get_parameter("gps_odom_topic").as_string();
        global_odom_topic_ = this->get_parameter("global_odom_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        factor_graph_update_rate_ = this->get_parameter("factor_graph_update_rate").as_double();
        odom_publish_rate_ = this->get_parameter("odom_publish_rate").as_double();
        publish_global_tf_ = this->get_parameter("publish_global_tf").as_bool();
        smoother_lag_ = this->get_parameter("smoother_lag").as_double();

        accel_noise_sigma_ = this->get_parameter("accel_noise_sigma").as_double();
        gyro_noise_sigma_ = this->get_parameter("gyro_noise_sigma").as_double();
        accel_bias_rw_sigma_ = this->get_parameter("accel_bias_rw_sigma").as_double();
        gyro_bias_rw_sigma_ = this->get_parameter("gyro_bias_rw_sigma").as_double();
        gps_noise_sigma_ = this->get_parameter("gps_noise_sigma").as_double();

        prior_pose_rot_sigma_ = this->get_parameter("prior_pose_rot_sigma").as_double();
        prior_pose_pos_sigma_ = this->get_parameter("prior_pose_pos_sigma").as_double();
        prior_vel_sigma_ = this->get_parameter("prior_vel_sigma").as_double();
        prior_bias_sigma_ = this->get_parameter("prior_bias_sigma").as_double();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Before system is initialized, store the latest IMU orientation
        // to provide an initial heading estimate for other nodes.
        if (!system_initialized_)
        {
            latest_imu_rotation_ = gtsam::Rot3::Quaternion(
                msg->orientation.w, msg->orientation.x,
                msg->orientation.y, msg->orientation.z);
        }
        imu_queue_.push_back(msg);
    }

    void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        gps_queue_.push_back(msg);
    }

    void factorGraphTimerCallback()
    {
        if (imu_queue_.empty())
        {
            return;
        }

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
            if (latest_gps)
            {
                RCLCPP_INFO(this->get_logger(), "First GPS message received. Initializing system.");
                initializeSystem(latest_gps);
                system_initialized_ = true;
                RCLCPP_INFO(this->get_logger(), "System initialized and is now live.");
            }
            // Note: Pre-initialization publishing is handled in odomPublishTimerCallback
            return;
        }

        // Use the timestamp from the previous update as the starting point.
        double start_time = prev_time_;
        double last_imu_time = prev_time_;

        // Integrate each measurement with its own specific dt.
        for (const auto &imu_msg : imu_measurements)
        {
            double current_imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
            double dt = current_imu_time - last_imu_time;

            if (dt > 0)
            {
                gtsam::Vector3 accel(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                gtsam::Vector3 gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                imu_preintegrator_->integrateMeasurement(accel, gyro, dt);
            }
            last_imu_time = current_imu_time;
        }
        prev_time_ = last_imu_time;

        gtsam::NonlinearFactorGraph new_graph;
        gtsam::Values new_values;

        gtsam::Pose3 prev_pose = smoother_->calculateEstimate<gtsam::Pose3>(X(prev_step_));
        gtsam::Vector3 prev_vel = smoother_->calculateEstimate<gtsam::Vector3>(V(prev_step_));
        gtsam::NavState prev_state(prev_pose, prev_vel);

        auto predicted_state = imu_preintegrator_->predict(prev_state, prev_bias_);

        new_graph.emplace_shared<gtsam::CombinedImuFactor>(
            X(prev_step_), V(prev_step_), X(current_step_), V(current_step_),
            B(prev_step_), B(current_step_), *imu_preintegrator_);

        double total_dt_since_last_factor = last_imu_time - start_time;
        double dt_sqrt = sqrt(total_dt_since_last_factor);
        gtsam::Vector6 bias_sigmas;
        bias_sigmas << gtsam::Vector3::Constant(dt_sqrt * accel_bias_rw_sigma_),
            gtsam::Vector3::Constant(dt_sqrt * gyro_bias_rw_sigma_);

        new_graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            B(prev_step_), B(current_step_), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));


        if (latest_gps)
        {
            gtsam::Pose3 gps_pose = gtsam::Pose3(
                gtsam::Rot3::Quaternion(latest_gps->pose.pose.orientation.w, latest_gps->pose.pose.orientation.x, latest_gps->pose.pose.orientation.y, latest_gps->pose.pose.orientation.z),
                gtsam::Point3(latest_gps->pose.pose.position.x, latest_gps->pose.pose.position.y, latest_gps->pose.pose.position.z));

            gtsam::SharedDiagonal gps_noise = gtsam::noiseModel::Diagonal::Sigmas(
                gtsam::Vector3::Constant(gps_noise_sigma_));

            new_graph.emplace_shared<gtsam::GPSFactor>(X(current_step_), gps_pose.translation(), gps_noise);
        }

        new_values.insert(X(current_step_), predicted_state.pose());
        new_values.insert(V(current_step_), predicted_state.velocity());
        new_values.insert(B(current_step_), prev_bias_);

        smoother_->update(new_graph, new_values);

        prev_pose_ = smoother_->calculateEstimate<gtsam::Pose3>(X(current_step_));
        prev_vel_ = smoother_->calculateEstimate<gtsam::Vector3>(V(current_step_));
        prev_bias_ = smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_step_));

        imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);

        prev_step_ = current_step_;
        current_step_++;
    }

    void initializeSystem(const nav_msgs::msg::Odometry::SharedPtr &initial_gps)
    {
        gtsam::ISAM2Params isam2_params;
        isam2_params.relinearizeThreshold = 0.1;
        isam2_params.relinearizeSkip = 1;
        smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(smoother_lag_, isam2_params);

        auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        imu_params->n_gravity = gtsam::Vector3(0, 0, -9.81);
        imu_params->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(accel_noise_sigma_, 2);
        imu_params->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(gyro_noise_sigma_, 2);
        imu_params->biasAccCovariance = gtsam::Matrix33::Identity() * pow(accel_bias_rw_sigma_, 2);
        imu_params->biasOmegaCovariance = gtsam::Matrix33::Identity() * pow(gyro_bias_rw_sigma_, 2);
        imu_params->integrationCovariance = gtsam::Matrix33::Identity() * 1e-8;

        prev_time_ = rclcpp::Time(initial_gps->header.stamp).seconds();

        gtsam::Rot3 initial_rotation = gtsam::Rot3::Quaternion(
            initial_gps->pose.pose.orientation.w, initial_gps->pose.pose.orientation.x,
            initial_gps->pose.pose.orientation.y, initial_gps->pose.pose.orientation.z);
        gtsam::Point3 initial_position(
            initial_gps->pose.pose.position.x, initial_gps->pose.pose.position.y, initial_gps->pose.pose.position.z);
        prev_pose_ = gtsam::Pose3(initial_rotation, initial_position);
        prev_vel_ = gtsam::Vector3(0, 0, 0);
        prev_bias_ = gtsam::imuBias::ConstantBias();

        imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imu_params, prev_bias_);

        gtsam::NonlinearFactorGraph initial_graph;
        gtsam::Values initial_values;

        gtsam::SharedDiagonal pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector3::Constant(prior_pose_rot_sigma_), gtsam::Vector3::Constant(prior_pose_pos_sigma_)).finished());
        gtsam::SharedDiagonal vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, prior_vel_sigma_);
        gtsam::SharedDiagonal bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, prior_bias_sigma_);

        initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), prev_pose_, pose_noise);
        initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), prev_vel_, vel_noise);
        initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), prev_bias_, bias_noise);

        initial_values.insert(X(0), prev_pose_);
        initial_values.insert(V(0), prev_vel_);
        initial_values.insert(B(0), prev_bias_);

        smoother_->update(initial_graph, initial_values);

        prev_step_ = 0;
        current_step_ = 1;
    }

    void odomPublishTimerCallback()
    {
        if (!system_initialized_)
        {
            // Don't publish until we have received at least one IMU message
            // The check below compares the current rotation to a new identity rotation.
            if (latest_imu_rotation_.equals(gtsam::Rot3::Identity()))
            {
                return;
            }

            // Publish an odometry message with position at zero and orientation
            // from the latest IMU message. This provides an initial heading
            // for other nodes like navsat_transform_node.
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = this->get_clock()->now();
            odom_msg.header.frame_id = odom_frame_;
            odom_msg.child_frame_id = base_frame_;

            // Position at zero
            odom_msg.pose.pose.position.x = 0.0;
            odom_msg.pose.pose.position.y = 0.0;
            odom_msg.pose.pose.position.z = 0.0;

            // Orientation from the latest IMU message
            odom_msg.pose.pose.orientation.w = latest_imu_rotation_.toQuaternion().w();
            odom_msg.pose.pose.orientation.x = latest_imu_rotation_.toQuaternion().x();
            odom_msg.pose.pose.orientation.y = latest_imu_rotation_.toQuaternion().y();
            odom_msg.pose.pose.orientation.z = latest_imu_rotation_.toQuaternion().z();

            // Set a covariance matrix to indicate high confidence in yaw
            // and low confidence in everything else.
            odom_msg.pose.covariance.fill(1e3); // High variance for all by default
            odom_msg.pose.covariance[35] = 0.01; // Low variance for yaw

            // This topic can be remapped to navsat_transform_node's "odometry/filtered" input
            global_odom_pub_->publish(odom_msg);
        }
        else // System is initialized, run the normal publisher
        {
            publishTransformsAndOdometry();
        }
    }

    void publishTransformsAndOdometry()
    {
        if (publish_global_tf_)
        {
            geometry_msgs::msg::TransformStamped odom_to_base_transform;
            try
            {
                odom_to_base_transform = tf_buffer_->lookupTransform(
                    base_frame_, odom_frame_, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not lookup transform from %s to %s: %s",
                            odom_frame_.c_str(), base_frame_.c_str(), ex.what());
                return;
            }

            gtsam::Pose3 odom_to_base_gtsam(
                gtsam::Rot3::Quaternion(odom_to_base_transform.transform.rotation.w,
                                        odom_to_base_transform.transform.rotation.x,
                                        odom_to_base_transform.transform.rotation.y,
                                        odom_to_base_transform.transform.rotation.z),
                gtsam::Point3(odom_to_base_transform.transform.translation.x,
                              odom_to_base_transform.transform.translation.y,
                              odom_to_base_transform.transform.translation.z));

            gtsam::Pose3 map_to_odom_gtsam = prev_pose_ * odom_to_base_gtsam;

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
        odom_msg.child_frame_id = base_frame_;
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

    // Queues for incoming messages
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;

    // State
    bool system_initialized_ = false;
    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::imuBias::ConstantBias prev_bias_;
    gtsam::Rot3 latest_imu_rotation_; // ADDED: For pre-initialization heading
    double prev_time_ = 0.0;
    long unsigned int prev_step_ = 0;
    long unsigned int current_step_ = 1;

    // GTSAM
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;

    // Parameters
    std::string imu_topic_, gps_odom_topic_, global_odom_topic_;
    std::string map_frame_, odom_frame_, base_frame_;
    double factor_graph_update_rate_, odom_publish_rate_;
    bool publish_global_tf_;
    double smoother_lag_;

    // Noise Model Parameters
    double accel_noise_sigma_, gyro_noise_sigma_;
    double accel_bias_rw_sigma_, gyro_bias_rw_sigma_;
    double gps_noise_sigma_;
    double prior_pose_rot_sigma_, prior_pose_pos_sigma_;
    double prior_vel_sigma_;
    double prior_bias_sigma_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtsamLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
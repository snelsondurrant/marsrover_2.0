/**
 * @file gtsam_dvl_localizer_node.cpp
 * @brief Fuses IMU, GPS, DVL, depth, and heading data using a GTSAM Fixed-Lag Smoother with IMU pre-integration.
 * @author Nelson Durrant
 * @date July 2025
 *
 * Subscribes:
 * - /imu/data (sensor_msgs/msg/Imu)
 * - /odometry/gps (nav_msgs/msg/Odometry)
 * - 'odom' -> 'base_link' transform
 * Publishes:
 * - /odometry/global (nav_msgs/msg/Odometry)
 * - 'map' -> 'odom' transform
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <deque>
#include <cmath>

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

// GTSAM symbol shorthand
using gtsam::symbol_shorthand::B; // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Velocity (x,y,z)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

class GtsamLocalizerNode : public rclcpp::Node
{
public:
    GtsamLocalizerNode() : Node("gtsam_dvl_localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting GTSAM Localizer Node...");

        loadParameters();
        setupRosInterfaces();

        RCLCPP_INFO(this->get_logger(), "Initialization complete! Waiting for first IMU and GPS messages...");
    }

private:
    /**
     * @brief Declares and loads all ROS parameters for the node.
     */
    void loadParameters()
    {
        // --- ROS Topics and Frames ---
        imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu/data");
        gps_odom_topic_ = this->declare_parameter<std::string>("gps_odom_topic", "/odometry/gps");
        global_odom_topic_ = this->declare_parameter<std::string>("global_odom_topic", "/odometry/global");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        // --- Node Settings ---
        factor_graph_update_rate_ = this->declare_parameter<double>("factor_graph_update_rate", 10.0); // Hz
        odom_publish_rate_ = this->declare_parameter<double>("odom_publish_rate", 50.0);               // Hz
        publish_global_tf_ = this->declare_parameter<bool>("publish_global_tf", true);
        smoother_lag_ = this->declare_parameter<double>("smoother_lag", 3.0); // seconds

        // --- Measurement Noise (Standard Deviations) ---
        accel_noise_sigma_ = this->declare_parameter<double>("imu.accel_noise_sigma", 0.1);        // m/s^2
        gyro_noise_sigma_ = this->declare_parameter<double>("imu.gyro_noise_sigma", 0.01);         // rad/s
        accel_bias_rw_sigma_ = this->declare_parameter<double>("imu.accel_bias_rw_sigma", 1.0e-4); // m/s^3
        gyro_bias_rw_sigma_ = this->declare_parameter<double>("imu.gyro_bias_rw_sigma", 1.0e-5);   // rad/s^2
        gps_noise_sigma_ = this->declare_parameter<double>("gps.noise_sigma", 0.5);                // meters

        // --- Initial State Prior Uncertainty (Standard Deviations) ---
        prior_pose_rot_sigma_ = this->declare_parameter<double>("prior.pose_rot_sigma", 0.05); // rad
        prior_pose_pos_sigma_ = this->declare_parameter<double>("prior.pose_pos_sigma", 0.5);  // meters
        prior_vel_sigma_ = this->declare_parameter<double>("prior.vel_sigma", 0.1);            // m/s
        prior_bias_sigma_ = this->declare_parameter<double>("prior.bias_sigma", 1e-3);
    }

    /**
     * @brief Sets up publishers, subscribers, and timers.
     */
    void setupRosInterfaces()
    {
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
    }

    /**
     * @brief Stores incoming IMU messages in a (thread-safe) queue.
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        imu_queue_.push_back(msg);
    }

    /**
     * @brief Stores incoming GPS odometry messages in a (thread-safe) queue.
     */
    void gpsOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        gps_queue_.push_back(msg);
    }

    /**
     * @brief Main timer callback to process sensor data and update the factor graph.
     */
    void factorGraphTimerCallback()
    {
        // Wait until we have at least one IMU message.
        if (imu_queue_.empty())
        {
            return;
        }

        // Attempt to initialize the system on the first valid GPS message.
        if (!system_initialized_)
        {
            if (!gps_queue_.empty())
            {
                initializeSystem(gps_queue_.back());
                system_initialized_ = true;
                gps_queue_.clear();
                imu_queue_.clear();
            }
            return;
        }

        // --- Process Queued Data ---
        std::vector<sensor_msgs::msg::Imu::SharedPtr> imu_measurements;
        imu_measurements.assign(imu_queue_.begin(), imu_queue_.end());
        imu_queue_.clear();

        nav_msgs::msg::Odometry::SharedPtr latest_gps;
        if (!gps_queue_.empty())
        {
            latest_gps = gps_queue_.back();
            gps_queue_.clear();
        }

        // --- IMU Pre-integration ---
        double last_imu_time = prev_time_;
        for (const auto &imu_msg : imu_measurements)
        {
            double current_imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
            double dt = current_imu_time - last_imu_time; // Get the exact time difference

            if (dt > 1e-4) // Ensure we have a reasonable timestep to work with
            {
                gtsam::Vector3 accel(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
                gtsam::Vector3 gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
                
                // Transform IMU data into the base frame (to account for different mounting configs).
                try
                {
                    geometry_msgs::msg::TransformStamped tf_stamped = tf_buffer_->lookupTransform(
                        base_frame_, imu_msg->header.frame_id, imu_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));

                    geometry_msgs::msg::Vector3 transformed_accel, transformed_gyro;
                    // For vectors, doTransform() applies only the rotational part of the transform.
                    tf2::doTransform(imu_msg->linear_acceleration, transformed_accel, tf_stamped);
                    tf2::doTransform(imu_msg->angular_velocity, transformed_gyro, tf_stamped);

                    accel = gtsam::Vector3(transformed_accel.x, transformed_accel.y, transformed_accel.z);
                    gyro = gtsam::Vector3(transformed_gyro.x, transformed_gyro.y, transformed_gyro.z);
                }
                catch (const tf2::TransformException &ex)
                {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "IMU transform failed from '%s' to '%s': %s. Using untransformed data.",
                                         imu_msg->header.frame_id.c_str(), base_frame_.c_str(), ex.what());
                }
                imu_preintegrator_->integrateMeasurement(accel, gyro, dt); // Use GTSAM's pre-integrator
            }
            last_imu_time = current_imu_time;
        }

        // --- Factor Graph Construction ---
        gtsam::NonlinearFactorGraph new_graph;
        gtsam::Values new_values;

        // Predict the next state using the pre-integrated IMU measurements.
        auto predicted_state = imu_preintegrator_->predict(
            gtsam::NavState(prev_pose_, prev_vel_), prev_bias_);

        // Add a new IMU between factor to constrain the two state estimates.
        new_graph.emplace_shared<gtsam::CombinedImuFactor>(
            X(prev_step_), V(prev_step_), X(current_step_), V(current_step_),
            B(prev_step_), B(current_step_), *imu_preintegrator_);

        // Add a between factor on the IMU bias to model it as a random walk.
        // https://github.com/borglab/gtsam/blob/develop/examples/CombinedImuFactorsExample.cpp
        double dt_since_last_factor = last_imu_time - prev_time_;
        double dt_sqrt = sqrt(dt_since_last_factor);
        gtsam::Vector6 bias_sigmas;
        bias_sigmas << gtsam::Vector3::Constant(dt_sqrt * accel_bias_rw_sigma_),
            gtsam::Vector3::Constant(dt_sqrt * gyro_bias_rw_sigma_);
        new_graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
            B(prev_step_), B(current_step_), gtsam::imuBias::ConstantBias(),
            gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas));

        // If we have a GPS measurement, add a GPS unary factor to the closest node in time.
        if (latest_gps)
        {
            double gps_time = rclcpp::Time(latest_gps->header.stamp).seconds();

            // Determine which node (previous or current) is closer in time.
            // This seems like a simple way to solve the problem Matthew and Braden identified before.
            bool closer_to_prev = (gps_time - prev_time_) < (last_imu_time - gps_time);
            long unsigned int target_node_key = closer_to_prev ? prev_step_ : current_step_;

            RCLCPP_DEBUG(this->get_logger(), "GPS measurement at time %.4f. Attaching to node %lu (time %.4f).",
                         gps_time, target_node_key, closer_to_prev ? prev_time_ : last_imu_time);

            // No transform needed, GPS data arrives pre-converted into the 'map' frame.
            gtsam::Point3 gps_position(latest_gps->pose.pose.position.x,
                                       latest_gps->pose.pose.position.y,
                                       latest_gps->pose.pose.position.z);
            auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(gps_noise_sigma_));

            // Add the GPS unary factor to the graph, connected to the chosen target node.
            new_graph.emplace_shared<gtsam::GPSFactor>(X(target_node_key), gps_position, gps_noise);
        }

        // Insert the predicted state as an initial estimate for the new state.
        new_values.insert(X(current_step_), predicted_state.pose());
        new_values.insert(V(current_step_), predicted_state.velocity());
        new_values.insert(B(current_step_), prev_bias_);

        // Create timestamp map for new variables
        // The Fixed-lag smoother uses this to know when to drop old data.
        gtsam::IncrementalFixedLagSmoother::KeyTimestampMap new_timestamps;
        new_timestamps[X(current_step_)] = last_imu_time;
        new_timestamps[V(current_step_)] = last_imu_time;
        new_timestamps[B(current_step_)] = last_imu_time;

        // --- Update the Smoother ---
        smoother_->update(new_graph, new_values, new_timestamps);

        // Update the state with the new optimized estimate.
        prev_pose_ = smoother_->calculateEstimate<gtsam::Pose3>(X(current_step_));
        prev_vel_ = smoother_->calculateEstimate<gtsam::Vector3>(V(current_step_));
        prev_bias_ = smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(current_step_));

        // Reset the pre-integrator with the new bias estimate.
        imu_preintegrator_->resetIntegrationAndSetBias(prev_bias_);

        prev_time_ = last_imu_time;
        prev_step_ = current_step_++;
    }

    /**
     * @brief Initializes the GTSAM smoother and sets initial state from the first GPS message.
     */
    void initializeSystem(const nav_msgs::msg::Odometry::SharedPtr &initial_gps)
    {
        RCLCPP_INFO(this->get_logger(), "First GPS message received. Initializing system...");

        // --- Configure Fixed-Lag Smoother ---
        gtsam::ISAM2Params isam2_params;
        isam2_params.relinearizeThreshold = 0.1;
        isam2_params.relinearizeSkip = 1;
        smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(smoother_lag_, isam2_params);

        // --- Configure IMU Pre-integration ---
        auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        imu_params->n_gravity = gtsam::Vector3(0, 0, -9.81); // ENU, so gravity is in the negative Z direction
        imu_params->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(accel_noise_sigma_, 2);
        imu_params->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(gyro_noise_sigma_, 2);
        imu_params->biasAccCovariance = gtsam::Matrix33::Identity() * pow(accel_bias_rw_sigma_, 2);
        imu_params->biasOmegaCovariance = gtsam::Matrix33::Identity() * pow(gyro_bias_rw_sigma_, 2);
        imu_params->integrationCovariance = gtsam::Matrix33::Identity() * 1e-8; // Seems to work haha

        // --- Set Initial State ---
        prev_time_ = rclcpp::Time(initial_gps->header.stamp).seconds();

        gtsam::Rot3 initial_rotation = gtsam::Rot3::Quaternion(
            initial_gps->pose.pose.orientation.w, initial_gps->pose.pose.orientation.x,
            initial_gps->pose.pose.orientation.y, initial_gps->pose.pose.orientation.z);
        gtsam::Point3 initial_position(
            initial_gps->pose.pose.position.x, initial_gps->pose.pose.position.y, initial_gps->pose.pose.position.z);

        prev_pose_ = gtsam::Pose3(initial_rotation, initial_position);
        prev_vel_ = gtsam::Vector3(0, 0, 0);         // Assume starting from rest
        prev_bias_ = gtsam::imuBias::ConstantBias(); // Assume zero initial bias

        imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imu_params, prev_bias_);

        // --- Add Prior Factors to the Graph ---
        gtsam::NonlinearFactorGraph initial_graph;
        gtsam::Values initial_values;

        // Priors define the initial uncertainty in our state.
        auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector3::Constant(prior_pose_rot_sigma_),
             gtsam::Vector3::Constant(prior_pose_pos_sigma_))
                .finished());
        auto vel_noise = gtsam::noiseModel::Isotropic::Sigma(3, prior_vel_sigma_);
        auto bias_noise = gtsam::noiseModel::Isotropic::Sigma(6, prior_bias_sigma_);

        initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), prev_pose_, pose_noise);
        initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(V(0), prev_vel_, vel_noise);
        initial_graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(B(0), prev_bias_, bias_noise);

        initial_values.insert(X(0), prev_pose_);
        initial_values.insert(V(0), prev_vel_);
        initial_values.insert(B(0), prev_bias_);

        // Create timestamp map for initial variables
        // The Fixed-lag smoother uses this to know when to drop old data.
        gtsam::IncrementalFixedLagSmoother::KeyTimestampMap initial_timestamps;
        initial_timestamps[X(0)] = prev_time_;
        initial_timestamps[V(0)] = prev_time_;
        initial_timestamps[B(0)] = prev_time_;

        // --- Update the Smoother with the Initial State ---
        smoother_->update(initial_graph, initial_values, initial_timestamps);

        RCLCPP_INFO(this->get_logger(), "System initialized and is now live.");
    }

    /**
     * @brief Timer callback for publishing odometry and transforms.
     */
    void odomPublishTimerCallback()
    {
        // IMPORTANT! Fix for integration with the 'navsat_transform_node':
        // Before initialization, publish a zero-position odometry message with IMU heading.
        if (!system_initialized_)
        {
            if (!imu_queue_.empty())
            {
                auto last_imu = imu_queue_.back();
                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = this->get_clock()->now();
                odom_msg.header.frame_id = odom_frame_;
                odom_msg.child_frame_id = base_frame_;
                odom_msg.pose.pose.orientation = last_imu->orientation;
                // Indicate high confidence in yaw, low confidence elsewhere.
                odom_msg.pose.covariance.fill(1e3);
                odom_msg.pose.covariance[35] = 0.01; // Yaw covariance
                global_odom_pub_->publish(odom_msg);
            }
        }
        else // System is initialized, publish the fused state.
        {
            publishFusedState();
        }
    }

    /**
     * @brief Publishes the fused odometry message and the map->odom transform.
     */
    void publishFusedState()
    {
        // --- Publish map->odom transform ---
        if (publish_global_tf_)
        {
            geometry_msgs::msg::TransformStamped odom_to_base_tf;
            try
            {
                // Look up the odom->base_link transform from another source (e.g., local EKF node)
                odom_to_base_tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not lookup transform from '%s' to '%s': %s",
                            odom_frame_.c_str(), base_frame_.c_str(), ex.what());
                return;
            }

            // Convert the odom->base transform to a GTSAM type.
            tf2::Transform odom_to_base_tf2;
            tf2::fromMsg(odom_to_base_tf.transform, odom_to_base_tf2);
            gtsam::Pose3 odom_to_base_gtsam(
                gtsam::Rot3(odom_to_base_tf2.getRotation().w(), odom_to_base_tf2.getRotation().x(),
                            odom_to_base_tf2.getRotation().y(), odom_to_base_tf2.getRotation().z()),
                gtsam::Point3(odom_to_base_tf2.getOrigin().x(), odom_to_base_tf2.getOrigin().y(), odom_to_base_tf2.getOrigin().z()));

            // Calculate map->odom transform: T_map_odom = T_map_base * (T_odom_base)^-1
            // Mangelson's EN EN 433 class really seeing some direct application here haha.
            gtsam::Pose3 map_to_odom_gtsam = prev_pose_ * odom_to_base_gtsam.inverse();

            // Broadcast the calculated transform.
            geometry_msgs::msg::TransformStamped map_to_odom_tf_msg;
            map_to_odom_tf_msg.header.stamp = this->get_clock()->now();
            map_to_odom_tf_msg.header.frame_id = map_frame_;
            map_to_odom_tf_msg.child_frame_id = odom_frame_;
            map_to_odom_tf_msg.transform.translation.x = map_to_odom_gtsam.x();
            map_to_odom_tf_msg.transform.translation.y = map_to_odom_gtsam.y();
            map_to_odom_tf_msg.transform.translation.z = map_to_odom_gtsam.z();
            map_to_odom_tf_msg.transform.rotation.w = map_to_odom_gtsam.rotation().toQuaternion().w();
            map_to_odom_tf_msg.transform.rotation.x = map_to_odom_gtsam.rotation().toQuaternion().x();
            map_to_odom_tf_msg.transform.rotation.y = map_to_odom_gtsam.rotation().toQuaternion().y();
            map_to_odom_tf_msg.transform.rotation.z = map_to_odom_gtsam.rotation().toQuaternion().z();
            tf_broadcaster_->sendTransform(map_to_odom_tf_msg);
        }

        // --- Publish the global odometry message ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_frame_;
        // Pose
        odom_msg.pose.pose.position.x = prev_pose_.x();
        odom_msg.pose.pose.position.y = prev_pose_.y();
        odom_msg.pose.pose.position.z = prev_pose_.z();
        odom_msg.pose.pose.orientation.w = prev_pose_.rotation().toQuaternion().w();
        odom_msg.pose.pose.orientation.x = prev_pose_.rotation().toQuaternion().x();
        odom_msg.pose.pose.orientation.y = prev_pose_.rotation().toQuaternion().y();
        odom_msg.pose.pose.orientation.z = prev_pose_.rotation().toQuaternion().z();
        // Twist (velocity)
        odom_msg.twist.twist.linear.x = prev_vel_.x();
        odom_msg.twist.twist.linear.y = prev_vel_.y();
        odom_msg.twist.twist.linear.z = prev_vel_.z();

        global_odom_pub_->publish(odom_msg);
    }

    // --- ROS 2 Interfaces ---
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr factor_graph_timer_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    // --- Message Queues ---
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;

    // --- System State ---
    bool system_initialized_ = false;
    double prev_time_ = 0.0;
    long unsigned int prev_step_ = 0;
    long unsigned int current_step_ = 1;

    // --- GTSAM Objects ---
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::imuBias::ConstantBias prev_bias_;

    // --- Parameters ---
    std::string imu_topic_, gps_odom_topic_, global_odom_topic_;
    std::string map_frame_, odom_frame_, base_frame_;
    double factor_graph_update_rate_, odom_publish_rate_;
    bool publish_global_tf_;
    double smoother_lag_;
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
    // A single-threaded executor simplifies data access management.
    // We'd need to use a mutex or lock if we ever use a MuliThreadedExecutor.
    rclcpp::spin(std::make_shared<GtsamLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
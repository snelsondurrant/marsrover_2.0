/**
 * @file gtsam_localizer_node.cpp
 * @brief Fuses IMU, GPS, and Heading data using a GTSAM Fixed-Lag Smoother.
 * @author Nelson Durrant
 * @date July 2025
 *
 * NOTE: This takes too much compute to run at the same time as Nav2.
 *
 * Subscribes:
 * - /imu/data (sensor_msgs/msg/Imu)
 * - /odometry/gps (nav_msgs/msg/Odometry)
 * - /heading/data (sensor_msgs/msg/Imu)
 * - 'odom' -> 'base_link' transform
 * Publishes:
 * - /odometry/global (nav_msgs/msg/Odometry)
 * - /smoothed_path (nav_msgs/msg/Path)
 * - 'map' -> 'odom' transform
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
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
#include <gtsam/nonlinear/NonlinearFactor.h>

// GTSAM symbol shorthand
using gtsam::symbol_shorthand::B; // Bias (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Velocity (x,y,z)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

/**
 * @brief Unary factor for an absolute heading/orientation measurement.
 */
class HeadingFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
    gtsam::Rot3 measured_rot_;

public:
    HeadingFactor(gtsam::Key poseKey, const gtsam::Rot3 &measured_rot, const gtsam::SharedNoiseModel &model)
        : NoiseModelFactor1<gtsam::Pose3>(model, poseKey), measured_rot_(measured_rot) {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
        // TODO: I just used Gemini 2.5 Pro to convert these error functions from Matthew's code,
        // they need to be verified.

        // Get the rotation from the pose
        const gtsam::Rot3 &pose_rot = pose.rotation();

        // Calculate the error rotation: R_err = R_measured^{-1} * R_pose
        gtsam::Rot3 error_rot = measured_rot_.inverse() * pose_rot;

        // The error vector is the tangent space representation of the error rotation
        gtsam::Vector3 error = gtsam::Rot3::Logmap(error_rot);

        if (H)
        {
            // Jacobian of the error with respect to the pose
            // The error is e = Logmap(R_measured^{-1} * R_pose).
            // The derivative w.r.t. rotation is d Log(R_err * exp(w)) / dw at w=0.
            // GTSAM provides LogmapDerivative for this.
            gtsam::Matrix3 H_rot = gtsam::Rot3::LogmapDerivative(error);

            // The error does not depend on translation.
            gtsam::Matrix3 H_trans = gtsam::Matrix3::Zero();

            *H = (gtsam::Matrix(3, 6) << H_rot, H_trans).finished();
        }

        return error;
    }
};

class GtsamLocalizerNode : public rclcpp::Node
{
public:
    GtsamLocalizerNode() : Node("gtsam_localizer_node")
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
        smoothed_path_topic_ = this->declare_parameter<std::string>("smoothed_path_topic", "/smoothed_path");
        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        heading_topic_ = this->declare_parameter<std::string>("heading_topic", "/heading/data");

        // --- Node Settings ---
        factor_graph_update_rate_ = this->declare_parameter<double>("factor_graph_update_rate", 10.0); // Hz
        odom_publish_rate_ = this->declare_parameter<double>("odom_publish_rate", 50.0);               // Hz
        publish_global_tf_ = this->declare_parameter<bool>("publish_global_tf", true);
        publish_smoothed_path_ = this->declare_parameter<bool>("publish_smoothed_path", true);
        smoother_lag_ = this->declare_parameter<double>("smoother_lag", 15.0); // seconds

        // --- Measurement Noise (Standard Deviations) ---
        accel_noise_sigma_ = this->declare_parameter<double>("imu.accel_noise_sigma", 0.1);        // m/s^2
        gyro_noise_sigma_ = this->declare_parameter<double>("imu.gyro_noise_sigma", 0.01);         // rad/s
        accel_bias_rw_sigma_ = this->declare_parameter<double>("imu.accel_bias_rw_sigma", 1.0e-4); // m/s^3
        gyro_bias_rw_sigma_ = this->declare_parameter<double>("imu.gyro_bias_rw_sigma", 1.0e-5);   // rad/s^2
        gps_noise_sigma_ = this->declare_parameter<double>("gps.noise_sigma", 0.5);                // meters
        heading_noise_sigma_ = this->declare_parameter<double>("heading.noise_sigma", 0.05);       // rad

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
        global_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(smoothed_path_topic_, 10);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_, 200, 
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { imu_queue_.push_back(msg); });
        gps_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(gps_odom_topic_, 20, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { gps_queue_.push_back(msg); });
        heading_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(heading_topic_, 20,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) { heading_queue_.push_back(msg); });

        factor_graph_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / factor_graph_update_rate_)),
            std::bind(&GtsamLocalizerNode::factorGraphTimerCallback, this));

        odom_pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / odom_publish_rate_)),
            std::bind(&GtsamLocalizerNode::odomPublishTimerCallback, this));
    }

    /**
     * @brief Determines which graph node (previous or current) is closer in time to a measurement.
     */
    unsigned long getClosestNodeKey(const rclcpp::Time &msg_stamp, double current_update_time) const
    {
        double msg_time = msg_stamp.seconds();
        bool closer_to_prev = std::abs(msg_time - prev_time_) < std::abs(current_update_time - msg_time);
        return closer_to_prev ? prev_step_ : current_step_;
    }

    /**
     * @brief Main timer callback to process sensor data and update the factor graph.
     */
    void factorGraphTimerCallback()
    {
        if (imu_queue_.empty())
            return;

        // Attempt to initialize the system on the first valid GPS and IMU messages.
        if (!system_initialized_)
        {
            // We need both GPS and a few IMU messages to initialize
            if (!gps_queue_.empty() && !imu_queue_.empty())
            {
                // Pass the GPS message and the whole IMU queue
                initializeSystem(gps_queue_.back(), imu_queue_);

                // On successful initialization, clear queues
                if (system_initialized_)
                {
                    gps_queue_.clear();
                    imu_queue_.clear();
                }
            }
            return;
        }

        // --- One-Time Transform Lookups ---
        // Once initialized, look for the other static transforms if we haven't already.
        if (!have_heading_to_base_tf_ && !heading_queue_.empty())
        {
            try
            {
                // Use a zero timestamp to get the latest available static transform.
                heading_to_base_tf_ = tf_buffer_->lookupTransform(
                    base_frame_, heading_queue_.front()->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0));
                have_heading_to_base_tf_ = true;
                RCLCPP_INFO(this->get_logger(), "Successfully looked up static transform from '%s' to '%s'",
                            heading_queue_.front()->header.frame_id.c_str(), base_frame_.c_str());
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not get Heading transform: %s. Will keep trying.", ex.what());
            }
        }

        // Do not proceed until the static IMU -> base_link transform is available.
        if (!have_imu_to_base_tf_)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for IMU to base_frame transform...");
            return;
        }

        // --- Process Queued Data ---
        nav_msgs::msg::Odometry::SharedPtr latest_gps;
        if (!gps_queue_.empty())
        {
            latest_gps = gps_queue_.back();
            gps_queue_.clear();
        }

        sensor_msgs::msg::Imu::SharedPtr latest_heading;
        if (!heading_queue_.empty())
        {
            latest_heading = heading_queue_.back();
            heading_queue_.clear();
        }

        // --- IMU Pre-integration ---
        double last_imu_time = prev_time_;
        rclcpp::Time last_imu_stamp = this->get_clock()->now();
        for (const auto &imu_msg : imu_queue_)
        {
            double current_imu_time = rclcpp::Time(imu_msg->header.stamp).seconds();
            double dt = current_imu_time - last_imu_time; // Get the exact time difference

            if (dt > 1e-4) // Ensure we have a reasonable timestep to work with
            {
                // Transform IMU data to the base_link frame before integration
                geometry_msgs::msg::Vector3 transformed_accel, transformed_gyro;
                tf2::doTransform(imu_msg->linear_acceleration, transformed_accel, imu_to_base_tf_);
                tf2::doTransform(imu_msg->angular_velocity, transformed_gyro, imu_to_base_tf_);

                gtsam::Vector3 accel = toGtsam(transformed_accel);
                gtsam::Vector3 gyro = toGtsam(transformed_gyro);
                imu_preintegrator_->integrateMeasurement(accel, gyro, dt);
            }
            last_imu_time = current_imu_time;
            last_imu_stamp = imu_msg->header.stamp;
        }
        imu_queue_.clear();

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

        // If we have a GPS measurement, add a GPS unary factor.
        if (latest_gps)
        {
            // Determine which node (previous or current) is closer in time.
            // This seems like a simple way to solve the problem Matthew and Braden identified before.
            unsigned long target_node_key = getClosestNodeKey(latest_gps->header.stamp, last_imu_time);

            // GPS data is assumed to be in the 'map' frame, no transform needed.
            gtsam::Point3 gps_position = toGtsam(latest_gps->pose.pose.position);
            auto gps_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(gps_noise_sigma_));
            new_graph.emplace_shared<gtsam::GPSFactor>(X(target_node_key), gps_position, gps_noise);
        }
        
        // If we have a heading measurement, add a Heading unary factor.
        if (latest_heading)
        {
            gtsam::Rot3 heading_rot_sensor = toGtsam(latest_heading->orientation);

            // Transform heading into the base_link frame (upside-down mounting config, etc).
            // R_map_base = R_map_sensor * (R_base_sensor)^-1
            // Mangelson's EC EN 433 class again, haha.
            if (have_heading_to_base_tf_)
            {
                gtsam::Rot3 R_base_sensor = toGtsam(heading_to_base_tf_.transform.rotation);
                heading_rot_sensor = heading_rot_sensor * R_base_sensor.inverse();
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Heading transform from '%s' to '%s' not yet available. Using untransformed data.",
                                     latest_heading->header.frame_id.c_str(), base_frame_.c_str());
            }

            unsigned long target_node_key = getClosestNodeKey(latest_heading->header.stamp, last_imu_time);
            // TODO: We might not want an isotropic noise model here in the future.
            // We probably just want to use the yaw component of the heading measurement.
            auto heading_noise = gtsam::noiseModel::Isotropic::Sigma(3, heading_noise_sigma_);
            new_graph.emplace_shared<HeadingFactor>(X(target_node_key), heading_rot_sensor, heading_noise);
        }

        // Insert the predicted state as an initial estimate for the new state.
        new_values.insert(X(current_step_), predicted_state.pose());
        new_values.insert(V(current_step_), predicted_state.velocity());
        new_values.insert(B(current_step_), prev_bias_);

        // Create timestamp map for new variables.
        // The Fixed-lag smoother uses this to know when to drop old data.
        gtsam::IncrementalFixedLagSmoother::KeyTimestampMap new_timestamps;
        new_timestamps[X(current_step_)] = last_imu_time;
        new_timestamps[V(current_step_)] = last_imu_time;
        new_timestamps[B(current_step_)] = last_imu_time;
        key_timestamps_[X(current_step_)] = last_imu_stamp;

        // --- Update the Smoother ---
        smoother_->update(new_graph, new_values, new_timestamps);

        // Manually prune the key_timestamps_ map to remove old entries
        double prune_time = last_imu_time - smoother_lag_;
        for (auto it = key_timestamps_.begin(); it != key_timestamps_.end(); ) {
            if (it->second.seconds() < prune_time) {
                it = key_timestamps_.erase(it);
            } else {
                ++it;
            }
        }

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
     * @brief Initializes the GTSAM smoother and sets initial state dynamically.
     */
    void initializeSystem(const nav_msgs::msg::Odometry::SharedPtr &initial_gps,
                          const std::deque<sensor_msgs::msg::Imu::SharedPtr> &imu_buffer)
    {
        // IMPORTANT! This assumes the robot is stationary on startup.
        RCLCPP_INFO(this->get_logger(), "GPS and IMU data received. Dynamically initializing orientation...");

        // --- Get the static transform from the IMU frame to the base_link frame ---
        try
        {
            // Use a zero timestamp to get the latest available static transform.
            imu_to_base_tf_ = tf_buffer_->lookupTransform(
                base_frame_, imu_buffer.back()->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(5.0));
            have_imu_to_base_tf_ = true;
            RCLCPP_INFO(this->get_logger(), "Successfully looked up static transform from '%s' to '%s'",
                        imu_buffer.back()->header.frame_id.c_str(), base_frame_.c_str());
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s. Aborting initialization.", ex.what());
            return;
        }

        // --- Configure Fixed-Lag Smoother ---
        gtsam::ISAM2Params isam2_params;
        isam2_params.relinearizeThreshold = 0.1;
        isam2_params.relinearizeSkip = 1;
        smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(smoother_lag_, isam2_params);

        // --- Configure IMU Pre-integration ---
        auto imu_params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();
        imu_params->n_gravity = gtsam::Vector3(0, 0, -9.8); // Exactly 9.8 in Gazebo
        imu_params->accelerometerCovariance = gtsam::Matrix33::Identity() * pow(accel_noise_sigma_, 2);
        imu_params->gyroscopeCovariance = gtsam::Matrix33::Identity() * pow(gyro_noise_sigma_, 2);
        imu_params->biasAccCovariance = gtsam::Matrix33::Identity() * pow(accel_bias_rw_sigma_, 2);
        imu_params->biasOmegaCovariance = gtsam::Matrix33::Identity() * pow(gyro_bias_rw_sigma_, 2);
        imu_params->integrationCovariance = gtsam::Matrix33::Identity() * 1e-8;

        // --- Dynamically Determine Initial Orientation ---

        // Average recent IMU messages to get a stable gravity vector in the IMU frame.
        gtsam::Vector3 avg_accel_imu_frame(0, 0, 0);
        size_t num_to_average = std::min((size_t)50, imu_buffer.size());
        if (num_to_average == 0)
        {
            RCLCPP_WARN(this->get_logger(), "Cannot initialize dynamically, IMU buffer is empty. Aborting.");
            return;
        }
        for (size_t i = imu_buffer.size() - num_to_average; i < imu_buffer.size(); ++i)
        {
            avg_accel_imu_frame += toGtsam(imu_buffer[i]->linear_acceleration);
        }
        avg_accel_imu_frame = avg_accel_imu_frame / num_to_average;

        // Transform the averaged acceleration into the base frame.
        geometry_msgs::msg::Vector3 avg_accel_base_frame_msg;
        tf2::doTransform(toVectorMsg(avg_accel_imu_frame), avg_accel_base_frame_msg, imu_to_base_tf_);
        gtsam::Vector3 avg_accel_base_frame = toGtsam(avg_accel_base_frame_msg);

        // Get roll, pitch, and yaw from the average acceleration and GPS odometry.
        double roll = std::atan2(avg_accel_base_frame.y(), avg_accel_base_frame.z());
        double pitch = std::atan2(-avg_accel_base_frame.x(), std::sqrt(pow(avg_accel_base_frame.y(), 2) + pow(avg_accel_base_frame.z(), 2)));
        gtsam::Pose3 gps_pose = toGtsam(initial_gps->pose.pose);
        double yaw = gps_pose.rotation().yaw();

        // Construct the full initial orientation and pose.
        gtsam::Rot3 initial_orientation = gtsam::Rot3::Ypr(yaw, pitch, roll);
        prev_pose_ = gtsam::Pose3(initial_orientation, gps_pose.translation());

        // --- Set Initial State ---
        prev_time_ = rclcpp::Time(initial_gps->header.stamp).seconds();
        prev_pose_ = toGtsam(initial_gps->pose.pose);
        prev_vel_ = gtsam::Vector3(0, 0, 0);         // Assume starting from rest
        prev_bias_ = gtsam::imuBias::ConstantBias(); // Assume zero initial bias
        imu_preintegrator_ = std::make_unique<gtsam::PreintegratedCombinedMeasurements>(imu_params, prev_bias_);

        // --- Add Prior Factors to Graph ---
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

        // Add timestamps for initial variables.
        gtsam::IncrementalFixedLagSmoother::KeyTimestampMap initial_timestamps;
        initial_timestamps[X(0)] = prev_time_;
        initial_timestamps[V(0)] = prev_time_;
        initial_timestamps[B(0)] = prev_time_;
        key_timestamps_[X(0)] = initial_gps->header.stamp;

        // --- Update the Smoother with the Initial State ---
        smoother_->update(initial_graph, initial_values, initial_timestamps);
        system_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "System initialized and is now live.");
    }

    /**
     * @brief Timer callback for publishing odometry and transforms.
     */
    void odomPublishTimerCallback()
    {
        // IMPORTANT! Fix for integration with the 'navsat_transform_node':
        // Before initialization, publish a zero-position odometry message with IMU heading.
        if (!system_initialized_ && !imu_queue_.empty())
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
        else if (system_initialized_)
        {
            publishFusedState();
        }
    }

    /**
     * @brief Publishes the fused odometry message and the map -> odom transform.
     */
    void publishFusedState()
    {
        // --- Publish map -> odom transform ---
        if (publish_global_tf_)
        {
            geometry_msgs::msg::TransformStamped odom_to_base_tf_msg;
            try
            {
                // Look up the odom -> base_link transform from another source (e.g., local EKF node)
                odom_to_base_tf_msg = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_WARN(this->get_logger(), "Could not lookup transform from '%s' to '%s': %s",
                            odom_frame_.c_str(), base_frame_.c_str(), ex.what());
                return;
            }

            // Convert the odom -> base_link transform to a GTSAM type.
            tf2::Transform odom_to_base_tf2;
            tf2::fromMsg(odom_to_base_tf_msg.transform, odom_to_base_tf2);
            gtsam::Pose3 odom_to_base_gtsam = toGtsam(odom_to_base_tf2);

            // Calculate map -> odom transform: T_map_odom = T_map_base * (T_odom_base)^-1
            // Mangelson's EN EN 433 class really seeing some direct application here haha.
            gtsam::Pose3 map_to_odom_gtsam = prev_pose_ * odom_to_base_gtsam.inverse();

            // Broadcast the calculated transform.
            geometry_msgs::msg::TransformStamped map_to_odom_tf_msg;
            map_to_odom_tf_msg.header.stamp = this->get_clock()->now();
            map_to_odom_tf_msg.header.frame_id = map_frame_;
            map_to_odom_tf_msg.child_frame_id = odom_frame_;
            map_to_odom_tf_msg.transform.translation = toVectorMsg(map_to_odom_gtsam.translation());
            map_to_odom_tf_msg.transform.rotation = toQuatMsg(map_to_odom_gtsam.rotation());
            tf_broadcaster_->sendTransform(map_to_odom_tf_msg);
        }

        // --- Publish the global odometry message ---
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = map_frame_;
        odom_msg.child_frame_id = base_frame_;
        odom_msg.pose.pose = toPoseMsg(prev_pose_);
        odom_msg.twist.twist.linear = toVectorMsg(prev_vel_);
        global_odom_pub_->publish(odom_msg);

        // --- Publish the smoothed path for visualization ---
        if (publish_smoothed_path_)
            publishSmoothedPath();
    }

    /**
     * @brief Publishes the full smoothed trajectory as a nav_msgs::Path.
     */
    void publishSmoothedPath()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = map_frame_;

        gtsam::Values results = smoother_->calculateEstimate();

        for (const auto& key_value : results)
        {
            gtsam::Key key = key_value.key;
            // Check for the lowercase 'x' symbol used by the GTSAM shorthand
            if (gtsam::Symbol(key).chr() == 'x')
            {
                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = map_frame_;
                if (key_timestamps_.count(key))
                {
                    pose_stamped.header.stamp = key_timestamps_.at(key);
                }
                else
                {
                    pose_stamped.header.stamp = this->get_clock()->now();
                }
                pose_stamped.pose = toPoseMsg(key_value.value.cast<gtsam::Pose3>());
                path_msg.poses.push_back(pose_stamped);
            }
        }
        global_path_pub_->publish(path_msg);
    }

    // --- ROS 2 Interfaces ---
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr global_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr heading_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr factor_graph_timer_;
    rclcpp::TimerBase::SharedPtr odom_pub_timer_;

    // --- Message Queues ---
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::deque<nav_msgs::msg::Odometry::SharedPtr> gps_queue_;
    std::deque<sensor_msgs::msg::Imu::SharedPtr> heading_queue_;

    // --- System State ---
    bool system_initialized_ = false;
    double prev_time_ = 0.0;
    long unsigned int prev_step_ = 0;
    long unsigned int current_step_ = 1;
    std::map<gtsam::Key, rclcpp::Time> key_timestamps_;

    // --- GTSAM Objects ---
    std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
    std::unique_ptr<gtsam::PreintegratedCombinedMeasurements> imu_preintegrator_;
    gtsam::Pose3 prev_pose_;
    gtsam::Vector3 prev_vel_;
    gtsam::imuBias::ConstantBias prev_bias_;

    // --- Transformations ---
    bool have_imu_to_base_tf_ = false;
    bool have_heading_to_base_tf_ = false;
    geometry_msgs::msg::TransformStamped imu_to_base_tf_;
    geometry_msgs::msg::TransformStamped heading_to_base_tf_;

    // --- Parameters ---
    std::string imu_topic_, gps_odom_topic_, heading_topic_;
    std::string global_odom_topic_, smoothed_path_topic_;
    std::string map_frame_, odom_frame_, base_frame_;
    double factor_graph_update_rate_, odom_publish_rate_;
    bool publish_global_tf_, publish_smoothed_path_;
    double smoother_lag_;
    double accel_noise_sigma_, gyro_noise_sigma_;
    double accel_bias_rw_sigma_, gyro_bias_rw_sigma_;
    double gps_noise_sigma_, heading_noise_sigma_;
    double prior_pose_rot_sigma_, prior_pose_pos_sigma_;
    double prior_vel_sigma_, prior_bias_sigma_;

    // --- Conversion Utilities (Static Methods) ---
    static gtsam::Point3 toGtsam(const geometry_msgs::msg::Point &msg);
    static gtsam::Vector3 toGtsam(const geometry_msgs::msg::Vector3 &msg);
    static gtsam::Rot3 toGtsam(const geometry_msgs::msg::Quaternion &msg);
    static gtsam::Pose3 toGtsam(const geometry_msgs::msg::Pose &msg);
    static gtsam::Pose3 toGtsam(const tf2::Transform &tf);
    static geometry_msgs::msg::Point toPointMsg(const gtsam::Point3 &gtsam_obj);
    static geometry_msgs::msg::Vector3 toVectorMsg(const gtsam::Vector3 &gtsam_obj);
    static geometry_msgs::msg::Quaternion toQuatMsg(const gtsam::Rot3 &gtsam_obj);
    static geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3 &gtsam_obj);
};

// --- Definitions for Static Member Functions ---
gtsam::Point3 GtsamLocalizerNode::toGtsam(const geometry_msgs::msg::Point &msg) { return {msg.x, msg.y, msg.z}; }
gtsam::Vector3 GtsamLocalizerNode::toGtsam(const geometry_msgs::msg::Vector3 &msg) { return {msg.x, msg.y, msg.z}; }
gtsam::Rot3 GtsamLocalizerNode::toGtsam(const geometry_msgs::msg::Quaternion &msg) { return gtsam::Rot3::Quaternion(msg.w, msg.x, msg.y, msg.z); }
gtsam::Pose3 GtsamLocalizerNode::toGtsam(const geometry_msgs::msg::Pose &msg) { return {toGtsam(msg.orientation), toGtsam(msg.position)}; }
gtsam::Pose3 GtsamLocalizerNode::toGtsam(const tf2::Transform &tf)
{
    const auto &rot = tf.getRotation();
    const auto &trans = tf.getOrigin();
    return {gtsam::Rot3::Quaternion(rot.w(), rot.x(), rot.y(), rot.z()), gtsam::Point3(trans.x(), trans.y(), trans.z())};
}
geometry_msgs::msg::Point GtsamLocalizerNode::toPointMsg(const gtsam::Point3 &gtsam_obj)
{
    geometry_msgs::msg::Point msg;
    msg.x = gtsam_obj.x();
    msg.y = gtsam_obj.y();
    msg.z = gtsam_obj.z();
    return msg;
}
geometry_msgs::msg::Vector3 GtsamLocalizerNode::toVectorMsg(const gtsam::Vector3 &gtsam_obj)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = gtsam_obj.x();
    msg.y = gtsam_obj.y();
    msg.z = gtsam_obj.z();
    return msg;
}
geometry_msgs::msg::Quaternion GtsamLocalizerNode::toQuatMsg(const gtsam::Rot3 &gtsam_obj)
{
    gtsam::Quaternion q = gtsam_obj.toQuaternion();
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}
geometry_msgs::msg::Pose GtsamLocalizerNode::toPoseMsg(const gtsam::Pose3 &gtsam_obj)
{
    geometry_msgs::msg::Pose msg;
    msg.position = toPointMsg(gtsam_obj.translation());
    msg.orientation = toQuatMsg(gtsam_obj.rotation());
    return msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    // A single-threaded executor simplifies data access management.
    // We'd need to use a mutex or lock if we ever use a MultiThreadedExecutor.
    rclcpp::spin(std::make_shared<GtsamLocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
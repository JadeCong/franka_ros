#include <franka_controllers/franka_force_teaching_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_controllers {

bool FrankaForceTeachingController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
    ROS_WARN("FrankaForceTeachingController: Make sure the hand perception node is running for getting the hand force!");
    
    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR("FrankaForceTeachingController: Could not read parameter arm_id");
        return false;
    }
    
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "FrankaForceTeachingController: Invalid or no joint_names parameters provided, aborting "
            "controller init!");
        return false;
    }
    
    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM("FrankaForceTeachingController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FrankaForceTeachingController: Exception getting model handle from interface: " << ex.what());
        return false;
    }
    
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM("FrankaForceTeachingController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "FrankaForceTeachingController: Exception getting state handle from interface: " << ex.what());
        return false;
    }
    
    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM("FrankaForceTeachingController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM("FrankaForceTeachingController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }
    
    std::string hand_force_topic;
    node_handle.getParam("/manual_force_teaching/controller_spawner/hand_force_topic", hand_force_topic);
    sub_hand_force_ = node_handle.subscribe(hand_force_topic, 1, &FrankaForceTeachingController::handForceCallback, this, ros::TransportHints().reliable().tcpNoDelay());
    
    dynamic_reconfigure_hand_force_scale_param_node_ = ros::NodeHandle("dynamic_reconfigure_hand_force_scale_param_node");
    dynamic_server_hand_force_scale_param_ = std::make_unique<dynamic_reconfigure::Server<franka_controllers::hand_force_scale_paramConfig>>(dynamic_reconfigure_hand_force_scale_param_node_);
    dynamic_server_hand_force_scale_param_->setCallback(boost::bind(&FrankaForceTeachingController::handForceScaleParamCallback, this, _1, _2));
    
    return true;
}

void FrankaForceTeachingController::starting(const ros::Time& /*time*/) {
    // Get the current state(joint torque/gravity) of the robot
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> gravity_array = model_handle_->getGravity();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());  // without gravity and friction
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    // Bias correction for the current external torque
    tau_ext_initial_ = tau_measured - gravity;
    tau_error_.setZero();
}

void FrankaForceTeachingController::update(const ros::Time& /*time*/, const ros::Duration& period) {
    // Get the current state of the robot
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    std::array<double, 7> gravity_array = model_handle_->getGravity();
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());  // without gravity and friction
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());  // NOLINT (readability-identifier-naming)
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
    
    // Calculate the tau_d, tau_ext and tau_error_
    Eigen::Matrix<double, 7, 1> tau_d, tau_ext, tau_cmd;
    Eigen::Matrix<double, 6, 1> desired_force_torque;
    desired_force_torque.setZero();
    // TODO: set the desired_force_torque
    desired_force_torque << hand_force_, hand_torque_;  // set the cartesian force/torque in franka base coordinate system
    desired_force_torque(1) = -desired_force_torque(1);
    desired_force_torque(2) = -desired_force_torque(2) + 0.185 * 2 * 9.81;  // compensation for two finger gravity(each finger mass is 0.1179/0.1386/0.1565kg), maybe the bigger value is fine
    tau_d = jacobian.transpose() * desired_force_torque;
    tau_ext = tau_measured - gravity - tau_ext_initial_;
    tau_error_ = tau_error_ + period.toSec() * (tau_d - tau_ext);
    // FF + PI control (PI gains are initially all 0)
    tau_cmd = tau_d + k_p_ * (tau_d - tau_ext) + k_i_ * tau_error_;
    tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
    // Send the tau_cmd to the robot for control
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_cmd(i));
    }
    
    // Update signals changed online through dynamic reconfigure
    hand_force_scale_ = filter_gain_ * target_hand_force_scale_ + (1 - filter_gain_) * hand_force_scale_;
    hand_torque_scale_ = filter_gain_ * target_hand_torque_scale_ + (1 - filter_gain_) * hand_torque_scale_;
    k_p_ = filter_gain_ * target_k_p_ + (1 - filter_gain_) * k_p_;
    k_i_ = filter_gain_ * target_k_i_ + (1 - filter_gain_) * k_i_;
}

void FrankaForceTeachingController::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void FrankaForceTeachingController::handForceCallback(const geometry_msgs::WrenchStampedConstPtr& msg) {
    hand_force_ << hand_force_scale_(0) * msg->wrench.force.x, hand_force_scale_(1) * msg->wrench.force.y, hand_force_scale_(2) * msg->wrench.force.z;
    hand_torque_ << hand_torque_scale_(0) * msg->wrench.torque.x, hand_torque_scale_(1) * msg->wrench.torque.y, hand_torque_scale_(2) * msg->wrench.torque.z;
}

void FrankaForceTeachingController::handForceScaleParamCallback(franka_controllers::hand_force_scale_paramConfig& config, uint32_t /*level*/) {
    target_hand_force_scale_ << config.hand_force_scale_x, config.hand_force_scale_y, config.hand_force_scale_z;
    target_hand_torque_scale_ << config.hand_torque_scale_x, config.hand_torque_scale_y, config.hand_torque_scale_z;
    target_k_p_ = config.k_p;
    target_k_i_ = config.k_i;
}

Eigen::Matrix<double, 7, 1> FrankaForceTeachingController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
    }
    
    return tau_d_saturated;
}

}  // namespace franka_controllers

PLUGINLIB_EXPORT_CLASS(franka_controllers::FrankaForceTeachingController, controller_interface::ControllerBase)

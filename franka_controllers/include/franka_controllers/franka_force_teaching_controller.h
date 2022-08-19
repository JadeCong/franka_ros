#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Core>

#include <franka_controllers/hand_force_scale_paramConfig.h>
#include <geometry_msgs/WrenchStamped.h>

namespace franka_controllers {

class FrankaForceTeachingController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface,
                                        franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void stopping(const ros::Time&) override;
    
    private:
        // Saturation
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)
        
        // Franka handles
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        
        // Robot parameters
        double k_p_{0.0};
        double k_i_{0.0};
        double target_k_p_{0.0};
        double target_k_i_{0.0};
        double filter_gain_{0.001};
        Eigen::Matrix<double, 7, 1> tau_ext_initial_;
        Eigen::Matrix<double, 7, 1> tau_error_;
        static constexpr double kDeltaTauMax{1.0};
        
        // Hand force/torque
        Eigen::Vector3d hand_force_{0.0, 0.0, 0.0};
        Eigen::Vector3d hand_torque_{0.0, 0.0, 0.0};
        
        // Hand force/torque scale
        Eigen::Vector3d hand_force_scale_{1.0, 1.0, 1.0};
        Eigen::Vector3d hand_torque_scale_{1.0, 1.0, 1.0};
        Eigen::Vector3d target_hand_force_scale_{1.0, 1.0, 1.0};
        Eigen::Vector3d target_hand_torque_scale_{1.0, 1.0, 1.0};
        
        // Hand force/torque subscriber
        ros::Subscriber sub_hand_force_;
        void handForceCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
        
        // Dynamic reconfigure for hand force/torque scale
        ros::NodeHandle dynamic_reconfigure_hand_force_scale_param_node_;
        std::unique_ptr<dynamic_reconfigure::Server<franka_controllers::hand_force_scale_paramConfig>> dynamic_server_hand_force_scale_param_;
        void handForceScaleParamCallback(franka_controllers::hand_force_scale_paramConfig& config, uint32_t level);
};

}  // namespace franka_controllers

#ifndef M2006_ROS2_HPP_
#define M2006_ROS2_HPP_

#include "dji_can_communication.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/visibility_control.h>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace dji_ros_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class M2006Ros2 : public hardware_interface::SystemInterface,
                  public rclcpp_lifecycle::LifecycleNode,
                  public std::enable_shared_from_this<class M2006Ros2>
{
public:
  M2006Ros2();
  std::shared_ptr<DjiCanCommunication> getDjiCanCommunication() const
  {
    return dji_can_;
  }
  std::shared_ptr<DjiCanCommunication> dji_can_;

  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start();
  hardware_interface::return_type stop();

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  rclcpp::TimerBase::SharedPtr timer_;

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<std::string> joint_names_;
  std::string joint_name_;

  enum Params
  {
    POSITION_LEFT,
    VELOCITY_LEFT,
    EFFORT_LEFT,
    POSITION_RIGHT,
    VELOCITY_RIGHT,
    EFFORT_RIGHT
  };

  //モータの左右
  enum Side
  {
    LEFT,
    RIGHT
  };
};

}  // namespace dji_ros_controller

#endif  // M2006_ROS2_HPP

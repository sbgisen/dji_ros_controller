#include "M2006Ros2.hpp"

namespace dji_ros_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn M2006Ros2::on_init(const hardware_interface::HardwareInfo& info)
{
  RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "M2006Ros2->on_init");
  dji_can_->connect("can0");

  // デフォルトの構成を確認
  if (SystemInterface::on_init(info) !=
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  // 各ジョイントのサイズに合わせてコマンド、位置、速度、努力の配列をリサイズ
  joint_names_.clear();
  hw_commands_.resize(info.joints.size(), 0.0);
  hw_positions_.resize(info.joints.size(), 0.0);
  hw_velocities_.resize(info.joints.size(), 0.0);
  hw_efforts_.resize(info.joints.size(), 0.0);
  joint_name_ = info_.hardware_parameters["joint_name"];

  for (const auto& joint : info.joints)
  {
    joint_names_.push_back(joint.name);
  }

  RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "Configured with %zu joints", joint_names_.size());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn M2006Ros2::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "on_configure");
  // timer_ = this->create_wall_timer(
  //           std::chrono::milliseconds(10),
  //           std::bind(&DjiCanCommunication::timerCallback, dji_can_.get())
  // );
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn M2006Ros2::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "on_activate");
  rclcpp::Clock steady_clock(RCL_STEADY_TIME);
  last_keep_alive_time_ = steady_clock.now() - rclcpp::Duration::from_seconds(1000.0);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn M2006Ros2::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn M2006Ros2::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn M2006Ros2::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

CallbackReturn M2006Ros2::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> M2006Ros2::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "export_state_interfaces with %zu joints", hw_positions_.size());
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    // state_interfaces.emplace_back(
    //     hardware_interface::StateInterface(info_.joints[i].name,
    //     hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> M2006Ros2::export_command_interfaces()
{
  RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "export_command_interfaces with %zu joints", hw_commands_.size());
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type M2006Ros2::start()
{
  // デバイスのスタート処理
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type M2006Ros2::stop()
{
  // デバイスのストップ処理
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type M2006Ros2::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // モーターステータスの更新
  // RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "M2006Ros2->read");

  std::vector<double> status;

  dji_can_->updateMotorStatus(status);

  // RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "M2006Ros2->status, %d, %d,
  // %d, %d, %d, %d",
  // status[0],status[1],status[2],status[3],status[4],status[5]);

  if (status.size() != hw_positions_.size() * 3)
  {
    RCLCPP_ERROR(rclcpp::get_logger("M2006Ros2"), "Unexpected status size: %zu", status.size());
    return hardware_interface::return_type::ERROR;
  }

  // ステータスを各ジョイントの状態に反映
  // joint[0](right_wheel_joint)は左モーターエンコーダーを使用しており
  // 前進時に符号が逆になるため反転する
  for (size_t i = 0; i < hw_positions_.size(); ++i)
  {
    const double sign = (i == 0) ? -1.0 : 1.0;
    hw_positions_[i] = sign * status[i * 3 + 0];
    hw_velocities_[i] = sign * status[i * 3 + 1];
    hw_efforts_[i] = status[i * 3 + 2];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type M2006Ros2::write(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  rclcpp::Time now = time;
  rclcpp::Duration interval = now - last_keep_alive_time_;

  // コマンドがすべてゼロなら
  bool all_zero = std::all_of(hw_commands_.begin(), hw_commands_.end(), [](double v) {
    return std::abs(v) < 1e-5;
  });

  const double keepalive_interval_sec = 30.0;
  const double keepalive_velocity = 0.1;

  if (all_zero && interval.seconds() >= keepalive_interval_sec)
  {
    RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "Sending keep-alive pulse: %.2f rad/s", keepalive_velocity);
    dji_can_->sendVelocityCan(keepalive_velocity, -keepalive_velocity);

    last_keep_alive_time_ = now;
  }
  else
  {
    dji_can_->sendVelocityCan(hw_commands_[0], -1.0 * hw_commands_[1]);
  }

  // RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "write");

  // std::stringstream ss;
  // ss << "hw_commands_: [";
  // for (size_t i = 0; i < hw_commands_.size(); ++i) {
  //     ss << hw_commands_[i];
  //     if (i != hw_commands_.size() - 1) {
  //         ss << ", ";
  //     }
  // }
  // ss << "]";
  // RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "hw_commands: %s",
  // ss.str().c_str());

  // コマンドを送信
  // RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "CM1: %f, CM2: %f",
  // hw_commands_[0], hw_commands_[1]);
  // dji_can_->sendVelocityCan(hw_commands_[0], -1.0 * hw_commands_[1]);

  return hardware_interface::return_type::OK;
}

M2006Ros2::M2006Ros2() : rclcpp_lifecycle::LifecycleNode("M2006Ros2"), dji_can_(std::make_shared<DjiCanCommunication>())
{
  RCLCPP_INFO(rclcpp::get_logger("M2006Ros2"), "M2006Ros2->constructor");
  dji_can_ = std::make_shared<DjiCanCommunication>();
  // Constructor initialization
  timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                   std::bind(&DjiCanCommunication::timerCallback, dji_can_.get()));
}

}  // namespace dji_ros_controller

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dji_ros_controller::M2006Ros2>();

  // Use a lifecycle manager to manage the lifecycle nodes
  auto lifecycle_manager = std::make_shared<rclcpp_lifecycle::LifecycleNode>("lifecycle_manager");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(lifecycle_manager->get_node_base_interface());
  executor.add_node(node->get_node_base_interface());

  // Trigger the transition to configure state
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  // Start the executor
  executor.spin();

  rclcpp::shutdown();
  return 0;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dji_ros_controller::M2006Ros2, hardware_interface::SystemInterface)

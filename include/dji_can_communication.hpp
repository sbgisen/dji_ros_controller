#ifndef DJI_CAN_COMMUNICATION_HPP_
#define DJI_CAN_COMMUNICATION_HPP_

#include <can_msgs/msg/frame.hpp>
#include <chrono>
#include <controller_interface/controller_interface.hpp>
#include <cstdint>
#include <cstring>
#include <functional>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <memory>
// #include "controller_manager/controller_manager.hpp"
#include <control_toolbox/pid.hpp>

class DjiCanCommunication : public rclcpp::Node
{
public:
  DjiCanCommunication();
  ~DjiCanCommunication() override;
  int sendVelocityCan(double left_target_velocity, double right_target_velocity);
  void updateMotorStatus(std::vector<double>& status_arg);
  void timerCallback();

  int sendSocketCan(uint16_t dst_id, uint8_t* data);

  void connect(const std::string& can_id);
  void receive();

  double right_target_velocity_;
  double left_target_velocity_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::TimerBase::SharedPtr timer_;
  control_toolbox::Pid left_pid_;
  control_toolbox::Pid right_pid_;

  enum PositionStatus
  {
    RAD_NOW_LEFT,
    RAD_BEFORE_LEFT,
    RAD_NOW_RIGHT,
    RAD_BEFORE_RIGHT
  };
  enum Params
  {
    POSITION_LEFT,
    VELOCITY_LEFT,
    EFFORT_LEFT,
    POSITION_RIGHT,
    VELOCITY_RIGHT,
    EFFORT_RIGHT
  };
  enum Side
  {
    LEFT,
    RIGHT
  };

private:
  // private
  void startTimer();
  void stopTimer();
  std::atomic<bool> timer_running_;
  std::thread timer_thread_;

  std::unique_ptr<drivers::socketcan::SocketCanReceiver> can_receiver_;
  std::unique_ptr<drivers::socketcan::SocketCanSender> can_sender_;
  std::unique_ptr<std::thread> receiver_thread_;
  void initialize();

  void receivedCanCallback(const std::shared_ptr<const can_msgs::msg::Frame>& msg);
  int current2Data(double current_in);
  int createCanPacketAndSend(int left_data, int right_data);
  int sendCan(uint16_t dst_id, uint8_t* data);

  double right_target_current_;
  double left_target_current_;
  double TORQUE_COEFFICIENT_;
  double REDUCTION_RATIO_;
  double MAX_CURRENT_;
  std::vector<double> status_;
  std::vector<double> rad_vec_;
  uint8_t can_data_[8];
  rclcpp::Time last_time_;

  rclcpp::Time getTime();
  rclcpp::Duration getPeriod();

  int motor_id_;
  int motor_rpm_;
  double motor_current_;
  int motor_temperature_;
  double motor_torque_;
  double motor_rad_per_sec_;

  int motor_position_;
  int motor_position_right_;
  int motor_position_left_;

  double motor_position_rad_;
  double motor_rad_now_;
  double motor_rad_before_right_;
  double motor_rad_before_left_;
  double motor_rad_before_;

  double AccessCount_[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
};

#endif  // DJI_CAN_COMMUNICATION_HPP_

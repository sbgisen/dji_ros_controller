#include "dji_can_communication.hpp"

////////////////////////////////////////////
//                                        //
//       dji_can_communication.cpp        //
//              2020/11/18                //
//      airi.yokochi@g.softbank.co.jp     //
//                                        //
////////////////////////////////////////////

/*
Motor: DJI M2006
Motor Controller: C610
*/

//////////////////////////////////////
/*  Send Velocity to CAN */
//////////////////////////////////////
int DjiCanCommunication::sendVelocityCan(const double left_target_velocity, const double right_target_velocity)
{
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "left_velocity %f,
  // right_velocity %f", left_target_velocity,
  //             right_target_velocity);
  // PID制御に使用する速度指令値を更新する
  right_target_velocity_ = right_target_velocity;
  left_target_velocity_ = left_target_velocity;
  return 0;
}

// [謎] 電流[A]を引数とし、データにする
// -10[A]~10[A]の電流を-10000~10000で表現するため、1[A]で1000を送る
// double型の引数電流値current_inからint型のdata_outを返す。
// [TODO]CASTしたら終わり？
int DjiCanCommunication::current2Data(double current_in)
{
  int data_out = 0;
  //目標電流値が０以上の場合（正回転の場合？）
  if (current_in >= 0)
  {
    data_out = (int)(current_in * 1000);
    // 負回転の場合、符号なし16ビット整数に変換
  }
  else if (current_in < 0)
  {
    data_out = 0xFFFF + (int)(current_in * 1000);
  }
  return data_out;
}

// 左の右のモータのデータからパケットを生成する
int DjiCanCommunication::createCanPacketAndSend(int left_data, int right_data)
{
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "
  // createCanPacketAndSend");
  can_data_[0] = left_data >> 8 & 0xFF;
  can_data_[1] = left_data & 0xFF;
  can_data_[2] = right_data >> 8 & 0xFF;
  can_data_[3] = right_data & 0xFF;
  can_data_[4] = 0;
  can_data_[5] = 0;
  can_data_[6] = 0;
  can_data_[7] = 0;
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
  //             "Sending CAN data: [%02X, %02X, %02X, %02X, %02X, %02X, %02X,
  //             %02X]", can_data_[0], can_data_[1], can_data_[2], can_data_[3],
  //             can_data_[4], can_data_[5], can_data_[6], can_data_[7]);

  std::vector<uint8_t> can_socket_data;
  can_socket_data.push_back(static_cast<unsigned char>(left_data >> 8 & 0xFF));
  can_socket_data.push_back(static_cast<unsigned char>(left_data & 0xFF));
  can_socket_data.push_back(static_cast<unsigned char>(right_data >> 8 & 0xFF));
  can_socket_data.push_back(static_cast<unsigned char>(right_data & 0xFF));
  can_socket_data.push_back(0);
  can_socket_data.push_back(0);
  can_socket_data.push_back(0);
  can_socket_data.push_back(0);

  // try
  // {
  //   can_sender_->send(
  //       &can_socket_data, 8,
  //       drivers::socketcan::CanId(0x200, 0,
  //       drivers::socketcan::FrameType::DATA,
  //       drivers::socketcan::StandardFrame), std::chrono::milliseconds(10));
  // }
  // catch (const std::exception& ex)
  // {
  //   return false;
  // }

  if (sendCan(0x200, can_data_))
  {
    RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "write error");
  }
  return 0;
}

int DjiCanCommunication::sendSocketCan(uint16_t dst_id, uint8_t* data)
{
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), " sendSocketCan");
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
  //             "Sending to ID: 0x%03X, data: [%02X, %02X, %02X, %02X, %02X,
  //             %02X, %02X, %02X]", dst_id, data[0], data[1], data[2], data[3],
  //             data[4], data[5], data[6], data[7]);

  try
  {
    can_msgs::msg::Frame msg;
    msg.id = dst_id;
    msg.dlc = 8;
    // std::copy(data, data+8, msg.data.begin());
    RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "1");
    for (int i = 0; i < 8; i++)
    {
      msg.data[i] = data[i];
    }
    RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "2");

    can_sender_->send(
        msg.data.data(), msg.dlc,
        drivers::socketcan::CanId(0x200, 0, drivers::socketcan::FrameType::DATA, drivers::socketcan::StandardFrame),
        std::chrono::milliseconds(10));
    RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "3");
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Error sending CAN message: %s", ex.what());
    return false;
  }
  return true;
}

// CANデータをデバイスに送信する。引数はコントローラIDと電流値
int DjiCanCommunication::sendCan(uint16_t dst_id, uint8_t* data)
{
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), " sendCan");
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
  //             "Sending ID: 0x%03X, data: [%02X, %02X, %02X, %02X, %02X, %02X,
  //             %02X, %02X]", dst_id, data[0], data[1], data[2], data[3],
  //             data[4], data[5], data[6], data[7]);
  int s; /* can raw socket */
  int required_mtu = CAN_MTU;
  // int enable_canfd = 1;
  struct sockaddr_can addr;
  struct canfd_frame frame;
  struct ifreq ifr;

  frame.can_id = dst_id;
  for (int i = 0; i < 8; i++)
  {
    frame.data[i] = data[i];
  }
  frame.len = 8;

  /* open socket */
  if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    perror("socket");
    return 1;
  }

  // ToDo: Make the interface name (currently constant value can0) configurable
  strncpy(ifr.ifr_name, "can0", IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  ifr.ifr_ifindex = if_nametoindex(ifr.ifr_name);
  if (!ifr.ifr_ifindex)
  {
    perror("if_nametoindex");
    return 1;
  }

  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  /* disable default receive filter on this RAW socket */
  /* This is obsolete as we do not read from the socket at all, but for */
  /* this reason we can remove the receive list in the Kernel to save a */
  /* little (really a very little!) CPU usage.                          */
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);

  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
    return 1;
  }

  /* send frame */
  if (write(s, &frame, required_mtu) != required_mtu)
  {
    perror("write");
    RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "write error");
    return 1;
  }

  close(s);

  return 0;
}

void DjiCanCommunication::connect(const std::string& can_id)
{
  try
  {
    can_sender_ = std::make_unique<drivers::socketcan::SocketCanSender>(can_id, false, drivers::socketcan::CanId());
    can_receiver_ = std::make_unique<drivers::socketcan::SocketCanReceiver>(can_id);
  }
  catch (const std::exception& ex)
  {
    std::stringstream ss;
    ss << "Failed to connect to " << can_id.c_str();
    throw std::runtime_error(ss.str());
  }
  receiver_thread_ = std::make_unique<std::thread>(&DjiCanCommunication::receive, this);
}

// https://github.com/MiRoboticsLab/cyberdog_ros2/tree/011025c2ee43a144f32f42440b631019124a731b/cyberdog_common/cyberdog_utils/include/cyberdog_utils/canhttps://github.com/MiRoboticsLab/cyberdog_ros2/tree/011025c2ee43a144f32f42440b631019124a731b/cyberdog_common/cyberdog_utils/include/cyberdog_utils/can
// https://github.com/PITT-MIT-IAC/raptor-dbw-ros2/blob/c50bc8bf27242e5df0f719d838585343ef8cf7f9/raptor_dbw_can/src/DbwNode.cpp

void DjiCanCommunication::receive()
{
  std::array<unsigned char, 8> frame_data;
  drivers::socketcan::CanId receive_id{};

  while (true)
  {
    // auto receive_id = receiver_->receive(frame_data,
    // std::chrono::milliseconds(10));
    auto receive_id = can_receiver_->receive(frame_data, std::chrono::milliseconds(10));
    uint32_t can_id = receive_id.identifier();

    // std::size_t index = static_cast<std::size_t>(frame_data[0] - 1);

    std::ostringstream oss;
    for (const auto& byte : frame_data)
    {
      oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte) << " ";
    }
    // 512+id
    // id: 514
    // receive: 0f a9 00 00 00 0b 00 00
    // id: 513
    // receive: 0e 1b 00 00 ff ff 00 00

    // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "id: %d,
    // receive: %s", can_id, oss.str().c_str());

    // 左側 ////////////////////////////////////////////////////////////////
    if (can_id == (0x200 + 0x001))
    {
      //減速前の値
      double left_rad = ((frame_data[0] << 8) | frame_data[1]) * 2.0 * M_PI / 8191.0;  // 0から2*M_PIの範囲
      // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
      // "/received_left %f", left_rad);

      //減速後の値を出すため、これまでに何回転したかカウントする
      static double left_rad_prev = 0.0;     //前回の位置
      static int left_revolution_count = 0;  //回転数(0〜35。減速比が36のため)

      //正回転
      if (left_rad < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad_prev)
      {
        left_revolution_count++;
        if (left_revolution_count == 36)
        {
          left_revolution_count = 0;
        }
        // ROS_INFO("left_revolution_count++ = %d :
        // %f,%f",left_revolution_count, left_rad, left_rad_prev);
        //負回転
      }
      else if (left_rad_prev < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad)
      {
        left_revolution_count--;
        if (left_revolution_count == -1)
        {
          left_revolution_count = 35;
        }
        // ROS_INFO("left_revolution_count-- = %d :
        // %f,%f",left_revolution_count, left_rad, left_rad_prev);
      }
      left_rad_prev = left_rad;

      // 減速後の値(タイヤが何回転したか)を出す
      // 0から(36*2*M_PI)の範囲の値からREDUCTION_RARIOを割り、[0から2*M_PI]までの範囲にする
      double left_rad_reduced = (left_rad + (2 * M_PI * left_revolution_count)) / REDUCTION_RATIO_;
      // Cuboidくん用DiffDriveControllerに合わせるため値の範囲を(-M_PIからM_PI)に変更する
      left_rad_reduced -= M_PI;

      // タイヤのRPMを保存する
      int16_t left_rpm = (frame_data[2] << 8) | frame_data[3];
      double left_rad_per_sec = (double)left_rpm * 2.0 * M_PI / 60.0 / REDUCTION_RATIO_;
      // 電流値とトルクを保存する
      double left_current = ((frame_data[4] << 8) | frame_data[5]) / 1000.0;
      double left_torque = left_current * TORQUE_COEFFICIENT_;
      // 現在の温度を保存する
      // double left_temperature = frame_data[6];

      status_[POSITION_LEFT] = left_rad_reduced;
      status_[VELOCITY_LEFT] = left_rad_per_sec;
      status_[EFFORT_LEFT] = left_torque;
      //右側
      ////////////////////////////////////////////////////////////////////////////
    }
    else if (can_id == (0x200 + 0x002))
    {
      //減速前の値
      double right_rad = ((frame_data[0] << 8) | frame_data[1]) * 2.0 * M_PI / 8191.0;  // 0から2*M_PIの範囲
      // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
      // "/received_right %f", right_rad);
      //減速後の値を出すため、これまでに何回転したかカウントする
      static double right_rad_prev = 0.0;
      static int right_revolution_count = 0;
      //正回転
      if (right_rad < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad_prev)
      {
        right_revolution_count++;
        if (right_revolution_count == 36)
        {
          right_revolution_count = 0;
        }
        // ROS_INFO("right_revolution_count++ = %d :
        // %f,%f",right_revolution_count, right_rad, right_rad_prev);
        //負回転
      }
      else if (right_rad_prev < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad)
      {
        right_revolution_count--;
        if (right_revolution_count == -1)
        {
          right_revolution_count = 35;
        }
        // ROS_INFO("right_revolution_count-- = %d :
        // %f,%f",right_revolution_count, right_rad, right_rad_prev);
      }
      right_rad_prev = right_rad;
      // 減速後の値(タイヤが何回転したか)を出す
      // 0から(36*2*M_PI)の範囲の値からREDUCTION_RARIOを割り、[0から2*M_PI]までの範囲にする
      double right_rad_reduced = (right_rad + (2 * M_PI * right_revolution_count)) / REDUCTION_RATIO_;
      // 値の範囲を(-M_PIからM_PI)に変更する
      right_rad_reduced -= M_PI;

      // タイヤのRPMを保存する
      int16_t right_rpm = (frame_data[2] << 8) | frame_data[3];
      double right_rad_per_sec = (double)right_rpm * 2.0 * M_PI / 60.0 / REDUCTION_RATIO_;
      //電流値とトルクを保存する
      double right_current = ((frame_data[4] << 8) | frame_data[5]) / 1000.0;
      double right_torque = right_current * TORQUE_COEFFICIENT_;
      //現在の温度を保存する
      // double right_temperature = frame_data[6];

      status_[POSITION_RIGHT] = right_rad_reduced;
      status_[VELOCITY_RIGHT] = right_rad_per_sec;
      status_[EFFORT_RIGHT] = right_torque;
    }
    else if (can_id == (0x200))
    {
      // int current_1 = (frame_data[0] << 8) | frame_data[1];
      // int current_2 = (frame_data[2] << 8) | frame_data[3];
      // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
      // "/received_message id: %d, ,data1 %d, data2 %d", can_id,
      //             current_1, current_2);
    }
    else if (can_id == (0x1FF))
    {
      RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "write error");
    }
    else if (can_id == (0x004))
    {
      RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "Controller could not connect to motors.");
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "Invalid /received_message id: %d", can_id);
    }
  }

  // 201   8 1d 98 0 0 0 27 0 0
  // 202   8 e e 0 0 ff d8 0 0
  // ID: 200+idの数
  //  ４つの値で電流値を表す
  //  値の範囲は-16384〜16384
  //  電流の範囲は-20[A]〜20[A]

  // 0,1:目標のモータの角度(上位８ビット＋下位８ビット）
  // 2,3:目標の回転速度、１分間で回転する回数（上位８ビット＋下位８ビット）
  // 4,5:実際のトルク電流
  // 6:モータの温度[℃]
  // 7:なし

  // 角度の範囲は0~8191(0度〜３６０度)
}

//////////////////////////////////////
/*  Received Joint State from CAN */
//////////////////////////////////////
void DjiCanCommunication::updateMotorStatus(std::vector<double>& status_arg)
{
  status_arg.resize(6);
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
  // "updateMotorStatus, %d, %d, %d, %d, %d, %d",
  // status_arg[0],status_arg[1],status_arg[2],status_arg[3],status_arg[4],status_arg[5]);

  //引数のポインタの値を書き換える
  status_arg[VELOCITY_LEFT] = status_[VELOCITY_LEFT];
  status_arg[POSITION_LEFT] = status_[POSITION_LEFT];
  status_arg[EFFORT_LEFT] = status_[EFFORT_LEFT];
  status_arg[POSITION_RIGHT] = status_[POSITION_RIGHT];
  status_arg[VELOCITY_RIGHT] = status_[VELOCITY_RIGHT];
  status_arg[EFFORT_RIGHT] = status_[EFFORT_RIGHT];
}

// モータからデータを受け取ったら値を格納するコールバック関数
void DjiCanCommunication::receivedCanCallback(const std::shared_ptr<const can_msgs::msg::Frame>& msg)
{
  RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "dji_can_communication->receivedCanCallback");
  if (msg->dlc == 8)
  {
    // 左側 ////////////////////////////////////////////////////////////////
    if (msg->id == (0x200 + 0x001))
    {
      //減速前の値
      double left_rad = ((msg->data[0] << 8) | msg->data[1]) * 2.0 * M_PI / 8191.0;  // 0から2*M_PIの範囲

      //減速後の値を出すため、これまでに何回転したかカウントする
      static double left_rad_prev = 0.0;     //前回の位置
      static int left_revolution_count = 0;  //回転数(0〜35。減速比が36のため)

      //正回転
      if (left_rad < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad_prev)
      {
        left_revolution_count++;
        if (left_revolution_count == 36)
        {
          left_revolution_count = 0;
        }
        // ROS_INFO("left_revolution_count++ = %d :
        // %f,%f",left_revolution_count, left_rad, left_rad_prev);
        //負回転
      }
      else if (left_rad_prev < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < left_rad)
      {
        left_revolution_count--;
        if (left_revolution_count == -1)
        {
          left_revolution_count = 35;
        }
        // ROS_INFO("left_revolution_count-- = %d :
        // %f,%f",left_revolution_count, left_rad, left_rad_prev);
      }
      left_rad_prev = left_rad;

      // 減速後の値(タイヤが何回転したか)を出す
      // 0から(36*2*M_PI)の範囲の値からREDUCTION_RARIOを割り、[0から2*M_PI]までの範囲にする
      double left_rad_reduced = (left_rad + (2 * M_PI * left_revolution_count)) / REDUCTION_RATIO_;
      // Cuboidくん用DiffDriveControllerに合わせるため値の範囲を(-M_PIからM_PI)に変更する
      left_rad_reduced -= M_PI;

      // タイヤのRPMを保存する
      int16_t left_rpm = (msg->data[2] << 8) | msg->data[3];
      double left_rad_per_sec = (double)left_rpm * 2.0 * M_PI / 60.0 / REDUCTION_RATIO_;
      // 電流値とトルクを保存する
      double left_current = ((msg->data[4] << 8) | msg->data[5]) / 1000.0;
      double left_torque = left_current * TORQUE_COEFFICIENT_;
      // 現在の温度を保存する
      // double left_temperature = msg->data[6];

      status_[POSITION_LEFT] = left_rad_reduced;
      status_[VELOCITY_LEFT] = left_rad_per_sec;
      status_[EFFORT_LEFT] = left_torque;
      //右側
      ////////////////////////////////////////////////////////////////////////////
    }
    else if (msg->id == (0x200 + 0x002))
    {
      //減速前の値
      double right_rad = ((msg->data[0] << 8) | msg->data[1]) * 2.0 * M_PI / 8191.0;  // 0から2*M_PIの範囲
      //減速後の値を出すため、これまでに何回転したかカウントする
      static double right_rad_prev = 0.0;
      static int right_revolution_count = 0;
      //正回転
      if (right_rad < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad_prev)
      {
        right_revolution_count++;
        if (right_revolution_count == 36)
        {
          right_revolution_count = 0;
        }
        // ROS_INFO("right_revolution_count++ = %d :
        // %f,%f",right_revolution_count, right_rad, right_rad_prev);
        //負回転
      }
      else if (right_rad_prev < 1.0 / 4.0 * M_PI && 3.0 / 4.0 * M_PI < right_rad)
      {
        right_revolution_count--;
        if (right_revolution_count == -1)
        {
          right_revolution_count = 35;
        }
        // ROS_INFO("right_revolution_count-- = %d :
        // %f,%f",right_revolution_count, right_rad, right_rad_prev);
      }
      right_rad_prev = right_rad;
      // 減速後の値(タイヤが何回転したか)を出す
      // 0から(36*2*M_PI)の範囲の値からREDUCTION_RARIOを割り、[0から2*M_PI]までの範囲にする
      double right_rad_reduced = (right_rad + (2 * M_PI * right_revolution_count)) / REDUCTION_RATIO_;
      // 値の範囲を(-M_PIからM_PI)に変更する
      right_rad_reduced -= M_PI;

      // タイヤのRPMを保存する
      int16_t right_rpm = (msg->data[2] << 8) | msg->data[3];
      double right_rad_per_sec = (double)right_rpm * 2.0 * M_PI / 60.0 / REDUCTION_RATIO_;
      //電流値とトルクを保存する
      double right_current = ((msg->data[4] << 8) | msg->data[5]) / 1000.0;
      double right_torque = right_current * TORQUE_COEFFICIENT_;
      //現在の温度を保存する
      // double right_temperature = msg->data[6];

      status_[POSITION_RIGHT] = right_rad_reduced;
      status_[VELOCITY_RIGHT] = right_rad_per_sec;
      status_[EFFORT_RIGHT] = right_torque;
    }
    else if (msg->id == (0x200))
    {
      // int current_1 = (msg->data[0] << 8) | msg->data[1];
      // int current_2 = (msg->data[2] << 8) | msg->data[3];
      // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
      // "/received_message id: %d, ,data1 %d, data2 %d", msg->id, current_1,
      // current_2);
    }
    else if (msg->id == (0x1FF))
    {
      RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "write error");
    }
    else if (msg->id == (0x004))
    {
      RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "Controller could not connect to motors.");
    }
    else
    {
      RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "Invalid /received_message id: %d", msg->id);
    }
  }
  else
  {
    RCLCPP_WARN(rclcpp::get_logger("dji_can_communication"), "Invalid /received_message dlc %d", msg->dlc);
  }
}

// PID制御
void DjiCanCommunication::timerCallback()
{
  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
  // "DjiCanCommunication::timerCallback()");
  rclcpp::Time time_now = this->get_clock()->now();

  // 現在の速度と目標速度の更新
  double right_target_velocity = right_target_velocity_;
  double right_velocity = status_[VELOCITY_RIGHT];
  double left_target_velocity = left_target_velocity_;
  double left_velocity = status_[VELOCITY_LEFT];

  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "R_tar_vel: %f,
  // R_vel: %f, L_tar_vel: %f, L_vel: %f", right_target_velocity,
  // right_velocity, left_target_velocity, left_velocity);

  // PID制御
  rclcpp::Duration time_diff = time_now - last_time_;

  uint64_t dt = time_diff.nanoseconds();

  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "duration dt: %ld
  // nanoseconds", dt); RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"),
  // "duration diff: %s", std::to_string(time_diff.nanoseconds()).c_str());

  right_target_current_ = 1.0 * right_pid_.computeCommand(right_target_velocity - right_velocity, dt);
  left_target_current_ = 1.0 * left_pid_.computeCommand(left_target_velocity - left_velocity, dt);
  // ROS_INFO("timerCallback: right: %f -> %f, left: %f ->
  // %f",right_target_velocity , right_velocity, left_target_velocity  ,
  // left_velocity);

  // モータに送る電流値[A]
  double right_target_current = right_target_current_;
  double left_target_current = left_target_current_;
  // 最大電流値を超えないようにする
  if (right_target_current >= MAX_CURRENT_)
  {
    right_target_current = MAX_CURRENT_;
  }
  else if (-MAX_CURRENT_ >= right_target_current)
  {
    right_target_current = -MAX_CURRENT_;
  }
  if (left_target_current >= MAX_CURRENT_)
  {
    left_target_current = MAX_CURRENT_;
  }
  else if (-MAX_CURRENT_ >= left_target_current)
  {
    left_target_current = -MAX_CURRENT_;
  }

  // 電流指令値をデータに変換する
  double right_data = current2Data(right_target_current);
  double left_data = current2Data(left_target_current);

  // RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "R_tar_cur: %f,
  // L_tar_cur: %f, R_data: %f, L_data: %f",
  //             right_target_current, left_target_current, right_data,
  //             left_data);

  // パケットを生成してCANに送信する
  createCanPacketAndSend(left_data, right_data);
  last_time_ = time_now;
}

void DjiCanCommunication::initialize()
{
  RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "dji_can_communication->initialize");
  sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", 1000, std::bind(&DjiCanCommunication::receivedCanCallback, this, std::placeholders::_1));
  // left_pid_.initPid(3, 10.0, 0.005, 10.0, -10.0);   // i=0
  // right_pid_.initPid(3, 10.0, 0.005, 10.0, -10.0);  // i=0
  left_pid_.initialize(0.3, 1.0, 0.001, 0.5, -0.5);
  right_pid_.initialize(0.3, 1.0, 0.001, 0.5, -0.5);

  // 100HzでPID制御のコールバック関数を呼ぶタイマー
}

void DjiCanCommunication::startTimer()
{
  timer_running_ = true;
  timer_thread_ = std::thread([this]() {
    while (timer_running_)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      timerCallback();
    }
  });
}

void DjiCanCommunication::stopTimer()
{
  timer_running_ = false;
  if (timer_thread_.joinable())
  {
    timer_thread_.join();
  }
}

/////////////////////////////
/*  コンストラクタ         */
/////////////////////////////
DjiCanCommunication::DjiCanCommunication() : Node("dji_can_communication"), timer_running_(false)
{
  RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "Constructor");
  TORQUE_COEFFICIENT_ = 0.18;  //トルク計数
  REDUCTION_RATIO_ = 36;       //減速比
  MAX_CURRENT_ = 10.000;       // C610の最大電流
  // MAX_CURRENT_ = 1.0;            //今だけ
  status_.resize(6, 0.0);
  rad_vec_.resize(4, 0.0);
  right_target_velocity_ = 0;  // 目標速度
  left_target_velocity_ = 0;
  right_target_current_ = 0;  // 目標速度に必要な電流
  left_target_current_ = 0;

  last_time_ = this->get_clock()->now();

  initialize();
  startTimer();
  //   timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
  //   std::bind(&DjiCanCommunication::timerCallback, this));

  RCLCPP_INFO(rclcpp::get_logger("dji_can_communication"), "DjiCanCommunication::DjiCanCommunication() -> SUCCEED");
}

DjiCanCommunication::~DjiCanCommunication()
{
  left_target_current_ = 0.0;
  right_target_current_ = 0.0;
  double right_data = current2Data(right_target_current_);
  double left_data = current2Data(left_target_current_);
  // パケットを生成してCANに送信する
  createCanPacketAndSend(left_data, right_data);
  stopTimer();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<DjiCanCommunication>();
//   rclcpp::spin(node);
//   return 0;
// }

// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the ssc_interface class.

#ifndef SSC_INTERFACE__SSC_INTERFACE_HPP_
#define SSC_INTERFACE__SSC_INTERFACE_HPP_

#include <ssc_interface/visibility_control.hpp>

#include <common/types.hpp>
#include <vehicle_interface/platform_interface.hpp>

#include <automotive_platform_msgs/msg/gear_command.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>
#include <automotive_platform_msgs/msg/steering_feedback.hpp>
#include <automotive_platform_msgs/msg/steer_mode.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>
#include <automotive_platform_msgs/msg/velocity_accel_cov.hpp>
#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>
#include <std_msgs/msg/bool.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::TAU;

using automotive_platform_msgs::msg::GearCommand;     //档位命令
using automotive_platform_msgs::msg::GearFeedback;    //档位反馈
using automotive_platform_msgs::msg::SpeedMode;       //速度模式
using automotive_platform_msgs::msg::SteeringFeedback;//方向盘反馈
using automotive_platform_msgs::msg::SteerMode;       //转向模式
using automotive_platform_msgs::msg::TurnSignalCommand;//转向信号灯
using automotive_platform_msgs::msg::VelocityAccelCov; //速度加速度协方差
using autoware_auto_msgs::msg::HighLevelControlCommand;//高速控制命令
using autoware_auto_msgs::msg::RawControlCommand;      //原始控制命令
using autoware_auto_msgs::msg::TrajectoryPoint;        //轨迹点
using autoware_auto_msgs::msg::VehicleControlCommand;  //车辆控制命令
using autoware_auto_msgs::msg::VehicleKinematicState;  //车辆运动状态
using autoware_auto_msgs::msg::VehicleStateCommand;    //车辆状态命令
using autoware_auto_msgs::srv::AutonomyModeChange;     //自动模式更改
using ModeChangeRequest = autoware_auto_msgs::srv::AutonomyModeChange_Request;

namespace ssc_interface
{

static constexpr float32_t STEERING_TO_TIRE_RATIO = 0.533F / 8.6F; //方向盘轮胎比

enum class DbwState
{
  DISABLED = 0,
  ENABLE_REQUESTED = 1,
  ENABLE_SENT = 2,
  ENABLED = 3
};

/// \brief Class for maintaining the DBW state 用于维护DBW状态的类
class SSC_INTERFACE_PUBLIC DbwStateMachine
{
public:
  /// \brief Default constructor
  /// \param[in] dbw_disabled_debounce If state = ENABLE_SENT and DBW reports DISABLED, debounce this many msgs  // NOLINT
  // 如果state = ENABLE_SENT 且 DBW DISABLED 清除消息
  explicit DbwStateMachine(uint16_t dbw_disabled_debounce);

  /// \brief Returns true if state is ENABLED, ENABLE_SENT, or ENABLE_REQUESTED with conditions
  //如果state是ENABLED，ENABLED，ENABLE_SENT 或者 ENABLED_REQUESTED返回true
  bool8_t enabled() const;

  /// \brief Returns current internal state
  /// \return A DbwState object representing the current state
  //返回当前内部状态 一个DbwState对象代表当前状态
  DbwState get_state() const;

  /// \brief Notifies the state machine that feedback was received from the DBW system
  /// \param[in] enabled If true, DBW system reports enabled. If false, DBW system reports disabled
  //通知state machine已经收到来自DBW系统的反馈
  void dbw_feedback(bool8_t enabled);

  /// \brief Notifies the state machine that a control command was sent to the DBW system
  //通知state machine一个控制命令已经被发送到DBW系统
  void control_cmd_sent();

  /// \brief Notifies the state machine that a state command was sent to the DBW system
  //通知state machine一个状态命令已经被发送到DBW系统
  void state_cmd_sent();

  /// \brief The user has requested the DBW system to enable (true) or disable (false)
  /// \param[in] enable If true, request enable. If false, request disable
  // 用户已请求DBWX系统启用或者禁用  true是启用 false是禁用
  void user_request(bool8_t enable);

private:
  bool8_t m_first_control_cmd_sent; //首次发送的控制命令
  bool8_t m_first_state_cmd_sent;   //首次发送的状态命令
  uint16_t m_disabled_feedback_count; //禁用的反馈计数
  const uint16_t DISABLED_FEEDBACK_THRESH;  //禁用反馈阈值
  DbwState m_state;                 //DbwState 对象

  void disable_and_reset();         //禁用和重置
};

/// \brief Class for interfacing with AS SSC
//用于与SSC接口的类
class SSC_INTERFACE_PUBLIC SscInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  /// \brief Default constructor. 默认构造函数
  /// \param[in] node Reference to node 引用node
  /// \param[in] front_axle_to_cog Distance from front axle to center-of-gravity in meters 前轴到重心的距离
  /// \param[in] rear_axle_to_cog Distance from rear axle to center-of-gravity in meters   后轴到重心的距离
  /// \param[in] max_accel_mps2 Maximum acceleration in m/s^2 最大加速度
  /// \param[in] max_decel_mps2 Maximum deceleration in m/s^2 最小减速度
  /// \param[in] max_yaw_rate_rad Maximum rate of change of heading in radians/sec  最大转角速率
  explicit SscInterface(
    rclcpp::Node & node,
    float32_t front_axle_to_cog,
    float32_t rear_axle_to_cog,
    float32_t max_accel_mps2,
    float32_t max_decel_mps2,
    float32_t max_yaw_rate_rad
  );
  /// \brief Default destructor 默认系够函数
  ~SscInterface() noexcept override = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  //尝试从车辆平台接收数据，并更新 StateReport 和 Odometry。
  //错误时可能会引发异常
  //检查/接收数据的最长时间 timeout
  //如果在超时之前收到数据，则为 true，否则为 false
  bool8_t update(std::chrono::nanoseconds timeout) override;

  /// \brief Send the state command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  //将状态命令发送到车辆平台。
  //错误时可能会引发异常
  //要发送到车辆的状态命令 msg
  //如果以某种方式发送失败，则为 false，否则为 true
  bool8_t send_state_command(const VehicleStateCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  //将控制命令发送到车辆平台。
  //错误时可能会引发异常
  //要发送到车辆的控制命令 msg
  //如果以某种方式发送失败，则为 false，否则为 true
  bool8_t send_control_command(const HighLevelControlCommand & msg);

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  //将控制命令发送到车辆平台。
  //错误时可能会引发异常
  //要发送到车辆的控制命令 msg（与上面不同）
  //如果以某种方式发送失败，则为 false，否则为 true
  bool8_t send_control_command(const RawControlCommand & msg) override;

  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  //将控制命令发送到车辆平台。
  //错误时可能会引发异常
  //要发送到车辆的控制命令 msg（与上面不同）
  //如果以某种方式发送失败，则为 false，否则为 true
  bool8_t send_control_command(const VehicleControlCommand & msg) override;

  /// \brief Handle a request from the user to enable or disable the DBW system.
  ///   Exceptions may be thrown on errors
  /// \param[in] request The requested autonomy mode
  /// \return false only if enabling the DBW system actually failed, true otherwise
  //处理来自用户的请求以启用或禁用 DBW 系统。
  //错误时可能会引发异常
  //请求的自动模式 request
  //仅当启用 DBW 系统实际失败时才为 false，否则为 true
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) override;

  //运动学自行车模型
  static void kinematic_bicycle_model(
    float32_t dt, float32_t l_r, float32_t l_f, VehicleKinematicState * vks);

private:

  // Publishers (to SSC) 发布者 输出 到SSC
  rclcpp::Publisher<GearCommand>::SharedPtr m_gear_cmd_pub;
  rclcpp::Publisher<SpeedMode>::SharedPtr m_speed_cmd_pub;
  rclcpp::Publisher<SteerMode>::SharedPtr m_steer_cmd_pub;
  rclcpp::Publisher<TurnSignalCommand>::SharedPtr m_turn_signal_cmd_pub;

  // Publishers (to Autoware) 发布者 输出 到Autoware
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr m_kinematic_state_pub;

  // Subscribers (from SSC) 订阅者 输入 从SSC
  rclcpp::SubscriptionBase::SharedPtr m_dbw_state_sub, m_gear_feedback_sub, m_vel_accel_sub,
    m_steer_sub;



  //定义一些成员变量
  rclcpp::Logger m_logger;
  float32_t m_front_axle_to_cog; //前轴到重心距离
  float32_t m_rear_axle_to_cog;  //后轴到重心距离
  float32_t m_accel_limit;       //最大加速度
  float32_t m_decel_limit;       //最大见速度
  float32_t m_max_yaw_rate;      //最大转向速率
  std::unique_ptr<DbwStateMachine> m_dbw_state_machine; //dbw_state_machine 对象 来自上面的一个类



  // The vehicle kinematic state is stored because it needs information from
  // both on_steer_report() and on_vel_accel_report().
  //车辆运动学状态被存储，因为它需要来自on_steer_report()和on_vel_accel_report()的信息
  VehicleKinematicState m_vehicle_kinematic_state;
  bool m_seen_steer{false};
  bool m_seen_vel_accel{false};
  // In case both arrive at the same time
  //如果两者同时到达
  std::mutex m_vehicle_kinematic_state_mutex;

  void on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg);
  void on_gear_report(const GearFeedback::SharedPtr & msg);
  void on_steer_report(const SteeringFeedback::SharedPtr & msg);
  void on_vel_accel_report(const VelocityAccelCov::SharedPtr & msg);
};

}  // namespace ssc_interface

#endif  // SSC_INTERFACE__SSC_INTERFACE_HPP_

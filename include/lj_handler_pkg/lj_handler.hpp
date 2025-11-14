#ifndef LJ_HANDLER_PKG__LJ_HANDLER_HPP_
#define LJ_HANDLER_PKG__LJ_HANDLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <LabJackM.h>
#include <string>
#include <vector>

class LJHandlerNode : public rclcpp::Node
{
public:
  LJHandlerNode();
  ~LJHandlerNode();

private:
  void steering_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void throttle_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void brake_callback(const std_msgs::msg::Float32::SharedPtr msg);
  void check_safety_timeout();
  
  void set_steering_angle(double steering_angle_deg);
  void set_throttle_brake(double throttle_value);
  void set_control_axis(double desired_voltage, 
                       double nom_vs_master, 
                       double nom_vs_slave,
                       const std::vector<std::string>& dac_names,
                       double min_perc,
                       double max_perc,
                       const std::string& axis_name,const bool opposition);
  
  int read_nominal_voltages(double& nom_vs_steer_master, 
                           double& nom_vs_steer_slave,
                           double& nom_vs_accbrake_master,
                           double& nom_vs_accbrake_slave);

  // LabJack handle and parameters
  bool wait_remove_brake;
  int handle_;
  std::string nominal_vs_steer_master_pin_;
  std::string nominal_vs_steer_slave_pin_;
  std::string nominal_vs_accbrake_master_pin_;
  std::string nominal_vs_accbrake_slave_pin_;
  
  std::vector<std::string> steering_dac_names_;
  std::vector<std::string> throttle_dac_names_;
  
  // Voltage limits for steering
  double steering_min_perc_;
  double steering_max_perc_;
  
  // Voltage limits for throttle/brake
  double throttle_min_perc_;
  double throttle_max_perc_;
  
  // Input voltage range
  double input_voltage_min_;
  double input_voltage_max_;
  double center_voltage_;
  
  // Steering parameters
  double max_steering_angle_;
  double steering_clip_;
  double old_throttle;

  // Steering subscription
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_sub_;
  rclcpp::Time last_steering_time_;
  double steering_timeout_sec_;
  double brake_perc;

  // Throttle/Brake subscription
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr throttle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr brake_sub_;
  rclcpp::Time last_throttle_time_;
  rclcpp::Time last_brake_time_;
  double brake_to_throttle_delay_;
  rclcpp::Time brake_release_time_;
  double throttle_timeout_sec_;
  
  // Safety timer
  rclcpp::TimerBase::SharedPtr safety_timer_;
  
  static constexpr int INITIAL_ERR_ADDRESS = -1;
};

#endif // LJ_HANDLER_PKG__LJ_HANDLER_HPP_
#ifndef LJ_HANDLER_PKG__RC_INPUT_NODE_HPP_
#define LJ_HANDLER_PKG__RC_INPUT_NODE_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <LabJackM.h>

/// Four-state throttle state machine driven by Button B on the RC transmitter.
/// Steering is always passed through regardless of state.
///
/// State transitions (forward-only, no going back):
///   WAIT_FOR_ACCEL  --[B press]-->  NORMAL_RUN  --[B press]-->  SLOW_RUN  --[B press]-->  STOP
///
/// STOP is terminal; restart the node to reset.
enum class RCState
{
  WAIT_FOR_ACCEL,  ///< Waiting for first Button B press; throttle = 0
  NORMAL_RUN,      ///< Fixed throttle at normal_run_throttle_perc
  SLOW_RUN,        ///< Ramp throttle down from normal to slow_run_throttle_perc
  STOP             ///< Brake only; terminal state
};

/// ROS 2 node that reads RC PWM channels from a LabJack T7 via DIO Extended
/// Features (DIO_EF mode 5) and publishes steering / pedal commands.
///
/// Pin mapping (default, configurable via parameters):
///   DIO0 (FIO0) — steering wheel (Ch1)
///   DIO1 (FIO1) — Button B      (Ch3, momentary)
///
/// PWM reading is based on the user-supplied Python reference snippet:
///   DIO#_EF_INDEX       = 5   (pulse-width capture)
///   DIO#_EF_CLOCK_SOURCE = 0  (internal 80 MHz clock)
///   DIO#_EF_CONFIG_A    = 1   (capture high-side pulse width)
///   DIO#_EF_READ_A          → raw tick count; offset from neutral = percentage
class RCInputNode : public rclcpp::Node
{
public:
  RCInputNode();
  ~RCInputNode();

private:
  // ---- LJM setup ----------------------------------------------------------
  void setup_pwm_inputs();
  void calibrate_initial_position();

  // ---- Main loop ----------------------------------------------------------
  void read_loop();

  // ---- State machine ------------------------------------------------------
  void run_state_machine(double steer_percent, bool btn_press_event);

  // ---- Emergency brake callback -------------------------------------------
  void emergency_callback(const std_msgs::msg::Bool::SharedPtr msg);

  // ---- Utilities ----------------------------------------------------------
  double convert_to_percent(double ticks, double initial_ticks, double max_ticks) const;
  std::string state_to_string(RCState state) const;
  void publish_state() const;

  // ---- LJM ----------------------------------------------------------------
  int handle_;

  // ---- Pin names (LJM register prefixes, e.g. "DIO0") --------------------
  std::string steering_pin_;
  std::string button_pin_;

  // ---- Calibration: neutral tick values read once at startup --------------
  double initial_steering_ticks_;
  double initial_button_ticks_;

  // ---- PWM conversion parameters ------------------------------------------
  double steering_max_ticks_;        ///< ±this tick range → ±100 %
  double button_max_ticks_;          ///< ±this tick range → ±100 %
  double button_threshold_percent_;  ///< |btn_pct| > this → button pressed
  double button_debounce_ms_;        ///< minimum ms between accepted press events
  double pwm_timeout_ms_;            ///< signal-loss timeout before safe fallback

  // ---- State-machine parameters -------------------------------------------
  double normal_run_throttle_perc_;  ///< throttle in NORMAL_RUN (%)
  double slow_run_throttle_perc_;    ///< target throttle in SLOW_RUN (%)
  double slow_run_ramp_time_s_;      ///< ramp duration NORMAL→SLOW (s)
  double stop_brake_perc_;           ///< brake force in STOP (%)
  double stop_brake_ramp_time_s_;    ///< 0 = instant; >0 = gradual ramp

  // ---- State-machine runtime state ----------------------------------------
  RCState state_;
  rclcpp::Time slow_run_start_time_;
  rclcpp::Time stop_start_time_;
  double throttle_at_slow_entry_;    ///< throttle value when SLOW_RUN was entered

  // ---- Button debounce ----------------------------------------------------
  bool button_was_pressed_;
  rclcpp::Time last_button_press_time_;

  // ---- Signal loss --------------------------------------------------------
  rclcpp::Time last_valid_read_time_;
  bool signal_was_lost_;
  int consecutive_read_errors_;

  // ---- Emergency brake ----------------------------------------------------
  bool emergency_active_;

  // ---- Publishers ---------------------------------------------------------
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr button_raw_pub_;

  // ---- Subscriber ---------------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;

  // ---- Timer --------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr read_timer_;

  // ---- Configurable topic names -------------------------------------------
  std::string steering_topic_;
  std::string throttle_topic_;

  // ---- Constants ----------------------------------------------------------
  static constexpr int INITIAL_ERR_ADDRESS = -1;
};

#endif  // LJ_HANDLER_PKG__RC_INPUT_NODE_HPP_

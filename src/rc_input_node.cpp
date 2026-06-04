#include "lj_handler_pkg/rc_input_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <LabJackM.h>

// ============================================================================
// Constructor
// ============================================================================

RCInputNode::RCInputNode()
: Node("rc_input_node"),
  handle_(-1),
  initial_steering_ticks_(0.0),
  initial_button_ticks_(0.0),
  steering_max_ticks_(24000.0),
  button_max_ticks_(30000.0),
  button_threshold_percent_(50.0),
  button_debounce_ms_(200.0),
  pwm_timeout_ms_(500.0),
  normal_run_throttle_perc_(66.0),
  slow_run_throttle_perc_(30.0),
  slow_run_ramp_time_s_(1.0),
  stop_brake_perc_(60.0),
  stop_brake_ramp_time_s_(0.0),
  state_(RCState::WAIT_FOR_ACCEL),
  throttle_at_slow_entry_(0.0),
  button_was_pressed_(false),
  signal_was_lost_(false),
  consecutive_read_errors_(0),
  emergency_active_(false)
{
  // ---- Declare parameters -------------------------------------------------
  this->declare_parameter<std::string>("steering_pin", "FIO0");
  this->declare_parameter<std::string>("button_pin", "FIO1");
  this->declare_parameter<double>("pwm_read_rate_hz", 50.0);
  this->declare_parameter<double>("steering_max_ticks", 24000.0);
  this->declare_parameter<double>("button_max_ticks", 30000.0);
  this->declare_parameter<double>("button_threshold_percent", 50.0);
  this->declare_parameter<double>("button_debounce_ms", 200.0);
  this->declare_parameter<double>("pwm_timeout_ms", 500.0);
  this->declare_parameter<double>("normal_run_throttle_perc", 66.0);
  this->declare_parameter<double>("slow_run_throttle_perc", 30.0);
  this->declare_parameter<double>("slow_run_ramp_time_s", 1.0);
  this->declare_parameter<double>("stop_brake_perc", 60.0);
  this->declare_parameter<double>("stop_brake_ramp_time_s", 0.0);
  this->declare_parameter<std::string>("steering_topic", "/follower/steering_cmd");
  this->declare_parameter<std::string>("throttle_topic", "/follower/pedal_cmd");

  // ---- Read parameters ----------------------------------------------------
  steering_pin_              = this->get_parameter("steering_pin").as_string();
  button_pin_                = this->get_parameter("button_pin").as_string();
  const double pwm_rate      = this->get_parameter("pwm_read_rate_hz").as_double();
  steering_max_ticks_        = this->get_parameter("steering_max_ticks").as_double();
  button_max_ticks_          = this->get_parameter("button_max_ticks").as_double();
  button_threshold_percent_  = this->get_parameter("button_threshold_percent").as_double();
  button_debounce_ms_        = this->get_parameter("button_debounce_ms").as_double();
  pwm_timeout_ms_            = this->get_parameter("pwm_timeout_ms").as_double();
  normal_run_throttle_perc_  = this->get_parameter("normal_run_throttle_perc").as_double();
  slow_run_throttle_perc_    = this->get_parameter("slow_run_throttle_perc").as_double();
  slow_run_ramp_time_s_      = this->get_parameter("slow_run_ramp_time_s").as_double();
  stop_brake_perc_           = this->get_parameter("stop_brake_perc").as_double();
  stop_brake_ramp_time_s_    = this->get_parameter("stop_brake_ramp_time_s").as_double();
  steering_topic_            = this->get_parameter("steering_topic").as_string();
  throttle_topic_            = this->get_parameter("throttle_topic").as_string();

  // ---- Publishers ---------------------------------------------------------
  steering_pub_     = this->create_publisher<std_msgs::msg::Float32>(steering_topic_, 10);
  throttle_pub_     = this->create_publisher<std_msgs::msg::Float32>(throttle_topic_, 10);
  state_pub_        = this->create_publisher<std_msgs::msg::String>("/rc_input/state", 10);
  steering_raw_pub_ = this->create_publisher<std_msgs::msg::Float32>("/rc_input/steering_raw", 10);
  button_raw_pub_   = this->create_publisher<std_msgs::msg::Float32>("/rc_input/button_raw", 10);

  // ---- Subscriber ---------------------------------------------------------
  emergency_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/emergency_brake", 10,
    std::bind(&RCInputNode::emergency_callback, this, std::placeholders::_1));

  // ---- Open LabJack T7 ----------------------------------------------------
  // LJM v1.20+ spawns a gRPC daemon automatically, so multiple processes can
  // safely open the same T7 over USB without conflict.
  int err = LJM_Open(LJM_dtT7, LJM_ctUSB, "470039659", &handle_);
  if (err != LJME_NOERROR) {
    char err_name[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, err_name);
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to open LabJack T7: %s. Node will shut down.", err_name);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "LabJack T7 opened (handle=%d).", handle_);

  // ---- Configure DIO_EF PWM inputs ----------------------------------------
  setup_pwm_inputs();

  // ---- Wait for PWM signal to stabilise, then read neutral position --------
  RCLCPP_INFO(
    this->get_logger(),
    "Waiting 1 s for PWM inputs to stabilise — keep controller at neutral.");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  calibrate_initial_position();

  // ---- Initialise timestamps ----------------------------------------------
  const rclcpp::Time now = this->now();
  last_valid_read_time_   = now;
  // Set last_button_press_time_ far in the past so first press is never suppressed
  last_button_press_time_ = now - rclcpp::Duration::from_seconds(10.0);
  slow_run_start_time_    = now;
  stop_start_time_        = now;

  // ---- Create main polling timer ------------------------------------------
  const double period_s = 1.0 / std::max(pwm_rate, 1.0);
  read_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_s),
    std::bind(&RCInputNode::read_loop, this));

  // ---- Publish initial state ----------------------------------------------
  publish_state();

  RCLCPP_INFO(
    this->get_logger(),
    "RC input node ready. steering=%s  button=%s  rate=%.0f Hz",
    steering_pin_.c_str(), button_pin_.c_str(), pwm_rate);
  RCLCPP_INFO(
    this->get_logger(),
    "Throttle levels — normal: %.0f %%  slow: %.0f %%  stop-brake: %.0f %%",
    normal_run_throttle_perc_, slow_run_throttle_perc_, stop_brake_perc_);
  RCLCPP_INFO(
    this->get_logger(),
    "State: WAIT_FOR_ACCEL — press Button B to start moving.");
}

// ============================================================================
// Destructor
// ============================================================================

RCInputNode::~RCInputNode()
{
  if (handle_ >= 0) {
    // Disable DIO_EF before closing
    LJM_eWriteName(handle_, (steering_pin_ + "_EF_ENABLE").c_str(), 0);
    LJM_eWriteName(handle_, (button_pin_   + "_EF_ENABLE").c_str(), 0);
    LJM_Close(handle_);
    RCLCPP_INFO(this->get_logger(), "LabJack T7 closed.");
  }
}

// ============================================================================
// setup_pwm_inputs
// Configures both DIO_EF channels for RC PWM pulse-width capture.
// Sequence follows the user-supplied Python reference snippet exactly.
// ============================================================================

void RCInputNode::setup_pwm_inputs()
{
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring DIO_EF PWM inputs: steering=%s  button=%s",
    steering_pin_.c_str(), button_pin_.c_str());

  // Error-checked write helper (warnings only — misconfigured DIO_EF is
  // non-fatal; the read loop will detect errors and apply safe fallback).
  auto write_ef = [this](const std::string & reg, double val) {
      int err = LJM_eWriteName(handle_, reg.c_str(), val);
      if (err != LJME_NOERROR) {
        char err_name[LJM_MAX_NAME_SIZE];
        LJM_ErrorToString(err, err_name);
        RCLCPP_WARN(
          this->get_logger(),
          "DIO_EF setup — write %s=%.0f failed: %s", reg.c_str(), val, err_name);
      }
    };

  for (const std::string & pin : {steering_pin_, button_pin_}) {
    write_ef(pin + "_EF_ENABLE",       0);  // disable before reconfiguring
    write_ef(pin + "_EF_INDEX",        5);  // mode 5: PWM pulse-width capture
    write_ef(pin + "_EF_CLOCK_SOURCE", 0);  // 0 = internal 80 MHz clock
    write_ef(pin + "_EF_CONFIG_A",     1);  // capture high-side pulse width
    write_ef(pin + "_EF_ENABLE",       1);  // enable
  }

  RCLCPP_INFO(this->get_logger(), "DIO_EF PWM inputs configured.");
}

// ============================================================================
// calibrate_initial_position
// Reads both channels once and stores the values as the neutral reference.
// Called after the 1 s stabilisation delay; controller must be at rest.
// ============================================================================

void RCInputNode::calibrate_initial_position()
{
  const std::string steer_reg = steering_pin_ + "_EF_READ_A";
  const std::string btn_reg   = button_pin_   + "_EF_READ_A";
  const char * names[2]       = {steer_reg.c_str(), btn_reg.c_str()};
  double values[2]            = {0.0, 0.0};
  int err_addr                = INITIAL_ERR_ADDRESS;

  const int err = LJM_eReadNames(handle_, 2, names, values, &err_addr);
  if (err != LJME_NOERROR) {
    char err_name[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, err_name);
    RCLCPP_WARN(
      this->get_logger(),
      "Calibration read failed: %s — using 0 as neutral for both channels.", err_name);
    initial_steering_ticks_ = 0.0;
    initial_button_ticks_   = 0.0;
  } else {
    initial_steering_ticks_ = values[0];
    initial_button_ticks_   = values[1];
    RCLCPP_INFO(
      this->get_logger(),
      "Calibration done. Neutral — steering: %.0f ticks  button: %.0f ticks",
      initial_steering_ticks_, initial_button_ticks_);
  }
}

// ============================================================================
// convert_to_percent
// Maps raw tick count to [-100, +100] % relative to the calibrated neutral.
// ============================================================================

double RCInputNode::convert_to_percent(
  double ticks, double initial_ticks, double max_ticks) const
{
  double offset = ticks - initial_ticks;
  offset = std::max(-max_ticks, std::min(max_ticks, offset));
  return (offset / max_ticks) * 100.0;
}

// ============================================================================
// state_to_string / publish_state
// ============================================================================

std::string RCInputNode::state_to_string(RCState state) const
{
  switch (state) {
    case RCState::WAIT_FOR_ACCEL: return "WAIT_FOR_ACCEL";
    case RCState::NORMAL_RUN:     return "NORMAL_RUN";
    case RCState::SLOW_RUN:       return "SLOW_RUN";
    case RCState::STOP:           return "STOP";
    default:                      return "UNKNOWN";
  }
}

void RCInputNode::publish_state() const
{
  auto msg = std_msgs::msg::String();
  msg.data = state_to_string(state_);
  state_pub_->publish(msg);
}

// ============================================================================
// emergency_callback
// ============================================================================

void RCInputNode::emergency_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool was_active = emergency_active_;
  emergency_active_ = msg->data;

  if (emergency_active_ && !was_active) {
    RCLCPP_ERROR(
      this->get_logger(),
      "EMERGENCY BRAKE active — throttle blocked. State reset to WAIT_FOR_ACCEL.");
    state_ = RCState::WAIT_FOR_ACCEL;
    publish_state();
  } else if (!emergency_active_ && was_active) {
    RCLCPP_INFO(
      this->get_logger(),
      "Emergency brake released. State: WAIT_FOR_ACCEL — press Button B to resume.");
  }
}

// ============================================================================
// read_loop  (50 Hz timer callback)
// Reads both DIO_EF channels, converts to %, detects button edge,
// runs the state machine, and publishes commands.
// ============================================================================

void RCInputNode::read_loop()
{
  const std::string steer_reg = steering_pin_ + "_EF_READ_A";
  const std::string btn_reg   = button_pin_   + "_EF_READ_A";
  const char * names[2]       = {steer_reg.c_str(), btn_reg.c_str()};
  double values[2]            = {0.0, 0.0};
  int err_addr                = INITIAL_ERR_ADDRESS;

  const int err = LJM_eReadNames(handle_, 2, names, values, &err_addr);

  if (err != LJME_NOERROR) {
    char err_name[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, err_name);
    consecutive_read_errors_++;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "PWM read error (consecutive=%d): %s", consecutive_read_errors_, err_name);

    // Check whether we have exceeded the signal-loss timeout
    const double elapsed_ms =
      (this->now() - last_valid_read_time_).seconds() * 1000.0;

    if (elapsed_ms > pwm_timeout_ms_) {
      if (!signal_was_lost_) {
        signal_was_lost_ = true;
        RCLCPP_ERROR(
          this->get_logger(),
          "RC signal lost for %.0f ms — publishing safe fallback commands.", elapsed_ms);
      }
      // In STOP state keep braking regardless of signal loss
      const float throttle_fallback = (state_ == RCState::STOP)
        ? static_cast<float>(-stop_brake_perc_ / 100.0)
        : 0.0f;

      auto steer_msg    = std_msgs::msg::Float32();
      auto throttle_msg = std_msgs::msg::Float32();
      steer_msg.data    = 0.0f;
      throttle_msg.data = throttle_fallback;
      steering_pub_->publish(steer_msg);
      throttle_pub_->publish(throttle_msg);
      publish_state();
    }
    return;
  }

  // ---- Good read ----------------------------------------------------------
  consecutive_read_errors_ = 0;
  last_valid_read_time_ = this->now();

  if (signal_was_lost_) {
    signal_was_lost_ = false;
    RCLCPP_INFO(this->get_logger(), "RC signal recovered.");
  }

  const double steer_ticks = values[0];
  const double btn_ticks   = values[1];

  // ---- Convert to percent -------------------------------------------------
  const double steer_percent =
    convert_to_percent(steer_ticks, initial_steering_ticks_, steering_max_ticks_); //TODO: this need to be radiants.
  const double btn_percent =
    convert_to_percent(btn_ticks, initial_button_ticks_, button_max_ticks_);

  // ---- Publish raw debug values -------------------------------------------
  {
    auto steer_raw_msg = std_msgs::msg::Float32();
    auto btn_raw_msg   = std_msgs::msg::Float32();
    steer_raw_msg.data = static_cast<float>(steer_percent);
    btn_raw_msg.data   = static_cast<float>(btn_percent);
    steering_raw_pub_->publish(steer_raw_msg);
    button_raw_pub_->publish(btn_raw_msg);
  }

  // ---- Button edge detection (momentary — rising edge only) ---------------
  const bool btn_pressed_now = (std::abs(btn_percent) > button_threshold_percent_);
  bool btn_press_event = false;

  if (btn_pressed_now && !button_was_pressed_) {
    // Rising edge — check debounce window
    const double ms_since_last =
      (this->now() - last_button_press_time_).seconds() * 1000.0;
    if (ms_since_last >= button_debounce_ms_) {
      btn_press_event = true;
      last_button_press_time_ = this->now();
      RCLCPP_INFO(
        this->get_logger(),
        "Button B pressed (raw=%.1f %%).", btn_percent);
    }
  }
  button_was_pressed_ = btn_pressed_now;

  // ---- Run state machine and publish commands -----------------------------
  run_state_machine(steer_percent, btn_press_event);
}

// ============================================================================
// run_state_machine
// Computes throttle output from the current state, handles transitions,
// and publishes steering + throttle commands.
// ============================================================================

void RCInputNode::run_state_machine(double steer_percent, bool btn_press_event)
{
  const RCState prev_state = state_;

  // Steering is a direct pass-through in all states: % → ratio [-1, +1]
  const double steer_ratio =
    std::max(-1.0, std::min(1.0, steer_percent / 100.0));

  double throttle_ratio = 0.0;

  switch (state_) {
    // -----------------------------------------------------------------------
    case RCState::WAIT_FOR_ACCEL:
    {
      throttle_ratio = 0.0;

      if (btn_press_event && !emergency_active_) {
        state_ = RCState::NORMAL_RUN;
        throttle_at_slow_entry_ = normal_run_throttle_perc_ / 100.0;
        RCLCPP_INFO(
          this->get_logger(),
          "WAIT_FOR_ACCEL -> NORMAL_RUN  (throttle=%.0f %%)",
          normal_run_throttle_perc_);
      }
      break;
    }

    // -----------------------------------------------------------------------
    case RCState::NORMAL_RUN:
    {
      throttle_ratio = normal_run_throttle_perc_ / 100.0;

      if (btn_press_event && !emergency_active_) {
        state_ = RCState::SLOW_RUN;
        slow_run_start_time_    = this->now();
        throttle_at_slow_entry_ = normal_run_throttle_perc_ / 100.0;
        RCLCPP_INFO(
          this->get_logger(),
          "NORMAL_RUN -> SLOW_RUN  (%.0f %% -> %.0f %% over %.1f s)",
          normal_run_throttle_perc_, slow_run_throttle_perc_, slow_run_ramp_time_s_);
      }
      break;
    }

    // -----------------------------------------------------------------------
    case RCState::SLOW_RUN:
    {
      // Linear ramp: entry throttle → slow target over slow_run_ramp_time_s_
      const double elapsed = (this->now() - slow_run_start_time_).seconds();
      const double t =
        std::min(1.0, elapsed / std::max(slow_run_ramp_time_s_, 1e-6));
      const double slow_target = slow_run_throttle_perc_ / 100.0;
      throttle_ratio = throttle_at_slow_entry_ + t * (slow_target - throttle_at_slow_entry_);

      if (btn_press_event && !emergency_active_) {
        state_ = RCState::STOP;
        stop_start_time_ = this->now();
        RCLCPP_INFO(
          this->get_logger(),
          "SLOW_RUN -> STOP  (brake=%.0f %%  ramp=%.1f s)",
          stop_brake_perc_, stop_brake_ramp_time_s_);
      }
      break;
    }

    // -----------------------------------------------------------------------
    case RCState::STOP:
    {
      // Terminal state. Applies braking — instant or ramped.
      const double target_brake = -(stop_brake_perc_ / 100.0);

      if (stop_brake_ramp_time_s_ <= 0.0) {
        throttle_ratio = target_brake;                          // instant
      } else {
        const double elapsed = (this->now() - stop_start_time_).seconds();
        const double t = std::min(1.0, elapsed / stop_brake_ramp_time_s_);
        throttle_ratio = t * target_brake;                      // smooth ramp
      }
      // No further button B handling — STOP is terminal.
      break;
    }
  }

  // ---- Log state transition -----------------------------------------------
  if (prev_state != state_) {
    publish_state();
  }

  // ---- Emergency brake guard: block throttle output (steering still flows) -
  if (emergency_active_) {
    throttle_ratio = 0.0;
  }

  // ---- Publish ------------------------------------------------------------
  {
    auto steer_msg    = std_msgs::msg::Float32();
    auto throttle_msg = std_msgs::msg::Float32();
    steer_msg.data    = static_cast<float>(steer_ratio);
    throttle_msg.data = static_cast<float>(throttle_ratio);
    steering_pub_->publish(steer_msg);
    throttle_pub_->publish(throttle_msg);
  }

  // Publish current state every tick for live monitoring
  publish_state();
}

// ============================================================================
// main
// ============================================================================

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RCInputNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

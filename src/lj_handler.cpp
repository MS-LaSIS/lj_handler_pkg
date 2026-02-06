#include "lj_handler_pkg/lj_handler.hpp"
#include <cmath>

float melex_max_deg = atan(2555.0/4500.0) * 180.0 / M_PI; // approx 29.74 degrees

/**
* @brief Linearly map a value from one range to another
 */
double map(double value, double in_min, double in_max, double out_min, double out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


LJHandlerNode::LJHandlerNode() : Node("lj_handler")
{
  // Declare parameters for pin assignments
  this->declare_parameter<std::string>("nominal_vs_steer_M_pin", "AIN10");
  this->declare_parameter<std::string>("nominal_vs_steer_S_pin", "AIN12");
  this->declare_parameter<std::string>("nominal_vs_accbrake_M_pin", "AIN6");
  this->declare_parameter<std::string>("nominal_vs_accbrake_S_pin", "AIN8");
  
  // Declare voltage range parameters
  this->declare_parameter<double>("input_voltage_min", 0.1);
  this->declare_parameter<double>("input_voltage_max", 5.26);
  
  // Declare percentage limits for steering
  this->declare_parameter<double>("steering_min_perc", 0.15);
  this->declare_parameter<double>("steering_max_perc", 0.85);
  
  // Declare percentage limits for throttle/brake
  this->declare_parameter<double>("throttle_min_perc", 0.23);
  this->declare_parameter<double>("throttle_max_perc", 0.77);
  
  // Declare steering parameters
  this->declare_parameter<double>("max_steering_angle", melex_max_deg); // degrees
  this->declare_parameter<double>("steering_clip", 20); // Steering clip factor in degrees
  
   // Declare input mode: "angle" (radians from controller) or "ratio" (direct -1 to 1)
   this->declare_parameter<std::string>("input_mode", "angle");
  
  // Declare offset parameters (used in ratio mode for calibration)
  this->declare_parameter<double>("steering_offset", 0.0);
  this->declare_parameter<double>("throttle_offset", 0.0);
  
  // Declare timeout parameters
  this->declare_parameter<double>("pose_timeout", 0.5); // seconds
  this->declare_parameter<double>("throttle_timeout", 0.5); // seconds
  this->declare_parameter<double>("safety_check_period", 0.1); // seconds
  
  // Declare topic parameters //TODO: set to actual topics
  this->declare_parameter<std::string>("steering_topic", "/follower/steering_cmd");
  this->declare_parameter<std::string>("throttle_topic", "/follower/acceleration_cmd");
  
  // Declare brake to throttle delay
  this->declare_parameter<double>("brake_to_throttle_delay", 0.3);  // 0.5 seconds
  
  // Get parameters
  nominal_vs_steer_master_pin_ = this->get_parameter("nominal_vs_steer_M_pin").as_string();
  nominal_vs_steer_slave_pin_ = this->get_parameter("nominal_vs_steer_S_pin").as_string();
  nominal_vs_accbrake_master_pin_ = this->get_parameter("nominal_vs_accbrake_M_pin").as_string();
  nominal_vs_accbrake_slave_pin_ = this->get_parameter("nominal_vs_accbrake_S_pin").as_string();
  
  input_voltage_min_ = this->get_parameter("input_voltage_min").as_double();
  input_voltage_max_ = this->get_parameter("input_voltage_max").as_double();
  center_voltage_ = (input_voltage_max_ + input_voltage_min_) / 2.0;
  
  steering_min_perc_ = this->get_parameter("steering_min_perc").as_double();
  steering_max_perc_ = this->get_parameter("steering_max_perc").as_double();
  steering_clip_ = this->get_parameter("steering_clip").as_double();
  
  throttle_min_perc_ = this->get_parameter("throttle_min_perc").as_double();
  throttle_max_perc_ = this->get_parameter("throttle_max_perc").as_double();
  
  max_steering_angle_ = melex_max_deg;
  
  // Get input mode and offsets
  input_mode_ = this->get_parameter("input_mode").as_string();
  steering_offset_ = this->get_parameter("steering_offset").as_double();
  throttle_offset_ = this->get_parameter("throttle_offset").as_double();
  
  steering_timeout_sec_ = this->get_parameter("pose_timeout").as_double();
  throttle_timeout_sec_ = this->get_parameter("throttle_timeout").as_double();
  double safety_check_period = this->get_parameter("safety_check_period").as_double();
  brake_to_throttle_delay_ = this->get_parameter("brake_to_throttle_delay").as_double();
  
  // Initialize state variables
  wait_remove_brake = false;
  old_throttle = 0.0;
  brake_perc = 0.0;
  throttle_timed_out_ = false;
  consecutive_lj_errors_ = 0;

  std::string steering_topic = this->get_parameter("steering_topic").as_string();
  std::string throttle_topic = this->get_parameter("throttle_topic").as_string();
  
  // Initialize last message times to current time
  last_steering_time_ = this->get_clock()->now();
  last_throttle_time_ = this->get_clock()->now();
  
  // Open LabJack T7
  int err = LJM_Open(LJM_dtT7, LJM_ctUSB, "ANY", &handle_);
  if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_ERROR(this->get_logger(), "Failed to open LabJack T7: %s", errName);
    rclcpp::shutdown();
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "LabJack T7 opened successfully");
  
  // DAC output names
  // Steering: TDAC0=Master1, TDAC1=Master2, TDAC2=Slave1, TDAC3=Slave2
  // Throttle: TDAC4=Master1, TDAC5=Master2, TDAC6=Slave1, TDAC7=Slave2
  steering_dac_names_ = {"TDAC0", "TDAC1", "TDAC2", "TDAC3"};
  throttle_dac_names_ = {"TDAC4", "TDAC5", "TDAC6", "TDAC7"};

  // Create subscription to steering topic
  steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    steering_topic, 10,
    std::bind(&LJHandlerNode::steering_callback, this, std::placeholders::_1));
  
  // Create subscription to throttle topic
  throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    throttle_topic, 10,
    std::bind(&LJHandlerNode::throttle_callback, this, std::placeholders::_1));
  
   // Create safety timer to check for timeouts
   safety_timer_ = this->create_wall_timer(
     std::chrono::duration<double>(safety_check_period),
     std::bind(&LJHandlerNode::check_safety_timeout, this));
   
   // Register parameter change callback for dynamic reconfiguration
   param_callback_handle_ = this->add_on_set_parameters_callback(
     std::bind(&LJHandlerNode::on_parameter_change, this, std::placeholders::_1));
   
   RCLCPP_INFO(this->get_logger(), "LJ Handler Node initialized");
  RCLCPP_INFO(this->get_logger(), "Input mode: '%s'", input_mode_.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribing to steering topic: '%s'", steering_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Subscribing to throttle topic: '%s'", throttle_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "Max steering angle: %.2f°", max_steering_angle_);
  RCLCPP_INFO(this->get_logger(), "Throttle timeout: %.2fs", throttle_timeout_sec_);
  RCLCPP_INFO(this->get_logger(), "Safety check period: %.2fs", safety_check_period);
  if (input_mode_ == "ratio") {
    RCLCPP_INFO(this->get_logger(), "Steering offset: %.3f, Throttle offset: %.3f", 
                steering_offset_, throttle_offset_);
  }
}

LJHandlerNode::~LJHandlerNode()
{
  // Set steering to center and throttle to zero before closing
  RCLCPP_INFO(this->get_logger(), "Setting steering to center and throttle to zero");
  set_steering_ratio(0.0);
  set_throttle_brake(0.0);
  
  // Close LabJack handle
  if (handle_ > 0) {
    LJM_Close(handle_);
    RCLCPP_INFO(this->get_logger(), "LabJack connection closed");
  }
}

rcl_interfaces::msg::SetParametersResult LJHandlerNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & param : parameters) {
    // Only allow reconfiguration of steering_offset and throttle_offset
    if (param.get_name() == "steering_offset") {
      // Check safety constraint: throttle must be zero
      if (std::abs(old_throttle) > 0.01) {
        result.successful = false;
        result.reason = "Cannot change steering_offset: throttle must be zero (current: " +
                       std::to_string(old_throttle) + ")";
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      
      double new_offset = param.as_double();
      auto now = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(),
                 "Parameter change: steering_offset %.3f -> %.3f (timestamp: %.6f)",
                 steering_offset_, new_offset,
                 now.seconds());
      steering_offset_ = new_offset;
    }
    else if (param.get_name() == "throttle_offset") {
      // Check safety constraint: throttle must be zero
      if (std::abs(old_throttle) > 0.01) {
        result.successful = false;
        result.reason = "Cannot change throttle_offset: throttle must be zero (current: " +
                       std::to_string(old_throttle) + ")";
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      
      double new_offset = param.as_double();
      auto now = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(),
                 "Parameter change: throttle_offset %.3f -> %.3f (timestamp: %.6f)",
                 throttle_offset_, new_offset,
                 now.seconds());
      throttle_offset_ = new_offset;
    }
  }

  return result;
}

void LJHandlerNode::steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Update last steering time
  last_steering_time_ = this->get_clock()->now();

  double steering_ratio;
  
  if (input_mode_ == "ratio") {
    // Input is already a ratio from -1.0 to 1.0
    // -1.0 = full left, 0.0 = center, 1.0 = full right
    steering_ratio = msg->data + steering_offset_;
    steering_ratio = std::max(-1.0, std::min(1.0, steering_ratio));
    RCLCPP_DEBUG(this->get_logger(), "Steering command (ratio mode): %.3f", steering_ratio);
  } else {
    // Default: "angle" mode - input is in radians from controller
    // Convert to degrees
    double steering_deg = msg->data * 180.0 / M_PI;
    // Clamp to steering clip
    steering_deg = std::max(-steering_clip_, std::min(steering_clip_, steering_deg));
    // Invert direction
    steering_deg = steering_deg * -1.0;
    // Convert degrees to ratio: -max_angle -> -1.0, 0 -> 0.0, +max_angle -> 1.0
    steering_ratio = steering_deg / max_steering_angle_;
    steering_ratio = std::max(-1.0, std::min(1.0, steering_ratio));
    RCLCPP_DEBUG(this->get_logger(), "Steering command (angle mode): %.2f rad -> %.2f deg -> %.3f ratio", 
                 msg->data, steering_deg, steering_ratio);
  }
  
  // Apply steering
  set_steering_ratio(steering_ratio);
}

void LJHandlerNode::throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Update last throttle time
  last_throttle_time_ = this->get_clock()->now();
  throttle_timed_out_ = false;
  
  // Get throttle value (expected range: -1.0 to 1.0)
  // -1.0 = full brake, 0.0 = neutral, 1.0 = full throttle
  double throttle_value = msg->data;
  
  // Apply offset in ratio mode
  if (input_mode_ == "ratio") {
    throttle_value += throttle_offset_;
  }
  
  // Detect transition from brake (negative) to throttle (positive)
  if (old_throttle < 0 && throttle_value > 0) {
    // Brake was pressed, now trying to accelerate
    // Start neutral waiting period
    brake_release_time_ = this->get_clock()->now();
    wait_remove_brake = true;
    brake_perc = old_throttle;
    RCLCPP_INFO(this->get_logger(), 
                "Brake-to-throttle transition detected. Enforcing %.2fs neutral period.",
                brake_to_throttle_delay_);
  }
  
  // If we're in the waiting period after brake release
  if (wait_remove_brake) {
    double time_since_brake_release = 
        (this->get_clock()->now() - brake_release_time_).seconds();

    if (time_since_brake_release < brake_to_throttle_delay_ + (std::abs(brake_perc))/2 ) {
      // Still in waiting period - force neutral
      throttle_value = 0.0;
      
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Neutral period: %.2fs / %.2fs",
                           time_since_brake_release, brake_to_throttle_delay_);
    } else {
      // Waiting period complete
      wait_remove_brake = false;
      RCLCPP_INFO(this->get_logger(), 
                  "Neutral period complete (%.2fs). Throttle enabled.",
                  time_since_brake_release);
    }
  }
  
  // Clamp to valid range
  throttle_value = std::max(-1.0, std::min(1.0, throttle_value));
  
  RCLCPP_DEBUG(this->get_logger(), "Throttle command: %.2f", throttle_value);
  
  // Apply throttle/brake
  set_throttle_brake(throttle_value);
  old_throttle = throttle_value;
}

void LJHandlerNode::check_safety_timeout()
{
  rclcpp::Time current_time = this->get_clock()->now();
  
  // Check LabJack health - escalate on consecutive failures
  if (consecutive_lj_errors_ >= LJ_ERROR_CRITICAL_THRESHOLD) {
    RCLCPP_ERROR(this->get_logger(),
                 "LabJack unresponsive after %d consecutive errors - shutting down",
                 consecutive_lj_errors_);
    rclcpp::shutdown();
    return;
  } else if (consecutive_lj_errors_ >= LJ_ERROR_WARN_THRESHOLD) {
    RCLCPP_ERROR(this->get_logger(),
                 "LabJack communication failing (%d consecutive errors) - attempting emergency stop",
                 consecutive_lj_errors_);
    set_throttle_brake(0.0);
  }

  // Check throttle timeout
  double throttle_time_diff = (current_time - last_throttle_time_).seconds();
  if (throttle_time_diff > throttle_timeout_sec_) {
    if (!throttle_timed_out_) {
      RCLCPP_WARN(this->get_logger(),
                  "Throttle timeout (%.2fs since last message) - setting to zero",
                  throttle_time_diff);
      set_throttle_brake(0.0);
      throttle_timed_out_ = true;
    }
  }

  // Check steering timeout - stop acceleration but don't touch steering
  // (changing trajectory blindly could make things worse)
  double steering_time_diff = (current_time - last_steering_time_).seconds();
  if (steering_time_diff > steering_timeout_sec_) {
    if (!throttle_timed_out_) {
      RCLCPP_WARN(this->get_logger(),
                  "Steering timeout (%.2fs since last message) - stopping acceleration",
                  steering_time_diff);
      set_throttle_brake(0.0);
      throttle_timed_out_ = true;
    }
  }
}

void LJHandlerNode::set_steering_ratio(double steering_ratio)
{
  // Convert steering ratio (-1.0 to 1.0) to internal ratio (0.0 to 1.0)
  // -1.0 -> 0.0, 0.0 -> 0.5, 1.0 -> 1.0
  double ratio = (steering_ratio + 1.0) / 2.0;
  ratio = std::max(0.0, std::min(1.0, ratio)); // Clamp to [0, 1]
  
  // Read nominal voltages
  double nom_vs_steer_master, nom_vs_steer_slave, ph1, ph2;
  int err = read_nominal_voltages(nom_vs_steer_master, nom_vs_steer_slave, ph1, ph2);
  if (err != LJME_NOERROR) {
    RCLCPP_WARN(this->get_logger(), "Failed to read nominal voltages for steering");
    return;
  }
  
  // Apply control using common function
  set_control_axis(ratio, 
                   nom_vs_steer_master, 
                   nom_vs_steer_slave,
                   steering_dac_names_, 
                   steering_min_perc_, 
                   steering_max_perc_,
                   "Steering",
                   true); // opposition mode
}

void LJHandlerNode::set_throttle_brake(double throttle_value)
{
  // Map throttle value (-1.0 to 1.0) to voltage ratio (0.0 to 1.0)
  // -1.0 -> 0.0, 0.0 -> 0.5, 1.0 -> 1.0
  double throttle_ratio = (throttle_value + 1.0) / 2.0;
  throttle_ratio = std::max(0.0, std::min(1.0, throttle_ratio)); // Clamp to [0, 1]
  
  // Read nominal voltages
  double ph1, ph2, nom_vs_accbrake_master, nom_vs_accbrake_slave;
  int err = read_nominal_voltages(ph1, ph2, nom_vs_accbrake_master, nom_vs_accbrake_slave);
  RCLCPP_DEBUG(this->get_logger(), "Nominal voltages - Acc/Brake Master: %.2f V, Slave: %.2f V",
               nom_vs_accbrake_master, nom_vs_accbrake_slave);
  if (err != LJME_NOERROR) {
    RCLCPP_WARN(this->get_logger(), "Failed to read nominal voltages for throttle");
    return;
  }
  
  // Apply control using common function
  set_control_axis(throttle_ratio,
                   nom_vs_accbrake_master, 
                   nom_vs_accbrake_slave,
                   throttle_dac_names_, 
                   throttle_min_perc_, 
                   throttle_max_perc_,
                   "Throttle",
                   false); // opposition mode
}

void LJHandlerNode::set_control_axis(double ratio,
                                     double nom_vs_master,
                                     double nom_vs_slave,
                                     const std::vector<std::string>& dac_names,
                                     double min_perc,
                                     double max_perc,
                                     const std::string& axis_name,
                                     const bool opposition)
{
  // Clamp the input voltage to the expected range
  ratio = std::max(0.0, std::min(1.0, ratio));
  
  // Normalize the desired voltage to a ratio from 0.0 to 1.0
  
  // Calculate voltage limits
  double master_max_out_v = nom_vs_master * max_perc;
  double master_min_out_v = nom_vs_master * min_perc;
  double slave_min_out_v = nom_vs_slave * min_perc;
  double slave_max_out_v = nom_vs_slave * max_perc;
  
  // First pair (M1, S1)
  double master1_voltage = map(ratio, 0.0, 1.0, master_min_out_v, master_max_out_v);
  master1_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master1_voltage));
  
  double slave1_voltage = map(1.0 - ratio, 0.0, 1.0, slave_min_out_v, slave_max_out_v);
  slave1_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave1_voltage));
  
  // Second pair (M2, S2) - can work in opposition or parallel
  double master2_ratio = opposition ? (1.0 - ratio) : ratio;
  double master2_voltage = map(master2_ratio, 0.0, 1.0, master_min_out_v, master_max_out_v);
  master2_voltage = std::max(master_min_out_v, std::min(master_max_out_v, master2_voltage));
  
  double slave2_voltage = map(1.0 - master2_ratio, 0.0, 1.0, slave_min_out_v, slave_max_out_v);
  slave2_voltage = std::max(slave_min_out_v, std::min(slave_max_out_v, slave2_voltage));
  
  // Write voltages to DACs
  double voltages[4] = {master1_voltage, master2_voltage, slave1_voltage, slave2_voltage};
  
  int errorAddress = INITIAL_ERR_ADDRESS;
  const char* dac_names_arr[4] = {dac_names[0].c_str(), dac_names[1].c_str(),
                                   dac_names[2].c_str(), dac_names[3].c_str()};
  int err = LJM_eWriteNames(handle_, 4, dac_names_arr, voltages, &errorAddress);
  
  if (err != LJME_NOERROR) {
    consecutive_lj_errors_++;
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_WARN(this->get_logger(), "Error writing to %s DACs: %s (address: %d)", 
                axis_name.c_str(), errName, errorAddress);
    return;
  }
  
  consecutive_lj_errors_ = 0;
  
  RCLCPP_INFO(this->get_logger(),
              "%s: %.1f%% -> M1: %.2fV, S1: %.2fV | M2: %.2fV, S2: %.2fV",
              axis_name.c_str(), ratio * 100.0,
              master1_voltage, slave1_voltage, master2_voltage, slave2_voltage);
}

int LJHandlerNode::read_nominal_voltages(double& nom_vs_steer_master,
                                        double& nom_vs_steer_slave,
                                        double& nom_vs_accbrake_master,
                                        double& nom_vs_accbrake_slave)
{
  const char* names[4] = {
    nominal_vs_steer_master_pin_.c_str(),
    nominal_vs_steer_slave_pin_.c_str(),
    nominal_vs_accbrake_master_pin_.c_str(),
    nominal_vs_accbrake_slave_pin_.c_str()
  };
  double values[4];
  int errorAddress = INITIAL_ERR_ADDRESS;
  
  int err = LJM_eReadNames(handle_, 4, names, values, &errorAddress);
  
  if (err == LJME_NOERROR) {
    nom_vs_steer_master = values[0];
    nom_vs_steer_slave = values[1];
    nom_vs_accbrake_master = values[2];
    nom_vs_accbrake_slave = values[3];
    consecutive_lj_errors_ = 0;
  } else {
    consecutive_lj_errors_++;
  }
  
  return err;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LJHandlerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
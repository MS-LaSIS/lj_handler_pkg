#include "lj_handler_pkg/lj_handler.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

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

  // Declare emergency brake (safety button) parameters
  this->declare_parameter<std::string>("safety_ain_pin", "AIN0");
  this->declare_parameter<double>("safety_voltage_threshold", 4.0); // V
  this->declare_parameter<double>("safety_ain_check_period", 0.02); // seconds (50 Hz)
  
  // Declare topic parameters //TODO: set to actual topics
  this->declare_parameter<std::string>("steering_topic", "/follower/steering_cmd");
  this->declare_parameter<std::string>("throttle_topic", "/follower/acceleration_cmd");
  
  // Declare brake to throttle delay
  this->declare_parameter<double>("brake_to_throttle_delay", 0.3);  // 0.5 seconds
  
  // Declare throttle LUT parameters (loaded from config/throttle_lut.yaml)
  this->declare_parameter<std::vector<double>>("throttle_lut_input",  std::vector<double>{});
  this->declare_parameter<std::vector<double>>("throttle_lut_output", std::vector<double>{});

  // Declare steering LUT parameters (loaded from config/steering_lut.yaml)
  // input:  steering angle in degrees (sorted ascending, negative=left, positive=right)
  // output: steering ratio in [-0.5, 0.5]
  this->declare_parameter<std::vector<double>>("steering_lut_input",  std::vector<double>{});
  this->declare_parameter<std::vector<double>>("steering_lut_output", std::vector<double>{});

  // Declare steering input unit: "rad" (default, backwards-compatible) or "deg"
  // Only applies in input_mode == "angle"; ignored in "ratio" mode.
  this->declare_parameter<std::string>("steering_input_unit", "rad");
  
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
  
  max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
  
  // Get input mode and offsets
  input_mode_ = this->get_parameter("input_mode").as_string();
  steering_offset_ = this->get_parameter("steering_offset").as_double();
  throttle_offset_ = this->get_parameter("throttle_offset").as_double();
  
  steering_timeout_sec_ = this->get_parameter("pose_timeout").as_double();
  throttle_timeout_sec_ = this->get_parameter("throttle_timeout").as_double();
  double safety_check_period = this->get_parameter("safety_check_period").as_double();
  brake_to_throttle_delay_ = this->get_parameter("brake_to_throttle_delay").as_double();

  // Emergency brake parameters
  safety_ain_pin_ = this->get_parameter("safety_ain_pin").as_string();
  safety_voltage_threshold_ = this->get_parameter("safety_voltage_threshold").as_double();
  safety_ain_check_period_ = this->get_parameter("safety_ain_check_period").as_double();
  
  // Load throttle LUT
  throttle_lut_enabled_ = false;
  {
    auto lut_in  = this->get_parameter("throttle_lut_input").as_double_array();
    auto lut_out = this->get_parameter("throttle_lut_output").as_double_array();
    if (!lut_in.empty() || !lut_out.empty()) {
      if (!validate_and_load_lut(lut_in, lut_out)) {
        RCLCPP_WARN(this->get_logger(),
                    "Throttle LUT invalid at startup - falling back to linear mapping");
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Throttle LUT not configured - using linear mapping");
    }
  }

  // Load steering LUT
  steering_lut_enabled_ = false;
  steering_input_unit_ = this->get_parameter("steering_input_unit").as_string();
  {
    auto slut_in  = this->get_parameter("steering_lut_input").as_double_array();
    auto slut_out = this->get_parameter("steering_lut_output").as_double_array();
    if (!slut_in.empty() || !slut_out.empty()) {
      if (!validate_and_load_steering_lut(slut_in, slut_out)) {
        RCLCPP_WARN(this->get_logger(),
                    "Steering LUT invalid at startup - falling back to linear angle mapping");
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Steering LUT not configured - using linear angle/max_steering_angle mapping");
    }
  }
  
  // Initialize state variables
  wait_remove_brake = false;
  old_throttle = 0.0;
  brake_perc = 0.0;
  throttle_timed_out_ = false;
  consecutive_lj_errors_ = 0;
  emergency_brake_active_ = false;
  consecutive_safety_ain_errors_ = 0;

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

   // Create dedicated high-frequency timer for emergency brake AIN polling
   safety_ain_timer_ = this->create_wall_timer(
     std::chrono::duration<double>(safety_ain_check_period_),
     std::bind(&LJHandlerNode::check_safety_ain, this));

   // Create emergency brake state publisher
   emergency_brake_pub_ = this->create_publisher<std_msgs::msg::Bool>(
     "/emergency_brake", 10);
   
   // Register parameter change callback for dynamic reconfiguration
   param_callback_handle_ = this->add_on_set_parameters_callback(
     std::bind(&LJHandlerNode::on_parameter_change, this, std::placeholders::_1));

   // Parse DEBUG env var into debug category set
   // Usage: DEBUG=1 or DEBUG=all  -> all categories
   //        DEBUG=steering,pedal  -> only those categories
   // Categories: steering, pedal, voltages, dac, safety, params
   const char* debug_env = std::getenv("DEBUG");
   if (debug_env != nullptr) {
     static const std::vector<std::string> ALL_CATEGORIES =
       {"steering", "pedal", "voltages", "dac", "safety", "params"};
     std::string debug_str(debug_env);
     if (debug_str == "1" || debug_str == "all" || debug_str == "true") {
       debug_categories_.insert(ALL_CATEGORIES.begin(), ALL_CATEGORIES.end());
     } else {
       std::istringstream ss(debug_str);
       std::string token;
       while (std::getline(ss, token, ',')) {
         // trim whitespace
         token.erase(0, token.find_first_not_of(" \t"));
         token.erase(token.find_last_not_of(" \t") + 1);
         if (!token.empty()) {
           debug_categories_.insert(token);
         }
       }
     }
   }
   
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
  if (!debug_categories_.empty()) {
    std::string cats;
    for (const auto & c : debug_categories_) {
      if (!cats.empty()) cats += ", ";
      cats += c;
    }
    RCLCPP_INFO(this->get_logger(), "Debug categories enabled: [%s]", cats.c_str());
  }
  if (throttle_lut_enabled_) {
    RCLCPP_INFO(this->get_logger(), "Throttle LUT active: %zu points, monotone cubic interpolation",
                throttle_lut_input_.size());
  } else {
    RCLCPP_INFO(this->get_logger(), "Throttle LUT disabled: using linear mapping for positive throttle");
  }
  if (steering_lut_enabled_) {
    RCLCPP_INFO(this->get_logger(),
                "Steering LUT active: %zu points, monotone cubic interpolation, input_unit=%s",
                steering_lut_input_.size(), steering_input_unit_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Steering LUT disabled: using linear angle/max_steering_angle mapping, input_unit=%s",
                steering_input_unit_.c_str());
  }
  RCLCPP_INFO(this->get_logger(),
              "Emergency brake: pin=%s threshold=%.2fV poll=%.0fHz",
              safety_ain_pin_.c_str(), safety_voltage_threshold_,
              1.0 / safety_ain_check_period_);
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

bool LJHandlerNode::is_debug(const std::string & category) const
{
  return debug_categories_.count(category) > 0;
}

rcl_interfaces::msg::SetParametersResult LJHandlerNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();

    // --- Offset parameters: require throttle to be zero (safety guard) ---
    if (name == "steering_offset") {
      if (std::abs(old_throttle) > 0.01) {
        result.successful = false;
        result.reason = "Cannot change steering_offset: throttle must be zero (current: " +
                        std::to_string(old_throttle) + ")";
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] steering_offset %.3f -> %.3f", steering_offset_, new_val);
      }
      steering_offset_ = new_val;
    }
    else if (name == "throttle_offset") {
      if (std::abs(old_throttle) > 0.01) {
        result.successful = false;
        result.reason = "Cannot change throttle_offset: throttle must be zero (current: " +
                        std::to_string(old_throttle) + ")";
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] throttle_offset %.3f -> %.3f", throttle_offset_, new_val);
      }
      throttle_offset_ = new_val;
    }

    // --- Voltage range parameters ---
    else if (name == "input_voltage_min") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] input_voltage_min %.3f -> %.3f", input_voltage_min_, new_val);
      }
      input_voltage_min_ = new_val;
      center_voltage_ = (input_voltage_max_ + input_voltage_min_) / 2.0;
    }
    else if (name == "input_voltage_max") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] input_voltage_max %.3f -> %.3f", input_voltage_max_, new_val);
      }
      input_voltage_max_ = new_val;
      center_voltage_ = (input_voltage_max_ + input_voltage_min_) / 2.0;
    }

    // --- Steering percentage limits ---
    else if (name == "steering_min_perc") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] steering_min_perc %.3f -> %.3f", steering_min_perc_, new_val);
      }
      steering_min_perc_ = new_val;
    }
    else if (name == "steering_max_perc") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] steering_max_perc %.3f -> %.3f", steering_max_perc_, new_val);
      }
      steering_max_perc_ = new_val;
    }

    // --- Throttle percentage limits ---
    else if (name == "throttle_min_perc") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] throttle_min_perc %.3f -> %.3f", throttle_min_perc_, new_val);
      }
      throttle_min_perc_ = new_val;
    }
    else if (name == "throttle_max_perc") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] throttle_max_perc %.3f -> %.3f", throttle_max_perc_, new_val);
      }
      throttle_max_perc_ = new_val;
    }

    // --- Steering geometry parameters ---
    else if (name == "max_steering_angle") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] max_steering_angle %.2f -> %.2f deg", max_steering_angle_, new_val);
      }
      max_steering_angle_ = new_val;
    }
    else if (name == "steering_clip") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] steering_clip %.2f -> %.2f deg", steering_clip_, new_val);
      }
      steering_clip_ = new_val;
    }

    // --- Input mode ---
    else if (name == "input_mode") {
      std::string new_val = param.as_string();
      if (new_val != "angle" && new_val != "ratio") {
        result.successful = false;
        result.reason = "input_mode must be \"angle\" or \"ratio\", got: " + new_val;
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] input_mode \"%s\" -> \"%s\"", input_mode_.c_str(), new_val.c_str());
      }
      input_mode_ = new_val;
    }

    // --- Steering input unit ---
    else if (name == "steering_input_unit") {
      std::string new_val = param.as_string();
      if (new_val != "rad" && new_val != "deg") {
        result.successful = false;
        result.reason = "steering_input_unit must be \"rad\" or \"deg\", got: " + new_val;
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] steering_input_unit \"%s\" -> \"%s\"",
                    steering_input_unit_.c_str(), new_val.c_str());
      }
      steering_input_unit_ = new_val;
    }

    // --- Steering LUT parameters ---
    else if (name == "steering_lut_input" || name == "steering_lut_output") {
      std::vector<double> new_in, new_out;
      if (name == "steering_lut_input") {
        new_in  = param.as_double_array();
        new_out = this->get_parameter("steering_lut_output").as_double_array();
      } else {
        new_in  = this->get_parameter("steering_lut_input").as_double_array();
        new_out = param.as_double_array();
      }

      if (new_in.empty() && new_out.empty()) {
        steering_lut_enabled_ = false;
        steering_lut_input_.clear();
        steering_lut_output_.clear();
        RCLCPP_INFO(this->get_logger(), "[params] Steering LUT cleared - using linear mapping");
      } else if (!validate_and_load_steering_lut(new_in, new_out)) {
        steering_lut_enabled_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "[params] Steering LUT update rejected - keeping previous state");
        result.successful = false;
        result.reason = "steering_lut invalid: check sizes match, input is strictly monotonic, "
                        "and output values are in [-1.0, 1.0]";
        break;
      } else {
        if (is_debug("params")) {
          RCLCPP_INFO(this->get_logger(),
                      "[params] Steering LUT updated: %zu points", steering_lut_input_.size());
        }
      }
    }

    // --- Throttle LUT parameters ---
    else if (name == "throttle_lut_input" || name == "throttle_lut_output") {
      // Safety guard: do not change LUT while throttle is active
      if (std::abs(old_throttle) > 0.01) {
        result.successful = false;
        result.reason = "Cannot change throttle LUT while throttle is active (current: " +
                        std::to_string(old_throttle) + "). Set throttle to zero first.";
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      // Re-read both arrays after potential update
      // Note: the new value is in 'param'; the other array is already in the parameter server
      std::vector<double> new_in, new_out;
      if (name == "throttle_lut_input") {
        new_in  = param.as_double_array();
        new_out = this->get_parameter("throttle_lut_output").as_double_array();
      } else {
        new_in  = this->get_parameter("throttle_lut_input").as_double_array();
        new_out = param.as_double_array();
      }

      if (new_in.empty() && new_out.empty()) {
        throttle_lut_enabled_ = false;
        throttle_lut_input_.clear();
        throttle_lut_output_.clear();
        RCLCPP_INFO(this->get_logger(), "[params] Throttle LUT cleared - using linear mapping");
      } else if (!validate_and_load_lut(new_in, new_out)) {
        // validate_and_load_lut already logged the reason; disable LUT for safety
        throttle_lut_enabled_ = false;
        RCLCPP_WARN(this->get_logger(),
                    "[params] Throttle LUT update rejected - keeping previous state");
        result.successful = false;
        result.reason = "throttle_lut invalid: check sizes match, input is strictly monotonic, "
                        "and all values are in [0.0, 1.0]";
        break;
      } else {
        if (is_debug("params")) {
          RCLCPP_INFO(this->get_logger(),
                      "[params] Throttle LUT updated: %zu points", throttle_lut_input_.size());
        }
      }
    }

    // --- Timeout parameters ---
    else if (name == "pose_timeout") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] pose_timeout %.3fs -> %.3fs", steering_timeout_sec_, new_val);
      }
      steering_timeout_sec_ = new_val;
    }
    else if (name == "throttle_timeout") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] throttle_timeout %.3fs -> %.3fs", throttle_timeout_sec_, new_val);
      }
      throttle_timeout_sec_ = new_val;
    }
    else if (name == "brake_to_throttle_delay") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] brake_to_throttle_delay %.3fs -> %.3fs",
                    brake_to_throttle_delay_, new_val);
      }
      brake_to_throttle_delay_ = new_val;
    }

    // --- Emergency brake parameters ---
    else if (name == "safety_ain_pin") {
      if (emergency_brake_active_) {
        result.successful = false;
        result.reason = "Cannot change safety_ain_pin while emergency brake is active";
        RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
        break;
      }
      std::string new_val = param.as_string();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] safety_ain_pin \"%s\" -> \"%s\"",
                    safety_ain_pin_.c_str(), new_val.c_str());
      }
      safety_ain_pin_ = new_val;
      consecutive_safety_ain_errors_ = 0;
    }
    else if (name == "safety_voltage_threshold") {
      double new_val = param.as_double();
      if (is_debug("params")) {
        RCLCPP_INFO(this->get_logger(),
                    "[params] safety_voltage_threshold %.3fV -> %.3fV",
                    safety_voltage_threshold_, new_val);
      }
      safety_voltage_threshold_ = new_val;
    }
    else if (name == "safety_ain_check_period") {
      result.successful = false;
      result.reason = "safety_ain_check_period cannot be changed at runtime - restart the node";
      RCLCPP_WARN(this->get_logger(), "%s", result.reason.c_str());
      break;
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
    // TODO: check the correct direction of the ratio (may need to invert)
    steering_ratio = -msg->data + steering_offset_;
    steering_ratio = std::max(-1.0, std::min(1.0, steering_ratio));
    if (is_debug("steering")) {
      RCLCPP_INFO(this->get_logger(), "[steering] ratio mode: raw=%.3f offset=%.3f -> ratio=%.3f",
                  msg->data, steering_offset_, steering_ratio);
    }
  } else {
    // Default: "angle" mode - input is an angle from controller
    // Convert to degrees based on steering_input_unit_
    double steering_deg;
    if (steering_input_unit_ == "deg") {
      steering_deg = msg->data;
    } else {
      // "rad" (default)
      steering_deg = msg->data * 180.0 / M_PI;
    }
    steering_deg = -steering_deg + steering_offset_; // Apply offset in degrees for angle mode

    if (steering_lut_enabled_) {
      // LUT path: angle_deg -> ratio directly from measured characterization data.
      // No clip, no direction inversion - the LUT encodes the full physical mapping.
      double angle_clamped = std::max(steering_lut_input_.front(),
                                      std::min(steering_lut_input_.back(), steering_deg));
      steering_ratio = apply_steering_lut(angle_clamped);
      steering_ratio += steering_offset_;
      if (is_debug("steering")) {
        RCLCPP_INFO(this->get_logger(),
                    "[steering] LUT mode: %.4f %s -> %.2f deg (clamped: %.2f) -> ratio=%.4f",
                    msg->data, steering_input_unit_.c_str(), steering_deg,
                    angle_clamped, steering_ratio);
      }
    } else {
      // Linear path (existing behaviour)
      // Clamp to steering clip
      steering_deg = std::max(-steering_clip_, std::min(steering_clip_, steering_deg));

      // Convert degrees to ratio: -max_angle -> -1.0, 0 -> 0.0, +max_angle -> 1.0
      steering_ratio = steering_deg / max_steering_angle_;
      steering_ratio += steering_offset_;
      steering_ratio = std::max(-1.0, std::min(1.0, steering_ratio));
      if (is_debug("steering")) {
        RCLCPP_INFO(this->get_logger(),
                    "[steering] linear mode: %.4f %s -> %.2f deg -> ratio=%.4f",
                    msg->data, steering_input_unit_.c_str(), steering_deg, steering_ratio);
      }
    }
  }
  
  // Apply steering
  set_steering_ratio(steering_ratio);
}

void LJHandlerNode::throttle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  // Safety guard: ignore all throttle commands while emergency brake is active
  if (emergency_brake_active_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Emergency brake active - ignoring throttle command (%.2f)", msg->data);
    return;
  }

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
  if(msg->data > 1.0){
    //RCLCPP_WARNING(this->get_logger(),"SOMETHING NASTY with acceleration")
    throttle_value = 0.0; 
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
      
      if (is_debug("pedal")) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "[pedal] neutral period: %.2fs / %.2fs",
                             time_since_brake_release, brake_to_throttle_delay_);
      }
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
  
  if (is_debug("pedal")) {
    RCLCPP_INFO(this->get_logger(), "[pedal] throttle command: %.2f", throttle_value);
  }
  
  // Apply throttle/brake
  set_throttle_brake(throttle_value);
  old_throttle = throttle_value;
}

void LJHandlerNode::check_safety_timeout()
{
  rclcpp::Time current_time = this->get_clock()->now();

  if (is_debug("safety")) {
    double steer_diff = (current_time - last_steering_time_).seconds();
    double thr_diff   = (current_time - last_throttle_time_).seconds();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[safety] steer_age=%.2fs (limit=%.2fs)  throttle_age=%.2fs (limit=%.2fs)",
                         steer_diff, steering_timeout_sec_, thr_diff, throttle_timeout_sec_);
  }
  
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

void LJHandlerNode::check_safety_ain()
{
  // Read the hardware safety button analog input
  double voltage = 0.0;
  int err = LJM_eReadName(handle_, safety_ain_pin_.c_str(), &voltage);

  if (err != LJME_NOERROR) {
    consecutive_safety_ain_errors_++;
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to read safety AIN (%s): %s (consecutive errors: %d)",
                         safety_ain_pin_.c_str(), errName, consecutive_safety_ain_errors_);

    if (consecutive_safety_ain_errors_ >= 5 && !emergency_brake_active_) {
      // 5 consecutive read failures = 100 ms of no signal confirmation.
      // Cannot verify button state - trigger emergency brake as fail-safe.
      emergency_brake_active_ = true;
      RCLCPP_ERROR(this->get_logger(),
                   "EMERGENCY BRAKE TRIGGERED: safety AIN (%s) unreadable for %d consecutive "
                   "reads (%.0f ms) - applying 100%% brake as fail-safe",
                   safety_ain_pin_.c_str(), consecutive_safety_ain_errors_,
                   consecutive_safety_ain_errors_ * 1000.0 * safety_ain_check_period_);
      std_msgs::msg::Bool emerg_msg;
      emerg_msg.data = true;
      emergency_brake_pub_->publish(emerg_msg);
    }

    // Keep braking if emergency is active (newly triggered or pre-existing)
    if (emergency_brake_active_) {
      set_throttle_brake(-1.0);
    }
    return;
  }

  // Successful read - reset error counter
  consecutive_safety_ain_errors_ = 0;

  if (voltage <= safety_voltage_threshold_) {
    // --- Button pressed ---
    if (!emergency_brake_active_) {
      // Transition: normal -> emergency
      emergency_brake_active_ = true;
      RCLCPP_ERROR(this->get_logger(),
                   "EMERGENCY BRAKE TRIGGERED: %s=%.2fV >= %.2fV - applying 100%% brake",
                   safety_ain_pin_.c_str(), voltage, safety_voltage_threshold_);
      std_msgs::msg::Bool msg;
      msg.data = true;
      emergency_brake_pub_->publish(msg);
    } else {
      // Already in emergency: keep braking and log at throttled rate
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Emergency brake ACTIVE: %s=%.2fV - holding 100%% brake",
                           safety_ain_pin_.c_str(), voltage);
    }
    // Apply 100% brake every tick while button is held
    set_throttle_brake(-0.8);
  } else {
    // --- Button released ---
    if (emergency_brake_active_) {
      // Transition: emergency -> normal
      emergency_brake_active_ = false;
      // Go to zero throttle and mark timed-out so normal operation
      // resumes only when a fresh throttle command arrives
      set_throttle_brake(0.0);
      throttle_timed_out_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Emergency brake RELEASED: %s=%.2fV < %.2fV - returning to zero throttle",
                  safety_ain_pin_.c_str(), voltage, safety_voltage_threshold_);
      std_msgs::msg::Bool msg;
      msg.data = false;
      emergency_brake_pub_->publish(msg);
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

  // Apply LUT correction on the positive (throttle) side only: ratio in (0.5, 1.0]
  // Rescale to local [0, 1], apply LUT, rescale back to [0.5, 1.0]
  if (throttle_lut_enabled_ && throttle_ratio > 0.5) {
    double local_in  = (throttle_ratio - 0.5) * 2.0;       // [0, 1]
    double local_out = apply_throttle_lut(local_in);         // [0, 1]
    throttle_ratio   = 0.5 + local_out * 0.5;               // back to [0.5, 1.0]
    if (is_debug("pedal")) {
      RCLCPP_INFO(this->get_logger(),
                  "[pedal] LUT: local_in=%.3f -> local_out=%.3f -> throttle_ratio=%.3f",
                  local_in, local_out, throttle_ratio);
    }
  }
  
  // Read nominal voltages
  double ph1, ph2, nom_vs_accbrake_master, nom_vs_accbrake_slave;
  int err = read_nominal_voltages(ph1, ph2, nom_vs_accbrake_master, nom_vs_accbrake_slave);
  if (err != LJME_NOERROR) {
    RCLCPP_WARN(this->get_logger(), "Failed to read nominal voltages for throttle");
    return;
  }
  if (is_debug("voltages")) {
    RCLCPP_INFO(this->get_logger(), "[voltages] Acc/Brake Master: %.3f V, Slave: %.3f V",
                nom_vs_accbrake_master, nom_vs_accbrake_slave);
  }
  
  // Apply control using common function
  set_control_axis(throttle_ratio,
                   nom_vs_accbrake_master, 
                   nom_vs_accbrake_slave,
                   throttle_dac_names_, 
                   throttle_min_perc_, 
                   throttle_max_perc_,
                   "Throttle",
                   true); // opposition mode
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
  
  if (is_debug("dac")) {
    RCLCPP_INFO(this->get_logger(),
                "[dac] %s: %.1f%% -> M1: %.3fV, S1: %.3fV | M2: %.3fV, S2: %.3fV",
                axis_name.c_str(), ratio * 100.0,
                master1_voltage, slave1_voltage, master2_voltage, slave2_voltage);
  }
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

bool LJHandlerNode::validate_and_load_lut(
  const std::vector<double> & in,
  const std::vector<double> & out)
{
  // Must have at least 2 points and equal size
  if (in.size() < 2 || in.size() != out.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Throttle LUT validation failed: need >= 2 points with matching sizes "
                "(got input=%zu, output=%zu)", in.size(), out.size());
    return false;
  }

  // All values must be in [0.0, 1.0] and input strictly monotonically increasing
  for (size_t i = 0; i < in.size(); ++i) {
    if (in[i] < 0.0 || in[i] > 1.0 || out[i] < 0.0 || out[i] > 1.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Throttle LUT validation failed: value out of [0.0, 1.0] at index %zu "
                  "(in=%.3f, out=%.3f)", i, in[i], out[i]);
      return false;
    }
    if (i > 0 && in[i] <= in[i - 1]) {
      RCLCPP_WARN(this->get_logger(),
                  "Throttle LUT validation failed: input not strictly monotonic at index %zu "
                  "(in[%zu]=%.3f <= in[%zu]=%.3f)", i, i, in[i], i - 1, in[i - 1]);
      return false;
    }
  }

  throttle_lut_input_   = in;
  throttle_lut_output_  = out;
  throttle_lut_enabled_ = true;
  return true;
}

double LJHandlerNode::apply_throttle_lut(double x)
{
  // Clamp input to [0.0, 1.0]
  x = std::max(0.0, std::min(1.0, x));

  const auto & xs = throttle_lut_input_;
  const auto & ys = throttle_lut_output_;
  const size_t n  = xs.size();

  // Boundary conditions: return exact endpoint values
  if (x <= xs.front()) { return ys.front(); }
  if (x >= xs.back())  { return ys.back(); }

  // Find segment: xs[i-1] <= x < xs[i]
  size_t i = static_cast<size_t>(
    std::lower_bound(xs.begin(), xs.end(), x) - xs.begin());
  if (i == 0) { i = 1; }

  // --- Fritsch-Carlson monotone cubic interpolation ---
  // Compute secant slopes for this segment and neighbors
  auto secant = [&](size_t k) {
    return (ys[k] - ys[k - 1]) / (xs[k] - xs[k - 1]);
  };

  // Tangent at left point (i-1)
  double m0;
  if (i == 1) {
    m0 = secant(1);
  } else {
    m0 = 0.5 * (secant(i - 1) + secant(i));
  }

  // Tangent at right point (i)
  double m1;
  if (i == n - 1) {
    m1 = secant(n - 1);
  } else {
    m1 = 0.5 * (secant(i) + secant(i + 1));
  }

  // Fritsch-Carlson monotonicity constraint
  double delta = secant(i);
  if (std::abs(delta) < 1e-12) {
    // Flat segment: force tangents to zero to avoid spurious oscillations
    m0 = 0.0;
    m1 = 0.0;
  } else {
    double alpha = m0 / delta;
    double beta  = m1 / delta;
    double r     = alpha * alpha + beta * beta;
    if (r > 9.0) {
      double tau = 3.0 / std::sqrt(r);
      m0 = tau * alpha * delta;
      m1 = tau * beta  * delta;
    }
  }

  // Cubic Hermite evaluation
  double h  = xs[i] - xs[i - 1];
  double t  = (x - xs[i - 1]) / h;
  double t2 = t * t;
  double t3 = t2 * t;

  double h00 =  2.0 * t3 - 3.0 * t2 + 1.0;
  double h10 =        t3 - 2.0 * t2 + t;
  double h01 = -2.0 * t3 + 3.0 * t2;
  double h11 =        t3 -       t2;

  double result = h00 * ys[i - 1] + h10 * h * m0 + h01 * ys[i] + h11 * h * m1;

  // Clamp output to [0.0, 1.0] as safety guard
  return std::max(0.0, std::min(1.0, result));
}

bool LJHandlerNode::validate_and_load_steering_lut(
  const std::vector<double> & in,
  const std::vector<double> & out)
{
  // Must have at least 2 points and equal size
  if (in.size() < 2 || in.size() != out.size()) {
    RCLCPP_WARN(this->get_logger(),
                "Steering LUT validation failed: need >= 2 points with matching sizes "
                "(got input=%zu, output=%zu)", in.size(), out.size());
    return false;
  }

  // Input (angles) must be strictly monotonically increasing;
  // Output (ratios) must be in [-1.0, 1.0]
  for (size_t i = 0; i < in.size(); ++i) {
    if (out[i] < -1.0 || out[i] > 1.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Steering LUT validation failed: output ratio out of [-1.0, 1.0] at index %zu "
                  "(in=%.3f, out=%.3f)", i, in[i], out[i]);
      return false;
    }
    if (i > 0 && in[i] <= in[i - 1]) {
      RCLCPP_WARN(this->get_logger(),
                  "Steering LUT validation failed: input not strictly monotonic at index %zu "
                  "(in[%zu]=%.3f <= in[%zu]=%.3f)", i, i, in[i], i - 1, in[i - 1]);
      return false;
    }
  }

  steering_lut_input_   = in;
  steering_lut_output_  = out;
  steering_lut_enabled_ = true;
  return true;
}

double LJHandlerNode::apply_steering_lut(double angle_deg)
{
  // Clamp input to LUT range
  angle_deg = std::max(steering_lut_input_.front(),
                       std::min(steering_lut_input_.back(), angle_deg));

  const auto & xs = steering_lut_input_;
  const auto & ys = steering_lut_output_;
  const size_t n  = xs.size();

  // Boundary conditions
  if (angle_deg <= xs.front()) { return ys.front(); }
  if (angle_deg >= xs.back())  { return ys.back(); }

  // Find segment: xs[i-1] <= angle_deg < xs[i]
  size_t i = static_cast<size_t>(
    std::lower_bound(xs.begin(), xs.end(), angle_deg) - xs.begin());
  if (i == 0) { i = 1; }

  // --- Fritsch-Carlson monotone cubic interpolation ---
  auto secant = [&](size_t k) {
    return (ys[k] - ys[k - 1]) / (xs[k] - xs[k - 1]);
  };

  // Tangent at left point (i-1)
  double m0;
  if (i == 1) {
    m0 = secant(1);
  } else {
    m0 = 0.5 * (secant(i - 1) + secant(i));
  }

  // Tangent at right point (i)
  double m1;
  if (i == n - 1) {
    m1 = secant(n - 1);
  } else {
    m1 = 0.5 * (secant(i) + secant(i + 1));
  }

  // Fritsch-Carlson monotonicity constraint
  double delta = secant(i);
  if (std::abs(delta) < 1e-12) {
    m0 = 0.0;
    m1 = 0.0;
  } else {
    double alpha = m0 / delta;
    double beta  = m1 / delta;
    double r     = alpha * alpha + beta * beta;
    if (r > 9.0) {
      double tau = 3.0 / std::sqrt(r);
      m0 = tau * alpha * delta;
      m1 = tau * beta  * delta;
    }
  }

  // Cubic Hermite evaluation
  double h  = xs[i] - xs[i - 1];
  double t  = (angle_deg - xs[i - 1]) / h;
  double t2 = t * t;
  double t3 = t2 * t;

  double h00 =  2.0 * t3 - 3.0 * t2 + 1.0;
  double h10 =        t3 - 2.0 * t2 + t;
  double h01 = -2.0 * t3 + 3.0 * t2;
  double h11 =        t3 -       t2;

  double result = h00 * ys[i - 1] + h10 * h * m0 + h01 * ys[i] + h11 * h * m1;

  // Clamp output to [-1.0, 1.0] as safety guard (final [-0.5, 0.5] clamp in caller)
  return std::max(-1.0, std::min(1.0, result));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LJHandlerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
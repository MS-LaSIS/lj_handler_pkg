# lj_handler_pkg

ROS 2 package for controlling vehicle steering and throttle/brake using LabJack T7 device.

## Overview

This package provides a ROS 2 node that interfaces with a LabJack T7 device to control vehicle actuators through analog voltage outputs. It implements safety features including timeout monitoring, brake-to-throttle transition delays, and configurable voltage limits.

## Features

- **Dual-axis control**: Independent steering and throttle/brake control
- **Multi-layer failsafe**: Timeout monitoring, LabJack health checks, and escalation logic
- **Brake transition delay**: Configurable neutral period when transitioning from brake to throttle
- **Voltage mapping**: Configurable min/max voltage percentages for safe operation
- **Master/Slave DAC control**: Redundant voltage control in opposition mode
- **Real-time monitoring**: Debug logging and safety check timer

## Dependencies

### System Dependencies
- ROS 2 Humble (Ubuntu 22.04)
- LabJack LJM Library (LabJack Modbus)

### ROS 2 Dependencies
- `rclcpp`
- `std_msgs`

### Installation of LabJack LJM

```bash
# Download and install LabJack LJM library
cd /tmp
wget https://labjack.com/sites/default/files/software/labjack_ljm_software_2019_07_16_x86_64.tar.gz
tar -xzf labjack_ljm_software_2019_07_16_x86_64.tar.gz
cd labjack_ljm_software_2019_07_16_x86_64
sudo ./labjack_ljm_installer.run
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select lj_handler_pkg
source install/setup.bash
```

## Configuration

### Parameters

#### Pin Assignments
- `nominal_vs_steer_M_pin` (string, default: "AIN10"): Master steering voltage input pin
- `nominal_vs_steer_S_pin` (string, default: "AIN12"): Slave steering voltage input pin
- `nominal_vs_accbrake_M_pin` (string, default: "AIN6"): Master throttle/brake voltage input pin
- `nominal_vs_accbrake_S_pin` (string, default: "AIN8"): Slave throttle/brake voltage input pin

#### Voltage Configuration
- `input_voltage_min` (double, default: 0.1): Minimum input voltage (V)
- `input_voltage_max` (double, default: 5.26): Maximum input voltage (V)

#### Steering Limits
(Default values were found via trial-and-error on GuidoSimplex Melex golf cars)
- `steering_min_perc` (double, default: 0.15): Minimum steering voltage percentage (15%)
- `steering_max_perc` (double, default: 0.85): Maximum steering voltage percentage (85%)
- `max_steering_angle` (double, default: 29.74): Maximum steering angle in degrees
- `steering_clip` (double, default: 20.0): Steering command clipping limit in degrees

#### Throttle/Brake Limits
- `throttle_min_perc` (double, default: 0.23): Minimum throttle voltage percentage (23%)
- `throttle_max_perc` (double, default: 0.77): Maximum throttle voltage percentage (77%)

#### Safety & Timing
- `pose_timeout` (double, default: 0.5): Pose timeout in seconds (reserved)
- `throttle_timeout` (double, default: 0.5): Throttle command timeout in seconds
- `safety_check_period` (double, default: 0.1): Safety check timer period in seconds
- `brake_to_throttle_delay` (double, default: 0.3): Neutral period duration when transitioning from brake to throttle (seconds)

#### Topics
- `steering_topic` (string, default: "/follower/steering_cmd"): Steering command topic
- `throttle_topic` (string, default: "/follower/pedal_cmd"): Throttle/brake command topic

#### Calibration Offsets (Dynamically Reconfigurable)
- `steering_offset` (double, default: 0.0): Steering offset for calibration (only "ratio" and "copy" modes). **Dynamically reconfigurable via `ros2 param set` or rqt dynamic reconfigure**
- `throttle_offset` (double, default: 0.0): Throttle offset for calibration (only "ratio" and "copy" modes). **Dynamically reconfigurable via `ros2 param set` or rqt dynamic reconfigure**

**Note on Dynamic Reconfiguration:** Only `steering_offset` and `throttle_offset` are dynamically reconfigurable. All other parameters are read-only (for now). Parameter changes are only allowed when the throttle command is at zero for safety reasons.

## Dynamic Reconfiguration

### Overview

The node supports dynamic reconfiguration of two calibration parameters without requiring a restart:

- `steering_offset`: Adjust steering center/neutral position
- `throttle_offset`: Adjust throttle center/neutral position

All other parameters are **read-only (for now)**.

### Using rqt Dynamic Reconfigure

Graphically adjust calibration offsets using rqt:

```bash
# Terminal 1: Launch the node
ros2 launch lj_handler_pkg lj_handler.launch.py input_mode:=ratio

# Terminal 2: Open rqt with dynamic reconfigure plugin
rqt --standalone rqt_reconfigure
```

Then in the rqt window:
1. Select `lj_handler_node` from the left panel
2. You will see sliders for `steering_offset` and `throttle_offset`
3. Adjust the sliders to calibrate
4. Other parameters are read-only (for now)

### Using Command Line

Adjust parameters via `ros2 param set`:

```bash
# Set steering offset
ros2 param set /lj_handler_node steering_offset 0.05

# Set throttle offset
ros2 param set /lj_handler_node throttle_offset -0.02

# View current values
ros2 param get /lj_handler_node steering_offset
ros2 param get /lj_handler_node throttle_offset
```

### Safety Constraints

Parameter changes are **only allowed when throttle is at zero** (threshold: 0.01). This prevents unintended vehicle behavior during calibration.

**Example error:**
```
Error: Cannot change steering_offset: throttle must be zero (current: 0.15)
```

To set parameters:
1. Ensure vehicle is stopped
2. Set throttle command to 0.0
3. Adjust the parameter
4. Vehicle will respond immediately with new calibration

### Parameter Change Logging

All parameter changes are logged with timestamps:

```
[INFO] Parameter change: steering_offset 0.000 -> 0.050 (timestamp: 1738865430.123456)
[INFO] Parameter change: throttle_offset 0.000 -> -0.020 (timestamp: 1738865431.654321)
```

View logs with:
```bash
ros2 launch lj_handler_pkg lj_handler.launch.py log_level:=info
```

## Usage

```bash
ros2 launch lj_handler_pkg lj_handler.launch.py
```

### Launch for Testing (No Hardware)

To test the node without connecting it to the Guidosimplex system, use:

```bash
ros2 launch lj_handler_pkg lj_handler_no_gs_attached.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch lj_handler_pkg lj_handler.launch.py \
  steering_clip:=25.0 \
  throttle_timeout:=1.0 \
  brake_to_throttle_delay:=0.5
```

### Launch in Debug Mode

For debugging, set the `DEBUG` environment variable before the launch command:

```bash
DEBUG=1 ros2 launch lj_handler_pkg lj_handler.launch.py
```
Of course, if you add personalized launch files, remember to configure the environment variable to keep them working.

### Run Node Directly

```bash
ros2 run lj_handler_pkg lj_handler --ros-args \
  --param steering_topic:=/vehicle/steering \
  --param throttle_topic:=/vehicle/throttle
```

## Topics

### Subscribed Topics

- **`/follower/steering_cmd`** (`std_msgs/Float32`)
  - Steering angle command in **radians**
  - Converted internally to degrees
  - Range: [-max_steering_angle, +max_steering_angle] degrees
  - Positive values = right turn (inverted internally)

- **`/follower/pedal_cmd`** (`std_msgs/Float32`)
  - Throttle/brake command
  - Range: [-1.0, +1.0]
  - `-1.0` = full brake
  - `0.0` = neutral
  - `+1.0` = full throttle


## Command Examples

### Steering Commands

```bash
# Center steering (0 degrees)
ros2 topic pub --once /follower/steering_cmd std_msgs/msg/Float32 "{data: 0.0}"

# Turn right (0.5 radians ≈ 28.65 degrees)
ros2 topic pub --once /follower/steering_cmd std_msgs/msg/Float32 "{data: 0.5}"

# Turn left (-0.5 radians)
ros2 topic pub --once /follower/steering_cmd std_msgs/msg/Float32 "{data: -0.5}"

# Maximum right turn (assuming 20° clip)
ros2 topic pub --once /follower/steering_cmd std_msgs/msg/Float32 "{data: 0.349}"  # 20° in radians
```

### Throttle/Brake Commands

```bash
# Neutral (coast)
ros2 topic pub --once /follower/acceleration_cmd std_msgs/msg/Float32 "{data: 0.0}"

# Half throttle
ros2 topic pub --once /follower/acceleration_cmd std_msgs/msg/Float32 "{data: 0.5}"

# Full throttle
ros2 topic pub --once /follower/acceleration_cmd std_msgs/msg/Float32 "{data: 1.0}"

# Half brake
ros2 topic pub --once /follower/acceleration_cmd std_msgs/msg/Float32 "{data: -0.5}"

# Full brake
ros2 topic pub --once /follower/acceleration_cmd std_msgs/msg/Float32 "{data: -1.0}"
```

## Safety Features

### Multi-Layer Failsafe System

The system implements a comprehensive failsafe architecture with three independent monitoring systems:

#### 1. Throttle Timeout Protection

If no throttle/acceleration command is received within the `throttle_timeout` period (default 0.5s), the throttle/brake is automatically set to zero (neutral). This prevents unintended acceleration due to communication loss.

**Behavior:**
- First timeout event: Throttle set to 0.0, warning logged
- Subsequent timeouts: No action (flag prevents redundant writes)
- Recovery: Flag resets when new throttle command arrives

```bash
# Monitor throttle timeout warnings
ros2 run lj_handler_pkg lj_handler --ros-args --log-level warn
```

#### 2. Steering Timeout Protection

If no steering command is received within the `pose_timeout` period (default 0.5s), the system stops acceleration (sets throttle to 0.0) but **does not modify steering**. This prevents blindly changing trajectory while already in motion.

**Rationale:** If steering communication fails, changing the steering angle without knowing the current vehicle state could make the situation worse. Stopping acceleration is the safer choice.

**Behavior:**
- First timeout event: Throttle set to 0.0, warning logged
- Steering remains at last known position
- Recovery: Flag resets when new steering command arrives

#### 3. LabJack Health Monitoring with Escalation

The system continuously monitors LabJack communication health. If the device becomes unresponsive, it escalates through two stages:

**Error Thresholds:**
- **0-2 consecutive errors**: Healthy state, normal operation
- **3-9 consecutive errors** (WARN threshold): Emergency stop attempted
  - Logs error message with error count
  - Attempts to write throttle = 0.0 via `set_throttle_brake(0.0)`
  - If write succeeds, error counter resets to 0
  - If write fails, error counter increments toward critical
- **10+ consecutive errors** (CRITICAL threshold): Device assumed offline
  - Logs final error message
  - Immediately shuts down node with `rclcpp::shutdown()`
  - Prevents further attempts to communicate with device

**Error Counter Reset:**
- Resets to 0 on successful LJM read/write operations
- Read operation failure: `read_nominal_voltages()` - increments counter
- Write operation failure: `set_control_axis()` (DAC writes) - increments counter

**Example Timeline:**
```
Error 1: Normal operation, counter = 1
Error 2: Normal operation, counter = 2
Error 3: WARNING logged, attempt emergency stop, counter = 3
Success: Counter resets to 0
Error 4-9: Each escalates warning, recovery attempts
Error 10: CRITICAL, node shutdown
```

### Throttle-to-Brake Transition Safety

When transitioning from braking (negative value) to acceleration (positive value), the system enforces a configurable neutral period to prevent drivetrain shock.

**Behavior:**
1. User is braking (`throttle < 0`)
2. User commands acceleration (`throttle > 0`)
3. System detects transition and starts neutral period
4. For `brake_to_throttle_delay` seconds (default 0.3s): forces neutral (`throttle = 0.0`)
5. Additional delay based on brake intensity: `delay + abs(brake_value)/2`
6. After total delay, acceleration command is applied

**Configuration:**
```bash
ros2 launch lj_handler_pkg lj_handler.launch.py brake_to_throttle_delay:=0.5
```

### Voltage Clamping

All output voltages are clamped to configured min/max percentages for safety reasons. This prevents accidental over-voltage or under-voltage conditions on the actuators.

## Hardware Setup

### LabJack T7 Connections

#### Analog Inputs (Reading nominal voltages)
- **AIN10**: Steering Master nominal voltage
- **AIN12**: Steering Slave nominal voltage
- **AIN6**: Throttle/Brake Master nominal voltage
- **AIN8**: Throttle/Brake Slave nominal voltage

#### Analog Outputs (DAC control)
The package uses 4 DAC channels per axis (8 total):

**Steering DACs:**
- DAC0, DAC1: Master steering control (opposition mode)
- FIO4, FIO5: Slave steering control (opposition mode)

**Throttle/Brake DACs:**
- FIO6, FIO7: Master throttle/brake control (opposition mode)
- FIO0, FIO1: Slave throttle/brake control (opposition mode)

### Wiring Diagram

```
LabJack T7
├── Steering System
│   ├── AIN10 ← Master nominal voltage sensor
│   ├── AIN12 ← Slave nominal voltage sensor
│   ├── DAC0 → Master control signal 1
│   ├── DAC1 → Master control signal 2
│   ├── FIO4 → Slave control signal 1
│   └── FIO5 → Slave control signal 2
│
└── Throttle/Brake System
    ├── AIN6 ← Master nominal voltage sensor
    ├── AIN8 ← Slave nominal voltage sensor
    ├── FIO6 → Master control signal 1
    ├── FIO7 → Master control signal 2
    ├── FIO0 → Slave control signal 1
    └── FIO1 → Slave control signal 2
```

## Troubleshooting

### Node fails to start

```bash
# Check LabJack connection
ljm_stream_test
ljm_list_all

# Verify LabJack permissions
sudo usermod -a -G dialout $USER
# Log out and log back in
```

### Steering not responding

```bash
# Check topic
ros2 topic echo /follower/steering_cmd

# Verify steering is within limits
ros2 param get /lj_handler max_steering_angle
ros2 param get /lj_handler steering_clip

# Enable debug logging
ros2 run lj_handler_pkg lj_handler --ros-args --log-level debug
```

### Throttle stuck at zero

```bash
# Check if timeout is triggered
ros2 topic hz /follower/acceleration_cmd  # Should be > 2 Hz (for 0.5s timeout)

# Check brake-to-throttle transition
ros2 run lj_handler_pkg lj_handler --ros-args --log-level info
# Look for "Brake-to-throttle transition detected" message
```

### "Failed to read nominal voltages"

- Check analog input connections (AIN6, AIN8, AIN10, AIN12)
- Verify voltage sensors are powered
- Check LabJack ground connections

## Development

### Building in Debug Mode

```bash
colcon build --packages-select lj_handler_pkg --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

### Code Structure

- `src/lj_handler.cpp` - Main node implementation
- `include/lj_handler_pkg/lj_handler.hpp` - Class declarations
- `launch/` - Launch file configurations

**Note on Testing:** This package currently does not include unit tests. Comprehensive failsafe logic is implemented and validated through real-world testing. Integration tests with actual LabJack hardware are recommended for future development.

## Vehicle Specifications

The package is configured for a **Melex vehicle** with the following characteristics:

- **Wheelbase**: 2555 mm
- **Track width**: 4500 mm
- **Maximum steering angle**: ~29.74° (calculated from geometry)
- **Default steering clip**: 20° (for safety)

To adjust for different vehicles:

```bash
ros2 launch lj_handler_pkg lj_handler.launch.py \
  max_steering_angle:=<your_max_angle> \
  steering_clip:=<your_safe_clip>
```

## License

Apache License 2.0

## See Also

- [LabJack T7 Documentation](https://labjack.com/pages/support?doc=/datasheets/t7-datasheet/)
- [LabJack LJM Library](https://labjack.com/ljm)
- ROS 2 Humble Documentation

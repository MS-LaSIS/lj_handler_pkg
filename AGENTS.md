# AGENTS.md - lj_handler_pkg

Instructions for AI coding agents working on this ROS 2 C++ package.

## Project Overview

ROS 2 package (ament_cmake) for controlling vehicle steering and throttle/brake using a LabJack T7 device. Target platform: ROS 2 Humble on Ubuntu 22.04.

## Docker Environment

**IMPORTANT**: ROS 2 and colcon are only available inside the `ros2_prova` Docker container.

```bash
# Start the Docker container (from host)
docker container run -it --rm --privileged \
  -v /dev/:/dev/ \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device /dev/video0 \
  --device /dev/video1 \
  -v ~/Documenti/Progetti/ros2:/home/emanuele/Documenti/Progetti/ros2 \
  --net=host \
  --name="prova" \
  --hostname="docker" \
  ros2_prova

# The workspace is mounted at: /home/emanuele/Documenti/Progetti/ros2/ros2_ws
```

**If the Docker image is not available**, the Dockerfile can be downloaded from:
https://github.com/emanuelenencioni/docker_images (inside `ros_humble_desktop_full_cpu` directory)

## LabJack Library Setup (User Action Required)

**IMPORTANT FOR AGENTS**: The LabJack LJM library cannot be pre-installed in the Dockerfile and must be installed manually by the user inside the running container. If the build fails with `LabJackM.h: No such file or directory`, prompt the user to install the library.

**Instructions for the user** (inside the Docker container):
Download the LabJack LJM installer from https://labjack.com/pages/support?doc=/software-driver/installer-downloads/ljm-software-installers-t4-t7-digit/ and run it with sudo:
```bash
sudo ./labjack_ljm_installer.run
```

## Build Commands (inside Docker)

```bash
# Navigate to workspace
cd /home/emanuele/Documenti/Progetti/ros2/ros2_ws

# Build the package
colcon build --packages-select lj_handler_pkg

# Build with debug symbols
colcon build --packages-select lj_handler_pkg --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Source the workspace after building
source install/setup.bash
```

## Test Commands (inside Docker)

```bash
# Run all tests (linting only - no unit tests currently)
colcon test --packages-select lj_handler_pkg

# View test results
colcon test-result --verbose

# Run specific linter
colcon test --packages-select lj_handler_pkg --ctest-args -R cpplint
colcon test --packages-select lj_handler_pkg --ctest-args -R cppcheck
```

## Launch Commands (inside Docker)

```bash
# Standard launch (leader topics)
ros2 launch lj_handler_pkg lj_handler.launch.py

# Follow mode launch
ros2 launch lj_handler_pkg lj_handler_follow.launch.py

# Testing without GuidoSimplex hardware
ros2 launch lj_handler_pkg lj_handler_no_gs_attached.launch.py

# Debug mode (verbose logging)
DEBUG=1 ros2 launch lj_handler_pkg lj_handler.launch.py

# Run node directly
ros2 run lj_handler_pkg lj_handler_pkg
```

## Directory Structure

```
lj_handler_pkg/
├── CMakeLists.txt              # Build configuration (ament_cmake)
├── package.xml                 # ROS 2 package manifest (format 3)
├── README.md                   # User documentation
├── AGENTS.md                   # This file
├── include/lj_handler_pkg/
│   └── lj_handler.hpp          # Class declarations
├── launch/
│   ├── lj_handler.launch.py    # Main launch file
│   ├── lj_handler_follow.launch.py
│   └── lj_handler_no_gs_attached.launch.py
└── src/
    └── lj_handler.cpp          # Node implementation
```

## Code Style Guidelines

### C++ Standards
- **C++ Standard**: C++17 (required via CMake)
- **C Standard**: C99
- **Compiler warnings**: `-Wall -Wextra -Wpedantic` enabled

### Naming Conventions
- **Classes**: PascalCase (e.g., `LJHandlerNode`)
- **Methods**: snake_case (e.g., `set_steering_ratio`, `check_safety_timeout`)
- **Member variables**: snake_case with trailing underscore (e.g., `handle_`, `steering_sub_`)
- **Parameters**: snake_case (e.g., `steering_min_perc`, `throttle_timeout`)
- **Constants**: SCREAMING_SNAKE_CASE (e.g., `INITIAL_ERR_ADDRESS`)
- **Local variables**: snake_case (e.g., `steering_ratio`, `throttle_value`)

### File Naming
- **Headers**: `<package_name>/<node_name>.hpp` in `include/`
- **Sources**: `<node_name>.cpp` in `src/`
- **Launch files**: `<descriptive_name>.launch.py` in `launch/`

### Header Guards
Use the ROS 2 recommended format:
```cpp
#ifndef LJ_HANDLER_PKG__LJ_HANDLER_HPP_
#define LJ_HANDLER_PKG__LJ_HANDLER_HPP_
// ...
#endif // LJ_HANDLER_PKG__LJ_HANDLER_HPP_
```

### Includes Order
1. Package headers (`#include "lj_handler_pkg/lj_handler.hpp"`)
2. Standard library (`#include <cmath>`, `#include <string>`)
3. ROS 2 headers (`#include <rclcpp/rclcpp.hpp>`)
4. Message headers (`#include <std_msgs/msg/float32.hpp>`)
5. External libraries (`#include <LabJackM.h>`)

### Type Usage
- Use `double` for floating-point values (not `float`)
- Use `std::string` for string parameters
- Use `std::vector<std::string>` for collections
- Use `SharedPtr` for message callbacks: `const std_msgs::msg::Float32::SharedPtr msg`
- Use `rclcpp::Time` for timestamps

### ROS 2 Patterns

**Parameter Declaration:**
```cpp
this->declare_parameter<double>("param_name", default_value);
double value = this->get_parameter("param_name").as_double();
```

**Subscriptions:**
```cpp
subscription_ = this->create_subscription<MsgType>(
    topic_name, qos_depth,
    std::bind(&ClassName::callback, this, std::placeholders::_1));
```

**Timers:**
```cpp
timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_seconds),
    std::bind(&ClassName::timer_callback, this));
```

**Logging:**
```cpp
RCLCPP_INFO(this->get_logger(), "Message: %s", value.c_str());
RCLCPP_DEBUG(this->get_logger(), "Debug: %.2f", value);
RCLCPP_WARN(this->get_logger(), "Warning message");
RCLCPP_ERROR(this->get_logger(), "Error: %s", error_name);
RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Throttled");
```

### Error Handling
- Check LabJack error codes and log descriptive messages
- Use early returns on error conditions
- Call `rclcpp::shutdown()` on critical initialization failures
- Clean up resources in destructors

```cpp
int err = LJM_Open(...);
if (err != LJME_NOERROR) {
    char errName[LJM_MAX_NAME_SIZE];
    LJM_ErrorToString(err, errName);
    RCLCPP_ERROR(this->get_logger(), "Failed: %s", errName);
    rclcpp::shutdown();
    return;
}
```

### Value Clamping
Always clamp input values to valid ranges:
```cpp
value = std::max(min_val, std::min(max_val, value));
```

## Dependencies

### ROS 2 Packages
- `rclcpp` - C++ client library
- `std_msgs` - Standard messages (Float32)
- `geometry_msgs` - Geometry messages
- `tf2`, `tf2_ros` - Transform library
- `apriltag_msgs` - AprilTag messages

### External Libraries
- `LabJackM` - LabJack Modbus library (linked in CMakeLists.txt)

### Test Dependencies
- `ament_lint_auto` - Automatic linting
- `ament_lint_common` - Common linters (cpplint, cppcheck, etc.)

## Launch File Patterns

Launch files use Python and follow this structure:
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    arg = DeclareLaunchArgument('name', default_value='value', description='...')
    
    # Create node
    node = Node(
        package='lj_handler_pkg',
        executable='lj_handler_pkg',
        name='lj_handler_node',
        output='screen',
        parameters=[{'param': LaunchConfiguration('name')}],
    )
    
    return LaunchDescription([arg, node])
```

## Important Notes

- Copyright check is disabled in CMakeLists.txt (`ament_cmake_copyright_FOUND TRUE`)
- No dedicated unit tests exist; testing relies on ament linters
- The node requires a LabJack T7 device connected via USB
- Safety timeout automatically zeros throttle if no commands received

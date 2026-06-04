"""
Combined launch: lj_handler_node (actuator control) + rc_input_node (RC PWM reader).

lj_handler_node parameters are forwarded to the existing lj_handler.launch.py so
they stay in sync automatically. rc_input_node parameters are declared here.

Usage:
  ros2 launch lj_handler_pkg lj_handler_with_rc.launch.py

Override any parameter at launch time, e.g.:
  ros2 launch lj_handler_pkg lj_handler_with_rc.launch.py \\
      normal_run_throttle_perc:=50.0 stop_brake_perc:=80.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('lj_handler_pkg')

    # =========================================================================
    # rc_input_node parameters
    # =========================================================================

    steering_pin_arg = DeclareLaunchArgument(
        'steering_pin', default_value='DIO0',
        description='LabJack T7 DIO prefix for steering PWM (FIO0)'
    )
    button_pin_arg = DeclareLaunchArgument(
        'button_pin', default_value='DIO1',
        description='LabJack T7 DIO prefix for Button B PWM (FIO1)'
    )
    pwm_read_rate_hz_arg = DeclareLaunchArgument(
        'pwm_read_rate_hz', default_value='50.0',
        description='PWM polling frequency (Hz)'
    )
    steering_max_ticks_arg = DeclareLaunchArgument(
        'steering_max_ticks', default_value='24000.0',
        description='Tick range = ±100 % for steering channel'
    )
    button_max_ticks_arg = DeclareLaunchArgument(
        'button_max_ticks', default_value='30000.0',
        description='Tick range = ±100 % for button channel'
    )
    button_threshold_percent_arg = DeclareLaunchArgument(
        'button_threshold_percent', default_value='50.0',
        description='|button_raw| above this (%) = pressed'
    )
    button_debounce_ms_arg = DeclareLaunchArgument(
        'button_debounce_ms', default_value='200.0',
        description='Min ms between accepted Button B press events'
    )
    pwm_timeout_ms_arg = DeclareLaunchArgument(
        'pwm_timeout_ms', default_value='500.0',
        description='Signal-loss timeout (ms) before safe fallback'
    )
    normal_run_throttle_perc_arg = DeclareLaunchArgument(
        'normal_run_throttle_perc', default_value='66.0',
        description='Throttle in NORMAL_RUN state (%)'
    )
    slow_run_throttle_perc_arg = DeclareLaunchArgument(
        'slow_run_throttle_perc', default_value='30.0',
        description='Target throttle at end of SLOW_RUN ramp (%)'
    )
    slow_run_ramp_time_s_arg = DeclareLaunchArgument(
        'slow_run_ramp_time_s', default_value='1.0',
        description='Duration of NORMAL->SLOW throttle ramp (s)'
    )
    stop_brake_perc_arg = DeclareLaunchArgument(
        'stop_brake_perc', default_value='60.0',
        description='Brake force in STOP state (%)'
    )
    stop_brake_ramp_time_s_arg = DeclareLaunchArgument(
        'stop_brake_ramp_time_s', default_value='0.0',
        description='0 = instant brake; >0 = ramp to full brake over N seconds'
    )
    rc_log_level_arg = DeclareLaunchArgument(
        'rc_log_level', default_value='info',
        description='Log level for rc_input_node'
    )

    # =========================================================================
    # rc_input_node
    # Publishes to /follower/steering_cmd and /follower/pedal_cmd, which is
    # exactly what lj_handler_node subscribes to in its default configuration.
    # =========================================================================

    rc_input_node = Node(
        package='lj_handler_pkg',
        executable='rc_input_node',
        name='rc_input_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'steering_pin':             LaunchConfiguration('steering_pin'),
            'button_pin':               LaunchConfiguration('button_pin'),
            'pwm_read_rate_hz':         LaunchConfiguration('pwm_read_rate_hz'),
            'steering_max_ticks':       LaunchConfiguration('steering_max_ticks'),
            'button_max_ticks':         LaunchConfiguration('button_max_ticks'),
            'button_threshold_percent': LaunchConfiguration('button_threshold_percent'),
            'button_debounce_ms':       LaunchConfiguration('button_debounce_ms'),
            'pwm_timeout_ms':           LaunchConfiguration('pwm_timeout_ms'),
            'normal_run_throttle_perc': LaunchConfiguration('normal_run_throttle_perc'),
            'slow_run_throttle_perc':   LaunchConfiguration('slow_run_throttle_perc'),
            'slow_run_ramp_time_s':     LaunchConfiguration('slow_run_ramp_time_s'),
            'stop_brake_perc':          LaunchConfiguration('stop_brake_perc'),
            'stop_brake_ramp_time_s':   LaunchConfiguration('stop_brake_ramp_time_s'),
            # Topics must match lj_handler_node defaults
            'steering_topic': '/follower/steering_cmd',
            'throttle_topic': '/follower/pedal_cmd',
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('rc_log_level')],
    )

    # =========================================================================
    # lj_handler_node  — included via its own launch file so all its defaults
    # and parameters are inherited without duplication.
    # =========================================================================

    lj_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lj_handler.launch.py')
        )
        # No launch_arguments here: lj_handler.launch.py keeps all its defaults.
        # Pass lj_handler-specific overrides directly, e.g.:
        #   ros2 launch ... safety_voltage_threshold:=0.25
        # Those will be forwarded automatically through the IncludeLaunchDescription.
    )

    return LaunchDescription([
        # rc_input_node args
        steering_pin_arg,
        button_pin_arg,
        pwm_read_rate_hz_arg,
        steering_max_ticks_arg,
        button_max_ticks_arg,
        button_threshold_percent_arg,
        button_debounce_ms_arg,
        pwm_timeout_ms_arg,
        normal_run_throttle_perc_arg,
        slow_run_throttle_perc_arg,
        slow_run_ramp_time_s_arg,
        stop_brake_perc_arg,
        stop_brake_ramp_time_s_arg,
        rc_log_level_arg,
        # Nodes
        rc_input_node,
        lj_handler_launch,
    ])

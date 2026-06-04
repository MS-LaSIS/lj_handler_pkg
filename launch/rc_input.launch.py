from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---- DIO pin assignments ------------------------------------------------
    steering_pin_arg = DeclareLaunchArgument(
        'steering_pin',
        default_value='DIO0',
        description='LabJack T7 DIO register prefix for the steering PWM channel (FIO0)'
    )

    button_pin_arg = DeclareLaunchArgument(
        'button_pin',
        default_value='DIO1',
        description='LabJack T7 DIO register prefix for Button B PWM channel (FIO1)'
    )

    # ---- PWM reading --------------------------------------------------------
    pwm_read_rate_hz_arg = DeclareLaunchArgument(
        'pwm_read_rate_hz',
        default_value='50.0',
        description='Timer polling frequency for PWM reads (Hz)'
    )

    steering_max_ticks_arg = DeclareLaunchArgument(
        'steering_max_ticks',
        default_value='24000.0',
        description='Tick range that maps to ±100 % for the steering channel'
    )

    button_max_ticks_arg = DeclareLaunchArgument(
        'button_max_ticks',
        default_value='30000.0',
        description='Tick range that maps to ±100 % for the button channel'
    )

    button_threshold_percent_arg = DeclareLaunchArgument(
        'button_threshold_percent',
        default_value='50.0',
        description='|button_raw| above this value (%) is considered a press'
    )

    button_debounce_ms_arg = DeclareLaunchArgument(
        'button_debounce_ms',
        default_value='200.0',
        description='Minimum time between accepted Button B press events (ms)'
    )

    pwm_timeout_ms_arg = DeclareLaunchArgument(
        'pwm_timeout_ms',
        default_value='500.0',
        description='Signal-loss timeout before safe fallback commands are published (ms)'
    )

    # ---- State machine ------------------------------------------------------
    normal_run_throttle_perc_arg = DeclareLaunchArgument(
        'normal_run_throttle_perc',
        default_value='66.0',
        description='Throttle command in NORMAL_RUN state (%)'
    )

    slow_run_throttle_perc_arg = DeclareLaunchArgument(
        'slow_run_throttle_perc',
        default_value='30.0',
        description='Target throttle command at end of SLOW_RUN ramp (%)'
    )

    slow_run_ramp_time_s_arg = DeclareLaunchArgument(
        'slow_run_ramp_time_s',
        default_value='1.0',
        description='Duration of the NORMAL_RUN -> SLOW_RUN throttle ramp (s)'
    )

    stop_brake_perc_arg = DeclareLaunchArgument(
        'stop_brake_perc',
        default_value='60.0',
        description='Brake force applied in STOP state (%). Maps to negative pedal_cmd.'
    )

    stop_brake_ramp_time_s_arg = DeclareLaunchArgument(
        'stop_brake_ramp_time_s',
        default_value='0.0',
        description='0 = instant full brake; >0 = ramp to full brake over this many seconds'
    )

    # ---- Output topics ------------------------------------------------------
    steering_topic_arg = DeclareLaunchArgument(
        'steering_topic',
        default_value='/follower/steering_cmd',
        description='Topic name for steering commands (std_msgs/Float32, ratio -1..+1)'
    )

    throttle_topic_arg = DeclareLaunchArgument(
        'throttle_topic',
        default_value='/follower/pedal_cmd',
        description='Topic name for throttle/brake commands (std_msgs/Float32, -1..+1)'
    )

    # ---- Log level ----------------------------------------------------------
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS log level (debug, info, warn, error, fatal)'
    )

    # ---- Node ---------------------------------------------------------------
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
            'steering_topic':           LaunchConfiguration('steering_topic'),
            'throttle_topic':           LaunchConfiguration('throttle_topic'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
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
        steering_topic_arg,
        throttle_topic_arg,
        log_level_arg,
        rc_input_node,
    ])

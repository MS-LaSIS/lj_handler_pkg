from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


DEBUG = os.getenv('DEBUG', default='False').lower() == 'true' or os.getenv('DEBUG', default='False').lower() == '1'


def generate_launch_description():
    # Declare launch arguments for pin assignments
    nominal_vs_steer_M_pin_arg = DeclareLaunchArgument(
        'nominal_vs_steer_M_pin',
        default_value='AIN10',
        description='LabJack analog input pin for steering master nominal voltage'
    )
    
    nominal_vs_steer_S_pin_arg = DeclareLaunchArgument(
        'nominal_vs_steer_S_pin',
        default_value='AIN12',
        description='LabJack analog input pin for steering slave nominal voltage'
    )
    
    nominal_vs_accbrake_M_pin_arg = DeclareLaunchArgument(
        'nominal_vs_accbrake_M_pin',
        default_value='AIN6',
        description='LabJack analog input pin for throttle/brake master nominal voltage'
    )
    
    nominal_vs_accbrake_S_pin_arg = DeclareLaunchArgument(
        'nominal_vs_accbrake_S_pin',
        default_value='AIN8',
        description='LabJack analog input pin for throttle/brake slave nominal voltage'
    )
    
    # Declare voltage range parameters
    input_voltage_min_arg = DeclareLaunchArgument(
        'input_voltage_min',
        default_value='0.1',
        description='Minimum input voltage for control signals (V)'
    )
    
    input_voltage_max_arg = DeclareLaunchArgument(
        'input_voltage_max',
        default_value='5.26',
        description='Maximum input voltage for control signals (V)'
    )
    
    # Declare percentage limits for steering
    steering_min_perc_arg = DeclareLaunchArgument(
        'steering_min_perc',
        default_value='0.0',
        description='Minimum percentage of nominal voltage for steering output'
    )
    
    steering_max_perc_arg = DeclareLaunchArgument(
        'steering_max_perc',
        default_value='1.0',
        description='Maximum percentage of nominal voltage for steering output'
    )
    
    # Declare percentage limits for throttle/brake
    throttle_min_perc_arg = DeclareLaunchArgument(
        'throttle_min_perc',
        default_value='0.23',
        description='Minimum percentage of nominal voltage for throttle/brake output'
    )
    
    throttle_max_perc_arg = DeclareLaunchArgument(
        'throttle_max_perc',
        default_value='0.77',
        description='Maximum percentage of nominal voltage for throttle/brake output'
    )
    
    # Declare steering parameters
    max_steering_angle_arg = DeclareLaunchArgument(
        'max_steering_angle',
        default_value='29.74',
        description='Maximum steering angle in degrees (calculated from vehicle geometry)'
    )

    steering_clip_arg = DeclareLaunchArgument(
        'steering_clip',
        default_value='20.0',
        description='Steering clip factor in degrees'
    )
    
    # Declare timeout parameters
    pose_timeout_arg = DeclareLaunchArgument(
        'pose_timeout',
        default_value='0.5',
        description='Timeout for pose/steering messages in seconds'
    )
    
    throttle_timeout_arg = DeclareLaunchArgument(
        'throttle_timeout',
        default_value='0.5',
        description='Timeout for throttle messages in seconds (returns to zero on timeout)'
    )
    
    safety_check_period_arg = DeclareLaunchArgument(
        'safety_check_period',
        default_value='0.1',
        description='Period for safety timeout checks in seconds'
    )
    
    # Declare topic parameters
    steering_topic_arg = DeclareLaunchArgument(
        'steering_topic',
        default_value='/delay/steering_cmd',
        description='Topic for steering commands (Float32 in radians)'
    )
    
    throttle_topic_arg = DeclareLaunchArgument(
        'throttle_topic',
        default_value='/leader/pedal_cmd',
        description='Topic for throttle/brake commands (Float32, -1.0 to 1.0)'
    )
    # Declare log level
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error, fatal)'
    )
    
    
    # Determine log level from DEBUG environment variable or launch argument
    log_level = 'debug' if DEBUG else 'info'
    
    # Create the lj_handler_node
    lj_handler_node = Node(
        package='lj_handler_pkg',
        executable='lj_handler_pkg',
        name='lj_handler_node',
        output='screen',
        parameters=[{
            'nominal_vs_steer_M_pin': LaunchConfiguration('nominal_vs_steer_M_pin'),
            'nominal_vs_steer_S_pin': LaunchConfiguration('nominal_vs_steer_S_pin'),
            'nominal_vs_accbrake_M_pin': LaunchConfiguration('nominal_vs_accbrake_M_pin'),
            'nominal_vs_accbrake_S_pin': LaunchConfiguration('nominal_vs_accbrake_S_pin'),
            'input_voltage_min': LaunchConfiguration('input_voltage_min'),
            'input_voltage_max': LaunchConfiguration('input_voltage_max'),
            'steering_min_perc': LaunchConfiguration('steering_min_perc'),
            'steering_max_perc': LaunchConfiguration('steering_max_perc'),
            'throttle_min_perc': LaunchConfiguration('throttle_min_perc'),
            'throttle_max_perc': LaunchConfiguration('throttle_max_perc'),
            'max_steering_angle': LaunchConfiguration('max_steering_angle'),
            'steering_clip': LaunchConfiguration('steering_clip'),
            'pose_timeout': LaunchConfiguration('pose_timeout'),
            'throttle_timeout': LaunchConfiguration('throttle_timeout'),
            'safety_check_period': LaunchConfiguration('safety_check_period'),
            'steering_topic': LaunchConfiguration('steering_topic'),
            'throttle_topic': LaunchConfiguration('throttle_topic'),
        }],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Pin assignments
        nominal_vs_steer_M_pin_arg,
        nominal_vs_steer_S_pin_arg,
        nominal_vs_accbrake_M_pin_arg,
        nominal_vs_accbrake_S_pin_arg,
        # Voltage ranges
        input_voltage_min_arg,
        input_voltage_max_arg,
        # Steering limits
        steering_min_perc_arg,
        steering_max_perc_arg,
        # Throttle limits
        throttle_min_perc_arg,
        throttle_max_perc_arg,
        # Steering parameters
        max_steering_angle_arg,
        steering_clip_arg,
        # Timeouts
        pose_timeout_arg,
        throttle_timeout_arg,
        safety_check_period_arg,
        # Topics
        steering_topic_arg,
        throttle_topic_arg,
        # Logging
        log_level_arg,
        # Node
        lj_handler_node,
    ])

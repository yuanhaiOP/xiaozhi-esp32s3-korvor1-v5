from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port name for ESP32 communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    # 位置控制参数
    position_tolerance_arg = DeclareLaunchArgument(
        'position_tolerance',
        default_value='0.1',
        description='Position tolerance in meters for target reaching'
    )
    
    angle_tolerance_arg = DeclareLaunchArgument(
        'angle_tolerance',
        default_value='0.1',
        description='Angle tolerance in radians for target reaching'
    )
    
    linear_p_gain_arg = DeclareLaunchArgument(
        'linear_p_gain',
        default_value='2.0',
        description='Linear velocity P gain for position control'
    )
    
    angular_p_gain_arg = DeclareLaunchArgument(
        'angular_p_gain',
        default_value='3.0',
        description='Angular velocity P gain for position control'
    )
    
    max_linear_accel_arg = DeclareLaunchArgument(
        'max_linear_accel',
        default_value='1.0',
        description='Maximum linear acceleration in m/s²'
    )
    
    max_angular_accel_arg = DeclareLaunchArgument(
        'max_angular_accel',
        default_value='2.0',
        description='Maximum angular acceleration in rad/s²'
    )
    
    # 创建 ESP32 桥接节点
    esp32_bridge_node = Node(
        package='esp32_robot_bridge',
        executable='esp32_robot_bridge',
        name='esp32_robot_bridge',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'position_tolerance': LaunchConfiguration('position_tolerance'),
            'angle_tolerance': LaunchConfiguration('angle_tolerance'),
            'linear_p_gain': LaunchConfiguration('linear_p_gain'),
            'angular_p_gain': LaunchConfiguration('angular_p_gain'),
            'max_linear_accel': LaunchConfiguration('max_linear_accel'),
            'max_angular_accel': LaunchConfiguration('max_angular_accel'),
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        position_tolerance_arg,
        angle_tolerance_arg,
        linear_p_gain_arg,
        angular_p_gain_arg,
        max_linear_accel_arg,
        max_angular_accel_arg,
        esp32_bridge_node,
    ]) 
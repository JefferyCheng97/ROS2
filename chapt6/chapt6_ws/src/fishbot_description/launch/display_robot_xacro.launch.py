import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros.parameter_descriptions

def generate_launch_description():
    # 获取功能包安装目录
    urdf_package_path = get_package_share_directory("fishbot_description")
    # 拼接出first_robot.urdf路径
    default_urdf_path = os.path.join(urdf_package_path,
        'urdf',
        'first_robot/first_robot.urdf'
    )

    # 拼接出保存的RViz配置文件，用于自动设置配置
    default_rviz_config_path = os.path.join(urdf_package_path,
        'config',
        'display_robot_model.rviz'
    )

    # 声明一个名为model的启动参数
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name = 'model', 
        default_value = str(default_urdf_path),
        description = '加载的模型文件路径'
    )

    # 获取URDF文件内容作为参数值。获取名为model的启动参数，Command会执行xacro命令并将LaunchConfiguration获取的内容作为参数，
    # 通过value_type=str指定这个参数值的类型是字符串
    robot_description_value = launch_ros.parameter_descriptions.ParameterValue(launch.substitutions.Command([
        'xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type = str
    )
    
    # 状态发布节点，并将参数值付给=左边
    action_robot_state_publisher = launch_ros.actions.Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters = [{'robot_description':robot_description_value}]
    )
    
    # 动态joint节点
    action_joint_state_publisher = launch_ros.actions.Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
    )

    # rviz启动节点+配置文件
    action_rviz_node = launch_ros.actions.Node(
        package = 'rviz2',
        executable = 'rviz2',
        arguments = ['-d', default_rviz_config_path]
    )

    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        action_robot_state_publisher,
        action_joint_state_publisher,
        action_rviz_node
    ])
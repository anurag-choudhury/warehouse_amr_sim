from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import os

def generate_launch_description():
    pkg_mr_robot_nav=get_package_share_directory("mr_robot_nav")
    pkg_nav2_bringup=get_package_share_directory("nav2_bringup")
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
	# with_rviz = LaunchConfiguration('with_rviz', default='true')
    map_yaml_file = LaunchConfiguration('map',
                default=os.path.join(pkg_mr_robot_nav, 'maps', 'warehouse_dummy_map.yaml'))
    nav2_config_file = LaunchConfiguration('params', 
                    default=os.path.join(pkg_mr_robot_nav, 'config', 'nav2_params.yaml'))
    
    nav2_bringup_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup,"launch","bringup_launch.py")),
        launch_arguments={
            "map":map_yaml_file,
            "use_sim_time":use_sim_time,
            "params_file":nav2_config_file
        }.items()
    )
    return LaunchDescription([
        DeclareLaunchArgument("map",default_value=map_yaml_file,description="map.yaml file"),
        DeclareLaunchArgument("params_file",default_value=nav2_config_file,description="params for nav2"),
        DeclareLaunchArgument("use_sim_time",default_value=use_sim_time,description="sim_time true or false"),
        nav2_bringup_launch_file
    ])
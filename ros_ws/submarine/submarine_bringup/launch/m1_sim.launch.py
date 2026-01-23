from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Package Paths
def get_package_paths():
    sim_pkg = FindPackageShare('submarine_simulation')
    desc_pkg = FindPackageShare('submarine_description')
    return sim_pkg, desc_pkg

# SDF Files
def get_sdf_files(sim_pkg, desc_pkg):
    world_file = PathJoinSubstitution([sim_pkg, 'worlds', 'm1_underwater.sdf'])
    submarine_file = PathJoinSubstitution([desc_pkg, 'models', 'm1_submarine', 'model.sdf'])
    return world_file, submarine_file

# ROS 2 Launch File's Entry Point
def generate_launch_description():
    return
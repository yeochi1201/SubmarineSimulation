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

# Gazebo Resource Path
def set_resource_path(sim_pkg, desc_pkg):
    set_gz_resource_path = SetEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value = [
            sim_pkg, ':', desc_pkg
        ]
    )

    set_ign_resource_path = SetEnvironmentVariable(
        name = 'IGN_GAZEBO_RESOURCE_PATH',
        value = [
            sim_pkg, ':', desc_pkg
        ]
    )
    return set_gz_resource_path, set_ign_resource_path

# Gazebo Execution
def gazebo_execution(world_file):
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'sim.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_file,
            'vervose': 'true'
        }.items()
    )
    return gazebo

#Spawn Submarine
def spawn_submarine(submarine_file):
    submarine = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-name', ',m1_submarine', '-world', 'm1_underwater',
            '-file', submarine_file,
            '-x', '0', '-y', '0', '-z', '10', '-R', '0', '-P', '0', '-Y', '0'
        ],
        output = 'screen'
    )
    return submarine

# ROS 2 Launch File's Entry Point
def generate_launch_description():
    sim_pkg, desc_pkg = get_package_paths()
    world_file, submarine_file = get_sdf_files(sim_pkg, desc_pkg)
    gz_resource_path, ign_resource_path = set_resource_path(sim_pkg, desc_pkg)
    gazebo = gazebo_execution(world_file)
    submarine = spawn_submarine(submarine_file)
    return LaunchDescription([
        gz_resource_path,
        ign_resource_path,
        gazebo,
        submarine
    ])
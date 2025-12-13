import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import  LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='fish_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_fishy_fish_navigation = get_package_share_directory('fishy_fish_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Add custom gazebo models path if environment variable is set
    # Users can set GZ_SIM_CUSTOM_MODELS_PATH environment variable to add their own models
    custom_models_path = os.environ.get('GZ_SIM_CUSTOM_MODELS_PATH', '')
    if custom_models_path:
        os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + custom_models_path


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_fishy_fish_navigation,
            'worlds',
            LaunchConfiguration('world')
        ]),
        TextSubstitution(text=' -r -v -v1')],
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        'on_exit_shutdown': 'true'}.items()
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject
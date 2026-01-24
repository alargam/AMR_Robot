import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # ========================================================================
    # 1. Define Paths to Config Files
    # ========================================================================
    # Get the path to your 'robot_bringup' package
    pkg_bringup = get_package_share_directory('robot_bringup')
    
    # Path to the Map file you saved earlier
    # Make sure 'my_first_map.yaml' exists in the /maps folder!
    map_file = os.path.join(pkg_bringup, 'maps', 'my_first_map.yaml')
    
    # Path to the Optimized Nav2 Parameters file
    # This reads the 'nav2_params.yaml' you just created
    params_file = os.path.join(pkg_bringup, 'config', 'nav2_params.yaml')

    # ========================================================================
    # 2. Include the Standard Nav2 Bringup
    # ========================================================================
    # Instead of defining all nodes (AMCL, Planner, Controller) manually,
    # we use the official launch file provided by the 'nav2_bringup' package.
    # We simply pass our map and params to it.
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py'
        )]),
        launch_arguments={
            'map': map_file,            # Load your map
            'params_file': params_file, # Load your optimized params
            'use_sim_time': 'false',    # Real robot = false
            'autostart': 'true'         # Start navigation immediately
        }.items()
    )

    # ========================================================================
    # 3. Return Launch Description
    # ========================================================================
    return LaunchDescription([
        nav2_bringup
    ])
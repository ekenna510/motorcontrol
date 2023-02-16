#!/usr/bin/env python3
#ros2 launch motorcontrol bigbotdescript.launch.py use_sim_time:=false
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition

import xacro
from launch.actions import OpaqueFunction




def generate_launch_description():

    # get prints to show in log file?
    #https://snorriheim.atlassian.net/wiki/spaces/TechnicalNotes/pages/549486593/Launch
    # ...
    myenv = os.environ
    myenv["PYTHONUNBUFFERED"] = "1"
    # ...




    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    a=(use_sim_time,'a' )
    print("a ",a)
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Use rviz if true')
    b=(use_rviz,'' )
    print("b ",b)


    urdf_file = os.path.join(get_package_share_directory('motorcontrol'), 'urdf', 'robot2.urdf')    
    assert os.path.exists(urdf_file), "The  doesnt exist in "+str(urdf_file)
    print(urdf_file)

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    sdf_file = os.path.join(get_package_share_directory('motorcontrol'), 'urdf', 'robot2.sdf')    
    assert os.path.exists(sdf_file), "The box_bot.xacro doesnt exist in "+str(sdf_file)
    print(sdf_file)

    rviz_file =os.path.join(get_package_share_directory('motorcontrol'), 'rviz', 'odom.rviz')   
    assert os.path.exists(rviz_file), "The rviz doesnt exist in "+str(rviz_file)
    print(rviz_file)

    return LaunchDescription([

    DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

     Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
                {"robot_description": robot_desc},{"publish_frequency":2.0}],
        output="both"),

    Node(
        package="joint_state_publisher", 
        executable="joint_state_publisher" ,
        name= "joint_state_publisher",
        parameters=[        
            {"robot_description": robot_desc},{"publish_frequency":2.0},
            {"use_gui": 'true' }    
                ],
        output="both",
        condition=IfCondition((use_sim_time,''))
        ),
    Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        output='both',
        condition=IfCondition(use_rviz)
        ),
    Node(package='spawnrobot', 
        executable='spawnrobot2', 
        arguments=['0.01','0.02','0.03' ], 
        output='screen',
        condition=IfCondition((use_sim_time,''))
        ),

  ])
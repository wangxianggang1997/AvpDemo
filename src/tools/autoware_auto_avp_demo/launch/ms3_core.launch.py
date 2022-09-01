# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    # 功能包文件前缀变量
    avp_demo_pkg_prefix = get_package_share_directory('autoware_auto_avp_demo')
    # 欧式点云聚类参数文件路径
    euclidean_cluster_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    # 地面分类参数文件路径
    ray_ground_classifier_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ray_ground_classifier.param.yaml')
    # rviz 配置文件
    rviz_cfg_path = os.path.join(avp_demo_pkg_prefix, 'config/ms3.rviz')
    # 点云下采样文件参数路径
    scan_downsampler_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/scan_downsampler_ms3.param.yaml')
    # lanlet2高精度地图发布者参数文件路径
    lanelet2_map_provider_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/lanelet2_map_provider.param.yaml')
    # 路径规划参数文件路径
    lane_planner_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/lane_planner.param.yaml')
    # 泊车规划参数文件路径
    parking_planner_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/parking_planner.param.yaml')
    # 物体碰撞检测参数文件路径
    object_collision_estimator_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/object_collision_estimator.param.yaml')
    # 行为规划参数文件
    behavior_planner_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/behavior_planner.param.yaml')
    # 地图外障碍物过滤器参数文件
    off_map_obstacles_filter_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/off_map_obstacles_filter.param.yaml')
    # 点云融合节点路径
    point_cloud_fusion_node_pkg_prefix = get_package_share_directory(
        'point_cloud_fusion_nodes')
    # avp web接口
    avp_web_interface_pkg_prefix = get_package_share_directory(
        'avp_web_interface')
    # web 源文件
    web_files_root = os.path.join(avp_web_interface_pkg_prefix, 'web')

    # Arguments
    # 参数声明
    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )
    with_obstacles_param = DeclareLaunchArgument(
        'with_obstacles',
        default_value='True',
        description='Enable obstacle detection'
    )
    scan_downsampler_param = DeclareLaunchArgument(
        'scan_downsampler_param_file',
        default_value=scan_downsampler_param_file,
        description='Path to config file for lidar scan downsampler'
    )
    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=lanelet2_map_provider_param_file,
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=lane_planner_param_file,
        description='Path to parameter file for lane planner'
    )
    parking_planner_param = DeclareLaunchArgument(
        'parking_planner_param_file',
        default_value=parking_planner_param_file,
        description='Path to parameter file for parking planner'
    )
    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=object_collision_estimator_param_file,
        description='Path to parameter file for object collision estimator'
    )
    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=behavior_planner_param_file,
        description='Path to parameter file for behavior planner'
    )
    off_map_obstacles_filter_param = DeclareLaunchArgument(
        'off_map_obstacles_filter_param_file',
        default_value=off_map_obstacles_filter_param_file,
        description='Path to parameter file for off-map obstacle filter'
    )

    # Nodes
    # 启动的节点
    # 节点1：欧式聚类
    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        node_executable='euclidean_cluster_node_exe',
        node_namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')],
        remappings=[
            ("points_in", "points_nonground")
        ]
    )
    # point cloud fusion runner to fuse front and rear lidar

    # 节点2： 前雷达点云和后雷达点云融合
    point_cloud_fusion_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(point_cloud_fusion_node_pkg_prefix,
                             'launch/vlp16_sim_lexus_pc_fusion.launch.py'))
    )

    # 节点3：地面点分类去除
    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        node_executable='ray_ground_classifier_cloud_node_exe',
        node_namespace='perception',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')],
        remappings=[("points_in", "/lidars/points_fused")]
    )

    #节点4：rviz2
    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz')),
        remappings=[("initialpose", "/localization/initialpose"),
                    ("goal_pose", "/planning/goal_pose")],
    )

    # 节点5： 点云下采样
    scan_downsampler = Node(
        package='voxel_grid_nodes',
        node_executable='voxel_grid_node_exe',
        node_namespace='lidars',
        node_name='voxel_grid_cloud_node',
        parameters=[LaunchConfiguration('scan_downsampler_param_file')],
        remappings=[
            ("points_in", "points_fused"),
            ("points_downsampled", "points_fused_downsampled")
        ]
    )

    # 节点5：lanelet2地图发布
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_provider_exe',
        node_namespace='had_maps',
        node_name='lanelet2_map_provider_node',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file')]
    )

    # 节点6：lanelet2地图可视化
    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_visualizer_exe',
        node_name='lanelet2_map_visualizer_node',
        node_namespace='had_maps'
    )

    #节点7：全局规划器
    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        node_name='lanelet2_global_planner_node',
        node_namespace='planning',
        node_executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )

    #节点8：路径规划
    lane_planner = Node(
        package='lane_planner_nodes',
        node_name='lane_planner_node',
        node_namespace='planning',
        node_executable='lane_planner_node_exe',
        parameters=[LaunchConfiguration('lane_planner_param_file')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    #节点9：泊车规划
    parking_planner = Node(
        package='parking_planner_nodes',
        node_name='parking_planner_node',
        node_namespace='planning',
        node_executable='parking_planner_node_exe',
        parameters=[LaunchConfiguration('parking_planner_param_file')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )
    
    # 节点10：物体检测碰撞
    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        node_name='object_collision_estimator_node',
        node_namespace='planning',
        node_executable='object_collision_estimator_node_exe',
        condition=IfCondition(LaunchConfiguration('with_obstacles')),
        parameters=[LaunchConfiguration('object_collision_estimator_param_file')],
        remappings=[
            ('obstacle_topic', '/perception/lidar_bounding_boxes_filtered'),
        ]
    )

    # 节点11：动作行为规划
    behavior_planner = Node(
        package='behavior_planner_nodes',
        node_name='behavior_planner_node',
        node_namespace='planning',
        node_executable='behavior_planner_node_exe',
        parameters=[
            LaunchConfiguration('behavior_planner_param_file'),
            {'enable_object_collision_estimator': LaunchConfiguration('with_obstacles')}
        ],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('vehicle_state_report', '/vehicle/state_report'),
            ('vehicle_state_command', '/vehicle/state_command')
        ]
    )

    # 地图外障碍物过滤
    off_map_obstacles_filter = Node(
        package='off_map_obstacles_filter_nodes',
        node_name='off_map_obstacles_filter_node',
        node_namespace='perception',
        node_executable='off_map_obstacles_filter_nodes_exe',
        parameters=[LaunchConfiguration('off_map_obstacles_filter_param_file')],
        output='screen',
        remappings=[
            ('bounding_boxes_in', 'lidar_bounding_boxes'),
            ('bounding_boxes_out', 'lidar_bounding_boxes_filtered'),
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
        ]
    )

    #网络链接
    web_bridge = Node(
        package='rosbridge_server',
        node_name='rosbridge_server_node',
        node_namespace='gui',
        node_executable='rosbridge_websocket'
    )

    web_server = ExecuteProcess(
      cmd=["python3", "-m", "http.server", "8000"],
      cwd=web_files_root
    )

    return LaunchDescription([
        euclidean_cluster_param,
        ray_ground_classifier_param,
        scan_downsampler_param,
        with_rviz_param,
        with_obstacles_param,
        lanelet2_map_provider_param,
        lane_planner_param,
        parking_planner_param,
        object_collision_estimator_param,
        behavior_planner_param,
        off_map_obstacles_filter_param,
        euclidean_clustering,
        ray_ground_classifier,
        scan_downsampler,
        point_cloud_fusion_node,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        global_planner,
        lane_planner,
        parking_planner,
        object_collision_estimator,
        behavior_planner,
        off_map_obstacles_filter,
        rviz2,
        web_server,
        web_bridge,
    ])

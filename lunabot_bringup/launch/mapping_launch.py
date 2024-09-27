from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    declare_rtabmap_args = DeclareLaunchArgument(
        "rtabmap_args",
        default_value="-d -Vis/Odom false -RGBD/OdomVarianceGuess false -RGBD/ProximityBySpace false -Odom/ScanMatching false -RGBD/OdomCorrection false -RGBD/ProximityOdomGuess false -RGBD/LoopClosure true -RGBD/LoopClosureMaxDistance 2.0 -Rtabmap/StartNewMapOnGoodSignature true -RGBD/OdomCorrection true  -RGBD/ProximityOdomGuess true -RGBD/NewMapOdomChangeDistance 0.5 -RGBD/OptimizeMaxError 0.1 -RGBD/NeighborLinkRefining true -RGBD/ProximityBySpace true -RGBD/StartAtOrigin true -RGBD/LinearUpdate 0.01 -RGBD/AngularUpdate 0.01 -RGBD/LinearSpeedUpdate 0.2 -RGBD/AngularSpeedUpdate 0.1 -RGBD/ProximityAngle 60 -RGBD/ProximityDistance 0.5 -Vis/MinInliers 10 -Vis/InlierDistance 0.1 -Vis/MaxFeatures 3000 -Rtabmap/StartNewMapOnGoodSignature true -Rtabmap/DetectionRate 10 -RGBD/CreateOccupancyGrid true -Grid/CellSize 0.035 -Grid/MaxGroundAngle 60 -Grid/Sensor 2 -Grid/RangeMin 0.5 -Grid/RangeMax 0.0 -Reg/Force3DoF true -Reg/Strategy 2 -Grid/MaxObstacleHeight 0.4 -Grid/RayTracing true",
        description="RTAB-Map custom arguments",
    )

    rtabmap_launch_file_path = os.path.join(
        get_package_share_path("rtabmap_launch"), "launch", "rtabmap.launch.py"
    )

    include_rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file_path),
        launch_arguments={
            "args": LaunchConfiguration("rtabmap_args"),
            "namespace": "",
            "rviz": "true",
            "rtabmap_viz": "true",
            "frame_id": "base_link",
            "odom_frame_id": "odom",
            "map_frame_id": "map",
            "publish_tf_map": "false",
            "publish_tf_odom": "false",
            "output": "screen",
            "approx_sync": "true",
            "rgb_topic": "/d456/color/image_raw",
            "depth_topic": "/d456/depth/image_rect_raw",
            "camera_info_topic": "/d456/depth/camera_info",
            "subscribe_scan": "true",
            "subscribe_scan_cloud": "false",
            "scan_topic": "/scan",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "imu_topic": "/imu/data",
            "tag_topic": "/tag_detections",
            "sync_queue_size": "1000",
        }.items(),
    )

    return LaunchDescription([declare_rtabmap_args, include_rtabmap_launch])

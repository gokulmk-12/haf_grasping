#!/usr/bin/env python3

import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Global variable to store the point cloud
point_cloud = None

# Callback function for the PointCloud2 message
def point_cloud_callback(msg):
    global point_cloud
    point_cloud = msg

    # Unsubscribe after receiving the first point cloud
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback, queue_size=1)

    # Convert PointCloud2 message to open3d PointCloud
    points = pc2.read_points(point_cloud, field_names=("x", "y", "z"))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    # Save the point cloud as a .pcd file
    o3d.io.write_point_cloud("point_cloud.pcd", pcd)

    print("Point cloud saved as point_cloud.pcd.")
    rospy.signal_shutdown("Point cloud saved")

# ROS initialization
rospy.init_node('point_cloud_subscriber', anonymous=True)
rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_cloud_callback, queue_size=1)

# Start the ROS spin loop
rospy.spin()
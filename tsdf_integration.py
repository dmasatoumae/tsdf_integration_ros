#!/usr/bin/python3
# -*- coding: utf-8 -*-

'''
This script gives exampe code for subscribing:
    (1) Color image.
    (2) Depth image.
    (3) Camera info.
    (4) Point cloud (subscribed as open3d format).
'''

from utils.lib_ros_rgbd_pub_and_sub import ColorImageSubscriber, DepthImageSubscriber, CameraInfoSubscriber
#from utils.lib_ros_point_cloud_pub_and_sub import PointCloudSubscriber
from utils.lib_rgbd import MyCameraInfo

import numpy as np
import rospy
import open3d as o3d

# -- Set ROS topic names for subscribing.
NS = "/camera"  # ROS topic namespace.
COLOR_TOPIC_NAME = NS + "/color/image_raw"
DEPTH_TOPIC_NAME = NS + "/aligned_depth_to_color/image_raw"
CAMERA_INFO_TOPIC_NAME = NS + "/color/camera_info"
#CLOUD_TOPIC_NAME = NS + "point_cloud"


# -- Subscribe data and print.
def main():

    # -- Set subscribers.
    sub_color = ColorImageSubscriber(COLOR_TOPIC_NAME)
    sub_depth = DepthImageSubscriber(DEPTH_TOPIC_NAME)
    sub_camera_info = CameraInfoSubscriber(CAMERA_INFO_TOPIC_NAME)
    #sub_cloud = PointCloudSubscriber(CLOUD_TOPIC_NAME)

    # -- Loop and subscribe.
    cnt_1, cnt_2, cnt_3, cnt_4 = 0, 0, 0, 0  # color, depth, camera_info, cloud

    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length = 7.0/512.0,
        sdf_trunc = 0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    
    cam = o3d.camera.PinholeCameraIntrinsic()
    
    while not rospy.is_shutdown():

        # Color.
        if sub_color.has_image():
            color = sub_color.get_image()
            cnt_1 += 1
            rospy.loginfo("Subscribe {}: color image, "
                          "shape={}".format(
                              cnt_1, color.shape))

        # Depth.
        if sub_depth.has_image():
            depth = sub_depth.get_image()
            cnt_2 += 1
            rospy.loginfo("Subscribe {}: depth image, "
                          "shape={}".format(
                              cnt_2, depth.shape))

        # Camera_info.
        if sub_camera_info.has_camera_info():
            ros_camera_info = sub_camera_info.get_camera_info()
            cnt_3 += 1
            rospy.loginfo("Subscribe {}: camera_info, "
                          "fx={}, fy={}.".format(
                              cnt_3,
                              ros_camera_info.K[0],
                              ros_camera_info.K[4],
                          ))
            #rospy.loginfo(type(ros_camera_info.width))
            #rospy.loginfo(type(ros_camera_info.K[4]))
            #rospy.loginfo(type(ros_camera_info.K[2]))
            my_camera_info = MyCameraInfo(ros_camera_info=ros_camera_info)
            cam.set_intrinsics(ros_camera_info.width,ros_camera_info.height,ros_camera_info.K[0],ros_camera_info.K[4],ros_camera_info.K[2],ros_camera_info.K[5])
            #cam.intrinsic = intrinsic
            #cam.intrinsic_matrix =[[ros_camera_info.K[0],ros_camera_info.K[1],ros_camera_info.K[2]],
            #                                [ros_camera_info.K[3],ros_camera_info.K[4],ros_camera_info.K[5]],
            #                                [ros_camera_info.K[6],ros_camera_info.K[7],ros_camera_info.K[8]]]

        if (cnt_1 >= 1 & cnt_2 >=1):
            cnt_4 += 1
            o3d_color = o3d.geometry.Image(color)
            o3d_depth = o3d.geometry.Image(depth)
            o3d_rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d_color, o3d_depth, depth_trunc=4.0, convert_rgb_to_intensity=False)
            extrinsic_params = np.array([[1., 0., 0., 0.],[0., 1., 0., 0.],[0., 0., 1., 0.],[0., 0., 0., 1.]])
            #rospy.loginfo(cam.intrinsic_matrix)
            #rospy.loginfo(extrinsic_params)
            volume.integrate(
                o3d_rgbd,
                cam,
                extrinsic_params
            )
            
        if (cnt_4==100):
            rospy.loginfo("Extract a triangle mesh from the volume and visualize it.")
            mesh = volume.extract_triangle_mesh()
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh("test.obj",mesh)

        '''
        # Point_cloud.
        if sub_cloud.has_cloud():
            open3d_cloud = sub_cloud.get_cloud()
            cnt_4 += 1
            num_points = np.asarray(open3d_cloud.points).shape[0]
            rospy.loginfo("Subscribe {}: point cloud, "
                          "{} points.".format(
                              cnt_4, num_points))

        rospy.sleep(0.1)
        '''

if __name__ == '__main__':
    node_name = "sub_rgbd_and_cloud"
    rospy.init_node(node_name)
    main()
    rospy.logwarn("Node `{}` stops.".format(node_name))

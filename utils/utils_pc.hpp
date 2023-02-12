// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

using namespace cv;
using namespace rs2;

float getZValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y);
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
Eigen::MatrixXd detectPoseObject(pipeline pipe);
double getZMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);




std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords)
{
    const int w = texture.get_width(), h = texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color){

    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(1280, 720), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", colorr);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    int idx = 0;
    for(unsigned int i = 0; i < cloud->height; ++i){
      for(unsigned int j = 0; j < cloud->width; ++j){
        cloud->points[idx].x = vertices[idx].x;
        cloud->points[idx].y = vertices[idx].y;
        cloud->points[idx].z = vertices[idx].z;

        std::tuple<uint8_t, uint8_t, uint8_t> current_color;
        current_color = get_texcolor(color, tex_coords[idx]);

        // Reversed order- 2-1-0 because of BGR model used in camera
        cloud->points[idx].r = std::get<2>(current_color);
        cloud->points[idx].g = std::get<1>(current_color);
        cloud->points[idx].b = std::get<0>(current_color);
        
        idx++;
      }
    }
    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (cloud);

    //waitKey(0);
    
   return cloud;
}

float getZValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y){
  pcl::PointXYZRGB point;
  float eps = 0.005;
  for (unsigned int i= 0; i < cloud->points.size(); i++){
    point = cloud->points[i];
    if((x-eps <= point.x && point.x <= x+eps) && (y-eps <= point.y && point.y <= y+eps)){
      return point.z;
    }
  }
  return 0.0f;
}

Eigen::MatrixXd detectPoseObject(pipeline pipe){
  rs2::pointcloud pc;
    rs2::points points;
    rs2::pipeline_profile pipeline_profile;
    rs2::frameset frameset;

    rs2::frame color_frame;
    cv::Mat color_mat;

    rs2::frame depth_frame;
    cv::Mat depth_mat;

    Eigen::MatrixXd T_obj;
    T_obj.setIdentity(4,4);
    rs2::colorizer color_map;

    int width = 1280;
    int height = 720;
    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::frameset frames;
    for(int i = 0; i < 50; i++) { frames = pipe.wait_for_frames();} 

    pcl::visualization::PCLVisualizer* viewer;
    viewer = new pcl::visualization::PCLVisualizer("3d viewer"); 
    viewer->setBackgroundColor(0.33f, 0.33f, 0.33f);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0.0f, 0.0f, 0.0f, //0.0f,
                0.0f, 0.0f, 1.0f,
                0.0f, -1.0f, 0.0f);

    int num = 0;
    int pp;
    while(num <5) {  
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
      viewer->removeCoordinateSystem();
      pp = rand();
      frames = pipe.wait_for_frames();
      frames = align_to_color.process(frames);

      cv::Mat rgbImage(Size(width, height), CV_8UC3, (void*)frames.get_color_frame().get_data(), Mat::AUTO_STEP);
      cv::Mat depth(Size(width, height), CV_8UC3, (void*)frames.get_depth_frame().apply_filter(color_map).get_data(), Mat::AUTO_STEP);
      rs2::depth_frame depthImage = frames.get_depth_frame();
            
      // Retrieve Color Flame
      color_frame = frames.get_color_frame();
      if( !color_frame ){
        exit(0);
      }

      pc.map_to(color_frame);
      // Retrieve Depth Flame
      depth_frame = frames.get_depth_frame();
      if( !depth_frame ){
        exit(0);
      }

      // Generate the pointcloud and texture mappings
      points = pc.calculate(depth_frame);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, color_frame);  

      pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter0; // Create the filtering object
      Cloud_Filter0.setInputCloud (cloud);           // Input generated cloud to filter
      Cloud_Filter0.setFilterFieldName ("z");        // Set field name to Z-coordinate
      double z_max = getZMax(cloud);
      Cloud_Filter0.setFilterLimits(0.05,z_max-0.05);// (-20, 20);      // metri
      Cloud_Filter0.filter (*cloud);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
      voxel_grid.setInputCloud(cloud);
      voxel_grid.setLeafSize(0.01,0.01,0.01); //0.01, 0.01, 0.01); //piu' e' grande e piu' pixel va a raggruppare
      voxel_grid.filter(*cloud_voxel); 


      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_temp(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*cloud_voxel, *pc_temp);

      std::stringstream sbuff1; sbuff1<<"cloud1"; sbuff1<<pp;
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud_voxel, sbuff1.str());
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, sbuff1.str());

      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud(cloud_voxel);

      // BB
      pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
      feature_extractor.setInputCloud (pc_temp);
      feature_extractor.compute ();

      std::vector <float> moment_of_inertia;
      std::vector <float> eccentricity;
      pcl::PointXYZ min_point_AABB;
      pcl::PointXYZ max_point_AABB;
      pcl::PointXYZ min_point_OBB;
      pcl::PointXYZ max_point_OBB;
      pcl::PointXYZ position_OBB;
      Eigen::Matrix3f rotational_matrix_OBB;
      float major_value, middle_value, minor_value;
      Eigen::Vector3f major_vector, middle_vector, minor_vector;
      Eigen::Vector3f mass_center;

      feature_extractor.getMomentOfInertia (moment_of_inertia);
      feature_extractor.getEccentricity (eccentricity);
      feature_extractor.getAABB (min_point_AABB, max_point_AABB);
      feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
      feature_extractor.getEigenValues (major_value, middle_value, minor_value);
      feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
      feature_extractor.getMassCenter (mass_center);

      std::stringstream sbuff; sbuff<<"cloud"; sbuff<<pp;
      std::stringstream sbuff_obb; sbuff_obb<<"obb"; sbuff_obb<<pp;
      viewer->addPointCloud<pcl::PointXYZ> (pc_temp, sbuff.str());
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, sbuff.str());
      Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
      Eigen::Quaternionf quat (rotational_matrix_OBB);
      
      viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, sbuff_obb.str());
      viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, sbuff_obb.str());
            
      pcl::PointXYZ center (mass_center (0), mass_center (1), mass_center (2));
      pcl::PointXYZ x_axis (major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));
      pcl::PointXYZ y_axis (middle_vector (0) + mass_center (0), middle_vector (1) + mass_center (1), middle_vector (2) + mass_center (2));
      pcl::PointXYZ z_axis (minor_vector (0) + mass_center (0), minor_vector (1) + mass_center (1), minor_vector (2) + mass_center (2));
        
      //Eigen::Vector3d asse_x(major_vector (0) + mass_center (0), major_vector (1) + mass_center (1), major_vector (2) + mass_center (2));;
      //Eigen::Vector3d asse_z(0,0,1);
      //Eigen::Vector3d asse_y;
      //asse_y = asse_z.cross(asse_x);
      //pcl::PointXYZ y_axis (asse_y[0], asse_y[1], asse_y[2]);
      //pcl::PointXYZ z_axis (asse_z[0], asse_z[1], asse_z[2]);
        

      std::stringstream sbuff_m1; sbuff_m1<<"major_eigen_vector_"; sbuff_m1<<pp;
      std::stringstream sbuff_m2; sbuff_m2<<"middle_eigen_vector_"; sbuff_m2<<pp;
      std::stringstream sbuff_m3; sbuff_m3<<"minor_eigen_vector_"; sbuff_m3<<pp;
      viewer->addLine (center, x_axis, 1.0f, 0.0f, 0.0f, sbuff_m1.str());
      viewer->addLine (center, y_axis, 0.0f, 1.0f, 0.0f, sbuff_m2.str());
      viewer->addLine (center, z_axis, 0.0f, 0.0f, 1.0f, sbuff_m3.str());

      if(num==3){
        T_obj(0,0) = rotational_matrix_OBB(0,0); T_obj(0,1) = rotational_matrix_OBB(0,1); T_obj(0,2) = rotational_matrix_OBB(0,2);
        T_obj(1,0) = rotational_matrix_OBB(1,0); T_obj(1,1) = rotational_matrix_OBB(1,1); T_obj(1,2) = rotational_matrix_OBB(1,2);
        T_obj(2,0) = rotational_matrix_OBB(2,0); T_obj(2,1) = rotational_matrix_OBB(2,1); T_obj(2,2) = rotational_matrix_OBB(2,2);
        T_obj(0,3) = position[0];
        T_obj(1,3) = position[1];
        T_obj(2,3) = getZValue(cloud, mass_center(0),mass_center (1));
        return T_obj;
      }

    viewer->spinOnce(100); 
    num++;
   }
   return T_obj;
}

double getZMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  pcl::PointXYZRGB point;
  double max = -1.0;
  for (unsigned int i= 0; i < cloud->points.size(); i++){
    point = cloud->points[i];
    if(point.z > max){
      max = point.z;
    }
  }
  return max;
}
 




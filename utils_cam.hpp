// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <algorithm>            // std::min, std::max
//#include <filesystem>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <Eigen/Core>
#include <ctime>

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
#include <pcl/io/ply_io.h>

using namespace cv;
using namespace rs2;

int width = 640; //1280;
int height = 480; //720;
int fps = 15;
std::string ply_folder="/home/labarea-franka/libfranka/unibas_insert_bh/resources/ply_data/";

pipeline startCamera();
void savePly(pipeline pipe, int pc_idx);
float getZValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y);
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color, cv::Mat& filtered);
double getZMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
void clearPlyFolder();
void filterAndSavePly_deeplab(pipeline pipe, int pc_idx, std::string savePath);
void savePly_noFilter(pipeline pipe, int pc_idx, std::string savePath);


void clearPlyFolder(){
  boost::filesystem::remove_all(ply_folder);
  boost::filesystem::create_directory(ply_folder);
}

pipeline startCamera() {
    pipeline pipe;
    rs2::config config;

    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);//.as<video_stream_profile>();
    //config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    //config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    
    rs2::pipeline_profile pipeline_profile = pipe.start(config);

    rs2::device dev = pipeline_profile.get_device();

    auto depth_sensor = dev.first<rs2::depth_sensor>();
    /*if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, 30000.f); //10715
    }
    if (depth_sensor.supports(RS2_OPTION_GAIN)) {
        depth_sensor.set_option(RS2_OPTION_GAIN, 148.f); //33
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER)){
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 30); //30
    }
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    }*/

	// Capture 30 frames to give autoexposure, etc. a chance to settle
    for (int k = 0; k < 50; ++k) pipe.wait_for_frames();

    return pipe;
}

pcl::visualization::PCLVisualizer::Ptr rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 255, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

void filterAndSavePly2(pipeline pipe, int pc_idx){
	rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::align align(RS2_STREAM_COLOR);

	auto data = pipe.wait_for_frames();
	auto color_frame = data.get_color_frame();
	auto depth_frame = data.get_depth_frame();

	auto aligned_frames = align.process(data);
	rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
	rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        
	//CREATE PC
	rs2::pointcloud pc;
	rs2::points points;
	pc.map_to(aligned_color_frame);
	points = pc.calculate(aligned_depth_frame);

  cv::Mat dst_rgb;//, dst_dep;
  // CREO E SALVO L'IMMAGINE DA PASSARE ALLA RETE
  cv::Mat color = cv::Mat( aligned_color_frame.as<rs2::video_frame>().get_height(), color_frame.as<rs2::video_frame>().get_width(), CV_8UC3, const_cast<void*>( color_frame.get_data() ) ).clone();
  cv::resize(color, dst_rgb, cv::Size(512,512));//for inference
  cv::imwrite("/home/labarea-franka/libfranka/unibas_insert_bh/segnet/image.png", dst_rgb);
  std::cout << "Python" << std::endl;
  
  // AVVIO LA RETE E FACCIO L'INFERENZA
  system("python3 /home/labarea-franka/libfranka/unibas_insert_bh/segnet/test.py --save_dir /home/labarea-franka/libfranka/unibas_insert_bh/segnet/mask/ --test_list /home/labarea-franka/libfranka/unibas_insert_bh/segnet/to_mask.txt --resume /home/labarea-franka/libfranka/unibas_insert_bh/segnet/model20.hdf5");
  system("python3 /home/labarea-franka/libfranka/unibas_insert_bh/segnet/inference.py");
  // CARICO LA MASCHERA BINARIA
  cv::Mat mask = cv::imread("/home/labarea-franka/libfranka/unibas_insert_bh/segnet/mask/image.png", cv::IMREAD_GRAYSCALE); 
  cv::Mat mask_res;
  int newW = 1280;
  int newH = 720;
  // FACCIO IL RESIZE DELLA MASCHERA E LA VISUALIZZO
  cv::resize(mask, mask_res, cv::Size(newW,newH));
  cv::imshow("Mask", mask_res);
  cv::waitKey(0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, aligned_color_frame, mask_res);


	std::string pc_idx_str = std::to_string(pc_idx);

  pcl::PLYWriter writer;
  std::string filename;
  filename = ply_folder + "pointCloud_" + pc_idx_str + ".ply";

  writer.write(filename, *cloud, false, true);
  
	points.export_to_ply(ply_folder + "pcex_" + pc_idx_str + ".ply", color_frame);
  
  cout << pc_idx << endl; 
  return;
}

///---------------- Modifiche 10 settembre
void filterAndSavePly_deeplab(pipeline pipe, int pc_idx, std::string savePath){
	std::string pc_idx_str = std::to_string(pc_idx);
	rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::align align(RS2_STREAM_COLOR);

	auto data = pipe.wait_for_frames();
	auto color_frame = data.get_color_frame();
	auto depth_frame = data.get_depth_frame();

	auto aligned_frames = align.process(data);
	rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
	rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        
	//CREATE PC
	rs2::pointcloud pc;
	rs2::points points;
	pc.map_to(aligned_color_frame);
	points = pc.calculate(aligned_depth_frame);

  cv::Mat dst_rgb;//, dst_dep;
  // CREO, CONVERTO E SALVO L'IMMAGINE DA PASSARE ALLA RETE
  cv::Mat color = cv::Mat( aligned_color_frame.as<rs2::video_frame>().get_height(), color_frame.as<rs2::video_frame>().get_width(), CV_8UC3, const_cast<void*>( color_frame.get_data() ) ).clone();
  cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
  cv::imwrite("/home/labarea-franka/libfranka/unibas_insert_bh/resources/image_for_filtering/image.jpg", color);
  std::string img_name = "/home/labarea-franka/libfranka/unibas_insert_bh/resources/mask_for_filtering/image_" + pc_idx_str + ".jpg";
  cv::imwrite(img_name, color);
  std::cout << "Python" << std::endl;
  
  // AVVIO LA RETE E FACCIO L'INFERENZA
  time_t tstart, tend; 
  tstart = time(0);
  system("python /mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/Paper_EAAI_2021/network_code/inference_image_from_folder_pipeline.py");
  tend = time(0); 
  cout << "\n\nTHE INFERENCE TOOKS "<< difftime(tend, tstart) <<" SECOND(s).\n"<< endl;
  // CARICO LA MASCHERA BINARIA
  cv::Mat mask = cv::imread("/home/labarea-franka/libfranka/unibas_insert_bh/resources/mask_for_filtering/image.jpg", cv::IMREAD_GRAYSCALE); 
  img_name = "/home/labarea-franka/libfranka/unibas_insert_bh/resources/mask_for_filtering/mask_" + pc_idx_str + ".jpg";
  cv::imwrite(img_name, mask);
  //cv::Mat mask_res;
  //int newW = 1280;
  //int newH = 720;
  // FACCIO IL RESIZE DELLA MASCHERA E LA VISUALIZZO
  //cv::resize(mask, mask_res, cv::Size(newW,newH));
  //cv::imshow("Mask", mask_res);
  //cv::waitKey(0);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, aligned_color_frame, mask);



  pcl::PLYWriter writer;
  std::string filename;
  filename = ply_folder + "pointCloud_" + pc_idx_str + ".ply";

  writer.write(filename, *cloud, false, true);
  filename= savePath + "pointCloud_" + pc_idx_str + ".ply";
  writer.write(filename, *cloud, false, true);
  
	points.export_to_ply(ply_folder + "pcex_" + pc_idx_str + ".ply", color_frame);
  
  cout << pc_idx << endl; 
  return;
}

void savePly_noFilter(pipeline pipe, int pc_idx, std::string savePath){
	std::string pc_idx_str = std::to_string(pc_idx);
	rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::align align(RS2_STREAM_COLOR);

	auto data = pipe.wait_for_frames();
	auto color_frame = data.get_color_frame();
	auto depth_frame = data.get_depth_frame();

	auto aligned_frames = align.process(data);
	rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
	rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        
	//CREATE PC
	rs2::pointcloud pc;
	rs2::points points;
	pc.map_to(aligned_color_frame);
	points = pc.calculate(aligned_depth_frame);

  cv::Mat dst_rgb;//, dst_dep;
  // CREO, CONVERTO E SALVO L'IMMAGINE DA PASSARE ALLA RETE
  cv::Mat color = cv::Mat( aligned_color_frame.as<rs2::video_frame>().get_height(), color_frame.as<rs2::video_frame>().get_width(), CV_8UC3, const_cast<void*>( color_frame.get_data() ) ).clone();
  cv::cvtColor(color, color, cv::COLOR_BGR2RGB);
  
  // CARICO LA MASCHERA BINARIA
  cv::Mat mask = cv::imread("/home/labarea-franka/libfranka/unibas_insert_bh/resources/mask_for_filtering/white.jpg", cv::IMREAD_GRAYSCALE); 


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, aligned_color_frame, mask);

  pcl::PLYWriter writer;
  std::string filename;
  filename = ply_folder + "pointCloud_" + pc_idx_str + ".ply";

  writer.write(filename, *cloud, false, true);
  filename= savePath + "pointCloud_" + pc_idx_str + ".ply";
  writer.write(filename, *cloud, false, true);
  
	points.export_to_ply(ply_folder + "pcex_" + pc_idx_str + ".ply", color_frame);
  
  cout << pc_idx << endl; 
  return;
}
//-----------------

void filterAndSavePly(pipeline pipe, int pc_idx, std::string savePath){
	rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::colorizer color_map;
  rs2::frame color_frame;
  rs2::frame depth_frame;
  rs2::pointcloud pc;
  rs2::points points;


	rs2::frameset frames = pipe.wait_for_frames();
  frames = align_to_color.process(frames);

  cv::Mat rgbImage(Size(1280, 720), CV_8UC3, (void*)frames.get_color_frame().get_data(), Mat::AUTO_STEP);
  cv::Mat depth(Size(1280, 720), CV_8UC3, (void*)frames.get_depth_frame().apply_filter(color_map).get_data(), Mat::AUTO_STEP);
  rs2::depth_frame depthImage = frames.get_depth_frame();
      
  // Retrieve Color Flame
  color_frame = frames.get_color_frame();
  if( !color_frame ){
    exit(0);
  }        
  // Tell pointcloud object to map to this color frame
  pc.map_to(color_frame);
  // Retrieve Depth Flame
  depth_frame = frames.get_depth_frame();
  if( !depth_frame ){
    exit(0);
  }

  // Generate the pointcloud and texture mappings
  points = pc.calculate(depth_frame);

  cv::Mat dst_rgb;//, dst_dep;
  // CREO E SALVO L'IMMAGINE DA PASSARE ALLA RETE
  cv::Mat color = cv::Mat( color_frame.as<rs2::video_frame>().get_height(), color_frame.as<rs2::video_frame>().get_width(), CV_8UC3, const_cast<void*>( color_frame.get_data() ) ).clone();
  cv::resize(color, dst_rgb, cv::Size(512,512));//for inference
  cv::imwrite("/home/labarea-franka/libfranka/unibas_insert_bh/segnet/image.png", dst_rgb);
  std::cout << "Python" << std::endl;
  
  // AVVIO LA RETE E FACCIO L'INFERENZA
  system("python3 /home/labarea-franka/libfranka/unibas_insert_bh/segnet/test.py --save_dir /home/labarea-franka/libfranka/unibas_insert_bh/segnet/mask/ --test_list /home/labarea-franka/libfranka/unibas_insert_bh/segnet/to_mask.txt --resume /home/labarea-franka/libfranka/unibas_insert_bh/segnet/model70.hdf5");
  system("python3 /home/labarea-franka/libfranka/unibas_insert_bh/segnet/inference.py");
  // CARICO LA MASCHERA BINARIA
  cv::Mat mask = cv::imread("/home/labarea-franka/libfranka/unibas_insert_bh/segnet/mask/image.png", cv::IMREAD_GRAYSCALE); 
  cv::Mat mask_res;
  int newW = 1280;
  int newH = 720;
  // FACCIO IL RESIZE DELLA MASCHERA E LA VISUALIZZO
  cv::resize(mask, mask_res, cv::Size(newW,newH));
  //cv::imwrite("resize.png", mask_res);

  cv::Mat mask_white = cv::imread("/mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/TESI/CeruzziFrancesco/test/white1280.png", cv::IMREAD_GRAYSCALE);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, color_frame, mask_res);
  
  
  
  
  /*pcl::PointCloud<pcl::PointXYZRGB>::Ptr full = points_to_pcl(points, color_frame, mask_white);
  cv::namedWindow("RGB", WINDOW_NORMAL );
  cv::imshow("RGB", color);
  cv::namedWindow("Depth", WINDOW_NORMAL );
  cv::imshow("Depth", depth);
  cv::namedWindow("Maschera", WINDOW_NORMAL );
  cv::imshow("Maschera", mask_res);
  cv::waitKey(0);
  

  pcl::visualization::PCLVisualizer::Ptr viewer1, viewer2;
  viewer1 = rgbVis(cloud);
  viewer2 = rgbVis(full);
  while (!viewer1->wasStopped()) {
    viewer1->spinOnce (100);
    viewer2->spinOnce (100);
  }*/





	std::string pc_idx_str = std::to_string(pc_idx);

  pcl::PLYWriter writer;
  std::string filename;
  filename = ply_folder + "pointCloud_" + pc_idx_str + ".ply";

  writer.write(filename, *cloud, false, true);
  filename= savePath + "pointCloud_" + pc_idx_str + ".ply";
  writer.write(filename, *cloud, false, true);

  
	//points.export_to_ply(ply_folder + "pcex_" + pc_idx_str + ".ply", color_frame);
  
  //cout << pc_idx << endl; 
  return;
}

void savePly(pipeline pipe, int pc_idx){
	rs2::align align_to_color(RS2_STREAM_COLOR);
  rs2::align align(RS2_STREAM_COLOR);

	auto data = pipe.wait_for_frames();
	auto color_frame = data.get_color_frame();
	auto depth_frame = data.get_depth_frame();

	auto aligned_frames = align.process(data);
	rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
	rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        
	//CREATE PC
	rs2::pointcloud pc;
	rs2::points points;
	pc.map_to(aligned_color_frame);
	points = pc.calculate(aligned_depth_frame);

	std::string pc_idx_str = std::to_string(pc_idx);
	points.export_to_ply(ply_folder + "pointcloud_" + pc_idx_str + ".ply", color_frame);
  
  cout << pc_idx << endl; 
  return;
}

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
    Mat colorr(Size(width, height), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
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
        cloud->points[idx].r = std::get<0>(current_color);
        cloud->points[idx].g = std::get<1>(current_color);
        cloud->points[idx].b = std::get<2>(current_color);
        
        idx++;
      }
    }
    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    //viewer.showCloud (cloud);

    //waitKey(0);
    
   return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color, cv::Mat& filtered){
    std::cout << "FUNZIONE\n";
  cv::imwrite("resize_in_func.png", filtered);
    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(1280, 720), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", colorr);
    //cv::waitKey(0);
    //destroyWindow("Display Image");
        
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
    for(int i = 0; i < cloud->height; ++i){
      for(int j = 0; j < cloud->width; ++j){
        double c = (double)filtered.at<uchar>(i,j);
        if((c >= 125 && c <= 130) || (c >= 250)){
          cloud->points[idx].x = vertices[idx].x;
          cloud->points[idx].y = vertices[idx].y;
          cloud->points[idx].z = vertices[idx].z;

          std::tuple<uint8_t, uint8_t, uint8_t> current_color;
          current_color = get_texcolor(color, tex_coords[idx]);

          // Reversed order- 2-1-0 because of BGR model used in camera
          cloud->points[idx].r = std::get<0>(current_color);
          cloud->points[idx].g = std::get<1>(current_color);
          cloud->points[idx].b = std::get<2>(current_color);
        }
        idx++;
      }
    }
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
 




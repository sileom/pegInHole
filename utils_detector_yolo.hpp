// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

#include <algorithm>            // std::min, std::max
//#include <filesystem>
#include <boost/filesystem.hpp>
#include <opencv2/dnn.hpp>
#include <ctime>
#include <chrono>

/*#include <opencv2/opencv.hpp>
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
#include <pcl/io/ply_io.h>*/

#include "utils_cam.hpp"

using namespace cv;
using namespace dnn;
using namespace rs2;

float confThreshold = 0.80;
float nmsThreshold = 0.40; 
float scale = 0.00392;
Scalar mean_v = Scalar(0,0,0,0);
bool swapRB = false;
int inpWidth = 416;
int inpHeight = 416;
size_t asyncNumReq = 0;

Rect boxMin;
std::vector<std::string> classes;



Eigen::Vector3d detectHole(Eigen::MatrixXd Ae, std::string path);
Net loadNet();
double getDistanceBetweenCenters(Point c1, Point c2);
Point getCenterOfBox(Rect box);
double getMeanZ(rs2::depth_frame aligned_depth_frame);
void postprocessAndFindCenterHole(Mat& frame, const std::vector<Mat>& outs, Net& net, rs2::points& points);
void preprocess(const Mat& frame, Net& net, Size inpSize, float scale, const Scalar& mean_v, bool swapRB);
void drawPredWithoutText(int classId, float conf, int left, int top, int right, int bottom, Mat& frame);
cv::Mat frame_to_mat(const rs2::frame& f);
Eigen::Vector3d convert2Dto3D(const rs2::depth_frame& frame, Point pixel, double distance);


Net loadNet(){
  const std::string weigthsFile = "/home/labarea-franka/Documents/yolo_test/test2/build/yolov3-unibas-gpu_final.weights";
  const std::string cfgFile = "/home/labarea-franka/Documents/yolo_test/test2/build/yolov3-unibas-gpu.cfg";
  const std::string classesFile = "/home/labarea-franka/Documents/yolo_test/test2/build/unibas-gpu.names";

  std::string modelPath = weigthsFile;
  std::string configPath = cfgFile;
  int backend = 0;
  int target = 0;

  // Open file with classes names.
  std::ifstream ifs(classesFile.c_str());
  if (!ifs.is_open())
      CV_Error(Error::StsError, "File " + classesFile + " not found");
  std::string line;
  while (std::getline(ifs, line)) {
      classes.push_back(line);
  }
  // Load a model.
  Net net = readNet(modelPath, configPath, "");
  net.setPreferableBackend(backend);
  net.setPreferableTarget(target);
  return net;
}

Eigen::Vector3d detectHole(Eigen::MatrixXd Ae, std::string path){
  pipeline pipe = startCamera();
  Net net = loadNet();
  std::vector<String> outNames = net.getUnconnectedOutLayersNames();
  std::cout << "Net loaded\n" << std::endl;
  for (int k = 0; k < 30; ++k) pipe.wait_for_frames();

  bool process = true;
  int i = 1;
  char c;

  rs2::pointcloud pc;
  rs2::points points;
  rs2::align align(RS2_STREAM_COLOR);
  //rs2::depth_frame aligned_depth_frame;

  Eigen::VectorXd hole_camera(4);
  hole_camera << 0,0,0,1;

  while(process){
    auto data = pipe.wait_for_frames();
    auto color_frame = data.get_color_frame();
    auto depth_frame = data.get_depth_frame();

    auto aligned_frames = align.process(data);
    rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        
    /*auto data = pipe.wait_for_frames();
    //data = align_to_color.process(data);
    
    auto color_frame = data.get_color_frame();
    auto depth_frame = data.get_depth_frame();

    auto aligned_frames = align.process(data);
    aligned_depth_frame = aligned_frames.get_depth_frame();*/
        
    pc.map_to(color_frame);
    //points = pc.calculate(depth_frame);
    points = pc.calculate(aligned_depth_frame);
        
    //COLOR
    auto frame = frame_to_mat(color_frame);
    //auto depth = frame_to_mat(depth_frame);
    std::cout << "Start pre-process\n" << std::endl;
    cv::imwrite("/mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/PegInHoleExperiments/d0107_deeplab_dati_video/noyolo.png", frame);
    preprocess(frame, net, Size(inpWidth, inpHeight), scale, mean_v, swapRB);
    std::cout << "End pre-process\n" << std::endl;
    std::vector<Mat> outs;
    std::cout << "Start process\n" << std::endl;

    time_t tstart, tend; 
    tstart = time(0);
    auto t0 = std::chrono::high_resolution_clock::now();
    net.forward(outs, outNames);
    auto t1 = std::chrono::high_resolution_clock::now();
    tend = time(0); 
    auto dt = 1.e-6*std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count();
    cout << "\n\nTHE INFERENCE TOOKS " << dt <<" MILLISECOND(s).\n"<< endl;
    //cout << "\n\nTHE INFERENCE TOOKS "<< difftime(tend, tstart) <<" SECOND(s).\n"<< endl;
    std::cout << "End process\n" << std::endl;
    std::cout << "Start post-process\n" << std::endl;
    postprocessAndFindCenterHole(frame, outs, net, points);
    std::cout << "End post-process\n" << std::endl;
    //video_color.write(frame);
    imshow("Color", frame);
    cv::imwrite("/mnt/f1e018d6-98bf-4b00-91de-f7b3f30ea52a/PegInHoleExperiments/d0107_deeplab_dati_video/yolo.png", frame);
    //waitKey(1);
     
    c = waitKey(0);
    if (c == 'g') {
      process = false;
      // ho dovuto inserire qui questa chiamata perché mi dava errore la dichiarazione di aligned_depth_frame fuori dal while
      hole_camera[2] = getMeanZ(aligned_depth_frame);
      hole_camera.block<3,1>(0,0) = convert2Dto3D(aligned_depth_frame, getCenterOfBox(boxMin), hole_camera[2]);
      std::cout << "Terna camera\n" << hole_camera << std::endl;
    }
  }
  
  Eigen::MatrixXd Ace(4,4); Ace.setIdentity();

  //Ace << 0.01999945272, -0.9990596861, 0.03846772089, 0.05834485203,
  //      0.9997803621, 0.01974315714, -0.007031030191, -0.03476564525,
  //      0.006264944557, 0.03859988867, 0.999235107, -0.06760482074,
  //      0, 0, 0, 1;

  //Ace << 0.01298035989,-0.9995171628,0.02823033132,0.05834485203, //0.0554335415,
  //      0.999908251,0.01308444161,0.003505272453,-0.03476564525, //-0.03373612252,
  //      -0.003872958099,0.02818224152,0.9995952988,-0.05430977191,
  //      0,0,0,1;

  // 10 giugno 2022
  //Ace << -0.008600103109, -0.9992217746, 0.0384952386, 0.06420071793,
  //       0.9999535015, -0.00842569125, 0.004690685436, -0.0373334897,
  //       -0.004362686031, 0.03853378901, 0.9992477741, -0.05248410703,
  //       0, 0, 0, 1;

  // 20 giugno 2022
  //Ace << -0.003670042231, -0.9992791083, 0.03778616916, 0.06342325983,
  //      0.9998983585, -0.004187657083, -0.013628506, -0.03885759766,
  //      0.01377691684, 0.03773231133, 0.9991929089, -0.0551657741,
  //      0,0,0,1;

  // 21 giugno 2022
  //Ace << -0.005154148655, -0.999188386, 0.03995001796, 0.06442325983, //0.06631491263,
  //      0.9999862925, -0.005113210083, 0.001126854918, -0.03744008249,
  //      -0.0009216675125, 0.03995527832, 0.999201044, -0.05384408518,
  //      0, 0, 0, 1;

  // 1 luglio 2022
  Ace << -0.007156772622,-0.9992723059,0.0374651753,0.06607069231,
          0.9999683998,-0.007022023019,0.003727016664,-0.0388433722,
         -0.003461223213, 0.0374906648, 0.9992909836, -0.05311303156,
          0, 0, 0, 1;

  Eigen::VectorXd foro_s_EE(4);
  foro_s_EE << 0,0,0,1;
  foro_s_EE = Ace*hole_camera;
  std::string filepath15 = path + "/dati_per_errore.txt"; 
  FILE* file15_ = fopen(filepath15.c_str(), "a");
  fprintf(file15_, "pe = [%f %f %f 1]';\n", foro_s_EE[0], foro_s_EE[1], foro_s_EE[2]);
  fclose(file15_);
  std::cout << "Terna EE\n" << Ace*hole_camera << std::endl;

  hole_camera = Ae*Ace*hole_camera;
  hole_camera[0] = 0.5193; 
  hole_camera[1] = 0.3997;
  hole_camera[2] = 0.2184;
  std::cout << "Terna mondo\n" << hole_camera << std::endl;
  pipe.stop();

  return hole_camera;

}

// in questo caso la distanza devo passarla io perche' in corrispondenza del centro del foro non c'e' la distanza
// quindi uso quella che ho calcolato prima
Eigen::Vector3d convert2Dto3D(const rs2::depth_frame& frame, Point pixel, double distance){
  float upixel[2]; // From pixel
  float upoint[3]; // From point (in 3D)

  // Copy pixels into the arrays (to match rsutil signatures)
  upixel[0] = static_cast<float>(pixel.x);
  upixel[1] = static_cast<float>(pixel.y);

  auto udist = frame.get_distance(static_cast<int>(upixel[0]), static_cast<int>(upixel[1]));
  if(udist < 0.01)
    udist = distance;

  // Deproject from pixel to point in 3D
  rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
  rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);

  return Eigen::Vector3d(upoint[0], upoint[1], upoint[2]);
}

void preprocess(const Mat& frame, Net& net, Size inpSize, float scale, const Scalar& mean_v, bool swapRB) {
    static Mat blob;
    // Create a 4D blob from a frame.
    if (inpSize.width <= 0) inpSize.width = frame.cols;
    if (inpSize.height <= 0) inpSize.height = frame.rows;
    blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false, CV_8U);

    // Run a model.
    net.setInput(blob, "", scale, mean_v);
    if (net.getLayer(0)->outputNameToIndex("im_info") != -1)  // Faster-RCNN or R-FCN
    {
        resize(frame, frame, inpSize);
        Mat imInfo = (Mat_<float>(1, 3) << inpSize.height, inpSize.width, 1.6f);
        net.setInput(imInfo, "im_info");
    }
}

void postprocessAndFindCenterHole(Mat& frame, const std::vector<Mat>& outs, Net& net, rs2::points& points) {
  static std::vector<int> outLayers = net.getUnconnectedOutLayers();
  static std::string outLayerType = net.getLayer(outLayers[0])->type;

  std::vector<int> classIds;
  std::vector<float> confidences;
  std::vector<Rect> boxes;
  if (outLayerType == "Region"){
    for (size_t i = 0; i < outs.size(); ++i) {
      // Network produces output blob with a shape NxC where N is a number of
      // detected objects and C is a number of classes + 4 where the first 4
      // numbers are [center_x, center_y, width, height]
      float* data = (float*)outs[i].data;
      for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
        Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
        Point classIdPoint;
        double confidence;
        minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
        if (confidence > confThreshold) {
          int centerX = (int)(data[0] * frame.cols);
          int centerY = (int)(data[1] * frame.rows);
          int width = (int)(data[2] * frame.cols);
          int height = (int)(data[3] * frame.rows);
          int left = centerX - width / 2;
          int top = centerY - height / 2;

          classIds.push_back(classIdPoint.x);
          confidences.push_back((float)confidence);
          boxes.push_back(Rect(left, top, width, height));
        }
      }
    }
  }
  else{
    CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);
  }


  std::vector<int> indices;
  NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
  double min_distance = 1000.0;
  int min_idx;
  Point centerBox;
  double d =0.0;
  std::cout << "Outs size: " <<  outs.size() << std::endl;
  std::cout << "Indices size: " <<  indices.size() << std::endl;
  std::cout << "Boxes size: " <<  boxes.size() << std::endl;
  for (size_t i = 0; i < indices.size(); ++i) {
    int idx = indices[i];
    Rect box = boxes[idx];
    // necessario per disegnare i box
    drawPredWithoutText(classIds[idx], confidences[idx], box.x, box.y, box.x + box.width, box.y + box.height, frame);
        
    centerBox = getCenterOfBox(box); 
    std::cout << "Center of box: " <<  centerBox << std::endl;
    d = getDistanceBetweenCenters(centerBox, Point(width/2, height/2));
    std::cout << "Distance between center: " <<  d << std::endl;
    if(d < min_distance){
      min_distance = d;
      min_idx = idx;
    }
  }
  boxMin = boxes[min_idx];
  std::cout << "BOX PIU' VICINO AL CENTRO\n" << boxMin.x << " " << boxMin.y << " " << boxMin.x + boxMin.width << " " << boxMin.y + boxMin.height << std::endl;
  std::cout << "Press 'g' to continue, any other key to detect again" << std::endl;
}

double getMeanZ(rs2::depth_frame aligned_depth_frame) {
  int radius = 100;
  int Left = (int)boxMin.x - radius;
  int Right = Left + radius * 2;
  int Top = (int)boxMin.y - radius;
  int Bottom = Top + radius * 2;

  double acc = 0.0;
  int count = 0;
  double z_temp = 0.0;
  rs2_error** err;

  for (int j = Top; j <= Bottom; ++j) {
    for (int k = Left; k <= Right; ++k) {
      double dist = pow(boxMin.x - k, 2.0) + pow(boxMin.y - j, 2.0);
      if (dist <= pow(radius, 2)) {
        z_temp = rs2_depth_frame_get_distance((rs2_frame*)aligned_depth_frame, k, j, err);
        //if(j==Top || j==Bottom || k==Left || k==Right)
        //  std::cout << "308 z: " << z_temp << std::endl;
        if(z_temp > 0.005){
          count++;
          acc += z_temp;
        }
      }
    }
  }
  std::cout << "ultima z_temp: " << z_temp  << " media: " << acc/count << endl;
  return acc/count;
}

Point getCenterOfBox(Rect box) {
  return Point(box.x + (box.width/2), box.y + (box.height/2));
}

double getDistanceBetweenCenters(Point c1, Point c2) {
  double distance = norm(c1-c2);
  return distance;
}

void drawPredWithoutText(int classId, float conf, int left, int top, int right, int bottom, Mat& frame){
  rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

  std::string label = format("%.2f", conf);
  if (!classes.empty()) {
      CV_Assert(classId < (int)classes.size());
      label = classes[classId] + ": " + label;
  }

  int baseLine;
  Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

  top = max(top, labelSize.height);
  //rectangle(frame, Point(left, top - labelSize.height), Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
  //putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
}

cv::Mat frame_to_mat(const rs2::frame& f) {
  using namespace cv;
  using namespace rs2;

  auto vf = f.as<video_frame>();
  const int w = vf.get_width();
  const int h = vf.get_height();

  if (f.get_profile().format() == RS2_FORMAT_BGR8) {
    return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_RGB8) {
    auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    cvtColor(r, r, COLOR_RGB2BGR);
    return r;
  } else if (f.get_profile().format() == RS2_FORMAT_Z16) {
    return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_Y8) {
    return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
  } else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32) {
    return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
  }

  throw std::runtime_error("Frame format is not supported yet!");
}




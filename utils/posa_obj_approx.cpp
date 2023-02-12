// Copyright (c) 2017 Franka Emika GmbH

/**
 * @unibas_single_robot ik.cpp
 * Inverse Kinematic. Muove il robot in una posa desiderata (specificata a riga 133) usando il calcolo dell'inversione cinematica
 */

// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// *************** LIBRERIE
  #include <iostream>
  #include <franka/exception.h>
  #include <franka/robot.h>
  #include <franka/gripper.h>
  #include <franka/model.h>
  #include "common_p.h"
  #include <Eigen/Core>
  #include <Eigen/Geometry>
  #include <Eigen/LU>
  #include <Eigen/QR>
  #include <Eigen/Dense>
  #include <math.h>

  //#include "dynModel.h"
  #include "utils_pc.hpp"
  #include "detection_rs.hpp"
  #include "utils_pc.hpp"
  
  #include <ctime>
  #include <stdio.h>
  #include <string>
  
  //#include "opencv2/highgui/highgui.hpp"
  //#include "opencv2/imgproc/imgproc.hpp"
  #include <librealsense2/rs.hpp> 
  #include <librealsense2/rsutil.h>

  //#include "MatlabEngine.hpp"
  //#include "MatlabDataArray.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;
//using namespace matlab::engine;
using namespace rs2;

FILE* file;


//argv: 
int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> "  << std::endl;
        return -1;
    }

    std::string robot_ip = argv[1];

    franka::Robot robot(robot_ip);
    franka::Gripper gripper(robot_ip);
    franka::RobotState robot_state = robot.readOnce();

    cout << "******* CALCOLO POSIZIONE APPROSSIMATIVA" << endl;
    Eigen::MatrixXd eMc;
    eMc.setIdentity(4,4);
    eMc << -0.01232433233,  0.999870768,  0.01032270411,  0.02208447947,
           -0.9999239627,  -0.01232805105,  0.0002966902714,  0.02902996678-0*0.021,
           0.0004239107528,  -0.01031826269,  0.9999466755,  -0.05600550771,
           0,  0,  0,  1;
 
    pipeline pipe = startCamera();
    Eigen::MatrixXd cMo = detectPoseObject(pipe);
    //cout << "cMo\n" << cMo << endl; 
    Eigen::Map<const Eigen::Matrix<double, 4, 4> > wMe(robot_state.O_T_EE.data());
    //cout << "wMe\n" << wMe << endl<< endl; 
    //cout << "wMc\n" << wMe * eMc << endl <<endl; 
 
    Eigen::MatrixXd oMw;
    oMw = wMe * eMc * cMo;
    file = fopen("/home/labarea-franka/libfranka/unibas_peginhole/resources/posa_obj.txt", "w");
    fprintf(file, "%f %f %f\n", oMw(0,3), oMw(1,3), oMw(2,3));
    fprintf(file, "Rd\n");
    fprintf(file, "%f %f %f\n", oMw(0,0), oMw(0,1), oMw(0,2));
    fprintf(file, "%f %f %f\n", oMw(1,0), oMw(1,1), oMw(1,2));
    fprintf(file, "%f %f %f\n", oMw(2,0), oMw(2,1), oMw(2,2));
    fclose(file);
    return 0;
}

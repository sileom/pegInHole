// *************** LIBRERIE
  #include <iostream>
  //#include <franka/exception.h>
  //#include <franka/robot.h>
  //#include <franka/gripper.h>
  //#include <franka/model.h>
  //#include "common_p.h"
  #include <Eigen/Core>
  #include <Eigen/Geometry>
  #include <Eigen/LU>
  #include <Eigen/QR>
  #include <Eigen/Dense>
  #include <math.h>

  //#include "dynModel.h"
  #include "utils_pc.hpp"
  //#include "detection_rs.hpp"
  //#include "utils_pc.hpp"
  
  #include <ctime>
  #include <stdio.h>
  #include <string>
  
  //#include "opencv2/highgui/highgui.hpp"
  //#include "opencv2/imgproc/imgproc.hpp"

  #include "MatlabEngine.hpp"
  #include "MatlabDataArray.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace matlab::engine;

FILE* file_i;


//argv: no
int main() {
    
    file_i = fopen("/home/labarea-franka/libfranka/unibas_peginhole/resources/posa_obj.txt", "r");
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    char stringa[80];
    int res;
    
    res = fscanf(file_i,"%s",stringa);
    p[0] = atof(stringa);
    res = fscanf(file_i,"%s",stringa);
    p[1] = atof(stringa);
    res = fscanf(file_i,"%s",stringa);
    p[2] = atof(stringa);
    res = fscanf(file_i,"%s",stringa); // riga Rd

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            res = fscanf(file_i,"%s",stringa);
            R(i,j) = atof(stringa);
        }
    }
    //cout << p << endl << endl;
    //cout << R << endl << endl;
    fclose(file_i);

    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();
    matlab::data::ArrayFactory factory;

    std::vector<matlab::data::Array> args({ factory.createArray<double>({ 3, 1 }, { p[0], p[1], p[2] }),
                                            factory.createArray<double>({ 3, 3 }, { R(0,0), R(1,0), R(2,0), R(0,1), R(1,1), R(2,1), R(0,2), R(1,2), R(2,2) })
                                            });    
    matlab::data::Array result = matlabPtr->feval(u"punti_scan_obj", args);
    //int16_t v = result[0];
    //std::cout << "Result: " << v << std::endl;
    
    return 0;
}
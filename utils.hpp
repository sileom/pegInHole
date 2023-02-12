// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.
#pragma once

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"          // Include short list of convenience functions for rendering

#include <algorithm>            // std::min, std::max

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include "common_i.h"
#include "dynModel_i.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Dense>
#include <math.h>
#include <thread>

#include <ctime>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>


using namespace Eigen;
using namespace std;


Eigen::MatrixXd fk(std::vector<double> a, std::vector<double> d, std::vector<double> alpha, std::vector<double> theta);
Eigen::MatrixXd DH(double a,double alpha,double d,double theta);
int sgn(double v);
Eigen::VectorXd getQuaternion(std::array<double, 16> pose);
Eigen::MatrixXd pinv(Eigen::MatrixXd J);
Eigen::VectorXd getColumn(int colonna, Eigen::MatrixXd *M);
Eigen::MatrixXd setColumn(int colonna, Eigen::MatrixXd M, Eigen::VectorXd v);
Eigen::MatrixXd setRow(int row, Eigen::MatrixXd M, Eigen::VectorXd v);
Eigen::VectorXd getVelocityProfile(double p_i, double p_f, double v_i, double v_f, double t_f, double t_i);
Eigen::VectorXd getPositionProfile(double p_i, double p_f, double v_i, double v_f, double t_f, double t_i);
Eigen::MatrixXd getJacobianMatrix(std::array<double, 42> jacobian_array);

Eigen::VectorXd getColJ(Eigen::Vector3d pos, Eigen::MatrixXd T0i_1);
Eigen::MatrixXd getJ(Eigen::Vector3d posizione, Eigen::VectorXd theta);

Eigen::MatrixXd getR(std::array<double, 16> posa);
Eigen::VectorXd r2asseangolo(Eigen::MatrixXd R);
Eigen::MatrixXd asseangolo2r(Eigen::Vector3d r_theta, double angle);
Eigen::VectorXd ascissa(double tk, double tf);
Eigen::VectorXd getQuaternion(Eigen::MatrixXd R);
Eigen::MatrixXd rotazioneElementari(int num, double angolo);

//Eigen::VectorXd getErrorWithMatrix(std::array<double,16> posa_endEffector, std::array<double,16> posa_desiderata);
Eigen::MatrixXd getDiagonalMatrix(double size, Eigen::VectorXd diag);
Eigen::Vector3d r2rpy(Eigen::MatrixXd R1);
Eigen::MatrixXd rpy2r(Eigen::Vector3d phi);
Eigen::VectorXd getError(std::array<double,16> posa_endEffector, Eigen::VectorXd x_r);
Eigen::VectorXd getErrorWithMatrix(Eigen::MatrixXd R_ee, Eigen::MatrixXd R_r);

std::array<double, 6> getArrayFromEigenVector(Eigen::VectorXd e);
Eigen::Vector3d calcolaPolinomioAcc(double ti, double tc, std::array<double, 6> &coeff_vel_x, std::array<double, 6> &coeff_vel_y, std::array<double, 6> &coeff_vel_z);
Eigen::Vector3d calcolaPolinomioPos(double ti, double tc, std::array<double, 6> &coeff_vel_x, std::array<double, 6> &coeff_vel_y, std::array<double, 6> &coeff_vel_z);
Eigen::Vector3d calcolaPolinomioVel(double ti, double tc, std::array<double, 6> &coeff_vel_x, std::array<double, 6> &coeff_vel_y, std::array<double, 6> &coeff_vel_z);

void calcolaCoeffTraiettoriaVel(double p_i, double p_f, double v_i, double v_f, double t_i, double t_f, std::array<double, 3> &coeff);
void calcolaCoeffTraiettoriaPos(double p_i, double p_f, double v_i, double v_f, double t_i, double t_f, std::array<double, 4> &coeff);
Eigen::Vector3d calcolaPolVel(double ti, double tc, std::array<double, 3> &coeff_vel_x, std::array<double, 3> &coeff_vel_y, std::array<double, 3> &coeff_vel_z);
Eigen::Vector3d calcolaPolPos(double ti, double tc, std::array<double, 4> &coeff_pos_x, std::array<double, 4> &coeff_pos_y, std::array<double, 4> &coeff_pos_z);

Eigen::VectorXd filter_utils(Eigen::VectorXd f_n_1, Eigen::VectorXd f_ext, double alpha);
Eigen::MatrixXd skew(Eigen::Vector3d a);
Eigen::Vector3d pixel2point(Eigen::Vector2d pixel, double depth);

Eigen::MatrixXd readMatrix6x6(FILE* file);
Eigen::Matrix3d readMatrix(FILE* file2);

//bool joint_lim(Eigen::VectorXd q,Eigen::VectorXd qd, double deltaT);
Eigen::VectorXd joint_lim(Eigen::VectorXd q,Eigen::VectorXd qd, double deltaT);
Eigen::MatrixXd Ti(Eigen::Vector3d phiv);

int countElement(FILE* file);
void poseAdjustment(std::string filename, std::string fileM);


// **************** VARIABILI GLOBALI
std::vector<double> a = {0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0};
std::vector<double> d = {0.333, 0, 0.316, 0.0, 0.384, 0.0, 0.107};
std::vector<double> alpha = {-M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0.0};
//durata del passo di campionamento [s]
const double delta_t = 0.001;
//tempo di inizio
//const double t_i = 0.0;
//tempo di assestamento per essere sicuri che l'errore vada a zero
double t_ass = 1;//1.5;
//costanti per indicare l'asse di rotazione nelle rotazioni elementari
const int ROTX = 1;
const int ROTY = 2;
const int ROTZ = 3;

// *************** PARAMETRI INTRINSECI
double px = 578.00549546;// 613.6060181;
double py = 575.42666411; //613.7553711;
double u0 = 327.06022754;//324.6341248;
double v0 = 238.04706309;//235.6944733;
double kud = 0.0; 
double kdu = 0.0;

void saveFile(std::string filename, Eigen::MatrixXd P, std::vector<Eigen::Matrix3d> Rf_vec){
  Eigen::Matrix3d Rc;
  FILE* file;
  file = fopen(filename.c_str(), "w");
  int numPti = Rf_vec.size();
  fprintf(file, "%d 10\n", numPti);
  for(int i=0; i < numPti; i++){
    fprintf(file, "%f %f %f\n", P(i,0), P(i,1), P(i,2));
    Rc = Rf_vec[i];
    fprintf(file, "Rd\n");
    fprintf(file, "%f %f %f\n%f %f %f\n%f %f %f\n", Rc(0,0),Rc(0,1),Rc(0,2), Rc(1,0),Rc(1,1),Rc(1,2), Rc(2,0),Rc(2,1),Rc(2,2));
  }
  fclose(file);
  return;
}

void poseAdjustment_old(std::string filename, std::string fileM){
  //LEGGO I PUNTI VECCHI
  FILE* file2;
  file2 = fopen(filename.c_str(), "r");
  Eigen::MatrixXd P;
  char stringa[80];
  int res;
  int i = 0;
  int j = 0;
  
  res = fscanf(file2,"%s",stringa);
  int numPti = stoi(stringa);
  res = fscanf(file2,"%s",stringa);
  double tempo = stod(stringa);
  std::vector<Eigen::Matrix3d> Rf_vec(numPti); 
  P.resize(numPti, 3);
  Eigen::Matrix3d Rd;

  res = fscanf(file2,"%s",stringa);
  while(i < numPti && res != EOF) {
    if(strcmp(stringa, "Rd") == 0){
      Rd = readMatrix(file2);
      Rf_vec[i] = Rd;
      j = 0;
      i++;
    } else {
      P(i,j) = atof(stringa);
      j++;
    }
    res = fscanf(file2,"%s",stringa);
  }
  fclose(file2);
  cout << P << endl;
  for(int k=0; k<Rf_vec.size(); k++){
    cout << Rf_vec[k] << endl;
  }

  //LEGGO I PUNTI MISURATI
  FILE* file1;
  file1 = fopen(fileM.c_str(), "r");
  Eigen::MatrixXd Pm(3,4);

  res = fscanf(file1,"%s",stringa);
  cout << stringa << endl;
  i = 0; j = 0;
  while(res != EOF) {
    if(j < 4){
      Pm(i,j) = atof(stringa);
      j++;
    } else{
      j = 0;
      i++;
      Pm(i,j) = atof(stringa);
      j++;
    }
    res = fscanf(file1,"%s",stringa);
    //cout << stringa << endl;
  }
  fclose(file1);
  cout << endl << "PM" << endl << Pm << endl;

  Eigen::Vector3d xm, vm, zm, ym;
  xm = Pm.block<1,3>(0,0) - Pm.block<1,3>(1,0); xm = xm /xm.norm();
  vm = Pm.block<1,3>(2,0) - Pm.block<1,3>(1,0); vm = vm /vm.norm();
  zm = vm.cross(xm); zm = zm/zm.norm();
  ym = zm.cross(xm); 

  cout << "vm" << endl << vm << endl;
  cout << "xm" << endl << xm << endl;
  cout << "ym" << endl << ym << endl;
  cout << "zm" << endl << zm << endl;

  Eigen::Vector3d xc, vc, zc, yc;
  xc = P.block<1,3>(0,0) - P.block<1,3>(1,0); xc = xc /xc.norm();
  vc = P.block<1,3>(2,0) - P.block<1,3>(1,0); vc = vc /vc.norm();
  zc = vc.cross(xc); zc = zc/zc.norm();
  yc = zc.cross(xc); 

  cout << "vc" << endl << vc << endl;
  cout << "x" << endl << xc << endl;
  cout << "y" << endl << yc << endl;
  cout << "z" << endl << zc << endl;

  Eigen::MatrixXd Am = Eigen::MatrixXd::Identity(4,4);
  Am.block<3,1>(0,0) = xm;
  Am.block<3,1>(0,1) = ym;
  Am.block<3,1>(0,2) = zm;
  Am.block<3,1>(0,3) = Pm.block<1,3>(1,0);
  cout << "Am" << endl << Am << endl;

  Eigen::MatrixXd Ac = Eigen::MatrixXd::Identity(4,4);
  Ac.block<3,1>(0,0) = xc;
  Ac.block<3,1>(0,1) = yc;
  Ac.block<3,1>(0,2) = zc;
  Ac.block<3,1>(0,3) = P.block<1,3>(1,0);
  cout << "Ac" << endl << Ac << endl;

  Eigen::MatrixXd Am_c(4,4);
  Am_c = Am.inverse() * Ac;
  cout << "am_c" << endl << Am_c << endl;
  Eigen::MatrixXd Padj(6,3);
  Eigen::VectorXd pc(4); pc[3]=1;
  for(int i = 0; i<6; i++){
    pc.head(3) = P.block<1,3>(i,0);
    Padj.block<1,3>(i,0) =  pc.head(3) + Am_c.block<3,3>(0,0)*Am_c.block<3,1>(0,3);
    P.block<1,3>(i,0) = Padj.block<1,3>(i,0);
  }
  cout << "PAdj" << endl << Padj << endl;

  saveFile(filename, P, Rf_vec);

  return;
}

void poseAdjustment(std::string filename, std::string fileM){
  //LEGGO I PUNTI VECCHI
  FILE* file2;
  file2 = fopen(filename.c_str(), "r");
  Eigen::MatrixXd P;
  char stringa[80];
  int res;
  int i = 0;
  int j = 0;
  
  res = fscanf(file2,"%s",stringa);
  int numPti = stoi(stringa);
  res = fscanf(file2,"%s",stringa);
  double tempo = stod(stringa);
  std::vector<Eigen::Matrix3d> Rf_vec(numPti); 
  P.resize(numPti, 3);
  Eigen::Matrix3d Rd;

  res = fscanf(file2,"%s",stringa);
  while(i < numPti && res != EOF) {
    if(strcmp(stringa, "Rd") == 0){
      Rd = readMatrix(file2);
      Rf_vec[i] = Rd;
      j = 0;
      i++;
    } else {
      P(i,j) = atof(stringa);
      j++;
    }
    res = fscanf(file2,"%s",stringa);
  }
  fclose(file2);
  cout << P << endl;
  for(int k=0; k<Rf_vec.size(); k++){
    cout << Rf_vec[k] << endl;
  }

  //LEGGO I PUNTI MISURATI
  FILE* file1;
  file1 = fopen(fileM.c_str(), "r");
  Eigen::MatrixXd Pm(3,4);

  res = fscanf(file1,"%s",stringa);
  cout << stringa << endl;
  i = 0; j = 0;
  while(res != EOF) {
    if(j < 4){
      Pm(i,j) = atof(stringa);
      j++;
    } else{
      j = 0;
      i++;
      Pm(i,j) = atof(stringa);
      j++;
    }
    res = fscanf(file1,"%s",stringa);
    //cout << stringa << endl;
  }
  fclose(file1);
  cout << endl << "PM" << endl << Pm << endl;

  // PM punti misurati 3x4
  // P  punti stimati  numPtix3

  Eigen::MatrixXd PM_ = Pm.block<3,3>(0,0); // 3x3 misurati
  Eigen::MatrixXd P_ = P.block<3,3>(0,0); // 3x3 stimati

  Eigen::MatrixXd diff = PM_ - P_;
  Eigen::Vector3d acc;
  Eigen::Vector3d meanError;
  acc << 0.0, 0.0, 0.0;
  for(int i = 0; i<3; i++){
    acc[i] += (diff(0,i) + diff(1,i) + diff(2,i));
  }
  for(int i = 0; i<3; i++){
    meanError[i] = acc[i]/3.0;
  }

  Eigen::MatrixXd P_new(numPti,3);
  for(int i = 0; i<numPti; i++){
    if(i == 2)
      P_new.block<1,3>(i,0) = P.block<1,3>(i,0) + (Rf_vec[i].transpose()*meanError).transpose();
    else
      P_new.block<1,3>(i,0) = P.block<1,3>(i,0) + meanError.transpose();
  }

  std::cout << "\nMedia errore: " << meanError << "\n\n" << P_new << "\n\n";

  saveFile(filename, P_new, Rf_vec);

  return;
}

int countElement(FILE* file){
  char stringa[80];
  int res;
  int num = 0;
  res = fscanf(file,"%s",stringa);
  while(res != EOF) {
    num++;
    res = fscanf(file,"%s",stringa);
  }
  return num;
}

Eigen::MatrixXd Ti(Eigen::Vector3d phiv){
  Eigen::MatrixXd T(3,3);
  double phi = phiv[0];
  double theta = phiv[1];
  //double psii = phiv[2];
  T(0,0) = 1; T(0,1) = 0;           T(0,2) = sin(theta); 
  T(1,0) = 0; T(1,1) = cos(phi); T(1,2) = -cos(theta)*sin(phi);
  T(2,0) = 0; T(2,1) = sin(phi); T(2,2) = cos(phi)*cos(theta); 
  return T;
}

Eigen::VectorXd joint_lim(Eigen::VectorXd q,Eigen::VectorXd qd, double deltaT){
  Eigen::VectorXd qLimMin(7);
  Eigen::VectorXd qLim(7); // giunti al limite
  qLimMin <<-2.81,-1.67,-2.81,-2.98,-2.50,0.07,-2.81;
  Eigen::VectorXd qLimMax(7);
  qLimMax <<2.81,1.67,2.81,-0.16,2.81,3.67,2.81;
  Eigen::VectorXd q_comm(7);
  q_comm=q+qd*deltaT;
  for (int i = 0; i < 7; i++){
    if(q_comm[i]<qLimMin[i] || q_comm[i]>qLimMax[i]){
      qLim[i]=1;
    }else{
      qLim[i]=0;
    }      
  }
  return qLim;
}

Eigen::Vector3d pixel2point(Eigen::Vector2d pixel, double depth){
  //double r_quad = pow(((pixel[0] - u0)/px),2) + pow(((pixel[1] - v0)/py),2);
  Eigen::Vector3d result;
  result[0] = (pixel[0] - u0)/px;//(pixel[0] - u0) * (1+ (kdu*r_quad))/px;
  result[1] = (pixel[1] - v0)/py;//(pixel[1] - v0) * (1+ (kdu*r_quad))/py;
  result[2] = depth;
  return result;
}

Eigen::Matrix3d readMatrix(FILE* file2){
  char stringa[80];
  int res;
  int k = 0;
  int c = 0;
  Eigen::Matrix3d R;

  while(k < 3) {
    res = fscanf(file2,"%s",stringa);
    R(k,c) = atof(stringa);
    c++;
    if(c == 3){
      c = 0;
      k++;
    }
  }
  return R;
}



Eigen::MatrixXd readMatrix6x6(FILE* file){
  char stringa[80];
  int res;
  int k = 0;
  int c = 0;
  Eigen::MatrixXd R(6,6);

  while(k < 6) {
    res = fscanf(file,"%s",stringa);
    R(k,c) = atof(stringa);
    c++;
    if(c == 6){
      c = 0;
      k++;
    }
  }
  return R;
}

Eigen::MatrixXd skew(Eigen::Vector3d a){
  Eigen::Matrix3d S;
  S <<    0,       -1*a[2],       a[1],
         a[2],       0,         -1*a[0],
       -1*a[1],     a[0],          0;
  return S;
}

Eigen::VectorXd filter_utils(Eigen::VectorXd f_n_1, Eigen::VectorXd f_ext, double alpha_filt){
    Eigen::VectorXd ff(6);
    for (int i = 0; i < f_ext.size(); i++) {
        ff[i] = alpha_filt*f_ext[i] + (1-alpha_filt)*f_n_1[i];
    }
    return ff;
}


Eigen::MatrixXd getJacobianMatrix(std::array<double, 42> jacobian_array){
  Eigen::MatrixXd J(6,7);

  /*J(0, 0) = jacobian_array[0];
  J(0, 1) = jacobian_array[1];
  J(0, 2) = jacobian_array[2];
  J(0, 3) = jacobian_array[3];
  J(0, 4) = jacobian_array[4];
  J(0, 5) = jacobian_array[5];

  J(1, 0) = jacobian_array[6];
  J(1, 1) = jacobian_array[7];
  J(1, 2) = jacobian_array[8];
  J(1, 3) = jacobian_array[9];
  J(1, 4) = jacobian_array[10];
  J(1, 5) = jacobian_array[11];

  J(2, 0) = jacobian_array[12];
  J(2, 1) = jacobian_array[13];
  J(2, 2) = jacobian_array[14];
  J(2, 3) = jacobian_array[15];
  J(2, 4) = jacobian_array[16];
  J(2, 5) = jacobian_array[17];

  J(3, 0) = jacobian_array[18];
  J(3, 1) = jacobian_array[19];
  J(3, 2) = jacobian_array[20];
  J(3, 3) = jacobian_array[21];
  J(3, 4) = jacobian_array[22];
  J(3, 5) = jacobian_array[23];

  J(4, 0) = jacobian_array[24];
  J(4, 1) = jacobian_array[25];
  J(4, 2) = jacobian_array[26];
  J(4, 3) = jacobian_array[27];
  J(4, 4) = jacobian_array[28];
  J(4, 5) = jacobian_array[29];

  J(5, 0) = jacobian_array[30];
  J(5, 1) = jacobian_array[31];
  J(5, 2) = jacobian_array[32];
  J(5, 3) = jacobian_array[33];
  J(5, 4) = jacobian_array[34];
  J(5, 5) = jacobian_array[35];

  J(6, 0) = jacobian_array[36];
  J(6, 1) = jacobian_array[37];
  J(6, 2) = jacobian_array[38];
  J(6, 3) = jacobian_array[39];
  J(6, 4) = jacobian_array[40];
  J(6, 5) = jacobian_array[41];
  */
  J(0, 0) = jacobian_array[0];
  J(1, 0) = jacobian_array[1];
  J(2, 0) = jacobian_array[2];
  J(3, 0) = jacobian_array[3];
  J(4, 0) = jacobian_array[4];
  J(5, 0) = jacobian_array[5];

  J(0, 1) = jacobian_array[6];
  J(1, 1) = jacobian_array[7];
  J(2, 1) = jacobian_array[8];
  J(3, 1) = jacobian_array[9];
  J(4, 1) = jacobian_array[10];
  J(5, 1) = jacobian_array[11];

  J(0, 2) = jacobian_array[12];
  J(1, 2) = jacobian_array[13];
  J(2, 2) = jacobian_array[14];
  J(3, 2) = jacobian_array[15];
  J(4, 2) = jacobian_array[16];
  J(5, 2) = jacobian_array[17];

  J(0, 3) = jacobian_array[18];
  J(1, 3) = jacobian_array[19];
  J(2, 3) = jacobian_array[20];
  J(3, 3) = jacobian_array[21];
  J(4, 3) = jacobian_array[22];
  J(5, 3) = jacobian_array[23];

  J(0, 4) = jacobian_array[24];
  J(1, 4) = jacobian_array[25];
  J(2, 4) = jacobian_array[26];
  J(3, 4) = jacobian_array[27];
  J(4, 4) = jacobian_array[28];
  J(5, 4) = jacobian_array[29];

  J(0, 5) = jacobian_array[30];
  J(1, 5) = jacobian_array[31];
  J(2, 5) = jacobian_array[32];
  J(3, 5) = jacobian_array[33];
  J(4, 5) = jacobian_array[34];
  J(5, 5) = jacobian_array[35];

  J(0, 6) = jacobian_array[36];
  J(1, 6) = jacobian_array[37];
  J(2, 6) = jacobian_array[38];
  J(3, 6) = jacobian_array[39];
  J(4, 6) = jacobian_array[40];
  J(5, 6) = jacobian_array[41];
  
  return J;
} 

Eigen::MatrixXd fk(std::vector<double> a, std::vector<double> d, std::vector<double> alpha, std::vector<double> theta){
  Eigen::MatrixXd T;
  T.setIdentity(4,4);
  for(unsigned int i = 0; i < theta.size(); i++){
    Eigen::MatrixXd Ai = DH(a[i], alpha[i], d[i], theta[i]);
    T = T*Ai;
  }
  //MOLTIPLICAZIONE DA AGGIUNGERE SE LA PINZA È MONTATA
  Eigen::MatrixXd A_EE(4, 4);
  A_EE(0, 0) = 0.7073883;   A_EE(0, 1) = 0.7068252;   A_EE(0, 2) = 0; A_EE(0, 3) = 0;
  A_EE(1, 0) = -0.7068252;  A_EE(1, 1) = 0.7073883;   A_EE(1, 2) = 0; A_EE(1, 3) = 0;
  A_EE(2, 0) = 0;       A_EE(2, 1) = 0;       A_EE(2, 2) = 1; A_EE(2, 3) = 0.1034;
  A_EE(3, 0) = 0;       A_EE(3, 1) = 0;       A_EE(3, 2) = 0; A_EE(3, 3) = 1; 
  T = T * A_EE;
  return T;
}

Eigen::MatrixXd DH(double a,double alpha,double d,double theta){
  double ct,st,ca,sa;
  Eigen::MatrixXd T(4, 4);
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);
  
  T(0, 0) = ct;
  T(0, 1) = -st*ca;
  T(0, 2) = st*sa;
  T(0, 3) = a*ct;

  T(1, 0) = st;
  T(1, 1) = ct*ca;
  T(1, 2) = -ct*sa;
  T(1, 3) = st*a;

  T(2, 0) = 0;
  T(2, 1) = sa;
  T(2, 2) = ca;
  T(2, 3) = d;

  T(3, 0) = 0;
  T(3, 1) = 0;
  T(3, 2) = 0;
  T(3, 3) = 1;

  return T;
}


int sgn(double v){
  return (v > 0 ) ? 1 : ((v < 0) ? -1 : 0);
}

Eigen::VectorXd getQuaternion(std::array<double, 16> pose){
  //double eta = 0.5*sqrt(pose[0]+pose[5]+pose[10]+1);
  //double epsilon_x = 0.5*sgn(pose[6]-pose[9])*sqrt(pose[0]-pose[5]-pose[10]+1);
  //double epsilon_y = 0.5*sgn(pose[8]-pose[2])*sqrt(pose[5]-pose[10]-pose[0]+1);
  //double epsilon_z = 0.5*sgn(pose[1]-pose[4])*sqrt(pose[10]-pose[0]-pose[5]+1);
  Eigen::VectorXd quaternion(4);
  //quaternion << eta, epsilon_x, epsilon_y, epsilon_z;
  //return quaternion; 
  double r1 = pose[0];
  double r2 = pose[5];
  double r3 = pose[10];
  double r4 = r1+r2+r3;
  int j = 1;
  double rj = r1;
  if(r2>rj){
    j=2;
    rj=r2;
  }
  if(r3>rj){
    j=3;
    rj=r3;
  }
  if(r4>rj){
    j=4;
    rj=r4;
  }  
  double pj = 2*sqrt(1+2*rj-r4);
  double p1, p2, p3, p4;
  if(j==1){
    p1 = pj/4;
    p2 = (pose[1] + pose[4])/pj; 
    p3 = (pose[8] + pose[2])/pj; 
    p4 = (pose[6] - pose[9])/pj; 
  } else if (j==2){
    p1 = (pose[1] + pose[4])/pj;
    p2 = pj/4;
    p3 = (pose[6] + pose[9])/pj;
    p4 = (pose[8] - pose[2])/pj;
  } else if (j ==3) {
    p1 = (pose[8] + pose[2])/pj;
    p2 = (pose[6] + pose[9])/pj;
    p3 = pj/4;
    p4 = (pose[1] - pose[4])/pj;
  } else {
    p1 = (pose[6] - pose[9])/pj;
    p2 = (pose[8] - pose[2])/pj;
    p3 = (pose[1] - pose[4])/pj;
    p4 = pj/4;
  }
  if(p4>0){
    quaternion[1] = p1;
    quaternion[2] = p2;
    quaternion[3] = p3;
    quaternion[0] = p4;
  } else {
    quaternion[1] = -p1;
    quaternion[2] = -p2;
    quaternion[3] = -p3;
    quaternion[0] = -p4;
  }
  return quaternion;

}

Eigen::VectorXd getErrorWithMatrix(Eigen::MatrixXd R_ee, Eigen::MatrixXd R_r){
  Eigen::MatrixXd R = R_ee;
  Eigen::MatrixXd Rd = R_r;
  Eigen::MatrixXd Rtilde = Rd * R.transpose();
  Eigen::VectorXd phi(3);
  phi[0] = 0.5 * (Rtilde(2,1) - Rtilde(1,2));
  phi[1] = 0.5 * (Rtilde(0,2) - Rtilde(2,0));
  phi[2] = 0.5 * (Rtilde(1,0) - Rtilde(0,1));
  return phi;
}

//Eigen::VectorXd getError(std::array<double,16> posa_endEffector, std::array<double,16> posa_desiderata)

Eigen::VectorXd getError(std::array<double,16> posa_endEffector, Eigen::VectorXd x_r){
  Eigen::Vector3d pos_x_d = Eigen::Vector3d(x_r[0],x_r[1],x_r[2]);

  Eigen::Vector3d pos_x_e = Eigen::Vector3d(posa_endEffector[12],
        posa_endEffector[13],
        posa_endEffector[14]);

  Eigen::Vector3d errore_pos = pos_x_d - pos_x_e;
/*
  Eigen::VectorXd quaternion_ee = getQuaternion(posa_endEffector);
  Eigen::VectorXd quaternion_d = getQuaternion(posa_desiderata);

  double eta_e = quaternion_ee[0];
  Eigen::Vector3d epsilon_e = Eigen::Vector3d(quaternion_ee[1], quaternion_ee[2], quaternion_ee[3]);

  double eta_d = quaternion_d[0];
  Eigen::Vector3d epsilon_d = Eigen::Vector3d(quaternion_d[1], quaternion_d[2], quaternion_d[3]);

  Eigen::Matrix3d S;
  S <<        0,       -1*epsilon_d[2],       epsilon_d[1],
         epsilon_d[2],       0,             -1*epsilon_d[0],
      -1*epsilon_d[1],    epsilon_d[0],         0       ;
  
  Eigen::Vector3d v1 = eta_e*epsilon_d;
  Eigen::Vector3d v2 = eta_d*epsilon_e;
  Eigen::Vector3d v3 = S*epsilon_e;
  Eigen::Vector3d errore_o = v1-v2-v3;
*/
  Eigen::Vector3d phi;
  phi << x_r[3], x_r[4], x_r[5];
  Eigen::MatrixXd R_r = rpy2r(phi);
  Eigen::MatrixXd R_ee = getR(posa_endEffector);


  Eigen::VectorXd err_or = getErrorWithMatrix(R_ee, R_r);
 
  Eigen::VectorXd errore(6);
  errore << errore_pos[0], errore_pos[1], errore_pos[2], err_or[0], err_or[1], err_or[2];//errore_o[0], errore_o[1], errore_o[2];
  
  return errore;
}



//calcola l'inversa dello Jacobiano
Eigen::MatrixXd pinv(Eigen::MatrixXd jacobian){
  Eigen::HouseholderQR<Eigen::MatrixXd> qr(jacobian.transpose());
  Eigen::MatrixXd p_inv;
  p_inv.setIdentity(jacobian.cols(), jacobian.rows());
  p_inv = qr.householderQ() * p_inv;
  p_inv = qr.matrixQR().topLeftCorner(jacobian.rows(),jacobian.rows()).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(p_inv);
  //Eigen::MatrixXd temp = jacobian*jacobian.transpose();
  //Eigen::MatrixXd p_inv = jacobian.transpose() * (temp.inverse());
  return p_inv;
}



Eigen::VectorXd getColumn(int colonna, Eigen::MatrixXd *M){
  int rows = M->rows();
  Eigen::VectorXd q_iesimo(rows);
  for(int i=0; i < rows; i++){
    q_iesimo[i] = M->operator()(i, colonna);
  }
  return q_iesimo;
}

Eigen::MatrixXd setColumn(int colonna, Eigen::MatrixXd M, Eigen::VectorXd v){
  Eigen::MatrixXd temp = M;
  for(int i=0; i < M.rows(); i++){
    temp(i, colonna) = v(i);
    
  }
  return temp;
}

Eigen::MatrixXd setRow(int row, Eigen::MatrixXd M, Eigen::VectorXd v){
  Eigen::MatrixXd temp = M;
  for(int i=0; i < M.cols(); i++){
    temp(row, i) = v(i);
  }
  return temp;
}

//getVelocityProfile(posa_corrente.position.x, posa_desiderata.position.x, 0.0, 0.0, 1.0, 0.0);
Eigen::VectorXd getVelocityProfile(double p_i, double p_f, double v_i, double v_f, double t_f, double t_i){
  double T = t_f - t_i;
  //double a0 = p_i;
  double a1 = v_i;
  
  double a3_num = 2*(p_i-p_f)+(v_i+v_f)*T;
  double a3 = a3_num/pow(T,3.0);
  
  double a2_num = -3*(p_i-p_f)-(2*v_i+v_f)*T;
  double a2 = a2_num/pow(T,2.0);
  double step;
  
  int numPassi = floor(t_f/delta_t)+floor(t_ass/delta_t);
  
  double t = 0;
  Eigen::VectorXd profile(numPassi);
  for(int i = 0; i <= numPassi; i++){
    if(i >= floor(t_f/delta_t)){
      profile[i] = profile[floor(t_f/delta_t)-1];
    } else {
      step = t-t_i;
      double aa = 3*a3*pow(step,2.0)+ 2*a2*step + a1;
      profile[i] = aa; 
      t = t + delta_t;
    }
  }
  
  return profile;
}

Eigen::VectorXd getPositionProfile(double p_i, double p_f, double v_i, double v_f, double t_f, double t_i){
  double T = t_f - t_i;
  double a0 = p_i;
  double a1 = v_i;
  
  double a3_num = 2*(p_i-p_f)+(v_i+v_f)*T;
  double a3 = a3_num/pow(T,3.0);
  
  double a2_num = -3*(p_i-p_f)-(2*v_i+v_f)*T;
  double a2 = a2_num/pow(T,2.0);
  
  
  std::vector<double> prof;
  double step;
  
  int numPassi = floor(t_f/delta_t)+floor(t_ass/delta_t);
  
  double t = 0;
  Eigen::VectorXd profile(numPassi);
  for(int i = 0; i < numPassi; i++){
    if(i >= floor(t_f/delta_t)){
      profile[i] = profile[floor(t_f/delta_t)-1];
    } else {
      step = t-t_i;
      double aa = a3*pow(step,3.0) + a2*pow(step,2.0)+a1*step +a0;
      profile[i] = aa; 
      t = t + delta_t;
    }
  }
  
  return profile;
}


Eigen::MatrixXd getJ(Eigen::Vector3d posizione, Eigen::VectorXd theta){
  Eigen::MatrixXd J(6,7);
  Eigen::MatrixXd Ai(4,4);
  Eigen::MatrixXd T01 = DH(a[0], alpha[0], d[0], theta[0]);
  Eigen::MatrixXd T02 = T01 * DH(a[1], alpha[1], d[1], theta[1]);
  Eigen::MatrixXd T03 = T02 * DH(a[2], alpha[2], d[2], theta[2]);
  Eigen::MatrixXd T04 = T03 * DH(a[3], alpha[3], d[3], theta[3]);
  Eigen::MatrixXd T05 = T04 * DH(a[4], alpha[4], d[4], theta[4]);
  Eigen::MatrixXd T06 = T05 * DH(a[5], alpha[5], d[5], theta[5]);
  Eigen::MatrixXd T07 = T06 * DH(a[6], alpha[6], d[6]+0.1034, theta[6]);

  Eigen::MatrixXd I(4,4);
  I.setIdentity(4,4);
  Eigen::Vector3d pe(T07(0,3), T07(1,3), T07(2,3));

  Eigen::VectorXd colonna12 = getColJ(pe, I);
  Eigen::VectorXd colonna23 = getColJ(pe, T01);
  Eigen::VectorXd colonna34 = getColJ(pe, T02);
  Eigen::VectorXd colonna45 = getColJ(pe, T03);
  Eigen::VectorXd colonna56 = getColJ(pe, T04);
  Eigen::VectorXd colonna67 = getColJ(pe, T05);
  Eigen::VectorXd colonna7 =  getColJ(pe, T06);
  
  J = setColumn(0, J, colonna12);
  J = setColumn(1, J, colonna23);
  J = setColumn(2, J, colonna34);
  J = setColumn(3, J, colonna45);
  J = setColumn(4, J, colonna56);
  J = setColumn(5, J, colonna67);
  J = setColumn(6, J, colonna7);

  return J;
}

Eigen::VectorXd getColJ(Eigen::Vector3d pos, Eigen::MatrixXd T0i_1){
  Eigen::Vector3d pi_1(T0i_1(0,3), T0i_1(1,3), T0i_1(2,3));
  Eigen::Vector3d zi_1(T0i_1(0,2), T0i_1(1,2), T0i_1(2,2));
  Eigen::Vector3d temp = pos-pi_1;
  Eigen::Vector3d vl = zi_1.cross(temp);
  Eigen::VectorXd temp1(6);
  temp1 << vl[0],vl[1],vl[2],zi_1[0],zi_1[1],zi_1[2];
  return temp1;
}

Eigen::MatrixXd getR(std::array<double, 16> posa){
  Eigen::MatrixXd R;
  R.setIdentity(3,3);
  R(0,0) = posa[0];
  R(0,1) = posa[4];
  R(0,2) = posa[8];
  R(1,0)= posa[1];
  R(1,1)= posa[5];
  R(1,2)= posa[9];
  R(2,0) = posa[2];
  R(2,1) = posa[6];
  R(2,2) = posa[10];
  return R;
}

Eigen::VectorXd r2asseangolo(Eigen::MatrixXd R){
   Eigen::Vector3d r0(0,0,0);
   double val = ((R(0, 0)+R(1,1) + R(2,2) - 1)*0.5) + 0.0;
   double theta = acos(std::min(std::max(val,-1.0),1.0));
   
   if(abs(theta-M_PI) <= 0.00001){
     r0[0] = -1*sqrt((R(0,0)+1) * 0.5); 
     r0[1] = sqrt((R(1,1)+1) * 0.5);
     r0[2] = sqrt(1-pow(r0(0),2)-pow(r0(1), 2));
   } else {
     if(theta >= 0.00001) {
      r0[0] = (R(2,1)-R(1, 2))/(2*sin(theta));
      r0[1] = (R(0,2)-R(2, 0))/(2*sin(theta));
      r0[2] = (R(1,0)-R(0, 1))/(2*sin(theta));
     }
   }
   Eigen::VectorXd result(4);
   result << r0(0), r0(1), r0(2), theta;
   return result;
 }

Eigen::VectorXd ascissa(double tk, double tf){
   double t = tk/tf;
   if(t > 1.0){
     t = 1.0;
   } 
   double s = pow(t, 3)*(6*pow(t,2)-15*t+10);
   double ds = (pow(t,2)*(30*pow(t,2)-60*t+30))/tf;
   Eigen::VectorXd result(2);
   result << s, ds;
   return result;

}

Eigen::MatrixXd asseangolo2r(Eigen::Vector3d r_theta, double angle){
   double a = cos(angle);
   double b = sin(angle);
   Eigen::MatrixXd R(3,3);
   R(0,0) = pow(r_theta[0], 2)*(1-a)+a;
   R(0,1) = (r_theta[0]*r_theta[1])*(1-a)-r_theta[2]*b;
   R(0,2) = (r_theta[0]*r_theta[2])*(1-a)+r_theta[1]*b;
   R(1,0) = (r_theta[0]*r_theta[1])*(1-a)+r_theta[2]*b;
   R(1,1) = pow(r_theta[1], 2)*(1-a)+a;
   R(1,2) = (r_theta[1]*r_theta[2])*(1-a)-r_theta[0]*b;
   R(2,0) = (r_theta[0]*r_theta[2])*(1-a)-r_theta[1]*b;
   R(2,1) = (r_theta[1]*r_theta[2])*(1-a)+r_theta[0]*b;
   R(2,2) = pow(r_theta[2], 2)*(1-a)+a;

   return R;
}

Eigen::MatrixXd rotazioneElementari(int num, double angolo){
  Eigen::MatrixXd R(3,3);
  if(num == 1){
    R << 1, 0, 0, 0, cos(angolo), -sin(angolo), 0, sin(angolo), cos(angolo);
  }
  if(num == 2){
    R << cos(angolo), 0, sin(angolo), 0, 1, 0, -sin(angolo), 0, cos(angolo);
  }
  if(num == 3){
    R << cos(angolo), -sin(angolo), 0, sin(angolo), cos(angolo), 0, 0, 0, 1;
  }

  return R;
}

Eigen::Vector3d r2rpy(Eigen::MatrixXd R1){
  Eigen::Vector3d phi;
  phi(0)=atan2(-R1(1,2),R1(2,2));
  if(phi(0) < 0){
    phi(0) += 2*M_PI;
  }
  phi(1)=atan2(R1(0,2), sqrt(pow(R1(0,0),2) + pow(R1(0,1),2)) );
  if(phi(1) < 0){
    phi(1) += 2*M_PI;
  }
  phi(2)=atan2(-R1(0,1),R1(0,0));
  if(phi(2) < 0){
    phi(2) += 2*M_PI;
  }

  return phi;
}

Eigen::MatrixXd rpy2r(Eigen::Vector3d phiv){
  Eigen::MatrixXd R(3,3); //phi, theta, psi
  double phi = phiv[0];
  double theta = phiv[1];
  double psii = phiv[2];
  R(0,0) = cos(psii)*cos(theta); R(0,1) = -cos(theta)*sin(psii); R(0,2) = sin(theta);
  R(1,0) = cos(phi)*sin(psii) + cos(psii)*sin(phi)*sin(theta); R(1,1) = cos(phi)*cos(psii) - sin(phi)*sin(psii)*sin(theta); R(1,2) = -cos(theta)*sin(phi);
  R(2,0) = sin(phi)*sin(psii) - cos(phi)*cos(psii)*sin(theta); R(2,1) = cos(psii)*sin(phi) + cos(phi)*sin(psii)*sin(theta); R(2,2) = cos(phi)*cos(theta);

  return R;
}

Eigen::MatrixXd getDiagonalMatrix(double size, Eigen::VectorXd diag){
  Eigen::MatrixXd M;
  M.resize(size,size);
  M.setIdentity(size,size);
  for (int i = 0; i < size; i++) {
    M(i,i) = diag[i];
  }
  return M;
}

std::array<double, 6> getArrayFromEigenVector(Eigen::VectorXd e){
  std::array<double, 6> a{e[0],e[1],e[2],e[3],e[4],e[5]};
  return a;
}

Eigen::Vector3d calcolaPolinomioVel(double ti, double tc, std::array<double, 6> &coeff_vel_x, std::array<double, 6> &coeff_vel_y, std::array<double, 6> &coeff_vel_z){
  Eigen::Vector3d v;
  double t = tc - ti;
  v[0] = coeff_vel_x[1] + 2*coeff_vel_x[2] * t + 3*coeff_vel_x[3] * pow(t,2) + 4*coeff_vel_x[4] * pow(t,3) + 5*coeff_vel_x[5] * pow(t,4);
  v[1] = coeff_vel_y[1] + 2*coeff_vel_y[2] * t + 3*coeff_vel_y[3] * pow(t,2) + 4*coeff_vel_y[4] * pow(t,3) + 5*coeff_vel_y[5] * pow(t,4);
  v[2] = coeff_vel_z[1] + 2*coeff_vel_z[2] * t + 3*coeff_vel_z[3] * pow(t,2) + 4*coeff_vel_z[4] * pow(t,3) + 5*coeff_vel_z[5] * pow(t,4);
  return v;
}

Eigen::Vector3d calcolaPolinomioPos(double ti, double tc, std::array<double, 6> &coeff_vel_x, std::array<double, 6> &coeff_vel_y, std::array<double, 6> &coeff_vel_z){
  Eigen::Vector3d p;
  double t = tc - ti;
  p[0] = coeff_vel_x[0] + coeff_vel_x[1]*t + coeff_vel_x[2] * pow(t,2) + coeff_vel_x[3] * pow(t,3) + coeff_vel_x[4] * pow(t,4) + coeff_vel_x[5] * pow(t,5);
  p[1] = coeff_vel_y[0] + coeff_vel_y[1]*t + coeff_vel_y[2] * pow(t,2) + coeff_vel_y[3] * pow(t,3) + coeff_vel_y[4] * pow(t,4) + coeff_vel_y[5] * pow(t,5);
  p[2] = coeff_vel_z[0] + coeff_vel_z[1]*t + coeff_vel_z[2] * pow(t,2) + coeff_vel_z[3] * pow(t,3) + coeff_vel_z[4] * pow(t,4) + coeff_vel_z[5] * pow(t,5);
  return p;
}

Eigen::Vector3d calcolaPolinomioAcc(double ti, double tc, std::array<double, 6> &coeff_vel_x, std::array<double, 6> &coeff_vel_y, std::array<double, 6> &coeff_vel_z){
  Eigen::Vector3d a;
  double t = tc - ti;
  a[0] = 2*coeff_vel_x[2] + 6*coeff_vel_x[3] * t + 12*coeff_vel_x[4] * pow(t,2) + 20*coeff_vel_x[5] * pow(t,3);
  a[1] = 2*coeff_vel_y[2] + 6*coeff_vel_y[3] * t + 12*coeff_vel_y[4] * pow(t,2) + 20*coeff_vel_y[5] * pow(t,3);
  a[2] = 2*coeff_vel_z[2] + 6*coeff_vel_z[3] * t + 12*coeff_vel_z[4] * pow(t,2) + 20*coeff_vel_z[5] * pow(t,3);
  return a;
}

void calcolaCoeffTraiettoriaPos(double p_i, double p_f, double v_i, double v_f, double t_i, double t_f, std::array<double, 4> &coeff){
  double T = t_f - t_i;
  coeff[0] = p_i;
  coeff[1] = v_i;
    
  double a2_num = -3*(p_i-p_f)-(2*v_i+v_f)*T;
  coeff[2] = a2_num/pow(T,2.0);

  double a3_num = 2*(p_i-p_f)+(v_i+v_f)*T;
  coeff[3] = a3_num/pow(T,3.0);
}

void calcolaCoeffTraiettoriaVel(double p_i, double p_f, double v_i, double v_f, double t_i, double t_f, std::array<double, 3> &coeff){
  double T = t_f - t_i;
  coeff[0] = v_i;
  
  double a2_num = -3*(p_i-p_f)-(2*v_i+v_f)*T;
  coeff[1] = a2_num/pow(T,2.0);
  
  double a3_num = 2*(p_i-p_f)+(v_i+v_f)*T;
  coeff[2] = a3_num/pow(T,3.0);
}

Eigen::Vector3d calcolaPolPos(double ti, double tc, std::array<double, 4> &coeff_pos_x, std::array<double, 4> &coeff_pos_y, std::array<double, 4> &coeff_pos_z){
  Eigen::Vector3d p;
  double t = tc - ti;
  p[0] = coeff_pos_x[0] + coeff_pos_x[1] * t + coeff_pos_x[2] * pow(t,2) + coeff_pos_x[3] * pow(t,3); 
  p[1] = coeff_pos_y[0] + coeff_pos_y[1] * t + coeff_pos_y[2] * pow(t,2) + coeff_pos_y[3] * pow(t,3); 
  p[2] = coeff_pos_z[0] + coeff_pos_z[1] * t + coeff_pos_z[2] * pow(t,2) + coeff_pos_z[3] * pow(t,3);  
  return p;
}

Eigen::Vector3d calcolaPolVel(double ti, double tc, std::array<double, 3> &coeff_vel_x, std::array<double, 3> &coeff_vel_y, std::array<double, 3> &coeff_vel_z){
  Eigen::Vector3d v;
  double t = tc - ti;
  v[0] = coeff_vel_x[0] + 2*coeff_vel_x[1] * t + 3*coeff_vel_x[2] * pow(t,2);
  v[1] = coeff_vel_y[0] + 2*coeff_vel_y[1] * t + 3*coeff_vel_y[2] * pow(t,2);
  v[2] = coeff_vel_z[0] + 2*coeff_vel_z[1] * t + 3*coeff_vel_z[2] * pow(t,2);
  return v;
}

// PSEUDO INVERSA SMORZATA
  /*MatrixXd pinv(MatrixXd J){
    MatrixXd J_inv, J_inv_;
    JacobiSVD<MatrixXd> svd(J, ComputeThinU | ComputeThinV);
    double lambda=1.6;//0.2;
    const VectorXd s= svd.singularValues();
    MatrixXd Sigma = s.asDiagonal();
    MatrixXd V,U;
    V = svd.matrixV();
    U = svd.matrixU();

    J_inv = V*( Sigma+lambda*MatrixXd::Identity( J.rows(),J.rows() ) ).inverse()*U.transpose();

    //    if (s(J.rows()-1) < 0.1 ) {
    //        //lambda = lambda_;
    //        FullPivLU<MatrixXd> lu_J(J);
    //        if (lu_J.rank() <J.rows()) {
    //            cout<<"J rank:"<<lu_J.rank()<<endl;
    //            cout<<"J rows:"<<J.rows()<<endl;
    //            cout<<"Damped Pseudo Inverse"<<endl;
    //            J_inv = V*(Sigma+lambda*MatrixXd::Zero(J.rows(),J.rows())).inverse()*U.transpose();
    //        } else {
    //            J_inv_ = J*J.transpose();
    //            J_inv = J.transpose()*J_inv_.inverse();
    //        }
    //    }else{
    //        //J_inv = V*Sigma.inverse()*U.transpose();
    //        //J_inv = W.inverse()*V*Sigma.inverse()*U.transpose();
    //        J_inv_ = J*J.transpose();
    //        J_inv = J.transpose()*J_inv_.inverse();
    //    }
    return J_inv;
 }*/


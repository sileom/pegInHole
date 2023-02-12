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

  #include "dynModel.h"
  //#include "utils_pc.hpp"
  //#include "detection_rs.hpp"
  //#include "utils_pc.hpp"
  
  #include <ctime>
  #include <stdio.h>
  #include <string>
  
  //#include "opencv2/highgui/highgui.hpp"
  //#include "opencv2/imgproc/imgproc.hpp"
  #include <librealsense2/rs.hpp> 
  #include <librealsense2/rsutil.h>

  #include "MatlabEngine.hpp"
  #include "MatlabDataArray.hpp"


// *************** DICHIARAZIONE FUNZIONI
 Eigen::MatrixXd fk(std::vector<double> a, std::vector<double> d, std::vector<double> alpha, std::vector<double> theta);
 Eigen::MatrixXd DH(double a,double alpha,double d,double theta);
 int sgn(double v);
 Eigen::VectorXd getQuaternion(std::array<double, 16> pose);
 //Eigen::VectorXd getError(std::array<double,16> posa_endEffector, std::array<double,16> posa_desiderata);
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
 Eigen::MatrixXd Ti(Eigen::Vector3d phiv);
 
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
 
 Eigen::VectorXd filter_(Eigen::VectorXd f_n_1, Eigen::VectorXd f_ext, double alpha);
 Eigen::MatrixXd skew(Eigen::Vector3d a);
 Eigen::Vector3d pixel2point(Eigen::Vector2d pixel, double depth);
 Eigen::Vector3d Transp(Eigen::Vector3d peg, Eigen::Vector3d forza,Eigen::Vector3d coppie);
 bool force_lim(int seg, Eigen::Vector3d forze);
 bool joint_lim(Eigen::VectorXd q,Eigen::VectorXd qd, double deltaT);
 int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix,Eigen::Vector3d nH, double ti, double tf, double vi, double vf, int seg, int num_seg,double t_ass);
 Eigen::Matrix3d readMatrix(FILE* file2);







// *************** VARIABILI GLOBALI
 std::vector<double> a = {0.0, 0.0, 0.0825, -0.0825, 0.0, 0.088, 0.0};
 std::vector<double> d = {0.333, 0, 0.316, 0.0, 0.384, 0.0, 0.107};
 std::vector<double> alpha = {-M_PI/2, M_PI/2, M_PI/2, -M_PI/2, M_PI/2, M_PI/2, 0.0};
 //durata del passo di campionamento [s]
 const double delta_t = 0.001;
 //tempo di inizio
 const double t_i = 0.0;
 //tempo di fine
 //const double t_f = 5.0;
 //tempo di assestamento per essere sicuri che l'errore vada a zero
 const double t_ass = 1;//1.5;
 //costanti per indicare l'asse di rotazione nelle rotazioni elementari
 const int ROTX = 1;
 const int ROTY = 2;
 const int ROTZ = 3;
 double ForzaLim=1; // FORZA DI CONTATTO LIMITE PER LA DETECTION DELL'ALBERO DA PRENDERE
 double tau_fl=0;   // COSTANTE DI TEMPO DEL GRADINO ESPONENZIALE UTILIZZATO PER AZZERARE LE q_dot
 bool peg_mounted=false;
 Eigen::Vector3d peg;
 Eigen::Vector3d ffr1;
 Eigen::Matrix3d RH;
 std::string path;


// *************** PARAMETRI INTRINSECI TELECAMERA
 double px = 578.00549546;// 613.6060181;
 double py = 575.42666411; //613.7553711;
 double u0 = 327.06022754;//324.6341248;
 double v0 = 238.04706309;//235.6944733;
 double kud = 0.0; 
 double kdu = 0.0;



// *************** VARIABILI PER IL SALVATAGGIO SU FILE
 FILE* file_pos;
 FILE* file11;
 FILE* file1_;
 FILE* file2_;
 FILE* file3_;
 FILE* file4_;
 FILE* file5_;
 FILE* file6_;
 FILE* file7_;
 FILE* file8_;
 FILE* file9_;
 FILE* file10_;
 FILE* file11_;
 FILE* file12_;


using namespace std;
//using namespace cv;
using namespace Eigen;
using namespace matlab::engine;
using namespace rs2;




int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> " << "<input file path holes PTI> " << "<path to save> " << std::endl;
    return -1;
  }

  //MoveRobotp2p move_robot;
  std::string robot_ip = argv[1];
  std::string filepath = argv[2];
  
  // *************** FILES PER IL SALVATAGGIO
   path = argv[3];

   std::string filepath1 = path + "/F_ext_dz.txt";      
   std::string filepath2 = path + "/xp_r.txt";    
   std::string filepath3 = path + "/xpd.txt";   
   std::string filepath4 = path + "/x_r.txt";    
   std::string filepath5 = path + "/xd.txt";   
   std::string filepath6 = path + "/xppd.txt";    
   std::string filepath7 = path + "/p_ee.txt"; 
   //std::string filepath7 = path + "/FICD.txt";   
   //std::string filepath7 = path + "/errPpp.txt"; 
   std::string filepath8 = path + "/F_ext_base.txt";   
   std::string filepath9 = path + "/xpp_r.txt"; 
   std::string filepath10 = path + "/F_ext_ee.txt";  
   //std::string filepath11 = path + "/errore.txt";   
   std::string filepath11 = path + "/Dp.txt";  
   std::string filepath12 = path + "/tau.txt";  
   file1_ = fopen(filepath1.c_str(), "a");
   file2_ = fopen(filepath2.c_str(), "a");
   file3_ = fopen(filepath3.c_str(), "a");
   file4_ = fopen(filepath4.c_str(), "a");
   file5_ = fopen(filepath5.c_str(), "a");
   file6_ = fopen(filepath6.c_str(), "a");
   file7_ = fopen(filepath7.c_str(), "a");
   file8_ = fopen(filepath8.c_str(), "a");
   file9_ = fopen(filepath9.c_str(), "a");
   file10_ = fopen(filepath10.c_str(), "a");
   file11_ = fopen(filepath11.c_str(), "a");
   file12_ = fopen(filepath12.c_str(), "a");

  
  // *************** COLLEGAMENTO CON IL ROBOT 
   cout << "******* COLLEGAMENTO CON IL ROBOT" << endl;
   franka::Robot robot(robot_ip);
   franka::Gripper gripper(robot_ip);
   franka::GripperState gripper_state=gripper.readOnce();
   franka::RobotState robot_state = robot.readOnce();
  

  //franka::Robot robot(robot_ip);
  //franka::Gripper gripper(robot_ip);
  //franka::GripperState gripper_state=gripper.readOnce();
  //franka::RobotState robot_state = robot.readOnce();

  
  peg<<0,0,0.07;
  ffr1 << 0,0,0; 

  if(peg_mounted==true){
  d = {0.333, 0, 0.316, 0.0, 0.384, 0.0, 0.107+peg[2]};

  }
  std::cout <<"d "<< d[6] << endl;
  
  
  // *************** SCANSIONE FILE PUNTI DESIDERATI
   FILE* file2;
   //file2 = fopen("/home/labarea/Scrivania/punti_calibrazione.txt", "r");
   //file2 = fopen("/home/labarea/Scrivania/puntiEMatrici.txt", "r");
   //file2 = fopen("/home/labarea/Scrivania/puntiRobot2.txt", "r");
   file2 = fopen(filepath.c_str(), "r");
   Eigen::MatrixXd P;
   char stringa[80];
   int res;
   int i = 1;
   int j = 0;
   res = fscanf(file2,"%s",stringa);
   int numPti = stoi(stringa) +1;
   res = fscanf(file2,"%s",stringa);
   double tempo = stod(stringa);
   std::vector<Eigen::Matrix3d> Rf_vec(numPti); 
   P.resize(numPti, 3);
   Eigen::MatrixXd F;
   Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q_d.data());
   std::vector<double> Theta;
   Theta = {q[0],q[1],q[2],q[3],q[4],q[5],q[6]};
   
   F = fk(a, d, alpha, Theta);
     P(0,0) = F(0,3);
     P(0,1) = F(1,3);
     P(0,2) = F(2,3);
   //P(0,0) = robot_state.O_T_EE_d[12]; 
   //P(0,1) = robot_state.O_T_EE_d[13]; 
   //P(0,2) = robot_state.O_T_EE_d[14];
   //P(0,3) = 0; P(0,4) = 0; P(0,4) = 0;
   Rf_vec[0] = Eigen::Matrix3d::Identity();
   Eigen::Matrix3d Rd;

  
   res = fscanf(file2,"%s",stringa);
   while(i < numPti && res != EOF) {
     //cout << stringa << endl;
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

   
   cout << P << endl;
   for(int k=0; k<Rf_vec.size(); k++){
     cout << Rf_vec[k] << endl;
   }
  
  cout << "Press enter to continue" << endl;
  cin.ignore();
  
  bool grasp_closed = false;
  int num_seg = numPti-1;
  cout << "Tempo: " << tempo << endl; 



  try{

    Eigen::Vector3d nH; 
    Eigen::Vector3d pHole;
    Eigen::Vector3d pApp;
    Eigen::Vector3d pIns;

    Rd = Rf_vec[1];

  // *************** VERSORE FORO
    nH[0]= Rd(0,2);
    nH[1]= Rd(1,2);
    nH[2]= Rd(2,2);
   
    //pHole << -0.022741, 0.434764, -0.0022355;
    pHole << P(1,0), P(1,1), P(1,2);

    Eigen::Matrix3d N;
    N.setIdentity(3,3);

    //pApp = pHole - nH*0.05;
    //pIns = pHole + nH*0.05;
    pApp = pHole - nH*0.3;
    pIns = pHole - nH*0.1;

    std::cout << "HOLE pos:\n" << pHole << endl << endl;
    std::cout << "HOLE or:\n" << Rf_vec[1] << endl << endl << endl;

    std::cout << "HOLE APP pos:\n" << pApp << endl << endl << endl;
    std::cout << "HOLE INS pos:\n" << pIns << endl << endl << endl;

    //return 0;
  
  
  //

    num_seg = 3;
    int seg=1;
    while(seg<=num_seg){
      if(seg==1){
        move(robot_ip,  Eigen::Vector3d(pApp[0], pApp[1], pApp[2]),Rf_vec[1], nH, 0, tempo, 0, 0,1,num_seg,t_ass);
      }else if(seg==2){    
        move(robot_ip,  Eigen::Vector3d(pHole[0], pHole[1], pHole[2]), RH, nH, 0, 2*tempo, 0, 0,2,num_seg,t_ass);    
      }else {
        move(robot_ip,  Eigen::Vector3d(pIns[0], pIns[1], pIns[2]),Rf_vec[1], nH, 0, tempo, 0, 0,3,num_seg,t_ass); 
      } 
      seg++;
    }
    //gripper.homing();
    //move(robot_ip,  Eigen::Vector3d(pHole[0], pHole[1],pHole[2]),Rd, nH, 0, tempo, 0, 0,1);
   /*
    while(seg<=num_seg){
      if(seg==1){
        move(robot_ip,  Eigen::Vector3d(P(1,0), P(1,1),P(1,2)),Rf_vec[1], nH, 0, tempo, 0, 0,1,num_seg,t_ass);
      }else if(seg==2){    
        move(robot_ip,  Eigen::Vector3d(P(2,0), P(2,1),P(2,2)), RH, nH, 0, 2*tempo, 0, 0,2,num_seg,t_ass);    
      }else {
        move(robot_ip,  Eigen::Vector3d(P(3,0), P(3,1),P(3,2)),Rf_vec[3], nH, 0, tempo, 0, 0,3,num_seg,t_ass); 
      } 
      seg++;
    }
    */

    cout << "FINE" << endl;

  // *************** CHIUSURA FILES
   } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    fclose(file2);
    fclose(file1_);
    fclose(file2_);
    fclose(file3_);
    fclose(file4_);
    fclose(file5_);
    fclose(file6_);
    fclose(file7_);
    fclose(file8_);
    fclose(file9_);
    fclose(file10_);
    fclose(file11_);
    fclose(file12_);
    return -1;
   }
   fclose(file2);
    fclose(file1_);
    fclose(file2_);
    fclose(file3_);
    fclose(file4_);
    fclose(file5_);
    fclose(file6_);
    fclose(file7_);
    fclose(file8_);
    fclose(file9_);
    fclose(file10_);
    fclose(file11_);
    fclose(file12_);
   return 0;
}



int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix,Eigen::Vector3d nH, double ti, double tf, double vi, double vf, int seg,int num_seg,double t_ass){
  double t_f = tf;
  
  // *************** GUADAGNI INVERSIONE CINEMATICA E OSSERVATORE E SPAZIO NULLO
   Eigen::MatrixXd K;
   K.setIdentity(7,7);
   K(0,0) = 10;     K(1,1) = 10;     K(2,2) = 10;     K(3,3) = 10;      K(4,4) = 15;     K(5,5) =15;     K(6,6)=15;
 
   Eigen::MatrixXd Kc;
   Kc.setIdentity(6,6);
   Kc(0,0) = 150;     Kc(1,1) = 150;     Kc(2,2) = 150;     Kc(3,3) = 30;      Kc(4,4) = 30;     Kc(5,5) = 30;

   Eigen::MatrixXd Knull;
   Knull.setIdentity(7,7);
   Knull(0,0) = 0.0;     Knull(1,1) = 0.0;     Knull(2,2) = 0.0;     Knull(3,3) = 0.0;      Knull(4,4) = 0.0;     Knull(5,5) =0.0;     Knull(6,6)=0.0;

  try {
    franka::Robot robot(robot_ip);
    franka::RobotState robot_state = robot.readOnce();
    DynModel modello(robot_state);
    
    if (seg==num_seg){
      t_ass=5;
    }    
 
  // ***************  SETTAGGIO COLLISIONI
    cout << "******* SETTAGGIO COLLISIONI" << endl;
    setDefaultBehavior(robot);
    robot.setCollisionBehavior(
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}}, {{200.0, 200.0, 180.0, 180.0, 160.0, 140.0, 120.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}},
        {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}}, {{200.0, 200.0, 200.0, 250.0, 250.0, 250.0}});
  // ***************  INVERSA DELLO JACOBIANO
    std::array<double, 7> joint_values = robot_state.q_d;
    Eigen::VectorXd q_0(7);
    q_0 << joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5],joint_values[6];

    franka::Model model(robot.loadModel());

    
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
    Eigen::MatrixXd jacobian = getJacobianMatrix(jacobian_array);


    Eigen::MatrixXd jacobian_inverse = /*jacobian.transpose();*/ pinv(jacobian); ///*************************************************************

  // ***************  CALCOLO DELLA POSA INIZIALE CON LA CINEMATICA DIRETTA
     Eigen::MatrixXd F;
     Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q_d.data());
     std::vector<double> Theta;
     Theta = {q[0],q[1],q[2],q[3],q[4],q[5],q[6]};
  
      F = fk(a, d, alpha, Theta);
      std::array<double, 16> initial_pose;//= robot_state.O_T_EE_d;
         initial_pose[0] = F(0,0);
         initial_pose[1] = F(1,0);      
         initial_pose[2] = F(2,0);
         initial_pose[3] = F(3,0);
         initial_pose[4] = F(0,1);
         initial_pose[5] = F(1,1);
         initial_pose[6] = F(2,1);
         initial_pose[7] = F(3,1);
         initial_pose[8] = F(0,2);
         initial_pose[9] = F(1,2);
         initial_pose[10] = F(2,2);
         initial_pose[11] = F(3,2);
         initial_pose[12] = F(0,3);
         initial_pose[13] = F(1,3);
         initial_pose[14] = F(2,3);
         initial_pose[15] = F(3,3);
   
       std::array<double, 16> posa_desiderata = initial_pose;
       Eigen::VectorXd quaternion_i = getQuaternion(initial_pose);
       Eigen::VectorXd quaternion_d = getQuaternion(posa_desiderata);



  // ***************  DIFINIZIONE SPOSTAMENTO CARTESIANO DESIDERATO
    posa_desiderata[12] = final_point[0];     //X
    posa_desiderata[13] = final_point[1];     //Y
    posa_desiderata[14] = final_point[2];     //Z


  // ***************  PASSIAMO IN RAPPRESENTAZIONE ASSE-ANGOLO
    Eigen::MatrixXd R0 = getR(initial_pose);
    Eigen::MatrixXd Rf = final_matrix;
    Eigen::MatrixXd Rf0 = R0.transpose()*Rf;
    Eigen::VectorXd r_theta = r2asseangolo(Rf0);
    Eigen::Vector3d r(r_theta[0], r_theta[1], r_theta[2]);
    double theta_f = r_theta[3]; 

  // ***************  INIZIALIZZAZIONE VARIABILI
    double time = 0.0;
    int i = 0;
    
    Eigen::VectorXd x_des(7);
    Eigen::VectorXd q_goal_eigen(7);
    Eigen::VectorXd q_dot_0(7);
    q_dot_0 << 0,0,0,0,0,0,0;
    Eigen::VectorXd qp(7);
    qp << 0,0,0,0,0,0,0;
    Eigen::VectorXd x_dot_d(7);
    Eigen::VectorXd errore(6);
    errore << 0,0,0,0,0,0;
    Eigen::VectorXd errore_f(6);
    Eigen::VectorXd s_ds(2);
    Eigen::MatrixXd R_d(3,3);
    Eigen::Vector3d omegad;
    omegad << 0,0,0;
    Eigen::VectorXd quat_des(4);


    

    time = 0.0;
    i = 0;
    double t = 0.0;
    //std::vector<double> theta;
    int count = 1;
    
  // ***************  CALCOLO DELLA TRAIETTORIA POLINOMIALE DI POSIZIONE
    std::array<double, 6> coeff_pos_x{};
    std::array<double, 6> coeff_pos_y{};
    std::array<double, 6> coeff_pos_z{};

    //Calcolo matrice
    Eigen::MatrixXd A(6,6);
    A(0,0) = 1; A(0,1) = t_i; A(0,2) = pow(t_i,2); A(0,3) = pow(t_i,3); A(0,4) = pow(t_i,4); A(0,5) = pow(t_i,5);
    A(1,0) = 1; A(1,1) = t_f; A(1,2) = pow(t_f,2); A(1,3) = pow(t_f,3); A(1,4) = pow(t_f,4); A(1,5) = pow(t_f,5);
    A(2,0) = 0; A(2,1) = 1; A(2,2) = 2*t_i; A(2,3) = 3*pow(t_i,2); A(2,4) = 4*pow(t_i,3); A(2,5) = 5*pow(t_i,4);
    A(3,0) = 0; A(3,1) = 1; A(3,2) = 2*t_f; A(3,3) = 3*pow(t_f,2); A(3,4) = 4*pow(t_f,3); A(3,5) = 5*pow(t_f,4);
    A(4,0) = 0; A(4,1) = 0; A(4,2) = 2; A(4,3) = 6*t_i; A(4,4) = 12*pow(t_i,2); A(4,5) = 20*pow(t_i,3);
    A(5,0) = 0; A(5,1) = 0; A(5,2) = 2; A(5,3) = 6*t_f; A(5,4) = 12*pow(t_f,2); A(5,5) = 20*pow(t_f,3);

    Eigen::MatrixXd A_inv(6,6);
    A_inv = A.inverse();

    Eigen::VectorXd b(6);
    Eigen::VectorXd m(6);
    b << initial_pose[12], posa_desiderata[12], 0.0, 0.0, 0.0, 0.0;
    m = A_inv*b;
    coeff_pos_x = getArrayFromEigenVector(m);

    b << initial_pose[13], posa_desiderata[13], 0.0, 0.0, 0.0, 0.0;
    m = A_inv*b;
    coeff_pos_y = getArrayFromEigenVector(m);

    b << initial_pose[14], posa_desiderata[14], 0.0, 0.0, 0.0, 0.0;
    m = A_inv*b;
    coeff_pos_z = getArrayFromEigenVector(m);

    //PASSIAMO IN RAPPRESENTAZIONE ASSE-ANGOLO E PIANIFICHIAMO L'ORIENTAMENTO
    //Eigen::MatrixXd R0 = getR(initial_pose);
    //Eigen::MatrixXd Rf = final_matrix; //R0*(R_i_x*R_i_y*R_i_z);
    //STAMPA PER VEDERE SE ASSEGNA BENE LE MATRICI
    //cout << Rf << endl;

    std::array<double, 6> coeff_phi_x{};
    std::array<double, 6> coeff_phi_y{};
    std::array<double, 6> coeff_phi_z{};

    Eigen::Vector3d phi_f = r2rpy(Rf);
    Eigen::Vector3d phi_0 = r2rpy(R0);

  // ***************  CONTROLLO CHE LA ROTAZIONE SIA DAL VERSO GIUSTO PERCORRENDO L'ANGOLO MINORE
    for(int i = 0; i < 3; i++){
         if (phi_f[i]-phi_0[i]>M_PI){
         phi_f[i]=phi_f[i]-2*M_PI;
         }
         if (phi_f[i]-phi_0[i]<-M_PI){
         phi_0[i]=phi_0[i]-2*M_PI;
         }
    }

  // ***************  STAMPA DEI PUNTI DI ORIENTAMENTO INIZIALE E FINALE E CALCOLO COEFFICIENTI POLINOMIALE
    std::cout << "Iniziale\n" << phi_0[0] << " " << phi_0[1] << " " << phi_0[2] << std::endl;
    std::cout << "Finale\n" << phi_f[0] << " " << phi_f[1] << " " << phi_f[2] << std::endl;

    b << phi_0[0], phi_f[0], 0.0, 0.0, 0.0, 0.0;
    m = A_inv*b;
    coeff_phi_x = getArrayFromEigenVector(m);

    b << phi_0[1], phi_f[1], 0.0, 0.0, 0.0, 0.0;
    m = A_inv*b;
    coeff_phi_y = getArrayFromEigenVector(m);

    b << phi_0[2], phi_f[2], 0.0, 0.0, 0.0, 0.0;
    m = A_inv*b;
    coeff_phi_z = getArrayFromEigenVector(m);
  
  // ***************  INIZIALIZZAZIONE VARIABILI POSIZIONI ECC
     cout << "******* INIZIALIZZAZIONE VARIABILI POSIZIONI" << endl;
     Eigen::VectorXd q_tk_1(7);
     q_tk_1 << Theta[0],Theta[1],Theta[2],Theta[3],Theta[4],Theta[5],Theta[6];
     std::vector<double> theta;
     Eigen::Vector3d phi_des;
     Eigen::Vector3d phip_des;
     Eigen::Vector3d phipp_des;
     Eigen::Vector3d p_des;
     Eigen::Vector3d rpyd_i = r2rpy(R0);
     Eigen::Vector3d v_des;
     Eigen::VectorXd xp_r(6); // velocita' di riferimento che saranno mandate al robot
     xp_r << 0,0,0,0,0,0;
     Eigen::VectorXd xpp_r(6);// accelerazioni' di riferimento
     xpp_r << 0,0,0,0,0,0;
     Eigen::Vector3d a_des;    
     Eigen::VectorXd res(7);
     res << 0.0,0.0,0.0,0.0,0.0,0.0;
     Eigen::VectorXd integrale_i_1(7);
     integrale_i_1 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0; 
     Eigen::VectorXd f_ext(6);
     f_ext << 0,0,0,0,0,0;
     Eigen::VectorXd f_n_1(6);
     f_n_1 << 0,0,0,0,0,0;
     double alpha_filt = 0.5;
     Eigen::VectorXd xppd(6); //accelerazioni desiderate
     xppd << 0,0,0,0,0,0;
     Eigen::VectorXd xpd(6);  //velocita' desiderate
     xpd << 0,0,0,0,0,0;
     Eigen::VectorXd xd(6);   //posizioni desiderate
     xd << initial_pose[12],initial_pose[13], initial_pose[14], rpyd_i[0],rpyd_i[1],rpyd_i[2];
     Eigen::VectorXd x_r(6);  // posizioni di riferimento 
     x_r << xd[0],xd[1], xd[2], xd[3],xd[4],xd[5];
     xp_r << xpd[0],xpd[1], xpd[2], xpd[3],xpd[4],xpd[5];
     Eigen::MatrixXd T(3,3);
  // ***************  INIZIALIZZAZIONE VARIABILI AMMETTENZA
     cout << "******* INIZIALIZZAZIONE VARIABILI AMMETTENZA" << endl;
     Eigen::VectorXd diag(6);
     diag << 15,15,15;//10,10,10
     Eigen::MatrixXd Ap = getDiagonalMatrix(3, diag);
     diag << 0.5,0.5,0.5;
     Eigen::MatrixXd Af = getDiagonalMatrix(3, diag);
     diag << 30,30,30; // 1000,1000,1000;
     Eigen::MatrixXd Dp = getDiagonalMatrix(3, diag);
     diag << 1,1,1;
     Eigen::MatrixXd Df = getDiagonalMatrix(3, diag);
     diag << 45,45,150;
     Eigen::MatrixXd Kp = getDiagonalMatrix(3, diag);
     diag << 1.5,1.5,1.5;
     Eigen::MatrixXd Kf = getDiagonalMatrix(3, diag);

  // ***************  INIZIALIZZAZIONE VARIABILI AMMETTENZA IN TERNA EE
      cout << "******* INIZIALIZZAZIONE VARIABILI AMMETTENZA IN TERNA EE" << endl;
      Eigen::Vector3d errP;
      errP <<  0,0,0;
      Eigen::Vector3d errPp;
      errPp <<  0,0,0;
      Eigen::Vector3d Fir;
      Fir <<   xd[3],xd[4],xd[5];
      Eigen::Vector3d Fipr;
      Fipr << 0,0,0;
      Eigen::Vector3d Ficd;
      Ficd << 0,0,0;      
      Eigen::Vector3d Fipcd;
      Fipcd << 0,0,0;  
      Eigen::Vector3d p_r;
      p_r << x_r[0], x_r[1], x_r[2];
      Eigen::Vector3d pp_r;
      pp_r << xp_r[0], xp_r[1], xp_r[2];
      Eigen::Vector3d pd;
      pd << xd[0], xd[1], xd[2];
      Eigen::Vector3d fid;
      fid << xd[3], xd[4], xd[5];
      Eigen::Vector3d ppd;
      ppd << xpd[0], xpd[1], xpd[2];
      Eigen::Vector3d fipd;
      fipd << xpd[3], xpd[4], xpd[5];
      Eigen::Vector3d int_ffr1;
      int_ffr1 <<0,0,0;
      Eigen::Vector3d int_ffr2;
      int_ffr2 <<0,0,0;
      Eigen::VectorXd int_prec(6);
      int_prec <<0,0,0,0,0,0;



  // ***************  VARIABILI WORKSPACE LIMITATION
    cout << "******* VARIABILI WORKSPACE LIMITATION" << endl;
    int in=0;
    int out=0;
    double thr=0.01; // soglia sulla distanza dalla superficie
    double s_wl=0; //surplus soglia
    double tau_wl=0;
    Eigen::Vector3d p;
    p << initial_pose[12],initial_pose[13], initial_pose[14];
    Eigen::Vector3d n; n << 0,-1,0; // array contenente tutte le normali dei triangoli
    Eigen::Vector3d Points; Points << 0,10,10; // array contenente le posizioni dei centri dei triangoli
    double d_wl;
    double dd_wl;
    Eigen::Vector3d pc;
    pc<< xd[0], xd[1], xd[2];
    Eigen::Vector3d ppc;
    ppc << xpd[0], xpd[1], xpd[2];
    Eigen::Vector3d int_pc;
    int_pc <<0,0,0;
  // ***************  SOGLIA FORZE LETTE DALL'OSSERVATORE
     cout << "******* SETTAGGIO SOGLIA FORZE LETTE" << endl;
     Eigen::VectorXd soglia(6);
     soglia << 6,6,4,1,1,1;
     //soglia << 16,16,14,10,10,10;


  // ***************  INIZIALIZZAZIONE VARIABILI PER IL SALVATAGGIO
   //NUMERO DI PASSI DERIVANTI DAL TEMPO TOTALE E DAL PASSO
    int aa = floor(t_f/delta_t) + floor(t_ass/delta_t);
    //Variabili per la stampa
    Eigen::VectorXd TIME;                                 TIME.resize(aa); 
    Eigen::MatrixXd XD = MatrixXd::Zero(6,aa);             XD.resize(6,aa); 
    Eigen::MatrixXd XpD = MatrixXd::Zero(6,aa);             XpD.resize(6,aa);   
    Eigen::MatrixXd Xp_R = MatrixXd::Zero(6,aa);             Xp_R.resize(6,aa);  
    Eigen::MatrixXd FICD = MatrixXd::Zero(3,aa);             FICD.resize(3,aa); 
    Eigen::MatrixXd ERRPpp = MatrixXd::Zero(3,aa);             ERRPpp.resize(3,aa); 
    Eigen::MatrixXd F_ext= MatrixXd::Zero(6,aa);           F_ext.resize(6,aa);
    Eigen::MatrixXd F_ext_f = MatrixXd::Zero(6,aa);           F_ext_f.resize(6,aa);
    Eigen::MatrixXd X_R= MatrixXd::Zero(6,aa);             X_R.resize(6,aa);
    Eigen::MatrixXd P_EE= MatrixXd::Zero(6,aa);            P_EE.resize(6,aa);
    Eigen::MatrixXd ERRORE= MatrixXd::Zero(1,aa);          ERRORE.resize(1,aa);
    Eigen::MatrixXd TAU;                                   TAU.resize(7,aa);
    bool release=false;

  // ***************  ROBOT CONTROL             
    cout << "******* ROBOT CONTROL" << endl;
    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities { //franka::JointPositions {
      time +=period.toSec();
      TIME[i] = period.toSec();

    // *************** CALCOLO DELLA POSA CORRENTE
       //cout << "******* CALCOLO DELLA POSA CORRENTE" << endl;
       Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q_d.data());
       Eigen:: VectorXd qvec(7);
       qvec<<q[0],q[1],q[2],q[3],q[4],q[5],q[6];
 
 
       q_tk_1 << q[0],q[1],q[2],q[3],q[4],q[5],q[6];
       theta = {q[0],q[1],q[2],q[3],q[4],q[5],q[6]};
 
       //initial_pose = robot_state.O_T_EE;
       F = fk(a, d, alpha, theta);
       initial_pose[0] = F(0,0);
       initial_pose[1] = F(1,0);      
       initial_pose[2] = F(2,0);
       initial_pose[3] = F(3,0);
       initial_pose[4] = F(0,1);
       initial_pose[5] = F(1,1);
       initial_pose[6] = F(2,1);
       initial_pose[7] = F(3,1);
       initial_pose[8] = F(0,2);
       initial_pose[9] = F(1,2);
       initial_pose[10] = F(2,2);
       initial_pose[11] = F(3,2);
       initial_pose[12] = F(0,3);
       initial_pose[13] = F(1,3);
       initial_pose[14] = F(2,3);
       initial_pose[15] = F(3,3);



       Eigen::Map<const Eigen::Matrix<double, 4, 4> > Tc(robot_state.O_T_EE_d.data());
       Eigen::MatrixXd R_current(3,3);
       R_current = getR(robot_state.O_T_EE);
       Eigen::Vector3d v_curr;
       v_curr<< R_current(0,2), R_current(1,2), R_current(2,2);
       Eigen::Vector3d rpy_c = r2rpy(R_current);
    // *************** SCRITTURA DEI VETTORI PER IL SALVATAGGIO
      X_R(0,i) = (x_r[0]); X_R(1,i) = (x_r[1]); X_R(2,i) = (x_r[2]); X_R(3,i) = (x_r[3]); X_R(4,i) = (x_r[4]); X_R(5,i) = (x_r[5]); 
      XD(0,i) = (xd[0]); XD(1,i) = (xd[1]); XD(2,i) = (xd[2]); XD(3,i) = (xd[3]); XD(4,i) = (xd[4]);  XD(5,i) = (xd[5]);
      //XpD(0,i) = (xpd[0]); XpD(1,i) = (xpd[1]); XpD(2,i) = (xpd[2]); XpD(3,i) = (xpd[3]); XpD(4,i) = (xpd[4]);  XpD(5,i) = (xpd[5]);
      Xp_R(0,i) = (xp_r[0]); Xp_R(1,i) = (xp_r[1]); Xp_R(2,i) = (xp_r[2]); Xp_R(3,i) = (xp_r[3]); Xp_R(4,i) = (xp_r[4]); Xp_R(5,i) = (xp_r[5]);
      P_EE(0,i) = (initial_pose[12]); P_EE(1,i) = (initial_pose[13]); P_EE(2,i) = (initial_pose[14]);
      P_EE(3,i) = (rpy_c[0]); P_EE(4,i) = (rpy_c[1]); P_EE(5,i) = (rpy_c[2]);

    // *************** CALCOLO DI M e g DALL'OSSERVATORE
      //cout << "******* CALCOLO DI M E g DALL'OSSERVATORE" << endl;
      Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq_(robot_state.q.data());

      Eigen::MatrixXd M = modello.getMassMatrix(q);
      Eigen::VectorXd g = modello.getGravityVector(q);

      //std::array<double, 7> acc = robot_state.ddq_d;
      //std::array<double, 7> tau = robot_state.tau_J;
      //std::array<double, 7> cor = model.coriolis(robot_state);
    // *************** CALCOLO DELLA TRAIETTORIA
        //cout << "******* CALCOLO DELLA TRAIETTORIA" << endl;
        if(time < (t_f-t_i) ) {
        p_des = calcolaPolinomioPos(t_i, time, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        v_des = calcolaPolinomioVel(t_i, time, coeff_pos_x, coeff_pos_y, coeff_pos_z); //i coefficienti sono gli stessi a meno del primo
        a_des = calcolaPolinomioAcc(t_i, time, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        
        phi_des = calcolaPolinomioPos(t_i, time, coeff_phi_x, coeff_phi_y, coeff_phi_z);
        phip_des = calcolaPolinomioVel(t_i, time, coeff_phi_x, coeff_phi_y, coeff_phi_z);
        phipp_des = calcolaPolinomioAcc(t_i, time, coeff_phi_x, coeff_phi_y, coeff_phi_z);

        /*s_ds = ascissa(time, t_f, t_i);
        omegad = R0*r;
        omegad = omegad * (s_ds[1]*theta_f);
        R_d = R0 * asseangolo2r(r, theta_f*s_ds[0]);
        domegad = R0*r * (s_ds[2]*theta_f);*/
      } else {
        p_des = calcolaPolinomioPos(t_i, t_f, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        v_des = calcolaPolinomioVel(t_i, t_f, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        a_des = calcolaPolinomioAcc(t_i, t_f, coeff_pos_x, coeff_pos_y, coeff_pos_z);

        phi_des = calcolaPolinomioPos(t_i, t_f, coeff_phi_x, coeff_phi_y, coeff_phi_z);
        phip_des = calcolaPolinomioVel(t_i, t_f, coeff_phi_x, coeff_phi_y, coeff_phi_z);
        phipp_des = calcolaPolinomioAcc(t_i, t_f, coeff_phi_x, coeff_phi_y, coeff_phi_z);

        /*s_ds = ascissa(t_f, t_f, t_i);
        omegad = R0*r;
        omegad = omegad * (s_ds[1]*theta_f);
        R_d = R0 * asseangolo2r(r, theta_f*s_ds[0]);
        domegad = R0*r * (s_ds[2]*theta_f);*/
      }

    // *************** CALCOLO DELLO JACOBIANO
       //cout << "******* CALCOLO DELLO JACOBIANO" << endl;
       Eigen::Vector3d posizione(robot_state.O_T_EE_d[12],robot_state.O_T_EE_d[13],robot_state.O_T_EE_d[14]);
       std::array<double, 7> pp = robot_state.q;
       Eigen::VectorXd qq(7);
       qq << pp[0],pp[1],pp[2],pp[3],pp[4],pp[5],pp[6];
       Eigen::MatrixXd jacobian = getJ(posizione, qq); 
       jacobian_inverse = pinv(jacobian);
    
    // *************** LETTURA DELLE FORZE ESTERNE DALL'OSSERVATORE
     //cout << "******* LETTURA DELLE FORZE ESTERNE DALL'OSSERVATORE" << endl;
     f_ext =  modello.observer(robot_state, res, integrale_i_1, jacobian, period.toSec(), K, model);
     TAU(0,i) = (res[0]); TAU(1,i) = (res[1]); TAU(2,i) = (res[2]); TAU(3,i) = (res[3]); TAU(4,i) = (res[4]);  TAU(5,i) = (res[5]); TAU(6,i) = (res[6]);
     
    // *************** CALCOLO DELLA MATRICE DI ROTAZIONE DI RIFERIMENTO
       //cout << "******* CALCOLO DELLA MATRICE DI ROTAZIONE DI RIFERIMENTO" << endl;
       Eigen::MatrixXd Rr(3,3);
       Rr=rpy2r(Fir);

    // *************** FILTRAGGIO DELLE FORZE LETTE DALL'OSSERVATORE
       //cout << "******* FILTRAGGIO DELLE FORZE LETTE DALL'OSSERVATORE" << endl;
       Eigen::VectorXd ff = filter_(f_n_1, f_ext, alpha_filt);
       f_n_1 << ff[0], ff[1], ff[2], ff[3], ff[4], ff[5];


    // *************** RIMOZIONE DELLA SOGLIA DAL WRENCH
      //cout << "******* RIMOZIONE DELLA SOGLIA DAL WRENCH" << endl;
      for(int i =0; i<6; i++){
        if(abs(ff[i]) < soglia[i]){
          ff[i] = 0;
        } else {
          ff[i] = sgn(ff[i]) * (abs(ff[i]) - soglia[i]); 
        }
      }

    // *************** ROTAZIONE DEL WRENCH IN TERNA END-EFFECTOR 
       //cout << "******* ROTAZIONE DEL WRENCH IN TERNA EE" << endl;
       Eigen::Vector3d forze;
       forze << ff[0], ff[1], ff[2];         
       Eigen::Vector3d f_mom;
       f_mom << ff[3], ff[4], ff[5];     
 
       ffr1=Rr.transpose()*forze;
       Eigen::Vector3d ffr2; 
       ffr2=Rr.transpose()*f_mom;
       Eigen::VectorXd ffr(6); 
       ffr[0]=ffr1[0];ffr[1]=ffr1[1];ffr[2]=ffr1[2];ffr[3]=ffr2[0];ffr[4]=ffr2[1];ffr[5]=ffr2[2];
       //ffr1[2]=ffr1[2]-1*pp_r[2];
    // *************** TRASPORTO DELLE FORZE, VELOCITA E DELLA POSIZIONE DALL'END EFFECTOR ALLA PUNTA DEL PEG
      //cout << "******* TRASPORTO GRANDEZZE DALL'EE ALLA PUNTA DEL PEG" << endl;
      if(peg_mounted==true){
      ffr2= Transp(peg, ffr1,ffr2);
      }

      
    // *************** CALCOLO DELL'ERRORE DI ORIENTAMENTO ESTRAENDO GLI ANGOLI DALLA MATRICE DI ROTAZIONE RELATIVA
      //cout << "******* CALCOLO ERRORE DI ORIENTAMENTO" << endl; 
      xppd << a_des[0], a_des[1], a_des[2], phipp_des[0], phipp_des[1], phipp_des[2]; //domegad[0], domegad[1], domegad[2];
      xpd << v_des[0], v_des[1], v_des[2], phip_des[0], phip_des[1], phip_des[2]; //omegad[0], omegad[1], omegad[2];
      xd << p_des[0], p_des[1], p_des[2], phi_des[0], phi_des[1], phi_des[2]; //rpyd[0], rpyd[1], rpyd[2];
      //Eigen::Vector3d rpyd = r2rpy(R_d);
      






    // *************** LIMITAZIONE WORKSPACE
      //cout << "******* LIMITAZIONE WORKSPACE" << endl;
      pd << xd[0], xd[1], xd[2];
      ppd << xpd[0], xpd[1], xpd[2];
 
      p << initial_pose[12],initial_pose[13], initial_pose[14];
      Eigen::Vector3d Vr;
      int mu=5;
      double n_norm;
      n_norm=sqrt(n.transpose()*n);
      Vr=n*(ppd.transpose()*n)/(n_norm*n_norm);

      d_wl=(p-Points).transpose()*n;
      d_wl=d_wl/n_norm;
      dd_wl=(pd-Points).transpose()*n;
      dd_wl=dd_wl/n_norm;

      if(d_wl<=(thr+s_wl)){
        if(d_wl*dd_wl>0 && abs(dd_wl>thr*1.5)){
          if(out==0){
            tau_wl=time;
            out=1;
          }
          ppc=ppd-Vr*(exp(mu*(tau_wl-time)));
         
          pc=pd;
          s_wl=0;
        }else{
          if(in==0){
            tau_wl=time;
            in=1;
          }  
          ppc=ppd-Vr*(1-exp(mu*(tau_wl-time)));
          int_pc=int_pc+Vr*(1-exp(mu*(tau_wl-time)))*period.toSec(); 
          pc=pd-int_pc;
          s_wl=0.005;                 
        }
      }else{
        if(out==1 && time<tau_wl+10){
          ppc=ppd-Vr*(exp(mu*(tau_wl-time)));          
        }else{
          ppc=ppd;
        }
        pc=pd;
        s_wl=0;
        int_pc << 0,0,0;
      }
      xd[0]=pc[0]; xd[1]=pc[1]; xd[2]=pc[2]; 
      xpd[0]=ppc[0];  xpd[1]=ppc[1];   xpd[2]=ppc[2];
      pd=pc; ppd=ppc;

    // 
    // *************** INIZIALIZZAIONE ERRORI DI POSIZIONE
      //cout << "******* INIZIALIZZAZIONE ERRORI DI POSIZIONE" << endl;
      Eigen::Vector3d ErP;
      ErP <<  xd[0] - x_r[0], xd[1] - x_r[1], xd[2] - x_r[2];
      Eigen::Vector3d ErPp;
      ErPp <<  xpd[0] - xp_r[0], xpd[1] - xp_r[1], xpd[2] - xp_r[2];

    //
      F_ext(0,i) = (ffr[0]); F_ext(1,i) = (ffr[1]); F_ext(2,i) = (ffr[2]); F_ext(3,i) = (ffr[3]); F_ext(4,i) = (ffr[4]); F_ext(5,i) = (ffr[5]);
      F_ext_f(0,i) = (ff[0]); F_ext_f(1,i) = (ff[1]); F_ext_f(2,i) = (ff[2]); F_ext_f(3,i) = (ff[3]); F_ext_f(4,i) = (ff[4]); F_ext_f(5,i) = (ff[5]);

      Eigen::MatrixXd Rd(3,3);
      Rd=rpy2r(fid);

      // *************** CALCOLO DI OMEGA DI RIFERIMENTO
      Eigen::Vector3d omegar;  
      omegar=Ti(Fir)*Fipr;   


    // *************** MODIFICA DEL GUADAGNO DAMPING POSIZIONE DA COSTANTE A VARIABILE
       //cout << "******* MODIFICA GUADAGNO DAMPING" << endl;
       int nc=1000;
       int gain=300;
       //INIZIALIZZAZIONE LIMITI DEI GUADAGNI DELL'AMMETENZA
       Eigen::Vector3d Dp_lim;
       Dp_lim << 500,500,1000;//250,250,250;
       Eigen::Vector3d Kp_lim;
       Kp_lim << 15,15,5;
       Eigen::Vector3d Df_lim;
       Df_lim << 500,500,500;//25,25,10;// 10,10,10;//50,50,50;//
       Eigen::Vector3d Kf_lim;
       Kf_lim << 0.5,0.5,0.5;
 
 
 
       int_ffr1= int_ffr1 +ffr1*period.toSec(); 
 
       if (count >nc) {
       int_prec[0] = int_prec[0]+ F_ext(0,i-nc)*period.toSec();int_prec[1] = int_prec[1]+ F_ext(1,i-nc)*period.toSec();int_prec[2] = int_prec[2]+ F_ext(2,i-nc)*period.toSec();
 
       diag[0] = 30 +gain*abs(int_ffr1[0]-int_prec[0]); diag[1] = 30 +gain*abs(int_ffr1[1]-int_prec[1]);diag[2] = 30 +gain*abs(int_ffr1[2]-int_prec[2]);
       
       //VERIFICA SUL VALORE DI Dp
       for(int i = 0; i < 3; i++){
         if (diag[i]>Dp_lim[i]){
           diag[i]=Dp_lim[i];
         }
       }
 
       Dp = getDiagonalMatrix(3, diag);
       int gain=-10;      
       diag[0] = 45 +gain*abs(int_ffr1[0]-int_prec[0]); diag[1] = 45 +gain*abs(int_ffr1[1]-int_prec[1]);diag[2] = 50 +gain*abs(int_ffr1[2]-int_prec[2]);
 
       //VERIFICA SUL VALORE DI Kp
       for(int i = 0; i < 3; i++){
          if (diag[i]<Kp_lim[i]){
          diag[i]=Kp_lim[i];
          }
       }
       Kp = getDiagonalMatrix(3, diag);
       }else{
       diag << 30,30,30;
       Dp = getDiagonalMatrix(3, diag);
       diag << 45,45,150;
       Kp = getDiagonalMatrix(3, diag);
       }
       FICD(0,i) = diag[0]; FICD(1,i) = diag[1]; FICD(2,i) = diag[2];
       ERRPpp(0,i) = int_ffr1[0]; ERRPpp(1,i) = int_ffr1[1]; ERRPpp(2,i) = int_ffr1[2]; 
 
       // MODIFICA DEL GUADAGNO DAMPING ORIENTAMENTO DA COSTANTE A VARIABILE
       int_ffr2= int_ffr2 +ffr2*period.toSec(); 
       nc=1000;//300;//500
       if (count >nc) {
       int_prec[3] = int_prec[3]+ F_ext(3,i-nc)*period.toSec();int_prec[4] = int_prec[4]+ F_ext(4,i-nc)*period.toSec();int_prec[5] = int_prec[5]+ F_ext(5,i-nc)*period.toSec();
 
       //VERIFICA SUL VALORE DI Df
       gain=10;//5
       diag[0] = 1 +gain*abs(int_ffr2[0]-int_prec[3]); diag[1] = 1 +gain*abs(int_ffr2[1]-int_prec[4]);diag[2] = 1 +gain*abs(int_ffr2[2]-int_prec[5]);
 
       for(int i = 0; i < 3; i++){
          if (diag[i]>Df_lim[i]){
           diag[i]=Df_lim[i];
          }
       }
       Df = getDiagonalMatrix(3, diag);
       int gain=-20;      
       diag[0] = 1.5 +gain*abs(int_ffr1[0]-int_prec[0]); diag[1] = 1.5 +gain*abs(int_ffr1[1]-int_prec[1]);diag[2] = 1.5 +gain*abs(int_ffr1[2]-int_prec[2]);
 
       //VERIFICA SUL VALORE DI Kf
       for(int i = 0; i < 3; i++){
          if (diag[i]<Kf_lim[i]){
          diag[i]=Kf_lim[i];
          }
       }
       Kf = getDiagonalMatrix(3, diag);
       }else{
       diag << 1,1,1;
       Df = getDiagonalMatrix(3, diag);
       diag << 1.5,1.5,1.5;
       Kf = getDiagonalMatrix(3, diag);
       }
      //ERRPpp(0,i) = int_ffr2[0]; ERRPpp(1,i) = int_ffr2[1]; ERRPpp(2,i) = int_ffr2[2]; 

      //VARIABILI PER I TERMINI FUORI DIAGONALE
      double r=0.2;//0.35; // RAGGIO DEL REMOTE CENTRE OF COMPLIANCE
      Eigen::MatrixXd Kpf(3,3);
      diag << 0,0,0;
      Kpf = getDiagonalMatrix(3, diag);
      Kpf(0,1)=Kf(2,2)/r;
      //Kpf(0,1)=(-1)*Kf(2,2)/r;//
      Kpf(1,0)=(-1)*Kf(1,1)/r;
      //Kpf(1,0)=Kf(1,1)/r; //
      Eigen::MatrixXd Kfp(3,3);
      Kfp = getDiagonalMatrix(3, diag);
      Kfp(0,1)=(-1)*Kf(1,1)/r;
      //Kfp(0,1)=Kf(1,1)/r;//
      Kfp(1,0)=Kf(2,2)/r; 
      //Kfp(1,0)=(-1)*Kf(2,2)/r; //


    // *************** AMMETTENZA IN TERNA END-EFFECTOR
       //cout << "******* AMMETTENZA IN TERNA EE" << endl;
       Eigen::Vector3d errPpp; 
       //std::cout << (Kpf*Ficd).transpose() << std::endl;
       errPpp=Ap.inverse()*(-ffr1 - Dp*errPp - Kp*errP+Kpf*Ficd);
 
       Eigen::Vector3d errFipp;       
       errFipp=Af.inverse()*(-Ti(Ficd).transpose()*ffr2-Df*Fipcd -Kf*Ficd-Kfp*errP);

      //INTEGRAZIONE
       errPp=errPp + errPpp*period.toSec();
       errP=errP + errPp*period.toSec();
    
       Fipcd=Fipcd + errFipp*period.toSec();
       Ficd=Ficd + Fipcd*period.toSec();
 
       Eigen::MatrixXd Rcd(3,3);
       Rcd=rpy2r(Ficd);
 
       fipd << xpd[3], xpd[4], xpd[5];
       ppd << xpd[0], xpd[1], xpd[2];
      // CALCOLO DI OMEGA DESIDERATA
       omegad=Ti(fid)*fipd;
 
       pp_r= ppd-Rr*(errPp +Rr.transpose()*skew(omegar)*Rr*errP);
 
       p_r = p_r + pp_r*period.toSec();

      // CALCOLO DEGLI ANGOLI DI RIFERIMENTO
       Fipr=Ti(Fir).inverse()*(omegad-Rr*Ti(Ficd)*Fipcd);
       Fir = Fir + Fipr*period.toSec();
      
      //COMPOSIZIONE DELLE GRANDEZZE DI RIFERIMENTO
       xpp_r << xpp_r[0], xpp_r[1], xpp_r[2], xppd[3], xppd[4], xppd[5];     
       xp_r[0]=pp_r[0]; xp_r[1]=pp_r[1]; xp_r[2]=pp_r[2];
       //xp_r[0]=ppc[0]; xp_r[1]=ppc[1]; xp_r[2]=ppc[2];
       xp_r[3]=Fipr[0];xp_r[4]=Fipr[1];xp_r[5]=Fipr[2];
       x_r[0]=p_r[0]; x_r[1]=p_r[1]; x_r[2]=p_r[2];
       //x_r[0]=pc[0]; x_r[1]=pc[1]; x_r[2]=pc[2];
       x_r[3]=Fir[0];x_r[4]=Fir[1];x_r[5]=Fir[2];

    // *************** SCELTA DELLE VARIABILI DA MANDARE AL ROBOT des=1 => variabili desiderate al robot
      //cout << "******* SCELTA DELLE VARIABILI DA MANDARE AL ROBOT" << endl;
      int des=0;
      if (des==0 && seg!=1){
      x_dot_d << pp_r[0], pp_r[1], pp_r[2], omegar[0], omegar[1], omegar[2];
      //x_dot_d << pp_r[0], pp_r[1], pp_r[2], omegad[0], omegad[1], omegad[2];x_r[3]=xd[3];x_r[4]=xd[4];x_r[5]=xd[5];
      //x_dot_d << ppd[0], ppd[1], ppd[2], omegad[0], omegad[1], omegad[2];
      errore = getError(initial_pose,x_r);
      }else{
      x_dot_d << ppd[0], ppd[1], ppd[2], omegad[0], omegad[1], omegad[2];
      errore = getError(initial_pose,xd);
      }     
    

    // *************** INVERSIONE CINEMATICA
      //cout << "******* INVERSIONE CINEMATICA" << endl;
      Eigen::MatrixXd Id(7,7);
      Id.setIdentity(7,7);

      double sigma = (nH - v_curr).transpose() * (nH - v_curr);
      Eigen::MatrixXd Jfov(1, 7);
      Jfov = 2 * (nH - v_curr).transpose() * skew(v_curr) * jacobian.block<3,7>(3,0);
      Eigen::MatrixXd Jfov_inv = pinv(Jfov);
      Eigen::MatrixXd Jp(3, 7);
      Jp = jacobian.block<3,7>(0,0);
      Eigen::VectorXd q_dot_prec;   
      q_dot_prec = q_dot_0;

      double err_sigma = 0 - sigma;
      double Kfov = 5*(1-exp(-time/(0.5*t_f/5)));
      ERRORE(0,i) = err_sigma;

      if (seg==1){
        q_dot_0 = Jfov_inv * Kfov *err_sigma + (Id-Jfov_inv*Jfov)* pinv(Jp)*(x_dot_d.head(3)+Kc.block<3,3>(0,0)*errore.head(3));
      }else{
        q_dot_0 =  jacobian_inverse*(x_dot_d +Kc*errore)-(Id-jacobian_inverse*jacobian)*Knull*q_dot_prec;
       // std::cout <<"q_dot_prec"<< endl << q_dot_prec << endl;
      }
      
      XpD(0,i) = (err_sigma); XpD(1,i) = (0); XpD(2,i) = (0); XpD(3,i) = (0); XpD(4,i) = (0);  XpD(5,i) = (0);
      
      //q_dot_0 =  jacobian_inverse*(x_dot_d +Kc*errore);


      if (time > t_f+t_ass-0.1 && seg!=num_seg){//|| (seg==2 && time>10)){
          q_dot_0 << 0,0,0,0,0,0,0;             
      }else if(seg==num_seg && time > t_f+t_ass-0.1 ){
          q_dot_0 << 0,0,0,0,0,0,0; 
      }
      
      if (seg==1 && time > t_f+t_ass-0.1 ){
         RH= getR(robot_state.O_T_EE);
         std::cout <<"Sono in RH"<<  endl;
      }
    // *************** VERIFICA SUI LIMITI DI GIUNTO
      //cout << "******* VERIFICA SUI LIMITI DI GIUNTO" << endl;
      if(joint_lim(q, q_dot_0, period.toSec())){
         franka::JointVelocities output = {{0,0,0,0,0,0,0}};
         std::cout << "Alcuni giunti sono al limite"  << std::endl;
         return franka::MotionFinished(output);
      }

    //
    //TAU(0,i) = (q_dot_0[0]); TAU(1,i) = (q_dot_0[1]); TAU(2,i) = (q_dot_0[2]); TAU(3,i) = (q_dot_0[3]); TAU(4,i) = (q_dot_0[4]);  TAU(5,i) = (q_dot_0[5]); TAU(6,i) = (q_dot_0[6]);
      
    // *************** INCREMENTO DEL TEMPO
      t = t + delta_t;
      i++;

      count++;

    // *************** ASSEGNAZIONE DEI VALORI CALCOLATI AI GIUNTI (POSIZIONE O VELOCITA')
      //franka::JointPositions output = {{q_goal_eigen[0],q_goal_eigen[1],q_goal_eigen[2],q_goal_eigen[3],q_goal_eigen[4],q_goal_eigen[5],q_goal_eigen[6]}};
      franka::JointVelocities output = {{q_dot_0[0], q_dot_0[1], q_dot_0[2], q_dot_0[3], q_dot_0[4], q_dot_0[5], q_dot_0[6]}};

    // *************** CONDIZIONE DI USCITA DAL CICLO DI CONTROLLO

      if ( time >= t_f+t_ass || i > floor(t_f/delta_t)+floor(t_ass/delta_t)) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;

        return franka::MotionFinished(output);
      }
      return output;
    }, franka::ControllerMode::kJointImpedance, true, 10.0);//franka::ControllerMode::kJointImpedance, false, 50.0);
 
    // *************** SALVATAGGIO NEI FILES 
      for (int i = 0; i < aa; i++) {
        fprintf(file10_, "%f %f %f %f %f %f\n", F_ext(0,i), F_ext(1,i), F_ext(2,i), F_ext(3,i), F_ext(4,i), F_ext(5,i));
        fprintf(file8_, "%f %f %f %f %f %f\n", F_ext_f(0,i), F_ext_f(1,i), F_ext_f(2,i), F_ext_f(3,i), F_ext_f(4,i), F_ext_f(5,i));
        fprintf(file5_, "%f %f %f %f %f %f\n", XD(0,i), XD(1,i), XD(2,i), XD(3,i), XD(4,i), XD(5,i));
        fprintf(file3_, "%f %f %f %f %f %f\n", XpD(0,i), XpD(1,i), XpD(2,i), XpD(3,i), XpD(4,i), XpD(5,i));
        fprintf(file4_, "%f %f %f %f %f %f\n", X_R(0,i), X_R(1,i), X_R(2,i), X_R(3,i), X_R(4,i), X_R(5,i));
        fprintf(file2_, "%f %f %f %f %f %f\n", Xp_R(0,i), Xp_R(1,i), Xp_R(2,i), Xp_R(3,i), Xp_R(4,i), Xp_R(5,i));
        fprintf(file7_, "%f %f %f %f %f %f\n", P_EE(0,i), P_EE(1,i), P_EE(2,i), P_EE(3,i), P_EE(4,i), P_EE(5,i));
        fprintf(file11_, "%f %f %f\n", FICD(0,i), FICD(1,i), FICD(2,i));
        //fprintf(file7_, "%f %f %f\n", ERRPpp(0,i), ERRPpp(1,i), ERRPpp(2,i));        
        //fprintf(file11_, "%f \n", ERRORE(0,i)); // ERRORE(1,i), ERRORE(2,i), ERRORE(3,i), ERRORE(4,i), ERRORE(5,i));
        fprintf(file12_, "%f %f %f %f %f %f %f\n", TAU(0,i), TAU(1,i), TAU(2,i), TAU(3,i), TAU(4,i), TAU(5,i), TAU(6,i));

      }
 
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    std::cout << "Done." << std::endl;
  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
   
  return 0;
}

// ***************  FUNZIONI
 Eigen::MatrixXd skew(Eigen::Vector3d a){
  Eigen::Matrix3d S;
  S <<    0,       -1*a[2],       a[1],
         a[2],       0,         -1*a[0],
       -1*a[1],     a[0],          0;
  return S;
 }

 bool joint_lim(Eigen::VectorXd q,Eigen::VectorXd qd, double deltaT){
  Eigen::VectorXd qLimMin(7);
  qLimMin <<-2.81,-0.4,-2.81,-2.98,-2.50,0.07,-2.81;
  Eigen::VectorXd qLimMax(7);
  qLimMax <<2.81,1.67,2.81,-0.16,2.81,3.67,2.81;
  Eigen::VectorXd q_comm(7);
  q_comm=q+qd*deltaT;
  for (int i = 0; i < 7; i++){
    if(q_comm[i]<qLimMin[i] || q_comm[i]>qLimMax[i]){
      return true;
    }      
  }
  return false;
 }

 Eigen::VectorXd filter_(Eigen::VectorXd f_n_1, Eigen::VectorXd f_ext, double alpha_filt){
    Eigen::VectorXd ff(6);
    for (int i = 0; i < f_ext.size(); i++) {
        ff[i] = alpha_filt*f_ext[i] + (1-alpha_filt)*f_n_1[i];
    }
    return ff;
 }

 Eigen::MatrixXd getJacobianMatrix(std::array<double, 42> jacobian_array){
  Eigen::MatrixXd J(6,7);

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
         epsilon_d[2],       0, 	          -1*epsilon_d[0],
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

 Eigen::VectorXd getColumn(int colonna, Eigen::MatrixXd *M){
	int rows = M->rows();
	Eigen::VectorXd q_iesimo(rows);
	for(int i=0; i < rows; i++){
		q_iesimo[i] = M->operator()(i, colonna);
	}
	return q_iesimo;
 }

  // VALORE DI FORZA LIMITE PER LO SCAMBIO DELL'ALBERO
 bool force_lim(int seg, Eigen::Vector3d forze){
  if(seg==3 && forze[0]>ForzaLim+5){   
    return true;
  }
  return false;
 }

 // TRASPORTO DELLE FORZE
 Eigen::Vector3d Transp(Eigen::Vector3d peg, Eigen::Vector3d forza,Eigen::Vector3d coppie){
  Eigen::Vector3d muTrans;
  peg[2]=peg[2]+0.107;
  muTrans= coppie-skew(peg)*forza;
  return muTrans;
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
  if(peg_mounted==true){
    T07 = T06 * DH(a[6], alpha[6], d[6]+0.1034+peg[2], theta[6]);
  }

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
  Eigen::MatrixXd R(3,3);
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
  res = 1;
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

 
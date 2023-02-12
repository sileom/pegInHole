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
  #include "common_i.h"
  #include "dynModel_i.h"
  #include <Eigen/Core>
  #include <Eigen/Geometry>
  #include <Eigen/LU>
  #include <Eigen/QR>
  #include <Eigen/Dense>
  #include <math.h>
  
  #include <ctime>
  #include <stdio.h>
  #include <string>
  
  #include "opencv2/highgui/highgui.hpp"
  #include "opencv2/imgproc/imgproc.hpp"
  #include <librealsense2/rs.hpp> 
  #include <librealsense2/rsutil.h>

  #include "utils.hpp"


// *************** DICHIARAZIONE FUNZIONI
 Eigen::Vector3d Transp(Eigen::Vector3d peg, Eigen::Vector3d forza,Eigen::Vector3d coppie);
 bool force_lim(int seg, Eigen::Vector3d forze);
 int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix,Eigen::Vector3d nH, double ti, double tf, double vi, double vf, int seg, int num_seg,double t_ass);
 bool contact(int seg, Eigen::Vector3d forze);





// *************** VARIABILI GLOBALI
 //tempo di inizio
 const double t_i = 0.0;
 //tempo di fine
 //const double t_f = 5.0;
 //tempo di assestamento per essere sicuri che l'errore vada a zero
 
 double ForzaLim=1; // FORZA DI CONTATTO LIMITE PER LA DETECTION DELL'ALBERO DA PRENDERE
 double tau_fl=0;   // COSTANTE DI TEMPO DEL GRADINO ESPONENZIALE UTILIZZATO PER AZZERARE LE q_dot
 bool peg_mounted=false;
 Eigen::Vector3d peg;
 Eigen::Vector3d ffr1;
 Eigen::Matrix3d RH;
 std::string path;
 const int GraspBh=10;
 const int Dispenser=20;
 const int Glue=30;
 const int Approach=1;
 const int Insertion=2;
 const int Bonding=4;
 bool searchC=false;
 bool bhInsertion = false;
 bool InsertionVar = false;
 double tcontact;
 double tcontactEnd;
 double z_contact;
 Eigen::Vector3d pContact(0,0,0);
 Eigen::Vector3d holeMeasured(0,0,0);
 Eigen::Vector3d pStop(0,0,0);
 Eigen::Vector3d pEE(0,0,0);
 
 Eigen::Vector3d pHole(0,0,0);

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
 FILE* file13_;
int bhToInsert;

using namespace std;
using namespace cv;
using namespace Eigen;



int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname> " << "<input file path> " << "<path to save> " << "<bigHead to insert> "<< std::endl;
    return -1;
  }

  //MoveRobotp2p move_robot;
  std::string robot_ip = argv[1];
  std::string filepath = argv[2];

  //BigHead da inserire (ci arrivera' dalla linea di terminale)
  bhToInsert = std::stoi(argv[4]);
  
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
    std::string filepath11 = "/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/Fori_measured.txt";  
    std::string filepath12 = path + "/tau.txt";  
    //file1_ = fopen(filepath1.c_str(), "a");
    //file2_ = fopen(filepath2.c_str(), "a");
    //file3_ = fopen(filepath3.c_str(), "a");
    file4_ = fopen(filepath4.c_str(), "a");
    file5_ = fopen(filepath5.c_str(), "a");
    //file6_ = fopen(filepath6.c_str(), "a");
    file7_ = fopen(filepath7.c_str(), "a");
    //file8_ = fopen(filepath8.c_str(), "a");
    //file9_ = fopen(filepath9.c_str(), "a");
    file10_ = fopen(filepath10.c_str(), "a");
    file11_ = fopen(filepath11.c_str(), "a+");
    //file12_ = fopen(filepath12.c_str(), "a");

  franka::Robot robot(robot_ip);
  franka::Gripper gripper(robot_ip);
  franka::GripperState gripper_state=gripper.readOnce();

  
  std::string filepath13 = path + "/foro_stimato_e_misurato.txt"; 
  file13_ = fopen(filepath13.c_str(), "a");

  peg<<0,0,0.07;
  ffr1 << 0,0,0; 

  if(peg_mounted==true){
  d = {0.333, 0, 0.316, 0.0, 0.384, 0.0, 0.107+peg[2]};

  }
  std::cout <<"d "<< d[6] << endl;
  franka::RobotState robot_state = robot.readOnce();
  
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
   /*P(0,0) = robot_state.O_T_EE_d[12]; 
   P(0,1) = robot_state.O_T_EE_d[13]; 
   P(0,2) = robot_state.O_T_EE_d[14];*/
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

   //In P abbiamo i punti che rappresentano i vari BH
   //cout << P << endl;
   //for(int k=0; k<Rf_vec.size(); k++){
   //  cout << Rf_vec[k] << endl;
   //}

   //poseAdjustment("/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/fori_w.txt", "/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/Fori_measured.txt"); //

  cout << "Press enter to continue" << endl;
  cin.ignore();
  
  bool grasp_closed = false;
  int num_seg = numPti-1;
  cout << "Tempo: " << tempo << endl; 



  try{

     // In input all'applicativo dovrebbe arrivarci il codice del bigHead da inserire supponiamo tra 1-6
  Eigen::Vector3d p_target(P(bhToInsert,0), P(bhToInsert,1),P(bhToInsert,2));
  Eigen::Matrix3d R_target;
  R_target = Rf_vec[bhToInsert];

    Eigen::Vector3d nH; 
   
  pHole=p_target;
  // *************** VERSORE FORO
    nH[0]= Rd(0,2);
    nH[1]= Rd(1,2);
    nH[2]= Rd(2,2);
     
  //
   Eigen::Vector3d p_temp; 



    cout << "\n\n\n\nPunto target: \n" << p_target << endl;
    cout << "Press enter to continue" << endl;
    cin.ignore();

    int seg=1;
    //gripper.homing();
    //move(robot_ip,  Eigen::Vector3d(pHole[0], pHole[1],pHole[2]),Rd, nH, 0, tempo, 0, 0,1);
    fprintf(file13_, "%f %f %f ", p_target[0], p_target[1], p_target[2]);

    std::string filepath15 = path + "/dati_per_errore_ny.txt"; 
    FILE* file15_ = fopen(filepath15.c_str(), "a");
    fprintf(file15_, "psW = [%f %f %f 1]';\n", p_target[0], p_target[1], p_target[2]);
    fclose(file15_);
    
    while(seg<=4){
      if(seg==Approach){
        cout << "Approach" << endl;
        //vado nel punto stabilito con un delta su z di 5 cm
        p_temp = p_target + R_target * Eigen::Vector3d(0,0,-0.05);
        move(robot_ip,  Eigen::Vector3d(p_temp[0], p_temp[1], p_temp[2]),R_target, nH, 0, tempo, 0, 0,Approach,num_seg,t_ass);
      } else if(seg==Insertion){  
        cout << "Insertion" << endl;
        //inserisco facendo la ricerca del foro e apro il gripper  
        p_temp = p_target + R_target * Eigen::Vector3d(0,0,0.02); //-0.018);
        std::cout << "P_temp: " << p_temp << std::endl;
        move(robot_ip,  Eigen::Vector3d(p_temp[0], p_temp[1], p_temp[2]), R_target, nH, 0, tempo, 0, 0,Insertion,num_seg,t_ass);
        if(bhInsertion==true){
          if (!gripper.move(gripper_state.max_width,0.1)){
            std::cout << "Failed to move gripper." << std::endl;
            return -1;
          } 
          //move(robot_ip,  Eigen::Vector3d(p_temp[0], p_temp[1], p_temp[2]+0.05), R_target, nH, 0, tempo, 0, 0,Approach,num_seg,t_ass);
        }  
      } else if(seg==Insertion+1){   
        cout << "Insertion+1" << endl;
        //vado nel punto che si trova in alto rispetto al foro (x e y uguali, z piu' in alto) e chiudo le pinze 
        //move(robot_ip,  Eigen::Vector3d(p_target[0], p_target[1], p_target[2]+0.05), R_target, nH, 0, tempo/2, 0, 0,Insertion+1,num_seg,t_ass);
        //if(bhInsertion==true){
        //  if (!gripper.move(0.0,0.1)){
        //    std::cout << "Failed to close gripper." << std::endl;
        //    return -1;
        //  } 
        //}   
      } else {
        //torno al punto in cui ho lasciato il bighead per spingerlo
        //move(robot_ip,  Eigen::Vector3d(p_target[0], p_target[1], p_target[2]-0.05),Rf_vec[Bonding], nH, 0, tempo, 0, 0,Bonding,num_seg,t_ass); 
        //cout << "SONO NEL TERZO"  << endl;
      } 
      seg++;
    }

    fclose(file13_);
    fclose(file11_);
    file11_ = fopen(filepath11.c_str(), "r");
    //int numElem = countElement(file11_); //
    //if(numElem == 12){
    //  std::cout << "STO FACENDO LA COMPENSAZIONE\n";
    //  poseAdjustment("/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/fori_w.txt", "/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/Fori_measured.txt"); //
    //}

    /*int seg=1;
    //gripper.homing();
    //move(robot_ip,  Eigen::Vector3d(pHole[0], pHole[1],pHole[2]),Rd, nH, 0, tempo, 0, 0,1);
    while(seg<=4){
      if(seg==Approach){
        move(robot_ip,  Eigen::Vector3d(P(Approach,0), P(Approach,1),P(Approach,2)),Rf_vec[Approach], nH, 0, tempo, 0, 0,Approach,num_seg,t_ass);
      } else if(seg==Insertion){    
        move(robot_ip,  Eigen::Vector3filterAnd
        }   
      } else if(seg==Insertion+1){    
        move(robot_ip,  Eigen::Vector3d(P(Bonding,0), P(Bonding,1),P(Bonding,2)+0.05), Rf_vec[Insertion], nH, 0, tempo/2, 0, 0,Insertion+1,num_seg,t_ass);
        if(bhInsertion==true){
          if (!gripper.move(0.0,0.1)){
            std::cout << "Failed to close gripper." << std::endl;
            return -1;
          } 
        }   
      } else {
        move(robot_ip,  Eigen::Vector3d(P(Bonding,0), P(Bonding,1),P(Bonding,2)),Rf_vec[Bonding], nH, 0, tempo, 0, 0,Bonding,num_seg,t_ass); 
        //cout << "SONO NEL TERZO"  << endl;
      } 
      seg++;
    }*/

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
   //K(0,0) = 10;     K(1,1) = 10;     K(2,2) = 10;     K(3,3) = 10;      K(4,4) = 15;     K(5,5) =15;     K(6,6)=15;
   K(0,0) = 30;     K(1,1) = 30;     K(2,2) = 30;     K(3,3) = 30;      K(4,4) = 30;     K(5,5) =30;     K(6,6)=30;
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
      //std::array<double, 16> initial_pose;//= robot_state.O_T_EE_d;
      std::array<double, 16> initial_pose= robot_state.O_T_EE_d;
         /*initial_pose[0] = F(0,0);
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
         initial_pose[15] = F(3,3);*/
   
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
     res << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;
     Eigen::VectorXd integrale_i_1(7);
     integrale_i_1 << 0.0,0.0,0.0,0.0,0.0,0.0,0.0; 
     Eigen::VectorXd f_ext(6);
     f_ext << 0,0,0,0,0,0;
     Eigen::VectorXd f_n_1(6);
     f_n_1 << 0,0,0,0,0,0;
     double alpha_filt = 1; //0.25;
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
     Eigen::VectorXd soglia(6);
     soglia << 1,1,1.5,1,1,1;
     soglia << 4,4,5,1.2,1.2,1;
     //soglia << 0.15,0.15,0.15,0.15,0.15,0.15;
     //soglia << 16,16,14,10,10,10;
      //soglia << 0,0,0,0,0,0;


  // ***************  INIZIALIZZAZIONE VARIABILI PER IL SALVATAGGIO
   //NUMERO DI PASSI DERIVANTI DAL TEMPO TOTALE E DAL PASSO
    int aa =10*floor(t_f/delta_t) + floor(t_ass/delta_t);
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
    int stop_cycles=0;
    z_contact=-0.80;

    Eigen::Vector3d p_foro;
    p_foro = final_point - final_matrix * Eigen::Vector3d(0,0,0.04); //-0.018);

  // ***************  ROBOT CONTROL             

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities { //franka::JointPositions {
      time +=period.toSec();
      TIME[i] = period.toSec();

    // *************** CALCOLO DELLA POSA CORRENTE
       Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q_d.data());
       Eigen:: VectorXd qvec(7);
       qvec<<q[0],q[1],q[2],q[3],q[4],q[5],q[6];
 
 
       q_tk_1 << q[0],q[1],q[2],q[3],q[4],q[5],q[6];
       theta = {q[0],q[1],q[2],q[3],q[4],q[5],q[6]};
 
       initial_pose = robot_state.O_T_EE_d;
       /*F = fk(a, d, alpha, theta);
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
       initial_pose[15] = F(3,3);*/



       Eigen::Map<const Eigen::Matrix<double, 4, 4> > Tc(robot_state.O_T_EE_d.data());
       Eigen::MatrixXd R_current(3,3);
       R_current = getR(robot_state.O_T_EE_d);
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

      Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq_(robot_state.q.data());

      Eigen::MatrixXd M = modello.getMassMatrix(q);
      Eigen::VectorXd g = modello.getGravityVector(q);

      //std::array<double, 7> acc = robot_state.ddq_d;
      //std::array<double, 7> tau = robot_state.tau_J;
      //std::array<double, 7> cor = model.coriolis(robot_state);
    // *************** CALCOLO DELLA TRAIETTORIA
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
       Eigen::Vector3d posizione(robot_state.O_T_EE_d[12],robot_state.O_T_EE_d[13],robot_state.O_T_EE_d[14]);
       std::array<double, 7> pp = robot_state.q;
       Eigen::VectorXd qq(7);
       qq << pp[0],pp[1],pp[2],pp[3],pp[4],pp[5],pp[6];
       Eigen::MatrixXd jacobian = getJ(posizione, qq); 
       jacobian_inverse = pinv(jacobian);
    
    // *************** LETTURA DELLE FORZE ESTERNE DALL'OSSERVATORE
      f_ext =  modello.observer(robot_state, res, integrale_i_1, jacobian, period.toSec(), K, model,0,false);
               //modello.observer(robot_state, res, integrale_i_1, jacobian, period.toSec(), K, model,ALBERO,grasp);
     Eigen::Map<const  Eigen::Matrix<double, 7, 1> > tau(robot_state.tau_J.data());
     //TAU(0,i) = (res[0]); TAU(1,i) = (res[1]); TAU(2,i) = (res[2]); TAU(3,i) = (res[3]); TAU(4,i) = (res[4]);  TAU(5,i) = (res[5]); TAU(6,i) = (res[6]);
     TAU(0,i) = (tau[0]); TAU(1,i) = (tau[1]); TAU(2,i) = (tau[2]); TAU(3,i) = (tau[3]); TAU(4,i) = (tau[4]);  TAU(5,i) = (tau[5]); TAU(6,i) = (tau[6]);
     
    // *************** CALCOLO DELLA MATRICE DI ROTAZIONE DI RIFERIMENTO
       Eigen::MatrixXd Rr(3,3);
       Rr=rpy2r(Fir);

    // *************** FILTRAGGIO DELLE FORZE LETTE DALL'OSSERVATORE
       Eigen::VectorXd ff = filter_utils(f_n_1, f_ext, alpha_filt);
       f_n_1 << ff[0], ff[1], ff[2], ff[3], ff[4], ff[5];

       ff=f_ext;
    // *************** RIMOZIONE DELLA SOGLIA DAL WRENCH
      for(int i =0; i<6; i++){
        if(abs(ff[i]) < soglia[i]){
          ff[i] = 0;
        } else {
          ff[i] = sgn(ff[i]) * (abs(ff[i]) - soglia[i]); 
        }
      }
      //ff<< 0,0,0,-2.64870,0,0,0;
    // *************** ROTAZIONE DEL WRENCH IN TERNA END-EFFECTOR 
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
      if(peg_mounted==true){
      ffr2= Transp(peg, ffr1,ffr2);
      }

      
    // *************** CALCOLO DELL'ERRORE DI ORIENTAMENTO ESTRAENDO GLI ANGOLI DALLA MATRICE DI ROTAZIONE RELATIVA
  

      xppd << a_des[0], a_des[1], a_des[2], phipp_des[0], phipp_des[1], phipp_des[2]; //domegad[0], domegad[1], domegad[2];
      xpd << v_des[0], v_des[1], v_des[2], phip_des[0], phip_des[1], phip_des[2]; //omegad[0], omegad[1], omegad[2];
      xd << p_des[0], p_des[1], p_des[2], phi_des[0], phi_des[1], phi_des[2]; //rpyd[0], rpyd[1], rpyd[2];
      //Eigen::Vector3d rpyd = r2rpy(R_d);
      






    // *************** LIMITAZIONE WORKSPACE
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
      Eigen::Vector3d ErP;
      ErP <<  xd[0] - x_r[0], xd[1] - x_r[1], xd[2] - x_r[2];
      Eigen::Vector3d ErPp;
      ErPp <<  xpd[0] - xp_r[0], xpd[1] - xp_r[1], xpd[2] - xp_r[2];

    //
      F_ext(0,i) = (ffr[0]); F_ext(1,i) = (ffr[1]); F_ext(2,i) = (ffr[2]); F_ext(3,i) = (ffr[3]); F_ext(4,i) = (ffr[4]); F_ext(5,i) = (ffr[5]);
      //F_ext_f(0,i) = (ff[0]); F_ext_f(1,i) = (ff[1]); F_ext_f(2,i) = (ff[2]); F_ext_f(3,i) = (ff[3]); F_ext_f(4,i) = (ff[4]); F_ext_f(5,i) = (ff[5]);

      Eigen::MatrixXd Rd(3,3);
      Rd=rpy2r(fid);

      // *************** CALCOLO DI OMEGA DI RIFERIMENTO
      Eigen::Vector3d omegar;  
      omegar=Ti(Fir)*Fipr;   


    // *************** MODIFICA DEL GUADAGNO DAMPING POSIZIONE DA COSTANTE A VARIABILE
       int nc=500;
       int gain=100;//300
       //INIZIALIZZAZIONE LIMITI DEI GUADAGNI DELL'AMMETENZA
       Eigen::Vector3d Dp_lim;
       Dp_lim << 500,500,2000;//250,250,250;
       Eigen::Vector3d Kp_lim;
       Kp_lim << 15,15,5;
       Eigen::Vector3d Df_lim;
       Df_lim << 500,500,500;//25,25,10;// 10,10,10;//50,50,50;//
       Eigen::Vector3d Kf_lim;
       Kf_lim << 0.5,0.5,0.5;
 
 
 
       int_ffr1= int_ffr1 +ffr1*period.toSec(); 
 
      if (count >nc) {
       int_prec[0] = int_prec[0]+ F_ext(0,i-nc)*period.toSec();int_prec[1] = int_prec[1]+ F_ext(1,i-nc)*period.toSec();int_prec[2] = int_prec[2]+ F_ext(2,i-nc)*period.toSec();
 
       diag[0] = 50 +0*gain*abs(int_ffr1[0]-int_prec[0]); diag[1] = 50 +0*gain*abs(int_ffr1[1]-int_prec[1]);diag[2] = 30 +gain*abs(int_ffr1[2]-int_prec[2]);
       
       //VERIFICA SUL VALORE DI Dp
       for(int i = 0; i < 3; i++){
         if (diag[i]>Dp_lim[i]){
           diag[i]=Dp_lim[i];
         }
       }
 
       Dp = getDiagonalMatrix(3, diag);
       int gain=-10*10;      
       diag[0] = 45 +0*gain*abs(int_ffr1[0]-int_prec[0]); diag[1] = 45 +0*gain*abs(int_ffr1[1]-int_prec[1]);diag[2] = 50 +0*gain*abs(int_ffr1[2]-int_prec[2]);
        if(seg==Insertion && searchC==true){
          diag[0] = 700; diag[1] = 700;
        }
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

       //FICD(0,i) = diag[0]; FICD(1,i) = diag[1]; FICD(2,i) = diag[2];
       //ERRPpp(0,i) = int_ffr1[0]; ERRPpp(1,i) = int_ffr1[1]; ERRPpp(2,i) = int_ffr1[2]; 
 
       // MODIFICA DEL GUADAGNO DAMPING ORIENTAMENTO DA COSTANTE A VARIABILE
       int_ffr2= int_ffr2 +ffr2*period.toSec(); 
       nc=100;//300;//500
       if (count >nc) {
       int_prec[3] = int_prec[3]+ F_ext(3,i-nc)*period.toSec();int_prec[4] = int_prec[4]+ F_ext(4,i-nc)*period.toSec();int_prec[5] = int_prec[5]+ F_ext(5,i-nc)*period.toSec();
 
       //VERIFICA SUL VALORE DI Df
       gain=10*0;//5
       diag[0] = 1 +gain*abs(int_ffr2[0]-int_prec[3]); diag[1] = 1 +gain*abs(int_ffr2[1]-int_prec[4]);diag[2] = 1 +gain*abs(int_ffr2[2]-int_prec[5]);
 
       for(int i = 0; i < 3; i++){
          if (diag[i]>Df_lim[i]){
           diag[i]=Df_lim[i];
          }
       }
       Df = getDiagonalMatrix(3, diag);
       int gain=-20*0;      
       diag[0] = 3*1.5 +gain*abs(int_ffr1[0]-int_prec[0]); diag[1] = 3*1.5 +gain*abs(int_ffr1[1]-int_prec[1]);diag[2] = 1.5 +gain*abs(int_ffr1[2]-int_prec[2]);
 
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
      ////Kpf(0,1)=Kf(2,2)/r;
      //Kpf(0,1)=(-1)*Kf(2,2)/r;//
      ////Kpf(1,0)=(-1)*Kf(1,1)/r;
      //Kpf(1,0)=Kf(1,1)/r; //
      Eigen::MatrixXd Kfp(3,3);
      Kfp = getDiagonalMatrix(3, diag);
      ////Kfp(0,1)=(-1)*Kf(1,1)/r;
      //Kfp(0,1)=Kf(1,1)/r;//
      ////Kfp(1,0)=Kf(2,2)/r; 
      //Kfp(1,0)=(-1)*Kf(2,2)/r; //


    // *************** AMMETTENZA IN TERNA END-EFFECTOR
       Eigen::Vector3d errPpp; 
       //if(seg==Insertion && searchC==true && (robot_state.O_T_EE[14]>=z_contact-0.002)){
       //  ffr1[0]*= z_contact - initial_pose[14];
       //  ffr1[1]*= z_contact - initial_pose[14];
       //}
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

      // MOTO DI RICERCA DEL FORO
       Eigen::Vector3d xC; 
       Eigen::Vector3d xCe;  

       if(seg == Insertion && contact(seg,ffr1)==true  && searchC==false && InsertionVar==false){// 
           searchC=true;
           InsertionVar=true;           
           tcontact=time;
           
           z_contact=robot_state.O_T_EE[14]; //initial_pose[14];
           pContact << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
           std::cout <<"forza"<< " "<< ffr1[2] << endl;


           //xCe=Rr.transpose()*xC;
        
         std::cout <<"Zcontact"<< " "<< pContact[2] << endl; 
       }

        /*xC = Eigen::Vector3d(initial_pose[12],
			    initial_pose[13],
				initial_pose[14]);
        xC=Rr.transpose()*xC;
        P_EE(0,i) = (xC[0]); P_EE(1,i) = (xC[1]); P_EE(2,i) = (xC[2]);*/
       if(seg == Insertion && searchC==true){
              
         
         double ax=0.007;
         double ay=0.007;
         if(bhToInsert==3) {
           ax=0.01;
           ay=0.01;
         }
         /*xC[0]=xCe[0]+ax*sin(20*(time-tcontact));
         xC[1]=xCe[1]+ay*sin(30*(time-tcontact));
         xC[2]=xCe[2];
         xC=Rr*xC;*/
         ppd << xpd[0], xpd[1], xpd[2];
         ppd=Rr.transpose()*ppd;
         ppd[0]+=2.5*ax*cos(2.5*(time-tcontact));
         ppd[1]+=3.5*ay*cos(3.5*(time-tcontact));
         
         ppd=Rr*ppd;
         //std::cout <<"Zeta"<< " "<< initial_pose[14] << endl;
        //std::cout <<"Zcontact"<< " "<< z_contact << endl; 

        //pStop = pContact + R_current * Eigen::Vector3d(0,0, 0.002);
        pEE << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
        pStop = pEE - pContact;
        R_current = getR(robot_state.O_T_EE);
        pStop = R_current.transpose() * pStop;
        //std::cout <<"zStop "<< pStop[2] <<" zContact "<< pContact[2]<< endl; 
         if(pStop[2] > 0.0015) { //robot_state.O_T_EE[14] < pStop[2]){ //initial_pose[14]<z_contact-0.003){ //---------------
             searchC=false;
             tcontactEnd=time;
             //pContact << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
             holeMeasured << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
              std::cout <<"ZUscente"<< " "<< pStop[2]<< endl; 
             std::cout <<"search=false"<< endl;
         }
        
       }
       else{
        ppd << xpd[0], xpd[1], xpd[2];
       }


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
      int des=0;
      if (des==0 && (seg==Insertion || seg==Bonding)){
      x_dot_d << pp_r[0], pp_r[1], pp_r[2], omegar[0], omegar[1], omegar[2];
      //x_dot_d << pp_r[0], pp_r[1], pp_r[2], omegad[0], omegad[1], omegad[2];//x_r[3]=xd[3];x_r[4]=xd[4];x_r[5]=xd[5];
      //x_dot_d << ppd[0], ppd[1], ppd[2], omegad[0], omegad[1], omegad[2];
      errore = getError(initial_pose,x_r);
      }else{
      x_dot_d << ppd[0], ppd[1], ppd[2], omegad[0], omegad[1], omegad[2];
      errore = getError(initial_pose,xd);
      }     
    

    // *************** INVERSIONE CINEMATICA

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
      //ERRORE(0,i) = err_sigma;

      /*if (seg==1){
        q_dot_0 = Jfov_inv * Kfov *err_sigma + (Id-Jfov_inv*Jfov)* pinv(Jp)*(x_dot_d.head(3)+Kc.block<3,3>(0,0)*errore.head(3));
      }else{
        q_dot_0 =  jacobian_inverse*(x_dot_d +Kc*errore)-(Id-jacobian_inverse*jacobian)*Knull*q_dot_prec;
       // std::cout <<"q_dot_prec"<< endl << q_dot_prec << endl;
      }*/
      
      XpD(0,i) = (ppd[0]); XpD(1,i) = (ppd[1]); XpD(2,i) = (ppd[2]); XpD(3,i) = (0); XpD(4,i) = (0);  XpD(5,i) = (0);
      
      q_dot_0 =  jacobian_inverse*(x_dot_d +Kc*errore);
      pEE << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
      R_current = getR(robot_state.O_T_EE);

      pStop = pEE - holeMeasured; // + R_current * Eigen::Vector3d(0,0, 0.005);
      pStop = R_current.transpose() * pStop;
      Eigen::Vector3d pH;
      pH = pEE - pHole;
      pH= R_current.transpose() * pH;

      /*if(seg==Insertion){
        std::cout <<"pH "<< pH[2] << endl;
        std::cout <<"pHole "<< pHole[2] << endl;
        std::cout <<"pEE "<< pEE[2] << endl;
      }*/
      //std::cout << "pEE-Hole " << pStop[2]  << std::endl; 
      if (time > t_f+t_ass-0.1 && seg!=Insertion){//|| (seg==2 && time>10)){
          q_dot_0 << 0,0,0,0,0,0,0;             
      }else if((seg==Insertion && pStop[2] > 0.006) || time > t_f+30 /*|| (seg==Insertion && pH[2] > -0.01) */) { ////initial_pose[14]<z_contact-0.005 ){
            //std::cout << time << " > " << t_f+30 << std::endl;
            //std::cout << "----pH " << pH[2]<< std::endl;  
            //std::cout << "----pStop " << pStop[2]<< std::endl;          
            q_dot_0 << 0,0,0,0,0,0,0;             
            stop_cycles++;          //std::cout <<"z contact in 1136"<< " " << z_contact << endl;   
      }else if((seg==Insertion && (pEE[2] - p_foro[2]) < 0.003) ){
        std::cout << "Esco per centratura perfetta \n\n\n"; 
        std::cout << "\n\n\n pEE " << pEE[2];
        std::cout << "\n\n\n p_foro " << p_foro[2]; 
        std::cout << "\n\n\n diff " << (pEE[2] - p_foro[2]); 
        holeMeasured << robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14];
        q_dot_0 << 0,0,0,0,0,0,0;    
        stop_cycles++;          //std::cout <<"z contact in 1136"<< " " << z_contact << endl;   
      }
      // Questo è necessario per far fermare il robot prima di uscire dal ciclo, adesso si aspetta 100 cicli  
      if(stop_cycles>100){
            bhInsertion=true;
            std::cout <<"bhInsertion"<< endl;
      }

      /*if( time >= 5*t_f ){
        q_dot_0 << 0,0,0,0,0,0,0;
      } */
    // *************** VERIFICA SUI LIMITI DI GIUNTO
    Eigen::VectorXd qLim(7);
    qLim= joint_lim(q, q_dot_0, period.toSec());
    for (int i = 0; i < 7; i++){
      if(qLim[i]==1){
        franka::JointVelocities output = {{0,0,0,0,0,0,0}};
        std::cout << "Alcuni giunti sono al limite"  << std::endl;
        std::cout << qLim[0] << " " << qLim[1] << " " << qLim[2] << " " << qLim[3] << " " << qLim[4] << " " << qLim[5] << " " << qLim[6] << " " << std::endl;
        return franka::MotionFinished(output);
      }    
     }
      /*if(joint_lim(q, q_dot_0, period.toSec())){
         franka::JointVelocities output = {{0,0,0,0,0,0,0}};
         std::cout << "Alcuni giunti sono al limite"  << std::endl;
         
      }
    
    //
      /*if (seg==Insertion && bhInsertion=true){ //force_lim(seg,ffr1)||
        if((seg == 2 && force_lim(seg,ffr1)==true)){
          if(tau_fl==0){
            tau_fl=time;
          }
          q_dot_0=exp((tau_fl-time)/0.1)*q_dot_0;
        }else{
          q_dot_0 << 0,0,0,0,0,0,0;
        } 
        q_dot_0 << 0,0,0,0,0,0,0;       
      }*/


    //TAU(0,i) = (q_dot_0[0]); TAU(1,i) = (q_dot_0[1]); TAU(2,i) = (q_dot_0[2]); TAU(3,i) = (q_dot_0[3]); TAU(4,i) = (q_dot_0[4]);  TAU(5,i) = (q_dot_0[5]); TAU(6,i) = (q_dot_0[6]);


      
    // *************** INCREMENTO DEL TEMPOseg==Insertion && pH[2]>0.008
      t = t + delta_t;
      i++;

      count++;

    // *************** ASSEGNAZIONE DEI VALORI CALCOLATI AI GIUNTI (POSIZIONE O VELOCITA')
      //franka::JointPositions output = {{q_goal_eigen[0],q_goal_eigen[1],q_goal_eigen[2],q_goal_eigen[3],q_goal_eigen[4],q_goal_eigen[5],q_goal_eigen[6]}};
      franka::JointVelocities output = {{q_dot_0[0], q_dot_0[1], q_dot_0[2], q_dot_0[3], q_dot_0[4], q_dot_0[5], q_dot_0[6]}};

    // *************** CONDIZIONE DI USCITA DAL CICLO DI CONTROLLO
      if(seg == Insertion  && bhInsertion==true  || time > t_f+t_ass+30 ){ // || seg==Insertion && pH[2]>0.008  time >= 5*t_f+t_ass
        if(pH[2]>0.014 ){
          std::cout << std::endl << "L'EE è 8mm sotto il foro misurato" << std::endl;
        }
        std::cout << std::endl << "Insertion completed: " << bhInsertion << std::endl;
        std::cout << tcontact << " " << tcontactEnd << std::endl;
        fprintf(file13_, "%f %f %f %f %f\n", holeMeasured[0], holeMeasured[1], holeMeasured[2], tcontact, tcontactEnd-tcontact);
        fprintf(file11_, "%f %f %f 1.0\n", holeMeasured[0], holeMeasured[1], holeMeasured[2]);
        return franka::MotionFinished(output);
      }
      if ( (time >= t_f+t_ass || i > floor(t_f/delta_t)+floor(t_ass/delta_t)) && seg!=Insertion) {
          //if(seg!=Insertion){
        
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;

        return franka::MotionFinished(output);
          //}
      }
      return output;
    }, franka::ControllerMode::kJointImpedance, true, 10.0);//franka::ControllerMode::kJointImpedance, false, 50.0);
 
    // *************** SALVATAGGIO NEI FILES 
    
      for (int i = 0; i < aa; i++) {
        //Forze sterne in terna EEseg==Insertion && pH[2]>0.008 XD(2,i), XD(3,i), XD(4,i), XD(5,i));
        //fprintf(file3_, "%f %f %f %f %f %f\n", XpD(0,i), XpD(1,i), XpD(2,i), XpD(3,i), XpD(4,i), XpD(5,i));
        fprintf(file4_, "%f %f %f %f %f %f\n", X_R(0,i), X_R(1,i), X_R(2,i), X_R(3,i), X_R(4,i), X_R(5,i));
        //fprintf(file2_, "%f %f %f %f %f %f\n", Xp_R(0,i), Xp_R(1,i), Xp_R(2,i), Xp_R(3,i), Xp_R(4,i), Xp_R(5,i));
        fprintf(file7_, "%f %f %f %f %f %f\n", P_EE(0,i), P_EE(1,i), P_EE(2,i), P_EE(3,i), P_EE(4,i), P_EE(5,i));
        //fprintf(file12_, "%f %f\n", tcontact, tcontactEnd);
        //fprintf(file11_, "%f %f %f\n", FICD(0,i), FICD(1,i), FICD(2,i));
        //fprintf(file7_, "%f %f %f\n", ERRPpp(0,i), ERRPpp(1,i), ERRPpp(2,i));        
        //fprintf(file11_, "%f \n", ERRORE(0,i)); // ERRORE(1,i), ERRORE(2,i), ERRORE(3,i), ERRORE(4,i), ERRORE(5,i));
        //fprintf(file12_, "%f %f %f %f %f %f %f\n", TAU(0,i), TAU(1,i), TAU(2,i), TAU(3,i), TAU(4,i), TAU(5,i), TAU(6,i));
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

  

  // VALORE DI FORZA LIMITE PER LO SCAMBIO DELL'ALBERO
 bool force_lim(int seg, Eigen::Vector3d forze){
  if(seg==3 && forze[0]>ForzaLim+5){   
    return true;
  }
  return false;
 }

 // DETECTION DEL CONTATTO
 bool contact(int seg, Eigen::Vector3d forze){
    if(seg==Insertion && forze[2]<-2){   
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
 
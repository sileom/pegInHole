// *************** LIBRERIE
  #include <iostream>
  #include <franka/exception.h>
  #include <franka/robot.h>
  #include <franka/gripper.h>
  #include <franka/model.h>
  #include "common_i.h"
  #include <Eigen/Core>
  #include <Eigen/Geometry>
  #include <Eigen/LU>
  #include <Eigen/QR>
  #include <Eigen/Dense>
  #include <math.h>

  //#include "dynModel.h"
  //#include "utils_pc.hpp"
  //#include "detection_rs.hpp"
  #include "utils.hpp"
  #include "utils_cam.hpp"
  
  #include <ctime>
  #include <stdio.h>
  #include <string>
  
  //#include "opencv2/highgui/highgui.hpp"
  //#include "opencv2/imgproc/imgproc.hpp"

  //#include "MatlabEngine.hpp"
  //#include "MatlabDataArray.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;
//using namespace matlab::engine;


/**
 * @unibas_single_robot plan_multi.cpp
 * Muove il robot a partire da pose lette da file (pianificazione multi_points)
 * 
 * STRUTTURA DEL FILE
 * <Numero punti presenti nel file> (sempre dispari) <Tempo per ogni segmento>
 * <x y z>
 * Rd
 * <R(0,0) R(0,1) R(0,2)>
 * <R(1,0) R(1,1) R(1,2)>
 * <R(2,0) R(2,1) R(2,2)>
 */

/*Eigen::MatrixXd getR(std::array<double, 16> posa);
Eigen::VectorXd r2asseangolo(Eigen::MatrixXd R);
Eigen::MatrixXd asseangolo2r(Eigen::Vector3d r_theta, double angle);
Eigen::VectorXd ascissa(double tk, double tf, double ti);
Eigen::MatrixXd rotazioneElementari(int num, double angolo);

void calcolaCoeffTraiettoriaVel(double p_i, double p_f, double v_i, double v_f, double t_i, double t_f, std::array<double, 3> &coeff);
void calcolaCoeffTraiettoriaPos(double p_i, double p_f, double v_i, double v_f, double t_i, double t_f, std::array<double, 4> &coeff);
Eigen::Vector3d calcolaPolinomioVel(double ti, double tc, std::array<double, 3> &coeff_vel_x, std::array<double, 3> &coeff_vel_y, std::array<double, 3> &coeff_vel_z);
Eigen::Vector3d calcolaPolinomioPos(double ti, double tc, std::array<double, 4> &coeff_pos_x, std::array<double, 4> &coeff_pos_y, std::array<double, 4> &coeff_pos_z);
*/


//tempo di inizio
const double t_i = 0.0;
//tempo di fine
const double t_f = 8.0;

//variabili per il salvataggio su file
FILE* file_i;
FILE* file_o;
FILE* file_o_fk;


int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, Eigen::Vector3d angles, double ti, double tf, double vi, double vf);

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname> " << "path to save" << std::endl;
        return -1;
    }

    std::string robot_ip = argv[1];
    std::string savePath= argv[2];
    franka::Robot robot(robot_ip);
    franka::RobotState robot_state = robot.readOnce();

    FILE* file_i;
    file_i = fopen("/home/labarea-franka/libfranka/unibas_insert_bh/resources/input/scanPoints.txt", "r");
    Eigen::MatrixXd P;
    char stringa[80];
    int res;
    int i = 1;
    int j = 0;
    res = fscanf(file_i,"%s",stringa);
    int numPti = stoi(stringa) +1;
    res = fscanf(file_i,"%s",stringa);
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
    Rf_vec[0] = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rd;

    res = fscanf(file_i,"%s",stringa);
    while(i < numPti && res != EOF) {
        //cout << stringa << endl;
        if(strcmp(stringa, "Rd") == 0){
			Rd = readMatrix(file_i);
			Rf_vec[i] = Rd;
			j = 0;
			i++;
        } else {
			P(i,j) = atof(stringa);
			j++;
        }
        res = fscanf(file_i,"%s",stringa);
    }

    //cout << "Stampa valori letti. " << P(1,0) << " " << P(1,1) << " " << P(1,2) <<  endl;
    //cout << P << endl;
    //for(int k=0; k<Rf_vec.size(); k++){
    //  cout << Rf_vec[k] << endl;
    //}

  
    //bool grasp_closed = false;
    //double ti_tot=0.0;
    Eigen::Vector3d angles(0,0,0);
    double t_single = 4.0;
    int num_seg = numPti-1;
    cout << "NUM seg: " << num_seg << endl; 

    //double tf_tot = num_seg * t_single;
    double vi_tot = 0.0;
    double vf_tot = 0.0;

  
    pipeline pipe = startCamera();
    cout << "Camera started." << endl; 
    clearPlyFolder();
    cout << "Folder cleared" << endl; 
    int pc_idx = 0;

    //file che contiene la posizione reale del bh1. Struttura:
    // <codice bigHead> <posizione (x,y,z)>
    FILE* file;
    file = fopen("/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/Fori_measured.txt", "w");
    fprintf(file, "");
    fclose(file);
    
    try{
        //FILE DI SALVATAGGIO (salvo all'interno di move altrimento dà errore di connessione)
        //file_o = fopen("/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/puntiSalv_franka.txt", "a");
        //file_o_fk = fopen("/home/labarea-franka/libfranka/unibas_insert_bh/resources/output/puntiSalv_fk.txt", "a");
        int i=0;
        while(i < num_seg){
            Eigen::Vector3d point_i(P(i+1,0), P(i+1,1), P(i+1,2));
            move(robot_ip, point_i, Rf_vec[i+1], angles, 0, tempo, vi_tot, vf_tot);
            if(i < (num_seg-2)){
              //savePly(pipe, pc_idx);
              //filterAndSavePly(pipe, pc_idx,savePath);         //segnet ceruzzi
              filterAndSavePly_deeplab(pipe, pc_idx,savePath); //deeplab
              //savePly_noFilter(pipe, pc_idx,savePath);
              pc_idx++;
            }            
            i++;
        }

        //fclose(file_o);
        //fclose(file_o_fk);


        pipe.stop();
        cout << "FINE" << endl;


    } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        fclose(file_i);
        
        return -1;
    }
    fclose(file_i);
    return 0;
}



int move(std::string &robot_ip, Eigen::Vector3d final_point, Eigen::MatrixXd final_matrix, Eigen::Vector3d angles, double ti, double tf, double vi, double vf){
  Eigen::MatrixXd K;
  K.setIdentity(6,6);
  K(0,0) = 100;     K(1,1) = 100;     K(2,2) = 100;     K(3,3) = 10;      K(4,4) = 10;     K(5,5) = 10;
   
  try {
    franka::Robot robot(robot_ip);
    franka::Model model = robot.loadModel();
    franka::RobotState robot_state = robot.readOnce();
    setDefaultBehavior(robot);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose = robot_state.O_T_EE;
    std::array<double, 16> posa_desiderata = initial_pose;

    //DIFINIZIONE SPOSTAMENTO CARTESIANO DESIDERATO
    posa_desiderata[12] = final_point[0];     //X
    posa_desiderata[13] = final_point[1];     //Y
    posa_desiderata[14] = final_point[2];     //Z

    //DIFINIZIONE DELLO SPOSTAMENTO DI ORIENTAMENTO DESIDERATO
    Eigen::MatrixXd R_i_x = rotazioneElementari(ROTX, angles[0]);
    Eigen::MatrixXd R_i_y = rotazioneElementari(ROTY, angles[1]);
    Eigen::MatrixXd R_i_z = rotazioneElementari(ROTZ, angles[2]);



    //calcolo della traiettoria polinomiale
    std::array<double, 4> coeff_pos_x{};
    std::array<double, 4> coeff_pos_y{};
    std::array<double, 4> coeff_pos_z{};
    std::array<double, 3> coeff_vel_x{};
    std::array<double, 3> coeff_vel_y{};
    std::array<double, 3> coeff_vel_z{};
    calcolaCoeffTraiettoriaPos(initial_pose[12], posa_desiderata[12], 0.0, 0.0, ti, tf, coeff_pos_x);
    calcolaCoeffTraiettoriaPos(initial_pose[13], posa_desiderata[13], 0.0, 0.0, ti, tf, coeff_pos_y);
    calcolaCoeffTraiettoriaPos(initial_pose[14], posa_desiderata[14], 0.0, 0.0, ti, tf, coeff_pos_z);
    calcolaCoeffTraiettoriaVel(initial_pose[12], posa_desiderata[12], 0.0, 0.0, ti, tf, coeff_vel_x);
    calcolaCoeffTraiettoriaVel(initial_pose[13], posa_desiderata[13], 0.0, 0.0, ti, tf, coeff_vel_y);
    calcolaCoeffTraiettoriaVel(initial_pose[14], posa_desiderata[14], 0.0, 0.0, ti, tf, coeff_vel_z);


    //PASSIAMO IN RAPPRESENTAZIONE ASSE-ANGOLO E PIANIFICHIAMO L'ORIENTAMENTO
    Eigen::MatrixXd R0 = getR(initial_pose);
    Eigen::MatrixXd Rf = final_matrix; //R0*(R_i_x*R_i_y*R_i_z);
    //STAMPA PER VEDERE SE ASSEGNA BENE LE MATRICI
    //cout << Rf << endl;

    Eigen::MatrixXd Rf0 = R0.transpose()*Rf;
    Eigen::VectorXd r_theta = r2asseangolo(Rf0);
    Eigen::Vector3d r(r_theta[0], r_theta[1], r_theta[2]);
    double theta_f = r_theta[3]; 

    Eigen::Vector3d p_des;
    Eigen::Vector3d v_des;
    Eigen::VectorXd s_ds;
    Eigen::MatrixXd R_d;
    Eigen::Vector3d omegad;

    double time = 0.0;

    robot.setJointImpedance({{3000, 3000, 3000, 3000, 3000, 3000, 3000}});

    robot.control([&](const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities { 
      time += period.toSec();

      //std::array<double, 7> q = robot_state.q;
      //std::array<double, 7> v = robot_state.dq;
      //std::array<double, 7> acc = robot_state.ddq_d;
      //std::array<double, 7> tau = robot_state.tau_J;
      //std::array<double, 7> cor = model.coriolis(robot_state);

      //Salvataggio
      //fprintf(file1_, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f \n", q[0], q[1], q[2], q[3], q[4], q[5], q[6], v[0], v[1], v[2], v[3], v[4], v[5], v[6], acc[0], acc[1], acc[2], acc[3], acc[4], acc[5], acc[6]);
      //fprintf(file2_, "%f %f %f %f %f %f %f \n", tau[0], tau[1], tau[2], tau[3], tau[4], tau[5], tau[6]);
      //fprintf(file3_, "%f %f %f %f %f %f %f \n", cor[0], cor[1], cor[2], cor[3], cor[4], cor[5], cor[6]);

      if(time < (tf-ti) ) {
        p_des = calcolaPolPos(ti, time, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        v_des = calcolaPolVel(ti, time, coeff_vel_x, coeff_vel_y, coeff_vel_z);
        s_ds = ascissa(time, tf);
        omegad = R0*r;
        omegad = omegad * (s_ds[1]*theta_f);
      } else {
        p_des = calcolaPolPos(ti, tf, coeff_pos_x, coeff_pos_y, coeff_pos_z);
        v_des = calcolaPolVel(ti, tf, coeff_vel_x, coeff_vel_y, coeff_vel_z);
        s_ds = ascissa(tf, tf);
        omegad = R0*r;
        omegad = omegad * (s_ds[1]*theta_f);
      }


      franka::CartesianVelocities output = {{v_des[0], v_des[1], v_des[2], omegad[0], omegad[1], omegad[2]}};

      if (time >= ((tf-ti)+t_ass)) {
        std::cout << std::endl << "Finished motion, shutting down example"  << std::endl;
        std::array<double, 7> q = robot_state.q;
        std::vector<double> Theta;
        Theta = {q[0],q[1],q[2],q[3],q[4],q[5],q[6]};
        Eigen::MatrixXd F = fk(a, d, alpha, Theta);
        //fprintf(file_o_fk, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", F(0,3), F(1,3), F(2,3), F(0,0),F(0,1),F(0,2), F(1,0),F(1,1),F(1,2),F(2,0),F(2,1),F(2,2));
        //fprintf(file_o, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", robot_state.O_T_EE[12], robot_state.O_T_EE[13], robot_state.O_T_EE[14], robot_state.O_T_EE[0], robot_state.O_T_EE[4], robot_state.O_T_EE[8], robot_state.O_T_EE[1], robot_state.O_T_EE[5], robot_state.O_T_EE[9], robot_state.O_T_EE[2], robot_state.O_T_EE[6], robot_state.O_T_EE[10]);
        return franka::MotionFinished(output);
      }
      return output;
    });
    

    std::cout << "Done." << std::endl;

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}

/*Eigen::Vector3d calcolaPolinomioPos(double ti, double tc, std::array<double, 4> &coeff_pos_x, std::array<double, 4> &coeff_pos_y, std::array<double, 4> &coeff_pos_z){
  Eigen::Vector3d p;
  double t = tc - ti;
  p[0] = coeff_pos_x[0] + coeff_pos_x[1] * t + coeff_pos_x[2] * pow(t,2) + coeff_pos_x[3] * pow(t,3); 
  p[1] = coeff_pos_y[0] + coeff_pos_y[1] * t + coeff_pos_y[2] * pow(t,2) + coeff_pos_y[3] * pow(t,3); 
  p[2] = coeff_pos_z[0] + coeff_pos_z[1] * t + coeff_pos_z[2] * pow(t,2) + coeff_pos_z[3] * pow(t,3);  
  return p;
}

Eigen::Vector3d calcolaPolinomioVel(double ti, double tc, std::array<double, 3> &coeff_vel_x, std::array<double, 3> &coeff_vel_y, std::array<double, 3> &coeff_vel_z){
  Eigen::Vector3d v;
  double t = tc - ti;
  v[0] = coeff_vel_x[0] + 2*coeff_vel_x[1] * t + 3*coeff_vel_x[2] * pow(t,2);
  v[1] = coeff_vel_y[0] + 2*coeff_vel_y[1] * t + 3*coeff_vel_y[2] * pow(t,2);
  v[2] = coeff_vel_z[0] + 2*coeff_vel_z[1] * t + 3*coeff_vel_z[2] * pow(t,2);
  return v;
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
   //cout << "\n\n\n\nAsse angolo nel metodo\n"<< r0(0) << " " << r0(1) << " " << r0(2) << " " << theta << endl << endl << endl;
   result << r0(0), r0(1), r0(2), theta;
   return result;
}*/


Eigen::VectorXd ascissa(double tk, double tf, double ti){
   double t = (tk-ti)/(tf-ti);
   if(t > 1.0){
     t = 1.0;
   } 
   double s = pow(t, 3)*(6*pow(t,2)-15*t+10);
   double ds = (pow(t,2)*(30*pow(t,2)-60*t+30))/tf;
   Eigen::VectorXd result(2);
   result << s, ds;
   return result;

}
/*
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
}*/

/*
//argv: no
int main(int argc, char** argv) {
    
    file_i = fopen("/home/labarea-franka/libfranka/unibas_peginhole/resources/posa_obj.txt", "r");
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    char stringa[80];
    int res;
    int i = 1;
    int j = 0;
    
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
    matlab::data::TypedArray<int16_t> result = matlabPtr->feval(u"punti_scan_obj", args);
    //int16_t v = result[0];
    //std::cout << "Result: " << v << std::endl;
    
    return 0;
}
*/

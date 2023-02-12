// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/robot_state.h>

/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class DynModel {
 public:
  
  DynModel(const franka::RobotState robot_state);
  Eigen::MatrixXd getMassMatrix(Eigen::VectorXd q);
  Eigen::VectorXd getGravityVector(Eigen::VectorXd q);
  Eigen::VectorXd getFrictionTorque(Eigen::VectorXd dq);
  //Eigen::VectorXd observer(const franka::RobotState& robot_state, Eigen::VectorXd &r, Eigen::VectorXd &integrale_i_1, Eigen::MatrixXd J, double delta_t, Eigen::MatrixXd K, franka::Model& model);
  Eigen::VectorXd observer(const franka::RobotState& robot_state, Eigen::VectorXd &r, Eigen::VectorXd &integrale_i_1, Eigen::MatrixXd J, double delta_t, Eigen::MatrixXd K, franka::Model& model,int albero, bool grasp);
  Eigen::MatrixXd pinv(Eigen::MatrixXd jacobian);

  private:
  franka::RobotState state;

};

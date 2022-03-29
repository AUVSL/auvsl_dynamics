#pragma once
#include <fstream>
#include <string.h>
#include <ros/ros.h>
#include <Eigen/Dense>

//Robcogen files
//just for Scalar type
#include "forward_dynamics.h"
#include "model_constants.h"

using Jackal::rcg::Scalar;


class HybridDynamics{
public:
  const static unsigned STATE_DIM = 6; //x,y,yaw, vx,vy,rad/s
  const static unsigned CNTRL_DIM = 2;
  const static unsigned num_in_features = 2;
  const static unsigned num_hidden_nodes = 8;
  const static unsigned num_out_features = 3;
  
  HybridDynamics();
  ~HybridDynamics();
  
  void log_vehicle_state();
  
  static void start_log();
  static void stop_log();
  
  void initState();
  void initState(Scalar *start_state);
  void step(Scalar vl, Scalar vr);
  
  void Euler(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  void RK4(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  void ODE(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xd, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  void ODE2(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xd, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  void forward_network(Eigen::Matrix<Scalar,num_in_features,1> &input_vec, Eigen::Matrix<Scalar,num_out_features,1> &output_vec);
  
  static std::ofstream log_file;
  static const Scalar timestep; //The rate that nn_model operates at.
  
  Eigen::Matrix<Scalar,STATE_DIM,1> state_;
  
  Eigen::Matrix<Scalar,num_hidden_nodes,num_in_features> weight0;
  Eigen::Matrix<Scalar,num_hidden_nodes,1> bias0;
  Eigen::Matrix<Scalar,num_hidden_nodes,num_hidden_nodes> weight1;
  Eigen::Matrix<Scalar,num_hidden_nodes,1> bias1;
  Eigen::Matrix<Scalar,num_out_features,num_hidden_nodes> weight2;
  Eigen::Matrix<Scalar,num_out_features,1> bias2;
  
  Eigen::Matrix<Scalar,3,2> linear_matrix;
};


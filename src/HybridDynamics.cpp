#include "HybridDynamics.h"
#include "declarations.h"
#include "miscellaneous.h"
#include "utils.h"
#include <iit/rbd/robcogen_commons.h>
#include <cppad/cppad.hpp>
#include <cppad/utility/to_string.hpp>

using CppAD::AD;

const Scalar HybridDynamics::timestep = .05;
const Scalar const_zero = 0;

inline Scalar tanh_scalar_wrapper(Scalar x){
  return CppAD::tanh(x);
  //CppAD::sin(x);
}


std::ofstream HybridDynamics::log_file;

HybridDynamics::HybridDynamics(){
  
  weight0 = .01*Eigen::Matrix<Scalar,num_hidden_nodes,num_in_features>::Random();
  bias0 = .01*Eigen::Matrix<Scalar,num_hidden_nodes,1>::Random();
  
  weight1 = .01*Eigen::Matrix<Scalar,num_hidden_nodes,num_hidden_nodes>::Random();
  bias1 = .01*Eigen::Matrix<Scalar,num_hidden_nodes,1>::Random();
  
  weight2 = .01*Eigen::Matrix<Scalar,num_out_features,num_hidden_nodes>::Random();
  bias2 = .01*Eigen::Matrix<Scalar,num_out_features,1>::Random();
  
  linear_matrix = .01*Eigen::Matrix<Scalar,3,2>::Random();
}

HybridDynamics::~HybridDynamics(){
  
}

void HybridDynamics::initState(){
  Scalar start_state[STATE_DIM] = {0,0,0,0,0,0};
  initState(start_state);
}

void HybridDynamics::initState(Scalar *start_state){
  for(unsigned i = 0; i < STATE_DIM; i++){
    state_[i] = start_state[i];
  }
}

//step is .05s
//THis matches up with the test dataset sample rate.
void HybridDynamics::step(Scalar vl, Scalar vr){  
  Eigen::Matrix<Scalar,STATE_DIM,1> Xt1;
  Eigen::Matrix<Scalar,CNTRL_DIM,1> u;
  
  u(0) = vl;
  u(1) = vr;
  
  const int num_steps = 1;
  for(int ii = 0; ii < num_steps; ii++){
    RK4(state_, Xt1, u);
    state_ = Xt1;
    //log_vehicle_state();
  }
}



void HybridDynamics::RK4(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
  Eigen::Matrix<Scalar,STATE_DIM,1> temp;
  Eigen::Matrix<Scalar,STATE_DIM,1> k1;
  Eigen::Matrix<Scalar,STATE_DIM,1> k2;
  Eigen::Matrix<Scalar,STATE_DIM,1> k3;
  Eigen::Matrix<Scalar,STATE_DIM,1> k4;
  Scalar temp_norm;
  Scalar ts = timestep;
  
  ODE(X, k1, u);
  temp = X + .5*ts*k1;
  
  ODE(temp, k2, u);
  temp = X + .5*ts*k2;
  
  ODE(temp, k3, u);
  temp = X + ts*k3;
  
  ODE(temp, k4, u);
  Xt1 = X + (ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);

}

void HybridDynamics::Euler(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
  Scalar ts = timestep;
  Eigen::Matrix<Scalar,STATE_DIM,1> Xd;
  ODE(X, Xd, u);
  Xt1 = X + (Xd*ts);
}




void HybridDynamics::ODE(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xd, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
    // Scalar vx = (u[0]*0.0542) + (u[1]*0.0370);
    // Scalar vy = (u[0]*0.0036) + (u[1]*-0.0035);
    // Scalar wz = (u[0]*-0.1743) + (u[1]*0.1749);
  Scalar vx = (u[0]*linear_matrix(0,0)) + (u[1]*linear_matrix(0,1));
  Scalar vy = (u[0]*linear_matrix(1,0)) + (u[1]*linear_matrix(1,1));
  Scalar wz = (u[0]*linear_matrix(2,0)) + (u[1]*linear_matrix(2,1));
  
  Xd[0] = vx*CppAD::cos(X[2]) - vy*CppAD::sin(X[2]);
  Xd[1] = vx*CppAD::sin(X[2]) + vy*CppAD::cos(X[2]);
  Xd[2] = wz;
  
  Xd[3] = 0;
  Xd[4] = 0;
  Xd[5] = 0;
}


//Vehicle ODE
//A combination of forward dynamics, external forces, and quaternion derivative.
void HybridDynamics::ODE2(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xd, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
  //input is vx_tire, vy_tire, tire_angular_vel
  //output is Fx, Fy
  Eigen::Matrix<Scalar,num_in_features,1> input_vec;
  Eigen::Matrix<Scalar,num_out_features,1> output_vec;

  // input_vec[0] = X[3];
  // input_vec[1] = X[4];
  // input_vec[2] = X[5];
  input_vec[0] = u[0];
  input_vec[1] = u[1];
  
  forward_network(input_vec, output_vec);
  
  Xd[0] = X[3]*CppAD::cos(X[2]) - X[4]*CppAD::sin(X[2]);
  Xd[1] = X[3]*CppAD::sin(X[2]) + X[4]*CppAD::cos(X[2]);
  Xd[2] = X[5];
  Xd[3] = output_vec[0]; //acceleration x
  Xd[4] = output_vec[1]; //y
  Xd[5] = output_vec[2]; //z
}

void HybridDynamics::forward_network(Eigen::Matrix<Scalar,num_in_features,1> &input_vec, Eigen::Matrix<Scalar,num_out_features,1> &output_vec){
  Eigen::Matrix<Scalar,HybridDynamics::num_hidden_nodes,1> layer0_out;
  Eigen::Matrix<Scalar,HybridDynamics::num_hidden_nodes,1> layer1_out;
  Eigen::Matrix<Scalar,HybridDynamics::num_out_features,1> layer2_out;

  
  layer0_out = weight0*input_vec + bias0;
  layer0_out = layer0_out.unaryExpr(&tanh_scalar_wrapper);
  layer1_out = (weight1*layer0_out) + bias1;
  layer1_out = layer1_out.unaryExpr(&tanh_scalar_wrapper);
  layer2_out = (weight2*layer1_out) + bias2;
  
  output_vec = layer2_out;
  
  //output_vec = weight0*input_vec; //layer2_out;
}

void HybridDynamics::start_log(){
  std::string filename;
  ros::param::get("/xout_log_file_name", filename);
  log_file.open(filename.c_str());
  log_file << "x,y,yaw,vx,vy,wz\n";
}

void HybridDynamics::log_vehicle_state(){
  if(!log_file.is_open()){
    return;
  }
  
  log_file << state_[0] << ',';
  log_file << state_[1] << ',';
  log_file << state_[2] << ',';
  
  log_file << state_[3] << ','; //position
  log_file << state_[4] << ',';
  log_file << state_[5] << '\n';
}

void HybridDynamics::stop_log(){
  log_file.close();
}

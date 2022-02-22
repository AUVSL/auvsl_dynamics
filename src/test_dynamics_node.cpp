#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include <cppad/cppad.hpp>
#include <cppad/utility/to_string.hpp>

#include "HybridDynamics.h"
#include "miscellaneous.h"
#include "model_constants.h"

#define EPS 1e-3f

//Quick test on the output of the tire network.
void test_tire_network(){
  Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> in_vec;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> out_vec;
  
  in_vec << .01, .5, 0, 29.76, 2083, .8, 0, .3930;
  
  TireNetwork tireNetwork;
  
  TireNetwork::forward(in_vec, out_vec);
  ROS_INFO("C++ NN:  %s %s %s %s",
           CppAD::to_string(out_vec[0]).c_str(),
           CppAD::to_string(out_vec[1]).c_str(),
           CppAD::to_string(out_vec[2]).c_str(),
           CppAD::to_string(out_vec[3]).c_str()
           );
  ROS_INFO("PyTorch: 64.21559, -0.35487288, 203.69678, -6.7131023");
  //The outputs match. Test passes.
}

//prints expected result. Tire COM is 0,0,0 in the tire frame.
/*
void print_tire_com(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  
  //just print two to see if theres any pattern.
  Jackal::rcg::Vector3 com_pos = g_hybrid_model.inertias.getCOM_front_left_wheel_link();
  ROS_INFO("%f   %f   %f", com_pos[0], com_pos[1], com_pos[2]);
  
  com_pos = g_hybrid_model.inertias.getCOM_front_right_wheel_link();
  ROS_INFO("%f   %f   %f", com_pos[0], com_pos[1], com_pos[2]);
}

//Start with an initial velocity in the x direction.
//Should show 0 acceleration for all components except in z, due to gravity.
//Velocity should stay the same
//passes
void print_ode_vx(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  g_hybrid_model.state_[6] = 10; //start above ground (0 normal force)
  g_hybrid_model.state_[14] = 1;
  
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> X = g_hybrid_model.state_;
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> Xd;
  Eigen::Matrix<Scalar,HybridDynamics::CNTRL_DIM,1> u(0,0);
  
  g_hybrid_model.ODE(X, Xd, u);

  int expected = 1;
  ROS_INFO("Testing ODE with Initial Condition vx = 1");
  for(int i = 0; i < 21; i++){
    ROS_INFO("%f     %f", X[i], Xd[i]);
    if(i == 16)
      expected &= fabs(Xd[16] + 9.81) < EPS;
    else if(i == 4)
      expected &= fabs(Xd[i] - 1) < EPS;
    else
      expected &= fabs(Xd[i]) < EPS;
  }
  if(expected){
    ROS_INFO("Test Passed");
  }
  else{
    ROS_INFO("Test Failed");
  }
}

//At first this greatly confused me, because I didn't expect to see any rotational
//acceleration at all. Then I realized that due to asymetric mass distribution,
//the principal axis of the vehicle doesn't align with the z axis.
//This leads to precession/nutation which creates an angular acceleration
//about the x and y axes.
void print_ode_wz(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  Scalar wz = 1;
  
  Jackal::rcg::Vector3 com_pos = getWholeBodyCOM(g_hybrid_model.inertias, g_hybrid_model.h_transforms);
  g_hybrid_model.state_[6] = 1000;
  g_hybrid_model.state_[13] = wz;
  g_hybrid_model.state_[14] = com_pos[1]*wz;
  g_hybrid_model.state_[15] = -com_pos[0]*wz;
  g_hybrid_model.state_[16] = 0;
  
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> X = g_hybrid_model.state_;
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> Xd;
  Eigen::Matrix<Scalar,HybridDynamics::CNTRL_DIM,1> u(0,0);
  
  g_hybrid_model.ODE(X, Xd, u);
  
  ROS_INFO("Testing ODE with Initial Condition wz = 1");
  for(int i = 0; i < 21; i++){
    ROS_INFO("%f     %f", X[i], Xd[i]);
  }
}

void test_ode_normal_force(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  g_hybrid_model.state_[12] = .1;
  
  //Scalar mass = g_hybrid_model.inertias.getTotalMass();
  //ROS_INFO("Total Mass %f", mass);
  
  Jackal::rcg::Vector3 com_pos = getWholeBodyCOM(g_hybrid_model.inertias, g_hybrid_model.h_transforms);
  
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> X = g_hybrid_model.state_;
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> Xd;
  Eigen::Matrix<Scalar,HybridDynamics::CNTRL_DIM,1> u(0,0);
  
  g_hybrid_model.ODE(X, Xd, u);
}

*/
void test_angular_integration(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  
  // Jackal::rcg::Vector3 com_pos = getWholeBodyCOM(g_hybrid_model.inertias, g_hybrid_model.h_transforms);
  // Scalar wz = 1;
  // g_hybrid_model.state_[13] = wz;
  // g_hybrid_model.state_[14] = (com_pos[1]*wz) + 1;
  // g_hybrid_model.state_[15] = -com_pos[0]*wz;
  
  ROS_INFO("Start Pos: %s %s %s",
           CppAD::to_string(g_hybrid_model.state_[4]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[5]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[6]).c_str());
  
  ROS_INFO("Start Quat: %s %s %s %s",
           CppAD::to_string(g_hybrid_model.state_[0]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[1]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[2]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[3]).c_str());
  
  //10s
  for(int i = 0; i < 20; i++){
    g_hybrid_model.step(0,0);
  }
  
  ROS_INFO("End Pos: %s %s %s",
           CppAD::to_string(g_hybrid_model.state_[4]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[5]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[6]).c_str());
  
  ROS_INFO("End Quat: %s %s %s %s",
           CppAD::to_string(g_hybrid_model.state_[0]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[1]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[2]).c_str(),
           CppAD::to_string(g_hybrid_model.state_[3]).c_str());
  
}



int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;  
  
  
  HybridDynamics::start_log();

  //use plot.py to visualize as well.

  //test_tire_network();
  //test_ode_normal_force();
  //test_angular_integration();
  //print_ode_vx();
  //print_ode_wz();
  
  HybridDynamics::stop_log();
  
  return 0;
}

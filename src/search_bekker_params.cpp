#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
//#include <iosfwd>
//#include <cppad/cg.hpp>

#include "VerifyModel.h"

#include <tf/tf.h>
#include <ros/ros.h>

static HybridDynamics *g_hybrid_model;


//Example of how to use CppAD with eigen.
//I couldn't find a good tutorial for this online.
//I did a lot of trial and error to find the right template parameters
//for the eigen::Matrix's and the CppAD functions

/*
int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "search_bekker_params");
  ros::NodeHandle nh;
  
  // using CGF = CppAD::cg::CG<float>;
  // using ADCG = CppAD::AD<CGF>;
  
  
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> X(3);
  Eigen::Matrix<Scalar,3,3> A;
  A << 1, 0, 0, 0, 2, 0, 0, 0, 3;
  
  CppAD::Independent(X);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> Y(3);
  Y = A*X;
  CppAD::ADFun<float> fun(X, Y);
  Eigen::Matrix<float,Eigen::Dynamic,1> x0(3);
  x0[0] = 1;
  x0[1] = 1;
  x0[2] = 1;
  
  Eigen::Matrix<float,9,1> jac;
  jac = fun.Jacobian(x0);
  
  
  ROS_INFO("Jacobian");
  ROS_INFO("%f %f %f\n%f %f %f\n %f %f %f",
           jac(0), jac(1), jac(2),
           jac(3), jac(4), jac(5),
           jac(6), jac(7), jac(8));

}
*/

/*
//I want to test if reverse and forward return same result. (I think they should)
int main(int argc, char **argv){
  ros::init(argc, argv, "search_bekker_params");
  ros::NodeHandle nh;
  
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> X(1);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> Y(1);
  
  X[0] = 1;
  
  CppAD::Independent(X);
  Y[0] = (X[0]*X[0]) + 3*X[0] + 6.0f;
  CppAD::ADFun<float> fun(X, Y);
  Eigen::Matrix<float,Eigen::Dynamic,1> x1(1);
  x1[0] = 1;
  
  Eigen::Matrix<float,Eigen::Dynamic,1> y0(1);
  y0[0] = 1;
  
  Eigen::Matrix<float,1,1> dydx;
  dydx = fun.Reverse(1, y0);
  ROS_INFO("Reverse %f", dydx[0]);

  dydx = fun.Forward(1, x1);
  ROS_INFO("Forward %f", dydx[0]);
}
*/

/*
int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "search_bekker_params");
  ros::NodeHandle nh;
  
  g_hybrid_model = new HybridDynamics();
  
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> x_bekker_param(1);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> y_Xd(HybridDynamics::STATE_DIM);
  
  CppAD::Independent(x_bekker_param);
  
  g_hybrid_model->bekker_params[0] = 29.76;
  g_hybrid_model->bekker_params[1] = 2083;
  g_hybrid_model->bekker_params[2] = x_bekker_param[0]; //.8;
  g_hybrid_model->bekker_params[3] = 0;
  g_hybrid_model->bekker_params[4] = 22.5*M_PI/180.0;
  
  Scalar start_state[HybridDynamics::STATE_DIM] = {0,0,0,1, 0,0,.06, 0,0,0,0, 0,0,0, 0,0,0, 0,0,0,0};
  
  g_hybrid_model->initState(start_state);
  g_hybrid_model->step(0,0); //(X, Xt1, u);
  
  for(unsigned i = 0; i < HybridDynamics::STATE_DIM; i++){
    y_Xd[i] = g_hybrid_model->state_[i];
  }
  
  CppAD::ADFun<float> ode_f(x_bekker_param, y_Xd);
  Eigen::Matrix<float,HybridDynamics::STATE_DIM,1> jac;
  Eigen::Matrix<float,Eigen::Dynamic,1> n0(1);
  n0[0] = .8f;
  jac = ode_f.Jacobian(n0);
  
  ROS_INFO("Partial wrt sinkage:");
  for(int i = 0; i < HybridDynamics::STATE_DIM; i++){
    ROS_INFO("Xt1[%d]: %f", i, jac[i]);
  }
  
  //init_tests();
  //Scalar y = test_train3_params(bekker_params);
  //del_tests();
  
  return 0;
}
*/


int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "search_bekker_params");
  ros::NodeHandle nh;
  
  
  
  init_tests();
  //test_CV3_paths();
    
  train_model_on_dataset(.01f);
  del_tests();
}

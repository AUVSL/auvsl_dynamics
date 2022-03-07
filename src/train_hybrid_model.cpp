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



/*
int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "train_hybrid_model");
  ros::NodeHandle nh;
  
  g_hybrid_model = new HybridDynamics();

  int num_weights = get_num_parameters();
  //THese are the input and output vectors.
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> x_weights(num_weights);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> y_x_pos(1);
  
  get_model_params(x_weights); //initializes the values.
  CppAD::Independent(x_weights);
  set_model_params(x_weights); //this creates the dependency needed to calculate the derivative wrt these params.
  
  Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> features;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> forces;
  
  features[0] = 1;
  features[1] = 1;
  features[2] = 1;
  features[3] = 1;
  features[4] = 1;
  TireNetwork::forward(features, forces);
  
  
  y_x_pos[0] = forces[0];
  
  CppAD::ADFun<float> ode_f(x_weights, y_x_pos);
  
  ROS_INFO("%f %f %f", CppAD::Value(forces[0]), CppAD::Value(forces[1]), CppAD::Value(forces[2]));
  
  Eigen::Matrix<float,Eigen::Dynamic,1> gradient(num_weights);
  Eigen::Matrix<float,Eigen::Dynamic,1> test_x_pos(1);
  test_x_pos[0] = 1;
  gradient = ode_f.Reverse(1, test_x_pos);
  
  ROS_INFO("Partial wrt sinkage:");
  for(int i = 0; i < num_weights; i++){
    ROS_INFO("gradient[%d]: %f", i, gradient[i]);
  }
  
  return 0;
}
*/



//Start by taking derivative wrt network weights
/*
int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "search_bekker_params");
  ros::NodeHandle nh;
  
  g_hybrid_model = new HybridDynamics();

  int num_weights = get_num_parameters();
  //THese are the input and output vectors.
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> x_weights(num_weights);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> y_x_pos(1);
  
  get_model_params(x_weights); //initializes the values.
  //x_weights[0] = TireNetwork::weight0(0,0);
  CppAD::Independent(x_weights);
  //TireNetwork::weight0(0,0) = x_weights[0];
  set_model_params(x_weights); //this creates the dependency needed to calculate the derivative wrt these params.
  
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> Xd;
  Eigen::Matrix<Scalar,HybridDynamics::CNTRL_DIM,1> u;
  Scalar start_state[HybridDynamics::STATE_DIM] = {0,0,0,1, 0,0,.01, 0,0,0,0, 0,0,0, 0,0,0, 0,0,0,0};
  g_hybrid_model->initState(start_state);
  //g_hybrid_model->step(1,1); //(X, Xt1, u);
  u(0) = 1;
  u(1) = 1;
  g_hybrid_model->state_[17] = u[0];
  g_hybrid_model->state_[18] = u[1];
  g_hybrid_model->state_[19] = u[0];
  g_hybrid_model->state_[20] = u[1];
  g_hybrid_model->ODE(g_hybrid_model->state_, Xd, u);
  
  //y_x_pos[0] = g_hybrid_model->state_[0];
  y_x_pos[0] = Xd[16];
  
  CppAD::ADFun<float> ode_f(x_weights, y_x_pos);

  for(int i = 0; i < HybridDynamics::STATE_DIM; i++){
    ROS_INFO("%f", CppAD::Value(Xd[i]));
  }
  
  Eigen::Matrix<float,Eigen::Dynamic,1> gradient(num_weights);
  Eigen::Matrix<float,Eigen::Dynamic,1> test_x_pos(1);
  test_x_pos[0] = 1;
  gradient = ode_f.Reverse(1, test_x_pos);
  
  ROS_INFO("Partial wrt sinkage:");
  for(int i = 0; i < num_weights; i++){
    ROS_INFO("gradient[%d]: %f", i, gradient[i]);
  }
  
  return 0;
}
*/

//compare the input outputs to what happens in python
void test_network(){
  Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> in_vec;
  in_vec << -3.94063,   -3.4051502, -1.79449,   -3.42353,    0.0734259;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> out_vec;
  TireNetwork::forward(in_vec, out_vec);
  
  ROS_INFO("Actual %f %f %f", CppAD::Value(out_vec[0]), CppAD::Value(out_vec[1]), CppAD::Value(out_vec[2]));
  ROS_INFO("Expected  -240.41273  -435.0391  929.8925");
}



int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "train_hybrid_model");
  ros::NodeHandle nh;

  g_hybrid_model = new HybridDynamics();
  
  g_hybrid_model->start_log();
  
  g_hybrid_model->initState();
  g_hybrid_model->settle();
  for(int i = 0; i < 20;i++){
    //  g_hybrid_model->step(1,1);
  }
  
  g_hybrid_model->stop_log();
  
  //init_tests();
  //test_CV3_paths();
  //train_model_on_dataset(.001f);
  //del_tests();
}


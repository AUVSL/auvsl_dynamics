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

#include "HybridDynamics.h"
#include "VerifyModel.h"
#include "AdjointMethod.h"

#include <tf/tf.h>
#include <ros/ros.h>

static HybridDynamics *g_hybrid_model;

//Start by taking derivative wrt network weights
/*
int mainly_ignore_me(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "search_bekker_params");
  ros::NodeHandle nh;
  
  g_hybrid_model = new HybridDynamics();

  int num_weights = getNumWeights();
  //THese are the input and output vectors.
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> x_weights(num_weights);
  Eigen::Matrix<float,Eigen::Dynamic,1> x_weights_float(num_weights);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> y_x_pos(1);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> start_state1(HybridDynamics::STATE_DIM); //dynamic parameter
  
  start_state1 << 0,0,0,1, 0,0,.04, 0,0,0,0, 0,0,0, 0,0,0, 0,0,0,0;
  
  getModelParams(x_weights_float); //initializes the values.
  for(int i = 0; i < num_weights; i++){
    x_weights[i] = x_weights_float[i];
  }
  CppAD::Independent(x_weights, start_state1);
  setModelParams(x_weights); //this creates the dependency needed to calculate the derivative wrt these params.
  
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> Xd;
  Eigen::Matrix<Scalar,HybridDynamics::CNTRL_DIM,1> u;
  
  g_hybrid_model->initStateCOM(start_state1);
  //g_hybrid_model->step(1,1); //(X, Xt1, u);
  u(0) = 1;
  u(1) = 1;
  g_hybrid_model->state_[17] = u[0];
  g_hybrid_model->state_[18] = u[1];
  g_hybrid_model->state_[19] = u[0];
  g_hybrid_model->state_[20] = u[1];
  g_hybrid_model->ODE(g_hybrid_model->state_, Xd, u);
  
  //y_x_pos[0] = g_hybrid_model->state_[0];
  y_x_pos[0] = Xd[16]; //resulting forward velocity, vx
  
  CppAD::ADFun<float> ode_f(x_weights, y_x_pos);

  // for(int i = 0; i < HybridDynamics::STATE_DIM; i++){
  //   ROS_INFO("%f", CppAD::Value(Xd[i]));
  // }
  
  Eigen::Matrix<float,Eigen::Dynamic,1> gradient1(num_weights);
  Eigen::Matrix<float,Eigen::Dynamic,1> gradient2(num_weights);
  Eigen::Matrix<float,Eigen::Dynamic,1> start_state2(HybridDynamics::STATE_DIM);
  Eigen::Matrix<float,Eigen::Dynamic,1> test_x_pos(1);
  
  test_x_pos[0] = 1;
  gradient1 = ode_f.Reverse(1, test_x_pos);
  
  start_state2 << 0,0,0,1, 0,0,.03, 0,0,0,0, 0,0,0, 0,0,0, 0,0,0,0;
  ode_f.new_dynamic(start_state2);
  gradient2 = ode_f.Reverse(1, test_x_pos);
  
  ROS_INFO("Partial wrt sinkage:");
  for(int i = 0; i < num_weights; i++){
    ROS_INFO("gradient[%d]: IC#1: %f     IC#2: %f", i, gradient1[i], gradient2[i]);
  }
  
  return 0;
}
*/



//need to understand this real quick.
//It checks out.
void test_dynamic_params(){
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> x(3); //independent vector
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> y(1); //dependent vector
  Eigen::Matrix<Scalar,1,3> A;
  A << 1,2,3;
  
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> b(1); //dynamic params
  b[0] = 5;
  
  CppAD::Independent(x, b);
  y = A*x + b*x[0];
  
  CppAD::ADFun<float> fun(x, y);

  Eigen::Matrix<float,Eigen::Dynamic,1> x1(3); //gradient
  Eigen::Matrix<float,Eigen::Dynamic,1> y1(1); //partial wrt itself.
  Eigen::Matrix<float,Eigen::Dynamic,1> b_new(1); //dynamic params
  
  y1[0] = 1;
  x1 = fun.Reverse(1, y1);
  ROS_INFO("%f %f %f", x1[0], x1[1], x1[2]);

  b_new[0] = 7;
  fun.new_dynamic(b_new);
  x1 = fun.Reverse(1, y1);
  ROS_INFO("%f %f %f", x1[0], x1[1], x1[2]);
}




//compare the input outputs to what happens in python
Eigen::Matrix<Scalar,TireNetwork::num_out_features,1>  test_network(){
  Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> in_vec;
  in_vec << -3.94063,   -3.4051502, -1.79449,   -3.42353,    0.0734259;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> out_vec;
  TireNetwork::forward(in_vec, out_vec);
  
  //ROS_INFO("Actual %f %f %f", CppAD::Value(out_vec[0]), CppAD::Value(out_vec[1]), CppAD::Value(out_vec[2]));
  //ROS_INFO("Expected  -240.41273  -435.0391  929.8925");

  return out_vec;
}


//Tests save load functions
//Tests model get/set functions
//Passes all 3 tests.
/*
void test_network_save_load_get_set(){
  //network already exists in memory, no need to load first.
  unsigned num_params = getNumWeights();
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> model_params(num_params);
  Eigen::Matrix<float,Eigen::Dynamic,1> test_model_params(num_params);
  
  for(unsigned ii = 0; ii < num_params; ii++){
    model_params(ii,0) = (float) ii;
  }
  
  setModelParams(model_params);
  getModelParams(test_model_params);
  
  for(unsigned ii = 0; ii < num_params; ii++){
    if(fabs(CppAD::Value(model_params[ii]) - test_model_params[ii]) > 1e-4){
      ROS_INFO("Test 1. Mismatch after set get at idx %u", ii);
      return;
    }
  }
  
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> labels_before;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> labels_after;
  
  setModelParams(model_params);
  labels_before = test_network();
  saveHybridNetwork();
  loadHybridNetwork();
  labels_after = test_network();
  getModelParams(test_model_params);

  for(unsigned ii = 0; ii < num_params; ii++){
    if(fabs(CppAD::Value(model_params[ii]) - test_model_params[ii]) > 1e-4){
      ROS_INFO("Test 2. Mismatch after set get at idx %u", ii);
      return;
    }
  }

  for(unsigned ii = 0; ii < labels_after.size(); ii++){
    if(CppAD::Value(CppAD::abs(labels_after[ii] - labels_before[ii])) > 1e-4){
      ROS_INFO("Test 3. Mismatch test_network at idx %u", ii);
      return;
    }
  }

  
}
*/

//essentially this is xd = Ax + theta*x
template <typename MatrixType>
void test_ode2(MatrixType &X, MatrixType &Xd, MatrixType &theta){
  Xd[0] = -.1*X[0]*X[1] + -2*X[1] + theta[0];
  Xd[1] = 3*X[0] + 4*X[1] + theta[1];
}

template <typename MatrixType>
void test_ode1(MatrixType &X, MatrixType &Xd, MatrixType &theta){
  Xd[0] = -theta[0]*X[0];
}


template <typename MatrixType1, typename MatrixType2>
void l2_loss(MatrixType1 &values, MatrixType1 &true_values, MatrixType2 &loss){
  MatrixType1 errors = values - true_values;
  loss = errors.transpose()*errors;
}

/*
void getGradient(){
  Eigen::Matrix<float,Eigen::Dynamic,1> theta(1);
  theta[0] = 2;
  
  AdjointMethod am;
  am.setODE(&test_ode1, 2, 2);
  am.setLossFunction(&l2_loss);
  am.performAD();
  
  am.theta_ = theta;
  
  Eigen::Matrix<float,Eigen::Dynamic,1> zt(am.dim_state_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W1(am.dim_state_); //a(t)
  Eigen::Matrix<float,Eigen::Dynamic,1> W2(am.dim_params_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W1d(am.dim_state_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W2d(am.dim_params_);
  Eigen::Matrix<float,Eigen::Dynamic,1> loss(1);
  
  zt[0] = 1;
  
  loss[0] = 1; //partial of loss wrt itself.
  am.loss_fun_->Forward(0,zt);
  W1 = am.loss_fun_->Reverse(1, loss); //initialize a(t1) = dL/dz(t1)
  W2 = Eigen::Matrix<float,Eigen::Dynamic,1>::Zero(am.dim_params_,1);
  
  am.augmentedODE(zt, W1, W2, W1d, W2d);
  
  ROS_INFO("Check em: %f %f", W1d[0], W2d[0]);
  //they check out.
}
*/


void fd_reference_gradient(){
  Eigen::Matrix<double,2,1> X_ic;
  Eigen::Matrix<double,2,1> X;
  Eigen::Matrix<double,2,1> Xd;
  Eigen::Matrix<double,1,1> loss_value1;
  Eigen::Matrix<double,1,1> loss_value2;
  Eigen::Matrix<double,2,1> theta_ic;
  Eigen::Matrix<double,2,1> theta;
  
  Eigen::Matrix<double,2,1> k1(2);
  Eigen::Matrix<double,2,1> k2(2);
  Eigen::Matrix<double,2,1> k3(2);
  Eigen::Matrix<double,2,1> k4(2);
  Eigen::Matrix<double,2,1> temp(2);
  
  theta_ic[0] = .1;
  theta_ic[1] = .2;
  theta = theta_ic;
  
  X_ic[0] = 1;
  X_ic[1] = 1;
  X = X_ic;
  
  double ts = .001;
  
  //euler integrate it.
  for(int i = 0; i < 999; i++){
    //test_ode2(X,Xd,theta);
    //X += ts*Xd;
    
    test_ode2(X,k1,theta);
    temp = X + .5*ts*k1;
    test_ode2(temp,k2,theta);
    temp = X + .5*ts*k2;
    test_ode2(temp,k3,theta);
    temp = X + ts*k3;
    test_ode2(temp,k4,theta);
    X += (ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
  }
  
  Eigen::Matrix<double,2,1> X_true;
  X_true[0] = 0;
  X_true[1] = 0;
  l2_loss(X, X_true, loss_value1);
  
  ROS_INFO("Final Values %f %f    Loss %f", X[0], X[1], loss_value1[0]);
  ROS_INFO("Finite Difference as h -> 0");
  
  
  double h;
  for(int j = 1; j < 10; j++){
    h = exp10(-j);
    
    X = X_ic;
    theta = theta_ic;
    theta[0] = theta[0] + h;
    
    for(int i = 0; i < 999; i++){
      // test_ode2(X,Xd,theta);
      // X += ts*Xd;
      
      test_ode2(X,k1,theta);
      temp = X + .5*ts*k1;
      test_ode2(temp,k2,theta);
      temp = X + .5*ts*k2;
      test_ode2(temp,k3,theta);
      temp = X + ts*k3;
      test_ode2(temp,k4,theta);
      X += (ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    }
    
    Eigen::Matrix<double,2,1> X_true;
    X_true[0] = 0;
    X_true[1] = 0;
    l2_loss(X, X_true, loss_value2);
    double fd_grad = (loss_value2[0] - loss_value1[0]) / h;
    
    ROS_INFO("FD Gradient %f\th=%f", fd_grad, h);
  }
}


void ad_reference_gradient(){
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> X(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> Xd(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> theta(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> loss(1);
  
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> k1(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> k2(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> k3(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> k4(2);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> temp(2);
  
  theta[0] = .1;
  theta[1] = .2;
  
  X[0] = 1;
  X[1] = 1;
  
  Scalar ts = .001;
  
  CppAD::Independent(theta);

  for(int i = 0; i < 999; i++){
    // test_ode2(X, Xd, theta);
    // X += ts*Xd;

    test_ode2(X,k1,theta);
    temp = X + .5*ts*k1;
    test_ode2(temp,k2,theta);
    temp = X + .5*ts*k2;
    test_ode2(temp,k3,theta);
    temp = X + ts*k3;
    test_ode2(temp,k4,theta);
    X += (ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
  }
  
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> X_true(2);
  X_true[0] = 0;
  X_true[1] = 0;
  l2_loss(X, X_true, loss);
  CppAD::ADFun<float> ode_f(theta, loss);
  
  Eigen::Matrix<float,Eigen::Dynamic,1> loss0(1);
  Eigen::Matrix<float,Eigen::Dynamic,1> ad_gradient;
  loss0[0] = 1;
  
  ad_gradient = ode_f.Reverse(1, loss0);
  ROS_INFO("AD Gradient %f %f", ad_gradient[0], ad_gradient[1]);
}



//test.
void test_asm_ode(){
  Eigen::Matrix<float,Eigen::Dynamic,1> theta(2);
  theta[0] = .1;
  theta[1] = .2;
  
  AdjointMethod am;
  am.setTimestep(.001);
  am.setODE(&test_ode2, 2, 2);
  am.setLossFunction(&l2_loss);
  am.performAD();
  
  //need to generate state history.
  
  std::vector<Eigen::Matrix<float,Eigen::Dynamic,1>> z_history;
  Eigen::Matrix<float,Eigen::Dynamic,1> X(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> Xd(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k1(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k2(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k3(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k4(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> temp(2);
  
  
  //initial condition
  X[0] = 1;
  X[1] = 1; 
  
  float ts = .001;
  
  //euler integrate it.
  z_history.push_back(X);
  for(int i = 0; i < 999; i++){
    //test_ode2(X,Xd,theta);
    //X += ts*Xd;
    
    test_ode2(X,k1,theta);
    temp = X + .5*ts*k1;
    test_ode2(temp,k2,theta);
    temp = X + .5*ts*k2;
    test_ode2(temp,k3,theta);
    temp = X + ts*k3;
    test_ode2(temp,k4,theta);
    X += (ts/6.0)*(k1 + 2*k2 + 2*k3 + k4);
    
    z_history.push_back(X);
  }
  
  am.setStateHistory(&z_history);
  
  Eigen::Matrix<float,Eigen::Dynamic,1> gradient(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> true_values(2);
  true_values[0] = 0;
  true_values[1] = 0;
  am.getGradient(theta, true_values, gradient);
  
  ROS_INFO("Adjoint Method gradient %f %f", gradient[0], gradient[1]);
  fd_reference_gradient();
  ad_reference_gradient();
}



int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "train_hybrid_model");
  ros::NodeHandle nh;
  
  test_asm_ode();
}




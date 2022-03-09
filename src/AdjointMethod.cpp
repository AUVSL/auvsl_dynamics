#include "AdjointMethod.h"
#include <ros/ros.h>

//I want this file to be generic so that I can test it with test ode's
//System:
//z_dot = f(z,theta), t in [t0,t1], z(t0) and z(t1) known.
//a(t) = dl/dz(t)
//theta is parameters


AdjointMethod::AdjointMethod(){
  df_dz_fun_ = 0;
  df_dtheta_fun_ = 0;
  loss_fun_ = 0;
}

AdjointMethod::~AdjointMethod(){
  if(df_dz_fun_){
    delete df_dz_fun_;
    delete df_dtheta_fun_;
    delete loss_fun_;
  }
}

//This is the final function.
void AdjointMethod::getGradient(Eigen::Matrix<float,Eigen::Dynamic,1> &theta){
  theta_ = theta;
  
  Eigen::Matrix<float,Eigen::Dynamic,1> zt(dim_state_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W1(dim_state_); //a(t)
  Eigen::Matrix<float,Eigen::Dynamic,1> W2(dim_params_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W1d(dim_state_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W2d(dim_params_);
  Eigen::Matrix<float,Eigen::Dynamic,1> loss(1);
  
  zt[0] = 1;
  
  loss[0] = 1; //partial of loss wrt itself.
  loss_fun_->Forward(0,zt);
  W1 = loss_fun_->Reverse(1, loss); //initialize a(t1) = dL/dz(t1)
  W2 = Eigen::Matrix<float,Eigen::Dynamic,1>::Zero(dim_params_,1);
  
  augmentedODE(zt, W1, W2, W1d, W2d);
  
  //
  
  
}

void AdjointMethod::augmentedODE(Eigen::Matrix<float,Eigen::Dynamic,1> &zt,
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W1, //a(t)
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W2, //idk?
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W1d,//-a_dot(t)
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W2d //-a(t)*df/dtheta
                                 ){
  
  df_dz_fun_->new_dynamic(theta_);
  W1d = -W1.transpose()*df_dz_fun_->Jacobian(zt);
  
  df_dtheta_fun_->new_dynamic(zt);
  W2d = -W1.transpose()*df_dtheta_fun_->Jacobian(theta_);
  
}

//obtain the f(z,theta) ode.
//This takes a vector valued function.
void AdjointMethod::setODE(void (*function)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&), int dim_state, int dim_params){
  ode_f_ = function;
  dim_state_ = dim_state;
  dim_params_ = dim_params;
}

void AdjointMethod::setLossFunction(void (*function)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&)){
  loss_f_ = function;
}

//Need to use AD to calculate two Jacobians.
//df/dz, df/dtheta
void AdjointMethod::performAD(){
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> Z(dim_state_);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> Zd(dim_state_);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> theta(dim_params_);
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> loss(1);

  CppAD::Independent(Z, theta);
  ode_f_(Z,Zd,theta);
  df_dz_fun_ = new CppAD::ADFun<float>(Z,Zd);

  CppAD::Independent(theta, Z);
  ode_f_(Z,Zd,theta);
  df_dtheta_fun_ = new CppAD::ADFun<float>(theta,Zd);

  CppAD::Independent(Z);
  loss_f_(Z, loss);
  loss_fun_ = new CppAD::ADFun<float>(Z, loss);
}

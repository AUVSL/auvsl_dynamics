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

void AdjointMethod::setTimestep(float timestep){
  ts_ = timestep;
}

//This is the final function.
void AdjointMethod::getGradient(const Eigen::Matrix<float,Eigen::Dynamic,1> &theta, const Eigen::Matrix<float,Eigen::Dynamic,1> &true_values, Eigen::Matrix<float,Eigen::Dynamic,1>& gradient){
  theta_ = theta;
  
  Eigen::Matrix<float,Eigen::Dynamic,1> zt(dim_state_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W1(dim_state_); //a(t)
  Eigen::Matrix<float,Eigen::Dynamic,1> W2(dim_params_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W1d(dim_state_);
  Eigen::Matrix<float,Eigen::Dynamic,1> W2d(dim_params_);
  Eigen::Matrix<float,Eigen::Dynamic,1> loss(1);

  Eigen::Matrix<float,Eigen::Dynamic,1> k1_1(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k2_1(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k3_1(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k4_1(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> temp_1(2);
  
  Eigen::Matrix<float,Eigen::Dynamic,1> k1_2(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k2_2(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k3_2(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> k4_2(2);
  Eigen::Matrix<float,Eigen::Dynamic,1> temp_2(2);

  
  //Set initial conditions
  unsigned t1 = z_history_->size()-1;
  zt = z_history_->at(t1); //z(t1)
  
  loss[0] = 1; //partial of loss wrt itself.
  loss_fun_->new_dynamic(true_values);
  loss_fun_->Forward(0,zt);
  W1 = loss_fun_->Reverse(1, loss); //initialize a(t1) = dL/dz(t1)
  W2 = Eigen::Matrix<float,Eigen::Dynamic,1>::Zero(dim_params_,1);
  
  
  float ts = -1*ts_;
  //integrate backwards from t1 to t0.
  for(int t = t1; t >= 1; t--){
    zt = z_history_->at(t);
    
    augmentedODE(zt, W1, W2, W1d, W2d);
    W1 += W1d*ts; //minus sign due to integrating backwards.
    W2 += W2d*ts;

    // augmentedODE(zt, W1, W2, k1_1, k1_2);
    // temp_1 = W1 + .5*ts*k1_1;
    // temp_2 = W2 + .5*ts*k1_2;
    
    // augmentedODE(zt, temp_1, temp_2, k2_1, k2_2);
    // temp_1 = W1 + .5*ts*k2_1;
    // temp_2 = W2 + .5*ts*k2_2;

    // augmentedODE(zt, temp_1, temp_2, k3_1, k3_2);
    // temp_1 = W1 + ts*k3_1;
    // temp_2 = W2 + ts*k3_2;
    
    // augmentedODE(zt, temp_1, temp_2, k4_1, k4_2);
    // W1 += (ts/6.0)*(k1_1 + 2*k2_1 + 2*k3_1 + k4_1);
    // W2 += (ts/6.0)*(k1_2 + 2*k2_2 + 2*k3_2 + k4_2);
  }
  
  
  gradient = W2;
  
}

void AdjointMethod::augmentedODE(Eigen::Matrix<float,Eigen::Dynamic,1> &zt,
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W1, //a(t)
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W2, //idk?
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W1d,//-a_dot(t)
                                 Eigen::Matrix<float,Eigen::Dynamic,1> &W2d //-a(t)*df/dtheta
                                 ){
  df_dz_fun_->new_dynamic(theta_);
  Eigen::Matrix<float,Eigen::Dynamic,1> df_dz_vec = df_dz_fun_->Jacobian(zt);
  Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> df_dz_mat(dim_state_, dim_state_);
  int cnt = 0;
  for(int i = 0; i < dim_state_; i++){
    for(int j = 0; j < dim_state_; j++){
      df_dz_mat(j,i) = df_dz_vec[cnt];
      cnt++;
    }
  }  
  W1d = -W1.transpose()*df_dz_mat;

  
  df_dtheta_fun_->new_dynamic(zt);
  Eigen::Matrix<float,Eigen::Dynamic,1> df_dtheta_vec = df_dtheta_fun_->Jacobian(theta_);
  Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic> df_dtheta_mat(dim_state_, dim_params_);
  cnt = 0;
  for(int i = 0; i < dim_state_; i++){
    for(int j = 0; j < dim_params_; j++){
      df_dtheta_mat(j,i) = df_dtheta_vec[cnt];
      cnt++;
    }
  }
  W2d = -W1.transpose()*df_dtheta_mat;
}


void AdjointMethod::setStateHistory(std::vector<Eigen::Matrix<float,Eigen::Dynamic,1>> *z_history){
  z_history_ = z_history;
}

//obtain the f(z,theta) ode.
//This takes a vector valued function.
void AdjointMethod::setODE(void (*function)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&), int dim_state, int dim_params){
  ode_f_ = function;
  dim_state_ = dim_state;
  dim_params_ = dim_params;
}

void AdjointMethod::setLossFunction(void (*function)(Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&, Eigen::Matrix<Scalar,Eigen::Dynamic,1>&)){
  loss_f_ = function;
}

//Need to use AD to calculate two Jacobians.
//df/dz, df/dtheta
void AdjointMethod::performAD(){
  Eigen::Matrix<Scalar,Eigen::Dynamic,1> true_zt1(dim_state_);
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

  CppAD::Independent(Z, true_zt1);
  loss_f_(Z, true_zt1, loss);
  loss_fun_ = new CppAD::ADFun<float>(Z, loss);
}

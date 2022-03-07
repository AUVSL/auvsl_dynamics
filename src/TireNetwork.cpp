#include "TireNetwork.h"
#include <cmath>
#include <cppad/cppad.hpp>

Eigen::Matrix<Scalar,TireNetwork::num_hidden_nodes,TireNetwork::num_in_features> TireNetwork::weight0;
Eigen::Matrix<Scalar,TireNetwork::num_hidden_nodes,TireNetwork::num_hidden_nodes> TireNetwork::weight2;
Eigen::Matrix<Scalar,TireNetwork::num_out_features,TireNetwork::num_hidden_nodes> TireNetwork::weight4;
Eigen::Matrix<Scalar,TireNetwork::num_hidden_nodes,1> TireNetwork::bias0;
Eigen::Matrix<Scalar,TireNetwork::num_hidden_nodes,1> TireNetwork::bias2;
Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> TireNetwork::bias4;
Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> TireNetwork::out_mean;
Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> TireNetwork::out_std;
Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> TireNetwork::in_mean;
Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> TireNetwork::in_std_inv;


int TireNetwork::is_loaded = 0;


TireNetwork::TireNetwork(){
  if(!is_loaded){
    load_model();
  }
}
TireNetwork::~TireNetwork(){}

//type overloading problem when taking address of std::tanh
//there is probably a smarter way to do this with templates.
inline double tanh_double_wrapper(double x){
  return std::tanh(x);
}

inline Scalar tanh_scalar_wrapper(Scalar x){
  return CppAD::tanh(x);
}

void TireNetwork::forward(const Eigen::Matrix<Scalar,num_in_features,1> &in_vec, Eigen::Matrix<Scalar,num_out_features,1> &out_vec){
  Eigen::Matrix<Scalar,TireNetwork::num_hidden_nodes,1> layer0_out;
  Eigen::Matrix<Scalar,TireNetwork::num_hidden_nodes,1> layer2_out;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> layer4_out;
  Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> scaled_features;
  
  scaled_features = (in_vec - in_mean).cwiseProduct(in_std_inv);
  layer0_out = (weight0*scaled_features) + bias0;
  layer0_out = layer0_out.unaryExpr(&tanh_scalar_wrapper);
  layer2_out = (weight2*layer0_out) + bias2;
  layer2_out = layer2_out.unaryExpr(&tanh_scalar_wrapper);
  layer4_out = (weight4*layer2_out) + bias4;        
  out_vec = layer4_out.cwiseProduct(out_std) + out_mean;
}





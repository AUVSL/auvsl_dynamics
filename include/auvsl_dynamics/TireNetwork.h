#pragma once
#include <fstream>
#include <string.h>
#include <ros/ros.h>
#include <Eigen/Dense>

#include "forward_dynamics.h"

using Jackal::rcg::Scalar;

//simple feedforward network with hard coded num of layers
//THis could have been a single function honestly
class TireNetwork {
public:
  TireNetwork();
  ~TireNetwork();

  static const int num_hidden_nodes = 32;
  static const int num_in_features = 8;
  static const int num_out_features = 4;
  
  static void forward(const Eigen::Matrix<Scalar,num_in_features,1> &in_vec, Eigen::Matrix<Scalar,num_out_features,1> &out_vec);
  
  static int is_loaded;
  static void load_model();
  
  static Eigen::Matrix<Scalar,num_hidden_nodes,num_in_features> weight0;
  static Eigen::Matrix<Scalar,num_hidden_nodes,1> bias0;
  static Eigen::Matrix<Scalar,num_hidden_nodes,num_hidden_nodes> weight2;
  static Eigen::Matrix<Scalar,num_hidden_nodes,1> bias2;
  static Eigen::Matrix<Scalar,num_out_features,num_hidden_nodes> weight4;
  static Eigen::Matrix<Scalar,num_out_features,1> bias4;

private:
  static Eigen::Matrix<Scalar,num_out_features,1> out_mean;
  static Eigen::Matrix<Scalar,num_out_features,1> out_std;
  static Eigen::Matrix<Scalar,num_in_features,1> in_mean;
  static Eigen::Matrix<Scalar,num_in_features,1> in_std_inv; //inverse of in_std. Because multiply is faster than divide.

};

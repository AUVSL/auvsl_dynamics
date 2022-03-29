#include <fstream>
#include <eigen3/Eigen/Dense>

#include "Model.h"

using Jackal::rcg::Scalar;

class NNJackalModel{
 public:
  static const int num_in_features = 16;
  static const int num_hidden_nodes = 10;
  static const int num_out_features = 3;
    
  static void load_nn_model();
  static Eigen::Matrix<Scalar,num_out_features,1> forward(Eigen::Matrix<Scalar,num_in_features,1> feature_vec);
  
  static Eigen::Matrix<Scalar,num_hidden_nodes,num_in_features> weight0;
  static Eigen::Matrix<Scalar,num_hidden_nodes,1> bias0;
  static Eigen::Matrix<Scalar,num_hidden_nodes,num_hidden_nodes> weight2;
  static Eigen::Matrix<Scalar,num_hidden_nodes,1> bias2;
  static Eigen::Matrix<Scalar,num_out_features,num_hidden_nodes> weight4;
  static Eigen::Matrix<Scalar,num_out_features,1> bias4;
  static Eigen::Matrix<Scalar,num_out_features,1> out_mean;
  static Eigen::Matrix<Scalar,num_out_features,1> out_std;
  static Eigen::Matrix<Scalar,num_in_features,1> in_mean;
  static Eigen::Matrix<Scalar,num_in_features,1> in_std;
 
};


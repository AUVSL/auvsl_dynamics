#include "forward_dynamics.h"
#include "model_constants.h"
#include "TireNetwork.h"


Eigen::Matrix<float, TireNetwork::num_out_features, 1> tire_model_bekker(const Eigen::Matrix<float, TireNetwork::num_in_features, 1> &features);

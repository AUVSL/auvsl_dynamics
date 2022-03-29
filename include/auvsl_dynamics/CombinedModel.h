#include <string>

#include "forward_dynamics.h"
#include "model_constants.h"
#include "NNJackalModel.h"
#include "HybridDynamics.h"

#pragma once


using Jackal::rcg::Scalar;
using Jackal::rcg::Velocity;
using Jackal::rcg::Acceleration;
using Jackal::rcg::Force;
using Jackal::rcg::JointState;
using Jackal::rcg::LinkDataMap;
using Jackal::rcg::orderedLinkIDs;

//Abstract Base Class.
class CombinedModel : public Model {
public:
  const static unsigned STATE_DIM = HybridDynamics::STATE_DIM;
  const static unsigned CNTRL_DIM = HybridDynamics::CNTRL_DIM;
  const static unsigned STATE_DIM_2D = 3;
  
  CombinedModel();
  ~CombinedModel();
  
  void initState();
  void initState(Scalar *start_state);
  void initStateCOM(Scalar *start_state);
  void step(Scalar vl, Scalar vr);
  void settle();
  void getState(Scalar *state);
  
  unsigned getStateDim() const;
  unsigned getControlDim() const;
  
private:
  Eigen::Matrix<float,NNJackalModel::num_in_features,1> feature_vec;
  
  float mix_; //1-0. 0 is 3d. 1 is 2d. .5 is equal weight of both.
  
  NNJackalModel nn_model_2d;
  HybridDynamics model_3d;
};

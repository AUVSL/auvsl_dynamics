#include "CombinedModel.h"
#include "utils.h"

CombinedModel::CombinedModel(){
  nn_model_2d.load_nn_model();
  //ros::param::get("/mix", mix_);
  //ROS_INFO("Mix value is %f", mix_);
  mix_ = 0;
}

CombinedModel::~CombinedModel(){
  
}

void CombinedModel::initState(){
  model_3d.initState();
  for(int i = 0; i < 16; i++){
    feature_vec[i] = 0;
  }
}

void CombinedModel::initState(Scalar *start_state){
  model_3d.initState(start_state);
  for(int i = 0; i < 16; i++){
    feature_vec[i] = 0;
  }
}

void CombinedModel::initStateCOM(Scalar *start_state){
  model_3d.initStateCOM(start_state);
  for(int i = 0; i < 16; i++){
    feature_vec[i] = 0;
  }
}



void CombinedModel::settle(){
  model_3d.settle();
}

void CombinedModel::getState(Scalar *state){
  model_3d.getState(state);
}
  
unsigned CombinedModel::getStateDim() const{
  return STATE_DIM;
}

unsigned CombinedModel::getControlDim() const{
  return CNTRL_DIM;
} 

//added current features to feature_vec like a queue
void CombinedModel::step(Scalar vl, Scalar vr){
  Eigen::Matrix<float,NNJackalModel::num_out_features,1> nn_prediction_vec;
  
  for(int i = 7; i > 0; i--){
    int idx = i*2;
    int prev_idx = (i-1)*2;
    feature_vec[idx] = feature_vec[prev_idx];
    feature_vec[idx+1] = feature_vec[prev_idx+1];
  }
  
  feature_vec[0] = vl;
  feature_vec[1] = vr;

  nn_prediction_vec = nn_model_2d.forward(feature_vec);
  
  if(vr == 0){
    vr = 1e-5d;
  }
  if(vl == 0){
    vl = 1e-5d;
  }
  Eigen::Matrix<Scalar,CombinedModel::CNTRL_DIM,1> u;
  u[0] = vl;
  u[1] = vr;
  
  Eigen::Matrix<Scalar,CombinedModel::STATE_DIM,1> next_state;
  Eigen::Matrix<Scalar,3,3> rot;
  
  int substeps = 50;
  for(int i = 0; i < substeps; i++){
    model_3d.state_[17] = model_3d.state_[19] = u[0];
    model_3d.state_[18] = model_3d.state_[20] = u[1];
    
    model_3d.RK4(model_3d.state_, next_state, u);
    
    rot = toMatrixRotation(next_state[0],next_state[1],next_state[2],next_state[3]);
    Eigen::Matrix<Scalar,3,1> ang_vel(next_state[11], next_state[12], next_state[13]);
    Eigen::Matrix<Scalar,3,1> lin_vel(next_state[14], next_state[15], next_state[16]);
    
    lin_vel = rot.transpose()*lin_vel; //convert from world to base
    
    //mix in the body frame.
    lin_vel[0] = mix_*lin_vel[0] + (1-mix_)*nn_prediction_vec[0];
    lin_vel[1] = mix_*lin_vel[1] + (1-mix_)*nn_prediction_vec[1];
    ang_vel[2] = mix_*ang_vel[2] + (1-mix_)*nn_prediction_vec[2];
    
    lin_vel = rot*lin_vel; //convert from base to world
    
    next_state[13] = ang_vel[2];
    
    next_state[14] = lin_vel[0];
    next_state[15] = lin_vel[1];
    //next_state[16] = lin_vel[2];
    
    model_3d.state_ = next_state;
       
  }
}







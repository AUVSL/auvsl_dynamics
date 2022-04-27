#include <iostream>
#include <fstream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include "VerifyModel.h"
#include <ros/ros.h>
#include <rbdl/rbdl.h>

float check_rollover(float qx, float qy, float qz, float qw){
  RigidBodyDynamics::Math::Quaternion quat(qx, qy, qz, qw);
  RigidBodyDynamics::Math::Vector3d vec = quat.rotate(RigidBodyDynamics::Math::Vector3d(0,0,1));
  
  float norm = sqrtf(vec[0]*vec[0] + vec[1]*vec[1] + vec[2]*vec[2]);
  
  return fabs(acosf(vec[2]/norm));
  
}

void test_safety(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  g_hybrid_model.settle();
  
  Eigen::Matrix<Scalar,HybridDynamics::STATE_DIM,1> state = g_hybrid_model.state_;
  
  std::ofstream log("/home/justin/safety.csv");
  log << "vl,vr,angle\n";
  
  for(int vl = 2; vl < 12; vl++){
    for(int vr = 2; vr < 12; vr++){
      ROS_INFO("vl %d   vr %d", vl, vr);
      g_hybrid_model.state_ = state;
      
      float temp_angle = 0;
      float max_angle = 0;
      for(int i = 0; i < 200; i++){
        g_hybrid_model.step((float)vl, (float)vr);
        
        temp_angle = check_rollover(g_hybrid_model.state_[0], g_hybrid_model.state_[1], g_hybrid_model.state_[2], g_hybrid_model.state_[3]);
        if(temp_angle > max_angle){
          max_angle = temp_angle;
        }
      }
      
      log << vl << ',' << vr << ',' << max_angle << '\n';
    }
  }

  log.close();
}

void test_slopes(){
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  g_hybrid_model.settle();
  
  for(int i = 0; i < 2000; i++){
    g_hybrid_model.step(6,8);
  }
}

int main(int argc, char **argv){
  //forgot to comment this line out when I ran with the bekker model.
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  //test_safety();
  //test_slopes();
  
  init_tests();
  test_CV3_paths();
  //test_LD3_path();
  del_tests();
  
  return 0;
}

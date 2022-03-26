#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include "VerifyModel.h"
#include <ros/ros.h>

void test_slopes(){
  HybridDynamics::start_log();
  HybridDynamics g_hybrid_model;
  g_hybrid_model.initState();
  
  for(int i = 0; i < 2000; i++){
    g_hybrid_model.step(10,6);
  }
  HybridDynamics::stop_log();
}


int main(int argc, char **argv){
  //forgot to comment this line out when I ran with the bekker model.
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;
  
  test_slopes();
  
  //init_tests();
  //test_CV3_paths();
  //test_LD3_path();
  //del_tests();
  
  return 0;
}

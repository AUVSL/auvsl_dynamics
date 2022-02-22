#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include "VerifyModel.h"
#include <ros/ros.h>


int main(int argc, char **argv){
  //forgot to comment this line out when I ran with the bekker model.
  feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;

  init_tests();
  test_CV3_paths();
  //test_LD3_path();
  del_tests();
  
  return 0;
}

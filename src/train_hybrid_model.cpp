#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
//#include <iosfwd>
//#include <cppad/cg.hpp>

#include "VerifyModel.h"

#include <tf/tf.h>
#include <ros/ros.h>





int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "train_hybrid_model");
  ros::NodeHandle nh;
  
  double lr;
  int load_from_file;
  int test_first;
  ros::param::get("/learn_rate", lr);
  ros::param::get("/load_from_file", load_from_file);
  ros::param::get("/test_first", test_first);
  
  init_tests();
  //test_network_save_load();
  
  if(load_from_file){
    loadHybridNetwork();
  }
  if(test_first){
    test_CV3_paths();
    test_LD3_path();
  }
  train_model_on_dataset((float)lr);
  del_tests();
}


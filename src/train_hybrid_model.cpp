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

  init_tests();
  //test_network_save_load();
  //test_CV3_paths();
  //loadHybridNetwork();
  train_model_on_dataset(1e-4f);
  test_CV3_paths();
  del_tests();
}


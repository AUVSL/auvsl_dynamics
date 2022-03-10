x#include <iostream>
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

#include "HybridDynamics.h"
#include "VerifyModel.h"
#include "AdjointMethod.h"

#include <tf/tf.h>
#include <ros/ros.h>


int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "train_hybrid_model");
  ros::NodeHandle nh;
  
  init_tests();
  //test_CV3_paths();
  train_model_on_dataset(.001f);
  del_tests();
}




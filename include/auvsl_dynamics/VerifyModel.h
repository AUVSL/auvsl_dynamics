#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <string>

#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>

#include "HybridDynamics.h"

#include <tf/tf.h>
#include <ros/ros.h>


#pragma once


//this evaluates the model over the datasets provided.
//Very simple.

//void load_files(const char *odom_fn, const char *imu_fn, const char *gt_fn);
//void simulatePeriod(double start_time, Scalar *X_start, Scalar *X_end);
//void getDisplacement(unsigned start_i, unsigned end_i, Scalar &lin_displacement, Scalar &ang_displacement);
//void simulateFile(Scalar &lin_err_sum_ret, Scalar &ang_err_sum_ret, unsigned &count_ret);
void test_CV3_paths();
void test_LD3_path();
void train_model_on_dataset(float lr);
void fileTrain(Eigen::Matrix<float,Eigen::Dynamic,1> &float_model_params);
void init_tests();
void del_tests();

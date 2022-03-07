#include <iostream>
#include <thread>
#include <vector>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <fenv.h>

#include "HybridDynamics.h"


//this was a mistake homie.


HybridDynamics *g_hybrid_model;

typedef struct{
  double vl;
  double vr;
  double ts;
} ODOM_LINE;

typedef struct{
  double ts;
  float x;
  float y;
  float vx; //will be approximated by derivative of x
  float vy;
  float yaw;
} GT_LINE;

typedef struct{
  double ts;
  double wx;
  double wy;
  double wz;
} IMU_LINE;


std::vector<ODOM_LINE> odom_vec;
std::vector<GT_LINE> gt_vec;
std::vector<IMU_LINE> imu_vec;

int readOdomFile(std::ifstream &odom_file, ODOM_LINE &odom_line){
  char comma;
  int cmd_mode;    //ignore
  double dist_left;  //ignore
  double dist_right; //ignore
  
  odom_file >> cmd_mode >> comma;
  odom_file >> odom_line.vl >> comma; //velocity left
  odom_file >> dist_left >> comma;
v  odom_file >> odom_line.vr >> comma; //velocity right
  odom_file >> dist_right >> comma;
  odom_file >> odom_line.ts; //time (s)
  return odom_file.peek() != EOF;
}

int readGTFile(std::ifstream &gt_file, GT_LINE &gt_line){
  char comma;
  gt_file >> gt_line.ts >> comma;
  gt_file >> gt_line.x >> comma;
  gt_file >> gt_line.y >> comma;
  gt_file >> gt_line.yaw;
  return gt_file.peek() != EOF;
}

int readIMUFile(std::ifstream &imu_file, IMU_LINE &imu_line){
  char comma;
  double ignore;
  imu_file >> ignore >> comma; //ax
  imu_file >> ignore >> comma; //ay
  imu_file >> ignore >> comma; //az
  
  imu_file >> imu_line.wx >> comma;
  imu_file >> imu_line.wy >> comma;
  imu_file >> imu_line.wz >> comma;

  imu_file >> ignore >> comma; //qx
  imu_file >> ignore >> comma; //qy
  imu_file >> ignore >> comma; //qz
  imu_file >> ignore >> comma; //qw

  imu_file >> imu_line.ts;
  
  return imu_file.peek() != EOF;
}

void load_files(const char *odom_fn, const char *imu_fn, const char *gt_fn){
  std::ifstream odom_file(odom_fn);
  std::ifstream gt_file(gt_fn);
  std::ifstream imu_file(imu_fn);
  
  ODOM_LINE odom_line;
  GT_LINE gt_line;
  IMU_LINE imu_line;

  odom_vec.clear();
  gt_vec.clear();
  imu_vec.clear();
  
  while(readOdomFile(odom_file, odom_line)){
    odom_vec.push_back(odom_line);
  }
  
  while(readGTFile(gt_file, gt_line)){
    gt_vec.push_back(gt_line);
  }
  
  while(readIMUFile(imu_file, imu_line)){
    imu_vec.push_back(imu_line);
  }
  
  
  float *vx_list = new float[gt_vec.size()];
  float *vy_list = new float[gt_vec.size()];
  
  float dt;
  for(int i = 0; i < gt_vec.size()-1; i++){
    dt = gt_vec[i+1].ts - gt_vec[i].ts;
    vx_list[i] = (gt_vec[i+1].x - gt_vec[i].x)/dt;
    vy_list[i] = (gt_vec[i+1].y - gt_vec[i].y)/dt;
  }
  
  vx_list[gt_vec.size()-1] = vx_list[gt_vec.size()-2];
  vy_list[gt_vec.size()-1] = vy_list[gt_vec.size()-2];
  
  
  //moving average
  float temp_x;
  float temp_y;
  int cnt;
  for(int i = 0; i < gt_vec.size()-1; i++){
    temp_x = 0;
    temp_y = 0;
    cnt = 0;
    for(int j = std::max(0, i-2); j <= std::min((unsigned)gt_vec.size()-1, (unsigned) i+2); j++){ 
      temp_x += vx_list[j];
      temp_y += vy_list[j];
      cnt++;
    }
    gt_vec[i].vx = temp_x / (float)cnt;
    gt_vec[i].vy = temp_y / (float)cnt;
  }
  
  
  ROS_INFO("gt vec size %lu", gt_vec.size());
  ROS_INFO("odom vec size %lu", odom_vec.size());
  ROS_INFO("imu vec size %lu", odom_vec.size());
  

  odom_file.close();
  gt_file.close();
  imu_file.close();
  
  delete[] vx_list;
  delete[] vy_list;
}



//simulate 6 second intervals just like the paper.
void simulatePeriod(double start_time, float *X_start, float *X_end){
  //find corresponding index in odom_vec. By finding timestamp with
  //minimum difference.
  unsigned start_idx = 0;
  double time_min = 100;
  double diff;
  for(unsigned i = 0; i < odom_vec.size(); i++){
    diff = fabs(odom_vec[i].ts - start_time);
    if(diff < time_min){
      time_min = diff;
      start_idx = i;
    }
  }
  
  g_hybrid_model->init_state(X_start);  
  //g_hybrid_model->settle();
  
  float vl, vr;
  for(unsigned idx = start_idx; (odom_vec[idx].ts - start_time) < 6; idx++){
    vl = odom_vec[idx].vl;
    vr = odom_vec[idx].vr;
    g_hybrid_model->step(vl, vr);
  }
  
  for(int i = 0; i < 21; i++){
    X_end[i] = g_hybrid_model->vehicle_state[i];
  }
  
}


void getDisplacement(unsigned start_i, unsigned end_i, float &lin_displacement, float &ang_displacement){
  lin_displacement = 0;
  ang_displacement = 0;
  
  float dx;
  float dy;
  float dyaw;
  
  for(unsigned i = start_i; (i < end_i) && (i < (gt_vec.size()-1)); i++){
    dx = gt_vec[i].x - gt_vec[i+1].x;
    dy = gt_vec[i].y - gt_vec[i+1].y;
    dyaw = gt_vec[i].yaw - gt_vec[i+1].yaw;
    dyaw = fabs(dyaw);
    
    dyaw = dyaw > M_PI ?  dyaw - (2*M_PI) : dyaw;
    
    lin_displacement += sqrtf((dx*dx) + (dy*dy));
    ang_displacement += fabs(dyaw);
  }
  
  
}


//simulate data and return results.
void simulateFile(float &lin_err_sum_ret, float &ang_err_sum_ret, unsigned &count_ret){
  float Xn[21];
  float Xn1[21];
  for(int i = 0; i < 21; i++){
    Xn[i] = 0;
  }
  
  tf::Quaternion initial_quat;
  tf::Quaternion temp_quat;
  
  float x_err, y_err, yaw_err, lin_displacement, ang_displacement;
  
  double dt;
  double trans_sum = 0;
  double ang_sum = 0;
  double ang_err;
  
  int count = 0;
  
  //so it doesnt skip the first one.
  double time = 0;
  for(unsigned i = 0; i < gt_vec.size(); i++){
    if((gt_vec[i].ts - time) < 6.0f){ //test over 6 second intervals.
      continue;
    }
    
    time = gt_vec[i].ts;
    
    if((gt_vec[gt_vec.size()-1].ts - time) < 6.0f){
      break; //If we reached the end of the gt_vec dataset, we're done.
    }
    
    if((odom_vec[odom_vec.size()-1].ts - time) < 6.0f){
      break; //If we reached the end of the odom_vec dataset, we're done.
    }
    
    
    Xn[0] = gt_vec[i].x;
    Xn[1] = gt_vec[i].y;
    Xn[2] = .16;
    
    initial_quat.setRPY(0,0,gt_vec[i].yaw);
    initial_quat = initial_quat.normalize();
      
    Xn[3] = initial_quat.getX();
    Xn[4] = initial_quat.getY();
    Xn[5] = initial_quat.getZ();
    Xn[10] = initial_quat.getW();
    
    
    if((imu_vec[imu_vec.size()-1].ts - time) < 6.0f){
      break; //If we reached the end of the imu_vec dataset, we can quit.
    }
    
    //find initial angular vel that matches with the start time "time"
    //get rotational velocity from IMU.
    double diff;
    unsigned imu_idx = 0;
    double time_min = 100;
    for(unsigned j = 0; j < imu_vec.size(); j++){
      diff = fabs(imu_vec[j].ts - time);
      if(diff < time_min){
        time_min = diff;
        imu_idx = j;
      }
    }
      
    Xn[14] = imu_vec[imu_idx].wx;
    Xn[15] = imu_vec[imu_idx].wy;
    Xn[16] = imu_vec[imu_idx].wz;
      
    Xn[11] = gt_vec[i].vx;
    Xn[12] = gt_vec[i].vy;
    Xn[13] = 0;

    ROS_INFO("Simulating over a 6 second horizon %d", i);
    simulatePeriod(time, Xn, Xn1);
    
    //Done simulating, now evaluate error.
    unsigned j;
    for(j = i; (gt_vec[j].ts - time) < 6.0f && j < gt_vec.size(); j++){}
        
    getDisplacement(i, j, lin_displacement, ang_displacement);
      
    x_err = Xn1[0] - gt_vec[j].x;
    y_err = Xn1[1] - gt_vec[j].y;
    
    temp_quat = tf::Quaternion(Xn1[3],Xn1[4],Xn1[5],Xn1[10]);
    tf::Matrix3x3 m(temp_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); //we just need yaw.

    yaw_err = fabs(yaw - gt_vec[j].yaw);
    yaw_err = yaw_err > M_PI ?  yaw_err - (2*M_PI) : yaw_err;
    yaw_err = fabs(yaw_err);
    
    //ROS_INFO("Actual Values %f   %f   %f", Xn1[0], Xn1[1], yaw);
    //ROS_INFO("Error lin %f    yaw %f", sqrtf(x_err*x_err + y_err*y_err), yaw_err);
    //ROS_INFO("Displacement lin %f    yaw %f", lin_displacement, ang_displacement);
    
    count++;

    float rel_lin_err = sqrtf(x_err*x_err + y_err*y_err) / lin_displacement;
    float rel_ang_err = yaw_err / ang_displacement;
      
    ROS_INFO("Relative lin err %f       ang err %f", rel_lin_err, rel_ang_err);
    ROS_INFO("\n\n\n");
    
    trans_sum += rel_lin_err;
    ang_sum += rel_ang_err;
    
    //break;
  }
  
  ROS_INFO(" ");
  ROS_INFO(" ");
  ROS_INFO(" ");
  //ROS_INFO("MARE Translation Error %f   MARE Heading Error %f", trans_sum/count, ang_sum/count);
  
  lin_err_sum_ret = trans_sum;
  ang_err_sum_ret = ang_sum;
  count_ret = count;
}



void test_CV3_paths(){
  float total_lin_err = 0;
  float total_ang_err = 0;
  
  float rel_lin_err;
  float rel_ang_err;
  
  char odom_fn[100];
  char imu_fn[100];
  char gt_fn[100];

  unsigned count;
  unsigned sum_count = 0;
  
  
  std::ofstream log_csv;
  log_csv.open("/home/justin/code/AUVSL_ROS/src/auvsl_planner/scripts/hybrid_err_cv3.csv", std::ofstream::out);
  log_csv << "lin_err,ang_err\n";
  
  //Remember this needs to start at 1.
  for(int jj = 1; jj <= 144; jj++){
    memset(odom_fn, 0, 100);
    sprintf(odom_fn, "/home/justin/Downloads/CV3/extracted_data/odometry/%04d_odom_data.txt", jj);
    ROS_INFO("Reading Odom File %s", odom_fn);
    
    memset(imu_fn, 0, 100);
    sprintf(imu_fn, "/home/justin/Downloads/CV3/extracted_data/imu/%04d_imu_data.txt", jj);
    ROS_INFO("Reading imu File %s", imu_fn);
    
    memset(gt_fn, 0, 100);
    sprintf(gt_fn, "/home/justin/Downloads/CV3/localization_ground_truth/%04d_CV_grass_GT.txt", jj);
    ROS_INFO("Reading gt File %s", gt_fn);
    
    load_files(odom_fn, imu_fn, gt_fn);
    simulateFile(rel_lin_err, rel_ang_err, count);
    
    log_csv << rel_lin_err/count << ',' << rel_ang_err/count << '\n';
    
    total_lin_err += rel_lin_err;
    total_ang_err += rel_ang_err;
    
    sum_count += count;
  }
  
  log_csv.close();
  
  ROS_INFO("CV3 MARE lin: %f     ang: %f", total_lin_err/(float)sum_count, total_ang_err/(float)sum_count);
  
}





int main(int argc, char **argv){
  //feenableexcept(FE_INVALID | FE_OVERFLOW);
  ros::init(argc, argv, "auvsl_global_planner");
  ros::NodeHandle nh;  

  HybridDynamics::init();
  HybridDynamics::start_log();
  
  g_hybrid_model = new HybridDynamics();
  
  g_hybrid_model->init_state(); //set start pos to 0,0,.16 and orientation to 0,0,0,1
  g_hybrid_model->settle();     //allow the 3d vehicle to come to rest and reach steady state, equillibrium sinkage for tires.
  
  ROS_INFO("CV3 Starting Timer");
  ros::Time start_time = ros::Time::now();
  test_CV3_paths();
  ros::Duration total_run_time = ros::Time::now() - start_time;
  ROS_INFO("CV3 Total Runtime %d", total_run_time.sec);
  
  
  load_files(
"/home/justin/Downloads/LD3/extracted_data/odometry/0001_odom_data.txt",
"/home/justin/Downloads/LD3/extracted_data/imu/0001_imu_data.txt",
"/home/justin/Downloads/LD3/localization_ground_truth/0001_LD_grass_GT.txt"
);
  float rel_lin_err;
  float rel_ang_err;
  unsigned count;
  
  ROS_INFO("LD3 Starting Timer");
  start_time = ros::Time::now();
  simulateFile(rel_lin_err, rel_ang_err, count);
  ROS_INFO("LD3 MARE lin: %f     ang: %f", rel_lin_err/(float)count, rel_ang_err/(float)count);  
  total_run_time = ros::Time::now() - start_time;
  ROS_INFO("LD3 Total Runtime %d", total_run_time.sec);
  
  HybridDynamics::stop_log();
  
  return 0;
}

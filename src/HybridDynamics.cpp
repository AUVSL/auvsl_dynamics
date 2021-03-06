#include "HybridDynamics.h"
#include "BekkerModel.h"
#include "declarations.h"
#include "miscellaneous.h"
#include "utils.h"
#include <iit/rbd/robcogen_commons.h>



using Jackal::rcg::tire_radius;
using Jackal::rcg::tx_front_left_wheel;
using Jackal::rcg::ty_front_left_wheel;
using Jackal::rcg::tz_front_left_wheel;

using Jackal::rcg::tx_front_right_wheel;
using Jackal::rcg::ty_front_right_wheel;
using Jackal::rcg::tz_front_right_wheel;

using Jackal::rcg::tx_rear_left_wheel;
using Jackal::rcg::ty_rear_left_wheel;
using Jackal::rcg::tz_rear_left_wheel;

using Jackal::rcg::tx_rear_right_wheel;
using Jackal::rcg::ty_rear_right_wheel;
using Jackal::rcg::tz_rear_right_wheel;

using Jackal::rcg::orderedJointIDs;


const Acceleration HybridDynamics::GRAVITY_VEC = (Acceleration() << 0,0,0,0,0,-9.81).finished();
//const Acceleration HybridDynamics::GRAVITY_VEC = (Acceleration() << 0,0,0,0,0,0).finished();


/*
std::ofstream HybridDynamics::log_file;
std::ofstream HybridDynamics::debug_file;
*/
Scalar HybridDynamics::timestep = .001;
int HybridDynamics::altitude_map_not_set_ = 1;
Scalar (*HybridDynamics::get_altitude_)(Scalar x, Scalar y, Scalar z_guess);


Scalar get_altitude_simple(Scalar x, Scalar y, Scalar z_guess){
  return 0; //std::max(-.5f*std::cos(2*x), 0.0f); //x*.2;
}




HybridDynamics::HybridDynamics() : Model(){
  fwd_dynamics = new Jackal::rcg::ForwardDynamics(inertias, m_transforms);
  
  // bekker_params[0] = 29.76;
  // bekker_params[1] = 2083;
  // bekker_params[2] = .8;
  // bekker_params[3] = 0;
  // bekker_params[4] = 22.5*M_PI/180.0;

  // bekker_params[0] = 29.760084;
  // bekker_params[1] = 2083.000000;
  // bekker_params[2] = 0.917826;
  // bekker_params[3] = 0.152998;
  // bekker_params[4] = 0.471988;
  
  
  bekker_params[0] = 29.758547;
  bekker_params[1] = 2083.000000;
  bekker_params[2] = 1.197933;
  bekker_params[3] = 0.102483;
  bekker_params[4] = 0.652405;
  
  // bekker_params[0] = 29.758434;
  // bekker_params[1] = 2083;
  // bekker_params[2] = 1.216381;
  // bekker_params[3] = 0.113669;
  // bekker_params[4] = 0.651287;

  if(altitude_map_not_set_){
    setAltitudeMap(get_altitude_simple);
  }
  
  JointState q(0,0,0,0);   //Joint position
  f_transforms.fr_front_left_wheel_link_X_fr_base_link.update(q);
  f_transforms.fr_base_link_X_fr_front_left_wheel_link_COM.update(q);

  std::string log_filename;
  std::string debug_filename;
  ros::param::get("/xout_log_file_name", log_filename);
  ros::param::get("/debug_log_file_name", debug_filename);
  //startLog(log_filename, debug_filename);
}

HybridDynamics::~HybridDynamics(){
  stopLog();
  delete fwd_dynamics;
}

void HybridDynamics::setAltitudeMap(Scalar (*alt_func)(Scalar x, Scalar y, Scalar z_guess)){
  get_altitude_ = alt_func;
  altitude_map_not_set_ = 0;
}


unsigned HybridDynamics::getStateDim() const{
  return STATE_DIM;
}

unsigned HybridDynamics::getControlDim() const{
  return CNTRL_DIM;
}

void HybridDynamics::getState(Scalar *state){
  for(int i = 0; i < STATE_DIM; i++){
    state[i] = state_[i];
  }
}

void HybridDynamics::initState(){
  Scalar start_state[STATE_DIM] = {0,0,0,1, 0,0,.16, 0,0,0,0, 0,0,0, 0,0,0, 0,0,0,0};
  initState(start_state);
}

void HybridDynamics::initState(Scalar *start_state){
  for(unsigned i = 0; i < STATE_DIM; i++){
    state_[i] = start_state[i];
  }
}

//velocities expressed in COM frame. more intuitive.
void HybridDynamics::initStateCOM(Scalar *start_state){
  Jackal::rcg::Vector3 com_pos = Jackal::rcg::getWholeBodyCOM(inertias, h_transforms);
  
  Eigen::Matrix<Scalar,3,1> base_lin_vel;
  Eigen::Matrix<Scalar,3,1> com_lin_vel(start_state[14], start_state[15], start_state[16]);
  Eigen::Matrix<Scalar,3,1> ang_vel(start_state[11], start_state[12], start_state[13]);

  Eigen::Matrix<Scalar,3,3> r_ss;
  r_ss << 0.,        -com_pos[2], com_pos[1],
          com_pos[2], 0.,        -com_pos[0],
         -com_pos[1], com_pos[0], 0.;
  
  //plus, because com_pos is the displacement from base_link to com
  base_lin_vel = com_lin_vel + (r_ss*ang_vel);
  
  Scalar base_state[STATE_DIM];
  for(int i = 0; i < STATE_DIM; i++){
    base_state[i] = start_state[i];
  }

  base_state[14] = base_lin_vel[0];
  base_state[15] = base_lin_vel[1];
  base_state[16] = base_lin_vel[2];

  initState(base_state);
}

void HybridDynamics::settle(){
  //reach equillibrium sinkage.
  for(int i = 0; i < 200; i++){
    step(0,0);
  }
}

//step is .05s
//THis matches up with the test dataset sample rate.
void HybridDynamics::step(Scalar vl, Scalar vr){  
  Eigen::Matrix<Scalar,STATE_DIM,1> Xt1;
  Eigen::Matrix<Scalar,CNTRL_DIM,1> u;

  if(vr == 0){
    vr = 1e-5d;
  }
  if(vl == 0){
    vl = 1e-5d;
  }
  
  u(0) = vl;
  u(1) = vr;
  
  const int num_steps = 50; //50*.001 = .05
  for(int ii = 0; ii < num_steps; ii++){
    state_[17] = state_[19] = u[0];
    state_[18] = state_[20] = u[1];
    
    RK4(state_, Xt1, u);
    //Euler(state_, Xt1, u);
    state_ = Xt1;
    //logVehicleState();
  }
}


//cpt rots express a vector in the tire cpt frame into the world frame
//cpt_pts is the displacement from the origin of the world frame to the center of the tire joints
//sinkages is the depth the tire cpt is below the soil
void HybridDynamics::get_tire_cpts_sinkages(const Eigen::Matrix<Scalar,STATE_DIM,1> &X,
                                            Eigen::Matrix<Scalar,3,1> *cpt_pts,
                                            Eigen::Matrix<Scalar,3,3> *cpt_rots,
                                            Scalar *sinkages){
  
  const Eigen::Matrix<Scalar,3,4> tire_translations = (Eigen::Matrix<Scalar,3,4>() <<
                                                       tx_front_left_wheel,tx_front_right_wheel,tx_rear_left_wheel,tx_rear_right_wheel,
                                                       ty_front_left_wheel,ty_front_right_wheel,ty_rear_left_wheel,ty_rear_right_wheel,
                                                       tz_front_left_wheel,tz_front_right_wheel,tz_rear_left_wheel,tz_rear_right_wheel).finished();
  
  const Eigen::Matrix<Scalar,3,1> radius_vec = (Eigen::Matrix<Scalar,3,1>() << 0,0,-tire_radius).finished();
  
  //get the matrix that transforms from base to world frame
  Eigen::Matrix<Scalar,3,3> base_rot = toMatrixRotation(X[0],X[1],X[2],X[3]);
  
  Eigen::Matrix<Scalar,3,4> end_pos_matrix = base_rot*tire_translations;
  Eigen::Matrix<Scalar,3,1> end_pos_tire_joint;
  
  int max_checks = 10;
  Scalar test_angle;
  Scalar max_angle = -M_PI*.00025;
  Scalar test_sinkage;
  Scalar max_sinkage;
  
  Eigen::Matrix<Scalar,3,1> test_pos;
  Eigen::Matrix<Scalar,3,3> test_rot;
  Eigen::Matrix<Scalar,3,3> best_rot;
  
  for(int ii = 0; ii < 4; ii++){
    end_pos_tire_joint[0] = end_pos_matrix(0,ii) + X[4]; //real world position of tire joint.
    end_pos_tire_joint[1] = end_pos_matrix(1,ii) + X[5];
    end_pos_tire_joint[2] = end_pos_matrix(2,ii) + X[6];
    
    cpt_pts[ii] = end_pos_tire_joint; //center of the tire joint. 
    
    //need to find the cpt point between tire and soil to determine the
    //orientation of the reaction forces frame.
    
    max_sinkage = 0;
    best_rot = Eigen::Matrix<Scalar,3,3>::Identity();
    for(int jj = 0; jj < max_checks; jj++){
      test_angle = max_angle - (2*max_angle*jj/((Scalar) max_checks - 1)); //test_angle ranges from +-max_angle
      roty(test_rot, test_angle);
      test_rot = base_rot*test_rot;
      
      test_pos = end_pos_tire_joint + test_rot*radius_vec; //a point on the edge of the tire.
      test_sinkage = get_altitude_(test_pos[0],test_pos[1],0) - test_pos[2];
      
      if(test_sinkage > max_sinkage){
        best_rot = test_rot; //orientation of cpt
        max_sinkage = test_sinkage;
      }
    }
    
    sinkages[ii] = max_sinkage;
    cpt_rots[ii] = best_rot;
  }
}

void HybridDynamics::get_tire_cpt_vels(const Eigen::Matrix<Scalar,STATE_DIM,1> &X,
                                       const Eigen::Matrix<Scalar,3,3> *cpt_rots,
                                       Eigen::Matrix<Scalar,3,1> *cpt_vels){
  
  const Eigen::Matrix<Scalar,3,4> tire_translations = (Eigen::Matrix<Scalar,3,4>() <<
                                                       tx_front_left_wheel,tx_front_right_wheel,tx_rear_left_wheel,tx_rear_right_wheel,
                                                       ty_front_left_wheel,ty_front_right_wheel,ty_rear_left_wheel,ty_rear_right_wheel,
                                                       tz_front_left_wheel,tz_front_right_wheel,tz_rear_left_wheel,tz_rear_right_wheel).finished();
  
  const Eigen::Matrix<Scalar,3,1> radius_vec = (Eigen::Matrix<Scalar,3,1>() << 0,0,-tire_radius).finished();
  Eigen::Matrix<Scalar,3,3> base_rot = toMatrixRotation(X[0],X[1],X[2],X[3]);
  Eigen::Matrix<Scalar,3,3> temp_rot;
  Eigen::Matrix<Scalar,3,1> temp_vec;
  
  //vel of base_link expressed in base link frame.
  Eigen::Matrix<Scalar,3,1> lin_vel(X[14], X[15], X[16]);
  Eigen::Matrix<Scalar,3,1> ang_vel(X[11], X[12], X[13]);

  Eigen::Matrix<Scalar,3,1> end_pos;
  for(int ii = 0; ii < 4; ii++){
    temp_vec[0] = tire_translations(0,ii);
    temp_vec[1] = tire_translations(1,ii);
    temp_vec[2] = tire_translations(2,ii);
    
    //temp_rot transforms from cpt frame to base frame
    //cpt_rots[ii] = base_rot*temp_rot
    temp_rot = base_rot.transpose()*cpt_rots[ii]; 
    //temp_vec is the displacement from vehicle base frame -> tire joint
    //second part of this line is the displacement from tire joint -> contact point
    end_pos = temp_vec + (temp_rot*radius_vec);
    
    Eigen::Matrix<Scalar,3,3> r_ss;
    r_ss << 0., -end_pos(2), end_pos(1),
      end_pos(2), 0., -end_pos(0),
      -end_pos(1), end_pos(0), 0.;
    
    //this is the linear velocity at the cpt point, expressed in the base frame
    cpt_vels[ii] = lin_vel - (r_ss*ang_vel);
    
    //go from base frame to cpt frame
    cpt_vels[ii] = temp_rot.transpose()*cpt_vels[ii];
  }
  
  //Only interested in the linear velocities of the cpt points. Maybe we can include angular later.
}

//Going to assume flat terrain for now.
//Going to use a pretrained network.
//Will need to apply a rotation to the calculated force because the tire frame rotates with the tire :(
//So essentially the rotation needs to undo the tire frame rotation.
void HybridDynamics::get_tire_f_ext(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, LinkDataMap<Force> &ext_forces){
  Eigen::Matrix<Scalar,3,1> cpt_points[4];  //world frame position of cpt frames
  Eigen::Matrix<Scalar,3,3> cpt_rots[4];    //world orientation of cpt frames
  Scalar sinkages[4];
  get_tire_cpts_sinkages(X, cpt_points, cpt_rots, sinkages);
  
  //get the velocity of each tire contact point expressed in the contact point frame
  Eigen::Matrix<Scalar,3,1> cpt_vels[4];
  get_tire_cpt_vels(X, cpt_rots, cpt_vels);
  
  Eigen::Matrix<Scalar,3,3> base_rot = toMatrixRotation(X[0],X[1],X[2],X[3]);
  
  //We now have sinkage and velocity of each tire contact point
  //Next we need to compute tire-soil reaction forces
  //Then we will transform these forces into the body frame of each tire
  //Due to the stupid way that the tire joint frame transforms are defined we
  //will need a transform that undos the rotation of the tire.
  //A smarter solution would be to permanently set the tire joint angles to zero
  //because those values literally change nothing about the simulation.
  //We're doing it: Joint positions are set to zero.
  //Transform is needed to tire frame because joints are oriented to that z is the joint axis.
  
  Eigen::Matrix<Scalar,TireNetwork::num_in_features,1> features;
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> forces;
  
  features[3] = bekker_params[0];
  features[4] = bekker_params[1];
  features[5] = bekker_params[2];
  features[6] = bekker_params[3];
  features[7] = bekker_params[4];
  
  //logValue(sinkages);
  
  for(int ii = 0; ii < 4; ii++){
    Scalar vel_x_tan = tire_radius*X[17+ii]; //17 is the idx that tire velocities start at.
    Scalar slip_ratio;  //longitudinal slip
    Scalar slip_angle;  //
    
    if(vel_x_tan == 0){
      if(cpt_vels[ii][0] == 0){
        slip_ratio = 0;
      }
      else{
        slip_ratio = 1. - (cpt_vels[ii][0]/1.0e-3);
      }
    }
    else{
      if(cpt_vels[ii][0] == 0){
        slip_ratio = 1. - (1.0e-3/vel_x_tan);  
      }
      else{
        slip_ratio = 1. - (cpt_vels[ii][0]/vel_x_tan);  
      }
    }
    
    if(cpt_vels[ii][0] == 0){
      slip_angle = atan(cpt_vels[ii][1] / 1.0e-3);
    }
    else{
      slip_angle = atan(cpt_vels[ii][1] / fabs(cpt_vels[ii][0]));
    }
    
    if(sinkages[ii] <= 0)
      continue;
    
    features[0] = sinkages[ii];
    features[1] = slip_ratio;
    features[2] = slip_angle;
    
    TireNetwork::forward(features, forces);
    //forces = tire_model_bekker(features);
    
    if(X[17+ii] > 0){
      forces[0] = forces[0];
      forces[3] = forces[3];
    }
    else{
      forces[0] = -forces[0];
      forces[3] = -forces[3];      
    }
    
    if(cpt_vels[ii][1] > 0){
      forces[1] = -fabs(forces[1]);
    }
    else{
      forces[1] = fabs(forces[1]);
    }
    
    //forces[2] = std::min(forces[2], 0); //Fz should never point up
    
    Eigen::Matrix<Scalar,3,1> lin_force;
    Eigen::Matrix<Scalar,3,1> ang_force;
    
    lin_force[0] = forces[0];
    lin_force[1] = forces[1];
    lin_force[2] = forces[2];

    //Convert from tire_cpt_frame to tire_joint_frame
    //cpt_rots = base_rot*tire_cpt_rot
    //next line equals tire_cpt_rot*lin_force
    lin_force = base_rot.transpose()*cpt_rots[ii]*lin_force;
    
    ang_force[0] = 0;
    ang_force[1] = 0; //forces[3];
    ang_force[2] = 0;
        
    //Convert from cpt frame to world frame.
    Eigen::Matrix<Scalar,3,1> cpt_vel_world = cpt_rots[ii]*cpt_vels[ii];
    
    if(cpt_vel_world[2] > 0){
      lin_force[2] *= .1;
    }
    
    Force wrench;
    //normal force Fz maps to force in Y direction due to Robcogen's choice of coordinate frame for joints.
    wrench[0] = ang_force[0];
    wrench[1] = -ang_force[2];
    wrench[2] = ang_force[1];
    wrench[3] = lin_force[0];
    wrench[4] = -lin_force[2];
    wrench[5] = lin_force[1];
    
    ext_forces[orderedLinkIDs[ii+1]] = wrench;  
  }
}

void HybridDynamics::RK4(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
  Eigen::Matrix<Scalar,STATE_DIM,1> temp;
  Eigen::Matrix<Scalar,STATE_DIM,1> k1;
  Eigen::Matrix<Scalar,STATE_DIM,1> k2;
  Eigen::Matrix<Scalar,STATE_DIM,1> k3;
  Eigen::Matrix<Scalar,STATE_DIM,1> k4;
  Scalar temp_norm;
  Scalar ts = timestep;
  
  ODE(X, k1, u);
  for(unsigned i = 0; i < STATE_DIM; i++) temp[i] = X[i]+.5*ts*k1[i];
  temp_norm = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
  temp[0] /= temp_norm;
  temp[1] /= temp_norm;
  temp[2] /= temp_norm;
  temp[3] /= temp_norm;
  
  ODE(temp, k2, u);
  for(unsigned i = 0; i < STATE_DIM; i++) temp[i] = X[i]+.5*ts*k2[i];
  temp_norm = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
  temp[0] /= temp_norm;
  temp[1] /= temp_norm;
  temp[2] /= temp_norm;
  temp[3] /= temp_norm;
  
  ODE(temp, k3, u);
  for(unsigned i = 0; i < STATE_DIM; i++) temp[i] = X[i]+ts*k3[i];
  temp_norm = sqrtf(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2] + temp[3]*temp[3]);
  temp[0] /= temp_norm;
  temp[1] /= temp_norm;
  temp[2] /= temp_norm;
  temp[3] /= temp_norm;
  
  ODE(temp, k4, u);
  for(unsigned i = 0; i < STATE_DIM; i++){
    Xt1[i] = X[i] + (ts/6.0)*(k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
  }
  temp_norm = sqrtf(Xt1[0]*Xt1[0] + Xt1[1]*Xt1[1] + Xt1[2]*Xt1[2] + Xt1[3]*Xt1[3]);
  Xt1[0] /= temp_norm;
  Xt1[1] /= temp_norm;
  Xt1[2] /= temp_norm;
  Xt1[3] /= temp_norm;
}

void HybridDynamics::Euler(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
  Scalar ts = timestep;
  Eigen::Matrix<Scalar,STATE_DIM,1> Xd;
  ODE(X, Xd, u);
  Xt1 = X + (Xd*ts);
  
  Scalar temp_norm = sqrtf(Xt1[0]*Xt1[0] + Xt1[1]*Xt1[1] + Xt1[2]*Xt1[2] + Xt1[3]*Xt1[3]);
  Xt1[0] /= temp_norm;
  Xt1[1] /= temp_norm;
  Xt1[2] /= temp_norm;
  Xt1[3] /= temp_norm;
}

//Vehicle ODE
//A combination of forward dynamics, external forces, and quaternion derivative.
void HybridDynamics::ODE(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xd, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u){
  const JointState q(0,0,0,0);   //Joint position
  JointState qd;  //velocity
  JointState qdd; //acceleration
  const JointState tau(0,0,0,0); //Joint torque
  //Not going to use this ^^^^, going to just set tire velocity directly.
  //Assumption is that tire motor controllers are really good and can reach whatever velocity we command.
  //Later on we can use neural networks to estimate joint torque.
  
  Acceleration base_acc; //base refers to the floating base. Not world frame. 
  Velocity base_vel;
  
  //Could experiment with setting these to 0, as they dont change inertia properties.
  //And setting them to a constant 0 could enable optimizations and cancel stuff. Idk.
  //I'm doing it.
  // q[0] = q1 //tire positions.
  // q[1] = q2
  // q[2] = q3
  // q[3] = q4
    
  qd[0] = u[0]; //q1 //tire velocities. These actually matter.
  qd[1] = u[1]; //q2
  qd[2] = u[0]; //q3
  qd[3] = u[1]; //q4
  
  base_vel[0] = X[11];
  base_vel[1] = X[12];
  base_vel[2] = X[13];
  base_vel[3] = X[14];
  base_vel[4] = X[15];
  base_vel[5] = X[16];
  
  //Each force is expressed in the respective link frame.
  //intializes all forces to 0.
  LinkDataMap<Force> ext_forces(Force::Zero());
  
  //Calculates ext forces
  get_tire_f_ext(X, ext_forces);

  //rot is the rotation matrix that transforms vectors from the base frame to world frame.
  //The transpose converts the world frame gravity to a base frame gravity vector.
  Eigen::Matrix<Scalar,4,1> quat(X[0], X[1], X[2], X[3]);
  Eigen::Matrix<Scalar,3,3> rot = toMatrixRotation(X[0],X[1],X[2],X[3]);
  Eigen::Matrix<Scalar,3,1> gravity_lin(GRAVITY_VEC[3], GRAVITY_VEC[4], GRAVITY_VEC[5]);
  gravity_lin = rot.transpose()*gravity_lin;
  
  Force gravity_b;
  gravity_b[0] = 0;
  gravity_b[1] = 0;
  gravity_b[2] = 0;
  gravity_b[3] = gravity_lin[0];
  gravity_b[4] = gravity_lin[1];
  gravity_b[5] = gravity_lin[2];
  
  //this calculates base_acc and qdd.
  fwd_dynamics->fd(qdd, base_acc, base_vel, gravity_b, q, qd, tau, ext_forces);
  
  //Velocity com_vel = m_transforms.fr_base_link_COM_X_fr_base_link*base_vel;
  //Acceleration com_acc = m_transforms.fr_base_link_COM_X_fr_base_link*base_acc;

  //ROS_INFO("Acc %f %f %f %f %f %f", base_acc[0], base_acc[1], base_acc[2], base_acc[3], base_acc[4], base_acc[5]);
  //ROS_INFO("Vel %f %f %f %f %f %f", com_vel[0], com_vel[1], com_vel[2], com_vel[3], com_vel[4], com_vel[5]);
  
  Eigen::Matrix<Scalar,3,1> ang_vel(base_vel[0], base_vel[1], base_vel[2]);
  Eigen::Matrix<Scalar,3,1> lin_vel(base_vel[3], base_vel[4], base_vel[5]);
  Eigen::Matrix<Scalar,4,1> quat_dot = calcQuatDot(quat, ang_vel);
  
  lin_vel = rot*lin_vel; //converts from base to world frame.
  
  Xd[0] = quat_dot[0];
  Xd[1] = quat_dot[1];
  Xd[2] = quat_dot[2];
  Xd[3] = quat_dot[3];
  
  Xd[4] = lin_vel[0];
  Xd[5] = lin_vel[1];
  Xd[6] = lin_vel[2];
  
  Xd[7] = u[0]; //X[17];
  Xd[8] = u[1]; //X[18];
  Xd[9] = u[0]; //X[19];
  Xd[10] = u[1]; //X[20];
  
  Xd[11] = base_acc[0];
  Xd[12] = base_acc[1];
  Xd[13] = base_acc[2];
  Xd[14] = base_acc[3];
  Xd[15] = base_acc[4];
  Xd[16] = base_acc[5];
  
  Xd[17] = 0; //qdd[0];
  Xd[18] = 0; //qdd[1];
  Xd[19] = 0; //qdd[2];
  Xd[20] = 0; //qdd[3];
}


void HybridDynamics::startLog(std::string log_filename, std::string debug_filename){
  log_file.open(log_filename.c_str());
  log_file << "qw,qx,qy,qz,x,y,z,wx,wy,wz,vx,vy,vz,q1,q2,q3,q4,qd1,qd2,qd3,qd4\n";
  
  debug_file.open(debug_filename.c_str());
  debug_file << "zr1,zr2,zr3,zr4\n";

  ROS_INFO("Opening log file %s: %d", log_filename.c_str(), log_file.is_open());
}


void HybridDynamics::logVehicleState(){
  if(!log_file.is_open()){
    return;
  }
  
  //Log vehicle state in the format that spatial v2 needs.
  log_file << state_[3] << ','; //spatial v2 needs the quaternion in w,x,y,z format.
  log_file << state_[0] << ',';
  log_file << state_[1] << ',';
  log_file << state_[2] << ',';
  
  log_file << state_[4] << ','; //position
  log_file << state_[5] << ',';
  log_file << state_[6] << ',';

  log_file << state_[11] << ','; //spatial velocity
  log_file << state_[12] << ',';
  log_file << state_[13] << ',';
  log_file << state_[14] << ',';
  log_file << state_[15] << ',';
  log_file << state_[16] << ',';
  
  log_file << state_[7] << ','; //joint positions (should be zero)
  log_file << state_[8] << ',';
  log_file << state_[9] << ',';
  log_file << state_[10] << ',';
  
  log_file << state_[17] << ','; //joint velocity
  log_file << state_[18] << ',';
  log_file << state_[19] << ',';
  log_file << state_[20] << '\n';
}

void HybridDynamics::logValue(Scalar *values){
  debug_file << values[0] << ',';
  debug_file << values[1] << ',';
  debug_file << values[2] << ',';
  debug_file << values[3] << '\n';
}


void HybridDynamics::stopLog(){
  if(log_file.is_open())
    log_file.close();
  if(debug_file.is_open())
    debug_file.close();
}


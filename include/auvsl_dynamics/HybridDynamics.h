#pragma once
#include <fstream>
#include <string.h>
#include <ros/ros.h>
#include <Eigen/Dense>

//Robcogen files
#include "forward_dynamics.h"
#include "model_constants.h"

//My files
#include "TireNetwork.h"
#include "Model.h"

//using namespace iit::Fancy;
using Jackal::rcg::Scalar;
using Jackal::rcg::Velocity;
using Jackal::rcg::Acceleration;
using Jackal::rcg::Force;
using Jackal::rcg::JointState;
using Jackal::rcg::LinkDataMap;
using Jackal::rcg::orderedLinkIDs;

class HybridDynamics : public Model{
public:
  const static unsigned STATE_DIM = 21;
  const static unsigned CNTRL_DIM = 2;
  
  HybridDynamics();
  ~HybridDynamics();

  static void setAltitudeMap(Scalar (*alt_func)(Scalar x, Scalar y, Scalar z_guess));
  
  void logVehicleState();
  void logValue(float *values);
  
  void startLog(std::string log_filename, std::string debug_filename);
  void stopLog();
  
  void initState() override;
  void initState(Scalar *start_state) override;
  void initStateCOM(Scalar *start_state) override;
  void step(Scalar vl, Scalar vr) override;
  void settle() override;
  void getState(Scalar *state) override;
  
  unsigned getStateDim() const override;
  unsigned getControlDim() const override;
  
  void Euler(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  void RK4(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xt1, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  void ODE(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,STATE_DIM,1> &Xd, Eigen::Matrix<Scalar,CNTRL_DIM,1> &u);
  
  void get_tire_sinkages(const Eigen::Matrix<Scalar,3,1> *cpt_points, Scalar *sinkages);
  void get_tire_cpt_vels(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, const Eigen::Matrix<Scalar,3,3> *cpt_rots, Eigen::Matrix<Scalar,3,1> *cpt_vels);
  void get_tire_f_ext(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, LinkDataMap<Force> &ext_forces);
  void get_tire_cpts_sinkages(const Eigen::Matrix<Scalar,STATE_DIM,1> &X, Eigen::Matrix<Scalar,3,1> *cpt_pts, Eigen::Matrix<Scalar,3,3> *cpt_rots, Scalar *sinkages);
  
  std::ofstream log_file;
  std::ofstream debug_file;
  static Scalar timestep; //The rate that nn_model operates at.
  constexpr static Scalar stepsize = .05;
  
  //angular and linear vel are expressed in the body frame (I think)
  //Quaternion is x,y,z,w
  //State is a vector: <quaternion, position, joint positions, angular vel, linear vel, joint velocities>
  //0  1  2  3  4 5 6 7  8  9  10 11 12 13 14 15 16 17  18  19  20
  //qx,qy,qz,qw,x,y,z,q1,q2,q3,q4,wx,wy,wz,vx,vy,vz,qd1,qd2,qd3,qd4
  Eigen::Matrix<Scalar,STATE_DIM,1> state_;
  
  const static Acceleration GRAVITY_VEC;
  
  Eigen::Matrix<Scalar,5,1> bekker_params;
  TireNetwork tire_network; //holds the tire-soil model

  static int altitude_map_not_set_;
  static Scalar (*get_altitude_)(Scalar x, Scalar y, Scalar z_guess);
  
  
  Jackal::rcg::HomogeneousTransforms h_transforms; //not actually used
  Jackal::rcg::MotionTransforms      m_transforms;
  Jackal::rcg::ForceTransforms       f_transforms; //Used to convert external forces to the right frame. Oh wait
  Jackal::rcg::InertiaProperties     inertias;
  Jackal::rcg::ForwardDynamics      *fwd_dynamics;
};


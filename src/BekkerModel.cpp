#include "BekkerModel.h"
#include <math.h>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <stdlib.h>


using Jackal::rcg::tire_radius;
float tire_thickness = .05;


struct BekkerModel{
public:
  static constexpr float k0 = 2195;
  static constexpr float Au = 192400;
  static constexpr float K = .0254;
  static constexpr float density = 1681;
  static constexpr float c = 8.62;
  static constexpr float pg = 100; //15psi

  
  static float zr;
  static float slip_ratio;
  static float slip_angle;
  
  static float kc;
  static float kphi;
  static float n0;
  static float n1;
  static float phi;

  static float n;
  static float pgc;
  static float R;
  static float b;
  static float ze;
  static float zu;
  static float ku;
  
  static float theta_f;
  static float theta_r;
  static float theta_c;
  
  static const int num_steps = 100;
  
  static float sigma_x_cf(float theta){
    //in some cases, cosf - cosf can return negative due to theta != theta_f, but very close.
    float diff = cosf(theta) - cosf(theta_f);//std::max(0.0f,cosf(theta) - cosf(theta_f));
    float temp = ((kc/b) + kphi)*std::pow(R*(diff), n);
    return temp;
  }
  static float sigma_x_cc(float theta){return ((kc/b)+kphi)*std::pow(ze, n); }
  static float sigma_x_cr(float theta){
    float diff = cosf(theta) - cosf(theta_r);
    float temp = ku*std::pow(R*(diff), n);
    return temp;
  }
  
  static float jx_cf(float theta){
    float temp = R*((theta - theta_f) - (1 - slip_ratio)*(sinf(theta)-sinf(theta_f)));
    return temp;
  }
  static float jx_cc(float theta){return R*((theta_c - theta_f) + (1 - slip_ratio)*sinf(theta_f) - sinf(theta_c) + (slip_ratio*cosf(theta_c)*tanf(theta))); }
  static float jx_cr(float theta){return R*((2*theta_c + theta - theta_f) - (1 - slip_ratio)*(sinf(theta) - sinf(theta_f)) - 2*sinf(theta_c)); }
  
  static float jy_cf(float theta){
    return R*(1-slip_ratio) * tanf(slip_angle) * (theta_f - theta);
  }
  static float jy_cc(float theta){return R*(1-slip_ratio) * tanf(slip_angle) * (theta_f - theta + sinf(theta_c) - cosf(theta_c)*tanf(theta)); }
  static float jy_cr(float theta){return R*(1-slip_ratio) * tanf(slip_angle) * (theta_f - theta - 2*theta_c + 2*sinf(theta_c)); }
  
  static float j_cf(float theta){
    float temp = sqrtf(std::pow(jy_cf(theta), 2) + std::pow(jx_cf(theta), 2));
    return temp;
  }
  static float j_cc(float theta){return sqrtf(std::pow(jy_cc(theta), 2) + std::pow(jx_cc(theta), 2)); }
  static float j_cr(float theta){return sqrtf(std::pow(jy_cr(theta), 2) + std::pow(jx_cr(theta), 2)); }
  
  static float tau_x_cf(float theta){
    //smart divide
    float j_cf_theta = j_cf(theta);
    float temp = j_cf_theta == 0 ? 0 : jx_cf(theta)/j_cf_theta;
    return (-temp) * (c + (sigma_x_cf(theta)*tanf(phi))) * (1 - (exp(-j_cf_theta/K)));
  }
  static float tau_x_cc(float theta){
    float j_cc_theta = j_cc(theta);
    float temp = j_cc_theta == 0 ? 0 : jx_cc(theta)/j_cc_theta;
    return (-temp) * (c + (sigma_x_cc(theta)*tanf(phi))) * (1 - (exp(-j_cc_theta/K)));
  } 
  static float tau_x_cr(float theta){
    float j_cr_theta = j_cr(theta);
    float temp = j_cr_theta == 0 ? 0 : jx_cr(theta)/j_cr_theta;
    return (-temp) * (c + (sigma_x_cr(theta)*tanf(phi))) * (1 - (exp(-j_cr_theta/K)));
  }
  
  static float tau_y_cf(float theta){
    float j_cf_theta = j_cf(theta);
    float temp = j_cf_theta == 0 ? 0 : jy_cf(theta)/j_cf_theta;
    return (-temp) * (c + (sigma_x_cf(theta)*tanf(phi))) * (1 - (exp(-j_cf_theta/K)));
  }
  static float tau_y_cc(float theta){
    float j_cc_theta = j_cc(theta);
    float temp = j_cc_theta == 0 ? 0 : jy_cc(theta)/j_cc_theta;
    return (-temp) * (c + (sigma_x_cc(theta)*tanf(phi))) * (1 - (exp(-j_cc_theta/K)));
  }
  static float tau_y_cr(float theta){
    float j_cr_theta = j_cr(theta);
    float temp = j_cr_theta == 0 ? 0 : jy_cr(theta)/j_cr_theta;
    return (-temp) * (c + (sigma_x_cr(theta)*tanf(phi))) * (1 - (exp(-j_cr_theta/K)));
  }


  static float Fy_eqn3(float theta){return tau_y_cc(theta)*std::pow(1.0f/cosf(theta), 2); }
  
  static float Fz_eqn1(float theta){return (sigma_x_cf(theta)*cosf(theta)) + (tau_x_cf(theta)*sinf(theta)); }
  static float Fz_eqn2(float theta){return (sigma_x_cr(theta)*cosf(theta)) + (tau_x_cr(theta)*sinf(theta)); }
  
  static float Fx_eqn1(float theta){return (tau_x_cf(theta)*cosf(theta)) - (sigma_x_cf(theta)*sinf(theta)); } 
  static float Fx_eqn2(float theta){return (tau_x_cr(theta)*cosf(theta)) - (sigma_x_cr(theta)*sinf(theta)); }
  static float Fx_eqn3(float theta){return tau_x_cc(theta)*(std::pow(1.0f/cosf(theta), 2)); }
  
  static float Ty_eqn1(float theta){return tau_x_cc(theta)*(std::pow(1.0f/cosf(theta), 2)); }

  static float integrate(float (*func)(float), float upper_b, float lower_b){
    float dtheta = (upper_b - lower_b) / num_steps;
    float eps = dtheta*.1; //adaptive machine epsilon
    
    //trapezoidal rule.
    float sum = 0;
    for(float theta = lower_b; theta < (upper_b - eps - dtheta); theta += dtheta){
      sum += .5*dtheta*(func(theta + dtheta) + func(theta));
    }

    //last iteration is different to ensure no floating point error occurs.
    //This ensures integration goes exactly to the upper bound and does not exceed it in the slightest.
    sum += .5*dtheta*(func(upper_b) + func(upper_b - dtheta));
    
    return sum;
  }
};



float BekkerModel::zr;
float BekkerModel::slip_ratio;
float BekkerModel::slip_angle;
  
float BekkerModel::kc;
float BekkerModel::kphi;
float BekkerModel::n0;
float BekkerModel::n1;
float BekkerModel::phi;

float BekkerModel::n;
float BekkerModel::pgc;
float BekkerModel::R;
float BekkerModel::b;
float BekkerModel::ze;
float BekkerModel::zu;
float BekkerModel::ku;
  
float BekkerModel::theta_f;
float BekkerModel::theta_r;
float BekkerModel::theta_c;



Eigen::Matrix<float, TireNetwork::num_out_features, 1> tire_model_bekker(const Eigen::Matrix<float,TireNetwork::num_in_features,1> &features){
  Eigen::Matrix<Scalar,TireNetwork::num_out_features,1> tire_wrench;
  float Fx = 0;
  float Fy = 0;
  float Fz = 0;
  float Ty = 0;

  BekkerModel bekker_model;
  bekker_model.R = tire_radius;
  bekker_model.b = tire_thickness;
  
  bekker_model.zr = std::min(features[0], tire_radius);
  bekker_model.slip_ratio = features[1];
  bekker_model.slip_angle = features[2];
  
  bekker_model.kc = features[3];
  bekker_model.kphi = features[4];
  bekker_model.n0 = features[5];
  bekker_model.n1 = features[6];
  bekker_model.phi = features[7];
  
  bekker_model.n = bekker_model.n0 + (bekker_model.n1 * fabs(bekker_model.slip_ratio));
  bekker_model.pgc = ((bekker_model.kc/bekker_model.b) + bekker_model.kphi) * std::pow(bekker_model.zr, bekker_model.n);
  
  if(bekker_model.pgc < bekker_model.pg){
    bekker_model.ze = bekker_model.zr;
  }
  else{
    bekker_model.ze = std::pow(bekker_model.pg/((bekker_model.kc/bekker_model.b) + bekker_model.kphi), 1.0f/bekker_model.n);
  }
  
  bekker_model.ku = bekker_model.k0 + (bekker_model.Au*bekker_model.ze);
  bekker_model.zu = std::pow(((bekker_model.kc/bekker_model.b) + bekker_model.kphi) / bekker_model.ku, 1.0f/bekker_model.n) * bekker_model.ze;
  
  float theta_f = bekker_model.theta_f =  acosf(1 - (bekker_model.zr/bekker_model.R));
  float theta_r = bekker_model.theta_r = -acosf(1 - ((bekker_model.zu + bekker_model.zr - bekker_model.ze)/bekker_model.R));
  float theta_c = bekker_model.theta_c =  acosf(1 - ((bekker_model.zr - bekker_model.ze)/bekker_model.R));

  //seems legit
  //ROS_INFO("f %f     r %f     c %f", theta_f*180.0f/M_PI, theta_r*180.0f/M_PI, theta_c*180.0f/M_PI);
  
  float bt_R = bekker_model.R * bekker_model.b;
  float bt_RR = bekker_model.R * bekker_model.R * bekker_model.b;
  
  Fx = bt_R*bekker_model.integrate(&bekker_model.Fx_eqn1, theta_f, theta_c) +
       bt_R*cosf(theta_c)*bekker_model.integrate(&bekker_model.Fx_eqn3, theta_c, -theta_c) +
       bt_R*bekker_model.integrate(&bekker_model.Fx_eqn2, -theta_c, theta_r);
  
  Fy = bt_R*bekker_model.integrate(&bekker_model.tau_y_cf, theta_f, theta_c) +
       bt_R*cosf(theta_c)*bekker_model.integrate(&bekker_model.Fy_eqn3, theta_c, -theta_c) + //original matlab code has a bug here. In the matlab code I integrated from -theta_c to theta_c which gives a negative result.
       bt_R*bekker_model.integrate(&bekker_model.tau_y_cr, -theta_c, theta_r);
  
  Fz = bt_R*bekker_model.integrate(&bekker_model.Fz_eqn1, theta_f, theta_c) +
       bt_R*bekker_model.integrate(&bekker_model.Fz_eqn2, -theta_c, theta_r) +
     2*bt_R*sinf(theta_c)*bekker_model.pg;
  
  Ty = -bt_RR*bekker_model.integrate(&bekker_model.tau_x_cf, theta_f, theta_c) + 
       -bt_RR*bekker_model.integrate(&bekker_model.tau_x_cr, -theta_c, theta_r) +
       -bt_RR*cosf(theta_c)*cosf(theta_c)*bekker_model.integrate(&bekker_model.Ty_eqn1, theta_c, -theta_c);

  tire_wrench[0] = 1000*Fx;
  tire_wrench[1] = 1000*Fy;
  tire_wrench[2] = 1000*Fz;
  tire_wrench[3] = 1000*Ty;
  
  return tire_wrench;
}

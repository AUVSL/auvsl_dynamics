#ifndef RCG_JACKAL_MODEL_CONSTANTS_H_
#define RCG_JACKAL_MODEL_CONSTANTS_H_

#include "rbd_types.h"

/**
 * \file
 * This file contains the definitions of all the non-zero numerical
 * constants of the robot model (i.e. the numbers appearing in the
 * .kindsl file).
 *
 * Varying these values (and recompiling) is a quick & dirty
 * way to vary the kinematics/dynamics model. For a much more
 * flexible way of exploring variations of the model, consider
 * using the parametrization feature of RobCoGen (see the wiki).
 *
 * Beware of inconsistencies when changing any of the inertia
 * properties.
 */

namespace Jackal {
namespace rcg {

const Scalar tire_radius = .098;
  
// Do not use 'constexpr' to allow for non-literal scalar types

const Scalar tx_front_left_wheel = 0.13099999725818634;
const Scalar ty_front_left_wheel = 0.1877949982881546;
const Scalar tz_front_left_wheel = 0.03449999913573265;
  
const Scalar tx_front_right_wheel = 0.13099999725818634;
const Scalar ty_front_right_wheel = -0.1877949982881546;
const Scalar tz_front_right_wheel = 0.03449999913573265;
  
const Scalar tx_rear_left_wheel = -0.13099999725818634;
const Scalar ty_rear_left_wheel = 0.1877949982881546;
const Scalar tz_rear_left_wheel = 0.03449999913573265;
  
const Scalar tx_rear_right_wheel = -0.13099999725818634;
const Scalar ty_rear_right_wheel = -0.1877949982881546;
const Scalar tz_rear_right_wheel = 0.03449999913573265;

  //these dont get used in the forward dynamics.
const Scalar tx_fr_base_link_COM = 0.011999273672699928;
const Scalar ty_fr_base_link_COM = 0.0019998790230602026;
const Scalar tz_fr_base_link_COM = 0.06699594855308533;

//const Scalar tx_fr_base_link_COM = 0.0;
//const Scalar ty_fr_base_link_COM = 0.0;
//const Scalar tz_fr_base_link_COM = 0.0;
  
const Scalar tx_fr_front_mount = 0.11999999731779099;
const Scalar tz_fr_front_mount = 0.18400000035762787;
const Scalar tz_fr_mid_mount = 0.18400000035762787;
const Scalar tx_fr_navsat_link = -0.18000000715255737;
const Scalar ty_fr_navsat_link = 0.12600000202655792;
const Scalar tz_fr_navsat_link = 0.18150000274181366;
const Scalar tx_fr_rear_mount = -0.11999999731779099;
const Scalar tz_fr_rear_mount = 0.18400000035762787;
const Scalar m_base_link = 16.52400016784668;
  
  const Scalar comx_base_link = 0.011999273672699928;
  const Scalar comy_base_link = 0.0019998790230602026;
  const Scalar comz_base_link = 0.06699594855308533;

  //const Scalar comx_base_link = 0.;
  //const Scalar comy_base_link = 0.;
  //const Scalar comz_base_link = 0.;

const Scalar ix_base_link = 0.38783785700798035;
const Scalar ixy_base_link = 0.0011965520679950714;
const Scalar ixz_base_link = -0.003115505911409855;
  //const Scalar ixy_base_link = 0.;
  //const Scalar ixz_base_link = 0.;
const Scalar iy_base_link = 0.46875107288360596;
const Scalar iyz_base_link = 0.0031140821520239115;
  //const Scalar iyz_base_link = 0.;
const Scalar iz_base_link = 0.4509454071521759;
  
const Scalar m_front_left_wheel_link = 0.47699999809265137;
const Scalar ix_front_left_wheel_link = 0.0013000000035390258;
const Scalar iy_front_left_wheel_link = 0.0013000000035390258;
const Scalar iyz_front_left_wheel_link = 4.808253448174149E-11;
const Scalar iz_front_left_wheel_link = 0.002400000113993883;
const Scalar m_front_right_wheel_link = 0.47699999809265137;
const Scalar ix_front_right_wheel_link = 0.0013000000035390258;
const Scalar iy_front_right_wheel_link = 0.0013000000035390258;
const Scalar iyz_front_right_wheel_link = 4.808253448174149E-11;
const Scalar iz_front_right_wheel_link = 0.002400000113993883;
const Scalar m_rear_left_wheel_link = 0.47699999809265137;
const Scalar ix_rear_left_wheel_link = 0.0013000000035390258;
const Scalar iy_rear_left_wheel_link = 0.0013000000035390258;
const Scalar iyz_rear_left_wheel_link = 4.808253448174149E-11;
const Scalar iz_rear_left_wheel_link = 0.002400000113993883;
const Scalar m_rear_right_wheel_link = 0.47699999809265137;
const Scalar ix_rear_right_wheel_link = 0.0013000000035390258;
const Scalar iy_rear_right_wheel_link = 0.0013000000035390258;
const Scalar iyz_rear_right_wheel_link = 4.808253448174149E-11;
const Scalar iz_rear_right_wheel_link = 0.002400000113993883;

}
}
#endif

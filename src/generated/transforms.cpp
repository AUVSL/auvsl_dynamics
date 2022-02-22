#include "transforms.h"

using namespace Jackal::rcg;

// Constructors

MotionTransforms::MotionTransforms()
 :  fr_base_link_X_fr_front_left_wheel_link(),
    fr_front_left_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_front_right_wheel_link(),
    fr_front_right_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_left_wheel_link(),
    fr_rear_left_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_right_wheel_link(),
    fr_rear_right_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_base_link_COM(),
    fr_base_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_chassis_link(),
    fr_chassis_link_X_fr_base_link(),
    fr_base_link_X_fr_front_fender_link(),
    fr_front_fender_link_X_fr_base_link(),
    fr_base_link_X_fr_front_left_wheel_link_COM(),
    fr_front_left_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_front_mount(),
    fr_front_mount_X_fr_base_link(),
    fr_base_link_X_fr_front_right_wheel_link_COM(),
    fr_front_right_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_imu_link(),
    fr_imu_link_X_fr_base_link(),
    fr_base_link_X_fr_mid_mount(),
    fr_mid_mount_X_fr_base_link(),
    fr_base_link_X_fr_navsat_link(),
    fr_navsat_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_fender_link(),
    fr_rear_fender_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_left_wheel_link_COM(),
    fr_rear_left_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_rear_mount(),
    fr_rear_mount_X_fr_base_link(),
    fr_base_link_X_fr_rear_right_wheel_link_COM(),
    fr_rear_right_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_front_left_wheel(),
    fr_base_link_X_fr_front_right_wheel(),
    fr_base_link_X_fr_rear_left_wheel(),
    fr_base_link_X_fr_rear_right_wheel()
{}
void MotionTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

ForceTransforms::ForceTransforms()
 :     fr_base_link_X_fr_front_left_wheel_link(),
    fr_front_left_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_front_right_wheel_link(),
    fr_front_right_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_left_wheel_link(),
    fr_rear_left_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_right_wheel_link(),
    fr_rear_right_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_base_link_COM(),
    fr_base_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_chassis_link(),
    fr_chassis_link_X_fr_base_link(),
    fr_base_link_X_fr_front_fender_link(),
    fr_front_fender_link_X_fr_base_link(),
    fr_base_link_X_fr_front_left_wheel_link_COM(),
    fr_front_left_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_front_mount(),
    fr_front_mount_X_fr_base_link(),
    fr_base_link_X_fr_front_right_wheel_link_COM(),
    fr_front_right_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_imu_link(),
    fr_imu_link_X_fr_base_link(),
    fr_base_link_X_fr_mid_mount(),
    fr_mid_mount_X_fr_base_link(),
    fr_base_link_X_fr_navsat_link(),
    fr_navsat_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_fender_link(),
    fr_rear_fender_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_left_wheel_link_COM(),
    fr_rear_left_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_rear_mount(),
    fr_rear_mount_X_fr_base_link(),
    fr_base_link_X_fr_rear_right_wheel_link_COM(),
    fr_rear_right_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_front_left_wheel(),
    fr_base_link_X_fr_front_right_wheel(),
    fr_base_link_X_fr_rear_left_wheel(),
    fr_base_link_X_fr_rear_right_wheel()
{}
void ForceTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

HomogeneousTransforms::HomogeneousTransforms()
 :     fr_base_link_X_fr_front_left_wheel_link(),
    fr_front_left_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_front_right_wheel_link(),
    fr_front_right_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_left_wheel_link(),
    fr_rear_left_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_right_wheel_link(),
    fr_rear_right_wheel_link_X_fr_base_link(),
    fr_base_link_X_fr_base_link_COM(),
    fr_base_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_chassis_link(),
    fr_chassis_link_X_fr_base_link(),
    fr_base_link_X_fr_front_fender_link(),
    fr_front_fender_link_X_fr_base_link(),
    fr_base_link_X_fr_front_left_wheel_link_COM(),
    fr_front_left_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_front_mount(),
    fr_front_mount_X_fr_base_link(),
    fr_base_link_X_fr_front_right_wheel_link_COM(),
    fr_front_right_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_imu_link(),
    fr_imu_link_X_fr_base_link(),
    fr_base_link_X_fr_mid_mount(),
    fr_mid_mount_X_fr_base_link(),
    fr_base_link_X_fr_navsat_link(),
    fr_navsat_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_fender_link(),
    fr_rear_fender_link_X_fr_base_link(),
    fr_base_link_X_fr_rear_left_wheel_link_COM(),
    fr_rear_left_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_rear_mount(),
    fr_rear_mount_X_fr_base_link(),
    fr_base_link_X_fr_rear_right_wheel_link_COM(),
    fr_rear_right_wheel_link_COM_X_fr_base_link(),
    fr_base_link_X_fr_front_left_wheel(),
    fr_base_link_X_fr_front_right_wheel(),
    fr_base_link_X_fr_rear_left_wheel(),
    fr_base_link_X_fr_rear_right_wheel()
{}
void HomogeneousTransforms::updateParams(const Params_lengths& v_lengths, const Params_angles& v_angles)
{
    params.lengths = v_lengths;
    params.angles = v_angles;
    params.trig.update();
}

MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel_link::Type_fr_base_link_X_fr_front_left_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel_link& MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,1) = -sin_q_front_left_wheel;
    (*this)(2,0) = -sin_q_front_left_wheel;
    (*this)(2,1) = -cos_q_front_left_wheel;
    (*this)(3,0) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,1) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,4) = -sin_q_front_left_wheel;
    (*this)(4,0) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(4,1) = ( tx_front_left_wheel * cos_q_front_left_wheel)-( tz_front_left_wheel * sin_q_front_left_wheel);
    (*this)(5,0) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(5,1) =  ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(5,3) = -sin_q_front_left_wheel;
    (*this)(5,4) = -cos_q_front_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_front_left_wheel_link_X_fr_base_link::Type_fr_front_left_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_front_left_wheel_link_X_fr_base_link& MotionTransforms::Type_fr_front_left_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = -sin_q_front_left_wheel;
    (*this)(1,0) = -sin_q_front_left_wheel;
    (*this)(1,2) = -cos_q_front_left_wheel;
    (*this)(3,0) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,1) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(3,2) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,5) = -sin_q_front_left_wheel;
    (*this)(4,0) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(4,1) = ( tx_front_left_wheel * cos_q_front_left_wheel)-( tz_front_left_wheel * sin_q_front_left_wheel);
    (*this)(4,2) =  ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(4,3) = -sin_q_front_left_wheel;
    (*this)(4,5) = -cos_q_front_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel_link::Type_fr_base_link_X_fr_front_right_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel_link& MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,1) = -sin_q_front_right_wheel;
    (*this)(2,0) = -sin_q_front_right_wheel;
    (*this)(2,1) = -cos_q_front_right_wheel;
    (*this)(3,0) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,1) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,4) = -sin_q_front_right_wheel;
    (*this)(4,0) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(4,1) = ( tx_front_right_wheel * cos_q_front_right_wheel)-( tz_front_right_wheel * sin_q_front_right_wheel);
    (*this)(5,0) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(5,1) =  ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(5,3) = -sin_q_front_right_wheel;
    (*this)(5,4) = -cos_q_front_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_front_right_wheel_link_X_fr_base_link::Type_fr_front_right_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_front_right_wheel_link_X_fr_base_link& MotionTransforms::Type_fr_front_right_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = -sin_q_front_right_wheel;
    (*this)(1,0) = -sin_q_front_right_wheel;
    (*this)(1,2) = -cos_q_front_right_wheel;
    (*this)(3,0) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,1) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(3,2) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,5) = -sin_q_front_right_wheel;
    (*this)(4,0) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(4,1) = ( tx_front_right_wheel * cos_q_front_right_wheel)-( tz_front_right_wheel * sin_q_front_right_wheel);
    (*this)(4,2) =  ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(4,3) = -sin_q_front_right_wheel;
    (*this)(4,5) = -cos_q_front_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link::Type_fr_base_link_X_fr_rear_left_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link& MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,1) = -sin_q_rear_left_wheel;
    (*this)(2,0) = -sin_q_rear_left_wheel;
    (*this)(2,1) = -cos_q_rear_left_wheel;
    (*this)(3,0) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,1) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,4) = -sin_q_rear_left_wheel;
    (*this)(4,0) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(4,1) = ( tx_rear_left_wheel * cos_q_rear_left_wheel)-( tz_rear_left_wheel * sin_q_rear_left_wheel);
    (*this)(5,0) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(5,1) =  ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(5,3) = -sin_q_rear_left_wheel;
    (*this)(5,4) = -cos_q_rear_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link::Type_fr_rear_left_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link& MotionTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = -sin_q_rear_left_wheel;
    (*this)(1,0) = -sin_q_rear_left_wheel;
    (*this)(1,2) = -cos_q_rear_left_wheel;
    (*this)(3,0) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,1) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(3,2) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,5) = -sin_q_rear_left_wheel;
    (*this)(4,0) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(4,1) = ( tx_rear_left_wheel * cos_q_rear_left_wheel)-( tz_rear_left_wheel * sin_q_rear_left_wheel);
    (*this)(4,2) =  ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(4,3) = -sin_q_rear_left_wheel;
    (*this)(4,5) = -cos_q_rear_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link::Type_fr_base_link_X_fr_rear_right_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,2) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(3,5) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,2) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link& MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,1) = -sin_q_rear_right_wheel;
    (*this)(2,0) = -sin_q_rear_right_wheel;
    (*this)(2,1) = -cos_q_rear_right_wheel;
    (*this)(3,0) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,1) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,4) = -sin_q_rear_right_wheel;
    (*this)(4,0) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(4,1) = ( tx_rear_right_wheel * cos_q_rear_right_wheel)-( tz_rear_right_wheel * sin_q_rear_right_wheel);
    (*this)(5,0) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(5,1) =  ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(5,3) = -sin_q_rear_right_wheel;
    (*this)(5,4) = -cos_q_rear_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link::Type_fr_rear_right_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link& MotionTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = -sin_q_rear_right_wheel;
    (*this)(1,0) = -sin_q_rear_right_wheel;
    (*this)(1,2) = -cos_q_rear_right_wheel;
    (*this)(3,0) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,1) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(3,2) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,5) = -sin_q_rear_right_wheel;
    (*this)(4,0) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(4,1) = ( tx_rear_right_wheel * cos_q_rear_right_wheel)-( tz_rear_right_wheel * sin_q_rear_right_wheel);
    (*this)(4,2) =  ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(4,3) = -sin_q_rear_right_wheel;
    (*this)(4,5) = -cos_q_rear_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_base_link_COM::Type_fr_base_link_X_fr_base_link_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_fr_base_link_COM;    // Maxima DSL: -_k__tz_fr_base_link_COM
    (*this)(3,2) =  ty_fr_base_link_COM;    // Maxima DSL: _k__ty_fr_base_link_COM
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_fr_base_link_COM;    // Maxima DSL: _k__tz_fr_base_link_COM
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_fr_base_link_COM;    // Maxima DSL: -_k__tx_fr_base_link_COM
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_fr_base_link_COM;    // Maxima DSL: -_k__ty_fr_base_link_COM
    (*this)(5,1) =  tx_fr_base_link_COM;    // Maxima DSL: _k__tx_fr_base_link_COM
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_base_link_COM& MotionTransforms::Type_fr_base_link_X_fr_base_link_COM::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_COM_X_fr_base_link::Type_fr_base_link_COM_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) =  tz_fr_base_link_COM;    // Maxima DSL: _k__tz_fr_base_link_COM
    (*this)(3,2) = - ty_fr_base_link_COM;    // Maxima DSL: -_k__ty_fr_base_link_COM
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = - tz_fr_base_link_COM;    // Maxima DSL: -_k__tz_fr_base_link_COM
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_fr_base_link_COM;    // Maxima DSL: _k__tx_fr_base_link_COM
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_fr_base_link_COM;    // Maxima DSL: _k__ty_fr_base_link_COM
    (*this)(5,1) = - tx_fr_base_link_COM;    // Maxima DSL: -_k__tx_fr_base_link_COM
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_COM_X_fr_base_link& MotionTransforms::Type_fr_base_link_COM_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_chassis_link::Type_fr_base_link_X_fr_chassis_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_chassis_link& MotionTransforms::Type_fr_base_link_X_fr_chassis_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_chassis_link_X_fr_base_link::Type_fr_chassis_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_chassis_link_X_fr_base_link& MotionTransforms::Type_fr_chassis_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_fender_link::Type_fr_base_link_X_fr_front_fender_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_fender_link& MotionTransforms::Type_fr_base_link_X_fr_front_fender_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_front_fender_link_X_fr_base_link::Type_fr_front_fender_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_front_fender_link_X_fr_base_link& MotionTransforms::Type_fr_front_fender_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM::Type_fr_base_link_X_fr_front_left_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,1) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM& MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = sin_q_front_left_wheel;
    (*this)(2,0) = -sin_q_front_left_wheel;
    (*this)(2,2) = cos_q_front_left_wheel;
    (*this)(3,0) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,2) =  ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,5) = sin_q_front_left_wheel;
    (*this)(4,0) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(4,2) = ( tz_front_left_wheel * sin_q_front_left_wheel)-( tx_front_left_wheel * cos_q_front_left_wheel);
    (*this)(5,0) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(5,2) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(5,3) = -sin_q_front_left_wheel;
    (*this)(5,5) = cos_q_front_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link::Type_fr_front_left_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link& MotionTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = -sin_q_front_left_wheel;
    (*this)(2,0) = sin_q_front_left_wheel;
    (*this)(2,2) = cos_q_front_left_wheel;
    (*this)(3,0) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,1) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(3,2) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,5) = -sin_q_front_left_wheel;
    (*this)(5,0) =  ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(5,1) = ( tz_front_left_wheel * sin_q_front_left_wheel)-( tx_front_left_wheel * cos_q_front_left_wheel);
    (*this)(5,2) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(5,3) = sin_q_front_left_wheel;
    (*this)(5,5) = cos_q_front_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_mount::Type_fr_base_link_X_fr_front_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_fr_front_mount;    // Maxima DSL: -_k__tz_fr_front_mount
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_fr_front_mount;    // Maxima DSL: _k__tz_fr_front_mount
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_fr_front_mount;    // Maxima DSL: -_k__tx_fr_front_mount
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_fr_front_mount;    // Maxima DSL: _k__tx_fr_front_mount
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_mount& MotionTransforms::Type_fr_base_link_X_fr_front_mount::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_front_mount_X_fr_base_link::Type_fr_front_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) =  tz_fr_front_mount;    // Maxima DSL: _k__tz_fr_front_mount
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = - tz_fr_front_mount;    // Maxima DSL: -_k__tz_fr_front_mount
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_fr_front_mount;    // Maxima DSL: _k__tx_fr_front_mount
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_fr_front_mount;    // Maxima DSL: -_k__tx_fr_front_mount
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_front_mount_X_fr_base_link& MotionTransforms::Type_fr_front_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM::Type_fr_base_link_X_fr_front_right_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,1) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM& MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = sin_q_front_right_wheel;
    (*this)(2,0) = -sin_q_front_right_wheel;
    (*this)(2,2) = cos_q_front_right_wheel;
    (*this)(3,0) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,2) =  ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,5) = sin_q_front_right_wheel;
    (*this)(4,0) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(4,2) = ( tz_front_right_wheel * sin_q_front_right_wheel)-( tx_front_right_wheel * cos_q_front_right_wheel);
    (*this)(5,0) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(5,2) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(5,3) = -sin_q_front_right_wheel;
    (*this)(5,5) = cos_q_front_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link::Type_fr_front_right_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link& MotionTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = -sin_q_front_right_wheel;
    (*this)(2,0) = sin_q_front_right_wheel;
    (*this)(2,2) = cos_q_front_right_wheel;
    (*this)(3,0) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,1) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(3,2) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,5) = -sin_q_front_right_wheel;
    (*this)(5,0) =  ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(5,1) = ( tz_front_right_wheel * sin_q_front_right_wheel)-( tx_front_right_wheel * cos_q_front_right_wheel);
    (*this)(5,2) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(5,3) = sin_q_front_right_wheel;
    (*this)(5,5) = cos_q_front_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_imu_link::Type_fr_base_link_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_imu_link& MotionTransforms::Type_fr_base_link_X_fr_imu_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_imu_link_X_fr_base_link::Type_fr_imu_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_imu_link_X_fr_base_link& MotionTransforms::Type_fr_imu_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_mid_mount::Type_fr_base_link_X_fr_mid_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_fr_mid_mount;    // Maxima DSL: -_k__tz_fr_mid_mount
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_fr_mid_mount;    // Maxima DSL: _k__tz_fr_mid_mount
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_mid_mount& MotionTransforms::Type_fr_base_link_X_fr_mid_mount::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_mid_mount_X_fr_base_link::Type_fr_mid_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) =  tz_fr_mid_mount;    // Maxima DSL: _k__tz_fr_mid_mount
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = - tz_fr_mid_mount;    // Maxima DSL: -_k__tz_fr_mid_mount
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_mid_mount_X_fr_base_link& MotionTransforms::Type_fr_mid_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_navsat_link::Type_fr_base_link_X_fr_navsat_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_fr_navsat_link;    // Maxima DSL: -_k__tz_fr_navsat_link
    (*this)(3,2) =  ty_fr_navsat_link;    // Maxima DSL: _k__ty_fr_navsat_link
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_fr_navsat_link;    // Maxima DSL: _k__tz_fr_navsat_link
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_fr_navsat_link;    // Maxima DSL: -_k__tx_fr_navsat_link
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = - ty_fr_navsat_link;    // Maxima DSL: -_k__ty_fr_navsat_link
    (*this)(5,1) =  tx_fr_navsat_link;    // Maxima DSL: _k__tx_fr_navsat_link
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_navsat_link& MotionTransforms::Type_fr_base_link_X_fr_navsat_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_navsat_link_X_fr_base_link::Type_fr_navsat_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) =  tz_fr_navsat_link;    // Maxima DSL: _k__tz_fr_navsat_link
    (*this)(3,2) = - ty_fr_navsat_link;    // Maxima DSL: -_k__ty_fr_navsat_link
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = - tz_fr_navsat_link;    // Maxima DSL: -_k__tz_fr_navsat_link
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_fr_navsat_link;    // Maxima DSL: _k__tx_fr_navsat_link
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) =  ty_fr_navsat_link;    // Maxima DSL: _k__ty_fr_navsat_link
    (*this)(5,1) = - tx_fr_navsat_link;    // Maxima DSL: -_k__tx_fr_navsat_link
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_navsat_link_X_fr_base_link& MotionTransforms::Type_fr_navsat_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_fender_link::Type_fr_base_link_X_fr_rear_fender_link()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = -1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = -1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = -1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_fender_link& MotionTransforms::Type_fr_base_link_X_fr_rear_fender_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_rear_fender_link_X_fr_base_link::Type_fr_rear_fender_link_X_fr_base_link()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = -1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = -1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = -1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_rear_fender_link_X_fr_base_link& MotionTransforms::Type_fr_rear_fender_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM::Type_fr_base_link_X_fr_rear_left_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,1) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM& MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = sin_q_rear_left_wheel;
    (*this)(2,0) = -sin_q_rear_left_wheel;
    (*this)(2,2) = cos_q_rear_left_wheel;
    (*this)(3,0) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,2) =  ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,5) = sin_q_rear_left_wheel;
    (*this)(4,0) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(4,2) = ( tz_rear_left_wheel * sin_q_rear_left_wheel)-( tx_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(5,0) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(5,2) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(5,3) = -sin_q_rear_left_wheel;
    (*this)(5,5) = cos_q_rear_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link::Type_fr_rear_left_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link& MotionTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = -sin_q_rear_left_wheel;
    (*this)(2,0) = sin_q_rear_left_wheel;
    (*this)(2,2) = cos_q_rear_left_wheel;
    (*this)(3,0) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,1) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(3,2) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,5) = -sin_q_rear_left_wheel;
    (*this)(5,0) =  ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(5,1) = ( tz_rear_left_wheel * sin_q_rear_left_wheel)-( tx_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(5,2) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(5,3) = sin_q_rear_left_wheel;
    (*this)(5,5) = cos_q_rear_left_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_mount::Type_fr_base_link_X_fr_rear_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - tz_fr_rear_mount;    // Maxima DSL: -_k__tz_fr_rear_mount
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_fr_rear_mount;    // Maxima DSL: _k__tz_fr_rear_mount
    (*this)(4,1) = 0.0;
    (*this)(4,2) = - tx_fr_rear_mount;    // Maxima DSL: -_k__tx_fr_rear_mount
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) =  tx_fr_rear_mount;    // Maxima DSL: _k__tx_fr_rear_mount
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_mount& MotionTransforms::Type_fr_base_link_X_fr_rear_mount::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_rear_mount_X_fr_base_link::Type_fr_rear_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) =  tz_fr_rear_mount;    // Maxima DSL: _k__tz_fr_rear_mount
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = - tz_fr_rear_mount;    // Maxima DSL: -_k__tz_fr_rear_mount
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_fr_rear_mount;    // Maxima DSL: _k__tx_fr_rear_mount
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = - tx_fr_rear_mount;    // Maxima DSL: -_k__tx_fr_rear_mount
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const MotionTransforms::Type_fr_rear_mount_X_fr_base_link& MotionTransforms::Type_fr_rear_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM::Type_fr_base_link_X_fr_rear_right_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,1) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(3,4) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,1) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM& MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = sin_q_rear_right_wheel;
    (*this)(2,0) = -sin_q_rear_right_wheel;
    (*this)(2,2) = cos_q_rear_right_wheel;
    (*this)(3,0) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,2) =  ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,5) = sin_q_rear_right_wheel;
    (*this)(4,0) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(4,2) = ( tz_rear_right_wheel * sin_q_rear_right_wheel)-( tx_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(5,0) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(5,2) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(5,3) = -sin_q_rear_right_wheel;
    (*this)(5,5) = cos_q_rear_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link::Type_fr_rear_right_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(4,1) = 0.0;
    (*this)(4,2) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,4) = 0.0;
}

const MotionTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link& MotionTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = -sin_q_rear_right_wheel;
    (*this)(2,0) = sin_q_rear_right_wheel;
    (*this)(2,2) = cos_q_rear_right_wheel;
    (*this)(3,0) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,1) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(3,2) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,5) = -sin_q_rear_right_wheel;
    (*this)(5,0) =  ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(5,1) = ( tz_rear_right_wheel * sin_q_rear_right_wheel)-( tx_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(5,2) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(5,3) = sin_q_rear_right_wheel;
    (*this)(5,5) = cos_q_rear_right_wheel;
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel::Type_fr_base_link_X_fr_front_left_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - ty_front_left_wheel;    // Maxima DSL: -_k__ty_front_left_wheel
    (*this)(3,2) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_front_left_wheel;    // Maxima DSL: _k__tz_front_left_wheel
    (*this)(4,1) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = - ty_front_left_wheel;    // Maxima DSL: -_k__ty_front_left_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel& MotionTransforms::Type_fr_base_link_X_fr_front_left_wheel::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel::Type_fr_base_link_X_fr_front_right_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - ty_front_right_wheel;    // Maxima DSL: -_k__ty_front_right_wheel
    (*this)(3,2) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_front_right_wheel;    // Maxima DSL: _k__tz_front_right_wheel
    (*this)(4,1) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = - ty_front_right_wheel;    // Maxima DSL: -_k__ty_front_right_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel& MotionTransforms::Type_fr_base_link_X_fr_front_right_wheel::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel::Type_fr_base_link_X_fr_rear_left_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - ty_rear_left_wheel;    // Maxima DSL: -_k__ty_rear_left_wheel
    (*this)(3,2) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_rear_left_wheel;    // Maxima DSL: _k__tz_rear_left_wheel
    (*this)(4,1) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = - ty_rear_left_wheel;    // Maxima DSL: -_k__ty_rear_left_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel& MotionTransforms::Type_fr_base_link_X_fr_rear_left_wheel::update(const state_t& q)
{
    return *this;
}
MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel::Type_fr_base_link_X_fr_rear_right_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = - ty_rear_right_wheel;    // Maxima DSL: -_k__ty_rear_right_wheel
    (*this)(3,2) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) =  tz_rear_right_wheel;    // Maxima DSL: _k__tz_rear_right_wheel
    (*this)(4,1) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = - ty_rear_right_wheel;    // Maxima DSL: -_k__ty_rear_right_wheel
    (*this)(5,1) = 0.0;
    (*this)(5,2) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel& MotionTransforms::Type_fr_base_link_X_fr_rear_right_wheel::update(const state_t& q)
{
    return *this;
}

ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel_link::Type_fr_base_link_X_fr_front_left_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel_link& ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,1) = -sin_q_front_left_wheel;
    (*this)(0,3) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(0,4) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(1,3) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(1,4) = ( tx_front_left_wheel * cos_q_front_left_wheel)-( tz_front_left_wheel * sin_q_front_left_wheel);
    (*this)(2,0) = -sin_q_front_left_wheel;
    (*this)(2,1) = -cos_q_front_left_wheel;
    (*this)(2,3) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(2,4) =  ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,4) = -sin_q_front_left_wheel;
    (*this)(5,3) = -sin_q_front_left_wheel;
    (*this)(5,4) = -cos_q_front_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_front_left_wheel_link_X_fr_base_link::Type_fr_front_left_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_front_left_wheel_link_X_fr_base_link& ForceTransforms::Type_fr_front_left_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = -sin_q_front_left_wheel;
    (*this)(0,3) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(0,4) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(0,5) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(1,0) = -sin_q_front_left_wheel;
    (*this)(1,2) = -cos_q_front_left_wheel;
    (*this)(1,3) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(1,4) = ( tx_front_left_wheel * cos_q_front_left_wheel)-( tz_front_left_wheel * sin_q_front_left_wheel);
    (*this)(1,5) =  ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,5) = -sin_q_front_left_wheel;
    (*this)(4,3) = -sin_q_front_left_wheel;
    (*this)(4,5) = -cos_q_front_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel_link::Type_fr_base_link_X_fr_front_right_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel_link& ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,1) = -sin_q_front_right_wheel;
    (*this)(0,3) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(0,4) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(1,3) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(1,4) = ( tx_front_right_wheel * cos_q_front_right_wheel)-( tz_front_right_wheel * sin_q_front_right_wheel);
    (*this)(2,0) = -sin_q_front_right_wheel;
    (*this)(2,1) = -cos_q_front_right_wheel;
    (*this)(2,3) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(2,4) =  ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,4) = -sin_q_front_right_wheel;
    (*this)(5,3) = -sin_q_front_right_wheel;
    (*this)(5,4) = -cos_q_front_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_front_right_wheel_link_X_fr_base_link::Type_fr_front_right_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_front_right_wheel_link_X_fr_base_link& ForceTransforms::Type_fr_front_right_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = -sin_q_front_right_wheel;
    (*this)(0,3) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(0,4) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(0,5) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(1,0) = -sin_q_front_right_wheel;
    (*this)(1,2) = -cos_q_front_right_wheel;
    (*this)(1,3) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(1,4) = ( tx_front_right_wheel * cos_q_front_right_wheel)-( tz_front_right_wheel * sin_q_front_right_wheel);
    (*this)(1,5) =  ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,5) = -sin_q_front_right_wheel;
    (*this)(4,3) = -sin_q_front_right_wheel;
    (*this)(4,5) = -cos_q_front_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link::Type_fr_base_link_X_fr_rear_left_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link& ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,1) = -sin_q_rear_left_wheel;
    (*this)(0,3) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(0,4) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(1,3) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(1,4) = ( tx_rear_left_wheel * cos_q_rear_left_wheel)-( tz_rear_left_wheel * sin_q_rear_left_wheel);
    (*this)(2,0) = -sin_q_rear_left_wheel;
    (*this)(2,1) = -cos_q_rear_left_wheel;
    (*this)(2,3) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(2,4) =  ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,4) = -sin_q_rear_left_wheel;
    (*this)(5,3) = -sin_q_rear_left_wheel;
    (*this)(5,4) = -cos_q_rear_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link::Type_fr_rear_left_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link& ForceTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = -sin_q_rear_left_wheel;
    (*this)(0,3) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(0,4) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(0,5) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(1,0) = -sin_q_rear_left_wheel;
    (*this)(1,2) = -cos_q_rear_left_wheel;
    (*this)(1,3) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(1,4) = ( tx_rear_left_wheel * cos_q_rear_left_wheel)-( tz_rear_left_wheel * sin_q_rear_left_wheel);
    (*this)(1,5) =  ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,5) = -sin_q_rear_left_wheel;
    (*this)(4,3) = -sin_q_rear_left_wheel;
    (*this)(4,5) = -cos_q_rear_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link::Type_fr_base_link_X_fr_rear_right_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,5) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,5) = 0.0;
    (*this)(2,2) = 0.0;
    (*this)(2,5) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link& ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,1) = -sin_q_rear_right_wheel;
    (*this)(0,3) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(0,4) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(1,3) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(1,4) = ( tx_rear_right_wheel * cos_q_rear_right_wheel)-( tz_rear_right_wheel * sin_q_rear_right_wheel);
    (*this)(2,0) = -sin_q_rear_right_wheel;
    (*this)(2,1) = -cos_q_rear_right_wheel;
    (*this)(2,3) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(2,4) =  ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,4) = -sin_q_rear_right_wheel;
    (*this)(5,3) = -sin_q_rear_right_wheel;
    (*this)(5,4) = -cos_q_rear_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link::Type_fr_rear_right_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link& ForceTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = -sin_q_rear_right_wheel;
    (*this)(0,3) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(0,4) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(0,5) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(1,0) = -sin_q_rear_right_wheel;
    (*this)(1,2) = -cos_q_rear_right_wheel;
    (*this)(1,3) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(1,4) = ( tx_rear_right_wheel * cos_q_rear_right_wheel)-( tz_rear_right_wheel * sin_q_rear_right_wheel);
    (*this)(1,5) =  ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,5) = -sin_q_rear_right_wheel;
    (*this)(4,3) = -sin_q_rear_right_wheel;
    (*this)(4,5) = -cos_q_rear_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_base_link_COM::Type_fr_base_link_X_fr_base_link_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_fr_base_link_COM;    // Maxima DSL: -_k__tz_fr_base_link_COM
    (*this)(0,5) =  ty_fr_base_link_COM;    // Maxima DSL: _k__ty_fr_base_link_COM
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_fr_base_link_COM;    // Maxima DSL: _k__tz_fr_base_link_COM
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_fr_base_link_COM;    // Maxima DSL: -_k__tx_fr_base_link_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - ty_fr_base_link_COM;    // Maxima DSL: -_k__ty_fr_base_link_COM
    (*this)(2,4) =  tx_fr_base_link_COM;    // Maxima DSL: _k__tx_fr_base_link_COM
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_base_link_COM& ForceTransforms::Type_fr_base_link_X_fr_base_link_COM::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_COM_X_fr_base_link::Type_fr_base_link_COM_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) =  tz_fr_base_link_COM;    // Maxima DSL: _k__tz_fr_base_link_COM
    (*this)(0,5) = - ty_fr_base_link_COM;    // Maxima DSL: -_k__ty_fr_base_link_COM
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_fr_base_link_COM;    // Maxima DSL: -_k__tz_fr_base_link_COM
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_fr_base_link_COM;    // Maxima DSL: _k__tx_fr_base_link_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_fr_base_link_COM;    // Maxima DSL: _k__ty_fr_base_link_COM
    (*this)(2,4) = - tx_fr_base_link_COM;    // Maxima DSL: -_k__tx_fr_base_link_COM
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_COM_X_fr_base_link& ForceTransforms::Type_fr_base_link_COM_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_chassis_link::Type_fr_base_link_X_fr_chassis_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_chassis_link& ForceTransforms::Type_fr_base_link_X_fr_chassis_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_chassis_link_X_fr_base_link::Type_fr_chassis_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_chassis_link_X_fr_base_link& ForceTransforms::Type_fr_chassis_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_fender_link::Type_fr_base_link_X_fr_front_fender_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_fender_link& ForceTransforms::Type_fr_base_link_X_fr_front_fender_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_front_fender_link_X_fr_base_link::Type_fr_front_fender_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_front_fender_link_X_fr_base_link& ForceTransforms::Type_fr_front_fender_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM::Type_fr_base_link_X_fr_front_left_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,4) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM& ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = sin_q_front_left_wheel;
    (*this)(0,3) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(0,5) =  ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(1,3) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(1,5) = ( tz_front_left_wheel * sin_q_front_left_wheel)-( tx_front_left_wheel * cos_q_front_left_wheel);
    (*this)(2,0) = -sin_q_front_left_wheel;
    (*this)(2,2) = cos_q_front_left_wheel;
    (*this)(2,3) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(2,5) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,5) = sin_q_front_left_wheel;
    (*this)(5,3) = -sin_q_front_left_wheel;
    (*this)(5,5) = cos_q_front_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link::Type_fr_front_left_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link& ForceTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = -sin_q_front_left_wheel;
    (*this)(0,3) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(0,4) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    (*this)(0,5) = - ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(2,0) = sin_q_front_left_wheel;
    (*this)(2,2) = cos_q_front_left_wheel;
    (*this)(2,3) =  ty_front_left_wheel * cos_q_front_left_wheel;
    (*this)(2,4) = ( tz_front_left_wheel * sin_q_front_left_wheel)-( tx_front_left_wheel * cos_q_front_left_wheel);
    (*this)(2,5) = - ty_front_left_wheel * sin_q_front_left_wheel;
    (*this)(3,3) = cos_q_front_left_wheel;
    (*this)(3,5) = -sin_q_front_left_wheel;
    (*this)(5,3) = sin_q_front_left_wheel;
    (*this)(5,5) = cos_q_front_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_mount::Type_fr_base_link_X_fr_front_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_fr_front_mount;    // Maxima DSL: -_k__tz_fr_front_mount
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_fr_front_mount;    // Maxima DSL: _k__tz_fr_front_mount
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_fr_front_mount;    // Maxima DSL: -_k__tx_fr_front_mount
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_fr_front_mount;    // Maxima DSL: _k__tx_fr_front_mount
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_mount& ForceTransforms::Type_fr_base_link_X_fr_front_mount::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_front_mount_X_fr_base_link::Type_fr_front_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) =  tz_fr_front_mount;    // Maxima DSL: _k__tz_fr_front_mount
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_fr_front_mount;    // Maxima DSL: -_k__tz_fr_front_mount
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_fr_front_mount;    // Maxima DSL: _k__tx_fr_front_mount
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_fr_front_mount;    // Maxima DSL: -_k__tx_fr_front_mount
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_front_mount_X_fr_base_link& ForceTransforms::Type_fr_front_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM::Type_fr_base_link_X_fr_front_right_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,4) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM& ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = sin_q_front_right_wheel;
    (*this)(0,3) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(0,5) =  ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(1,3) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(1,5) = ( tz_front_right_wheel * sin_q_front_right_wheel)-( tx_front_right_wheel * cos_q_front_right_wheel);
    (*this)(2,0) = -sin_q_front_right_wheel;
    (*this)(2,2) = cos_q_front_right_wheel;
    (*this)(2,3) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(2,5) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,5) = sin_q_front_right_wheel;
    (*this)(5,3) = -sin_q_front_right_wheel;
    (*this)(5,5) = cos_q_front_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link::Type_fr_front_right_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link& ForceTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = -sin_q_front_right_wheel;
    (*this)(0,3) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(0,4) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    (*this)(0,5) = - ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(2,0) = sin_q_front_right_wheel;
    (*this)(2,2) = cos_q_front_right_wheel;
    (*this)(2,3) =  ty_front_right_wheel * cos_q_front_right_wheel;
    (*this)(2,4) = ( tz_front_right_wheel * sin_q_front_right_wheel)-( tx_front_right_wheel * cos_q_front_right_wheel);
    (*this)(2,5) = - ty_front_right_wheel * sin_q_front_right_wheel;
    (*this)(3,3) = cos_q_front_right_wheel;
    (*this)(3,5) = -sin_q_front_right_wheel;
    (*this)(5,3) = sin_q_front_right_wheel;
    (*this)(5,5) = cos_q_front_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_imu_link::Type_fr_base_link_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_imu_link& ForceTransforms::Type_fr_base_link_X_fr_imu_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_imu_link_X_fr_base_link::Type_fr_imu_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_imu_link_X_fr_base_link& ForceTransforms::Type_fr_imu_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_mid_mount::Type_fr_base_link_X_fr_mid_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_fr_mid_mount;    // Maxima DSL: -_k__tz_fr_mid_mount
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_fr_mid_mount;    // Maxima DSL: _k__tz_fr_mid_mount
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_mid_mount& ForceTransforms::Type_fr_base_link_X_fr_mid_mount::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_mid_mount_X_fr_base_link::Type_fr_mid_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) =  tz_fr_mid_mount;    // Maxima DSL: _k__tz_fr_mid_mount
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_fr_mid_mount;    // Maxima DSL: -_k__tz_fr_mid_mount
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_mid_mount_X_fr_base_link& ForceTransforms::Type_fr_mid_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_navsat_link::Type_fr_base_link_X_fr_navsat_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_fr_navsat_link;    // Maxima DSL: -_k__tz_fr_navsat_link
    (*this)(0,5) =  ty_fr_navsat_link;    // Maxima DSL: _k__ty_fr_navsat_link
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_fr_navsat_link;    // Maxima DSL: _k__tz_fr_navsat_link
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_fr_navsat_link;    // Maxima DSL: -_k__tx_fr_navsat_link
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - ty_fr_navsat_link;    // Maxima DSL: -_k__ty_fr_navsat_link
    (*this)(2,4) =  tx_fr_navsat_link;    // Maxima DSL: _k__tx_fr_navsat_link
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_navsat_link& ForceTransforms::Type_fr_base_link_X_fr_navsat_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_navsat_link_X_fr_base_link::Type_fr_navsat_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) =  tz_fr_navsat_link;    // Maxima DSL: _k__tz_fr_navsat_link
    (*this)(0,5) = - ty_fr_navsat_link;    // Maxima DSL: -_k__ty_fr_navsat_link
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_fr_navsat_link;    // Maxima DSL: -_k__tz_fr_navsat_link
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_fr_navsat_link;    // Maxima DSL: _k__tx_fr_navsat_link
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  ty_fr_navsat_link;    // Maxima DSL: _k__ty_fr_navsat_link
    (*this)(2,4) = - tx_fr_navsat_link;    // Maxima DSL: -_k__tx_fr_navsat_link
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_navsat_link_X_fr_base_link& ForceTransforms::Type_fr_navsat_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_fender_link::Type_fr_base_link_X_fr_rear_fender_link()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = -1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = -1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = -1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_fender_link& ForceTransforms::Type_fr_base_link_X_fr_rear_fender_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_rear_fender_link_X_fr_base_link::Type_fr_rear_fender_link_X_fr_base_link()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = 0.0;
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = -1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = 0.0;
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = -1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = -1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_rear_fender_link_X_fr_base_link& ForceTransforms::Type_fr_rear_fender_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM::Type_fr_base_link_X_fr_rear_left_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,4) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM& ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = sin_q_rear_left_wheel;
    (*this)(0,3) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(0,5) =  ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(1,3) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(1,5) = ( tz_rear_left_wheel * sin_q_rear_left_wheel)-( tx_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(2,0) = -sin_q_rear_left_wheel;
    (*this)(2,2) = cos_q_rear_left_wheel;
    (*this)(2,3) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(2,5) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,5) = sin_q_rear_left_wheel;
    (*this)(5,3) = -sin_q_rear_left_wheel;
    (*this)(5,5) = cos_q_rear_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link::Type_fr_rear_left_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link& ForceTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = -sin_q_rear_left_wheel;
    (*this)(0,3) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(0,4) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(0,5) = - ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(2,0) = sin_q_rear_left_wheel;
    (*this)(2,2) = cos_q_rear_left_wheel;
    (*this)(2,3) =  ty_rear_left_wheel * cos_q_rear_left_wheel;
    (*this)(2,4) = ( tz_rear_left_wheel * sin_q_rear_left_wheel)-( tx_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(2,5) = - ty_rear_left_wheel * sin_q_rear_left_wheel;
    (*this)(3,3) = cos_q_rear_left_wheel;
    (*this)(3,5) = -sin_q_rear_left_wheel;
    (*this)(5,3) = sin_q_rear_left_wheel;
    (*this)(5,5) = cos_q_rear_left_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_mount::Type_fr_base_link_X_fr_rear_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - tz_fr_rear_mount;    // Maxima DSL: -_k__tz_fr_rear_mount
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  tz_fr_rear_mount;    // Maxima DSL: _k__tz_fr_rear_mount
    (*this)(1,4) = 0.0;
    (*this)(1,5) = - tx_fr_rear_mount;    // Maxima DSL: -_k__tx_fr_rear_mount
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) =  tx_fr_rear_mount;    // Maxima DSL: _k__tx_fr_rear_mount
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_mount& ForceTransforms::Type_fr_base_link_X_fr_rear_mount::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_rear_mount_X_fr_base_link::Type_fr_rear_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) =  tz_fr_rear_mount;    // Maxima DSL: _k__tz_fr_rear_mount
    (*this)(0,5) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_fr_rear_mount;    // Maxima DSL: -_k__tz_fr_rear_mount
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_fr_rear_mount;    // Maxima DSL: _k__tx_fr_rear_mount
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(2,4) = - tx_fr_rear_mount;    // Maxima DSL: -_k__tx_fr_rear_mount
    (*this)(2,5) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = 0.0;
    (*this)(5,5) = 1.0;
}

const ForceTransforms::Type_fr_rear_mount_X_fr_base_link& ForceTransforms::Type_fr_rear_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM::Type_fr_base_link_X_fr_rear_right_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,4) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,4) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,4) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM& ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = sin_q_rear_right_wheel;
    (*this)(0,3) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(0,5) =  ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(1,3) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(1,5) = ( tz_rear_right_wheel * sin_q_rear_right_wheel)-( tx_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(2,0) = -sin_q_rear_right_wheel;
    (*this)(2,2) = cos_q_rear_right_wheel;
    (*this)(2,3) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(2,5) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,5) = sin_q_rear_right_wheel;
    (*this)(5,3) = -sin_q_rear_right_wheel;
    (*this)(5,5) = cos_q_rear_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link::Type_fr_rear_right_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(1,4) = 0.0;
    (*this)(1,5) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,4) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 1.0;
    (*this)(4,5) = 0.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,4) = 0.0;
}

const ForceTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link& ForceTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = -sin_q_rear_right_wheel;
    (*this)(0,3) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(0,4) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(0,5) = - ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(2,0) = sin_q_rear_right_wheel;
    (*this)(2,2) = cos_q_rear_right_wheel;
    (*this)(2,3) =  ty_rear_right_wheel * cos_q_rear_right_wheel;
    (*this)(2,4) = ( tz_rear_right_wheel * sin_q_rear_right_wheel)-( tx_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(2,5) = - ty_rear_right_wheel * sin_q_rear_right_wheel;
    (*this)(3,3) = cos_q_rear_right_wheel;
    (*this)(3,5) = -sin_q_rear_right_wheel;
    (*this)(5,3) = sin_q_rear_right_wheel;
    (*this)(5,5) = cos_q_rear_right_wheel;
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel::Type_fr_base_link_X_fr_front_left_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - ty_front_left_wheel;    // Maxima DSL: -_k__ty_front_left_wheel
    (*this)(0,5) = - tz_front_left_wheel;    // Maxima DSL: -_k__tz_front_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  tz_front_left_wheel;    // Maxima DSL: _k__tz_front_left_wheel
    (*this)(1,4) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_front_left_wheel;    // Maxima DSL: -_k__ty_front_left_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel& ForceTransforms::Type_fr_base_link_X_fr_front_left_wheel::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel::Type_fr_base_link_X_fr_front_right_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - ty_front_right_wheel;    // Maxima DSL: -_k__ty_front_right_wheel
    (*this)(0,5) = - tz_front_right_wheel;    // Maxima DSL: -_k__tz_front_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  tz_front_right_wheel;    // Maxima DSL: _k__tz_front_right_wheel
    (*this)(1,4) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_front_right_wheel;    // Maxima DSL: -_k__ty_front_right_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel& ForceTransforms::Type_fr_base_link_X_fr_front_right_wheel::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel::Type_fr_base_link_X_fr_rear_left_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - ty_rear_left_wheel;    // Maxima DSL: -_k__ty_rear_left_wheel
    (*this)(0,5) = - tz_rear_left_wheel;    // Maxima DSL: -_k__tz_rear_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  tz_rear_left_wheel;    // Maxima DSL: _k__tz_rear_left_wheel
    (*this)(1,4) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_rear_left_wheel;    // Maxima DSL: -_k__ty_rear_left_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel& ForceTransforms::Type_fr_base_link_X_fr_rear_left_wheel::update(const state_t& q)
{
    return *this;
}
ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel::Type_fr_base_link_X_fr_rear_right_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(0,4) = - ty_rear_right_wheel;    // Maxima DSL: -_k__ty_rear_right_wheel
    (*this)(0,5) = - tz_rear_right_wheel;    // Maxima DSL: -_k__tz_rear_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  tz_rear_right_wheel;    // Maxima DSL: _k__tz_rear_right_wheel
    (*this)(1,4) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(1,5) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_rear_right_wheel;    // Maxima DSL: -_k__ty_rear_right_wheel
    (*this)(2,4) = 0.0;
    (*this)(2,5) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
    (*this)(3,4) = 0.0;
    (*this)(3,5) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(4,1) = 0.0;
    (*this)(4,2) = 0.0;
    (*this)(4,3) = 0.0;
    (*this)(4,4) = 0.0;
    (*this)(4,5) = 1.0;
    (*this)(5,0) = 0.0;
    (*this)(5,1) = 0.0;
    (*this)(5,2) = 0.0;
    (*this)(5,3) = 0.0;
    (*this)(5,4) = -1.0;
    (*this)(5,5) = 0.0;
}

const ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel& ForceTransforms::Type_fr_base_link_X_fr_rear_right_wheel::update(const state_t& q)
{
    return *this;
}

HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel_link::Type_fr_base_link_X_fr_front_left_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_front_left_wheel;    // Maxima DSL: _k__ty_front_left_wheel
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_front_left_wheel;    // Maxima DSL: _k__tz_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel_link& HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,1) = -sin_q_front_left_wheel;
    (*this)(2,0) = -sin_q_front_left_wheel;
    (*this)(2,1) = -cos_q_front_left_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_front_left_wheel_link_X_fr_base_link::Type_fr_front_left_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_front_left_wheel;    // Maxima DSL: -_k__ty_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_front_left_wheel_link_X_fr_base_link& HomogeneousTransforms::Type_fr_front_left_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = -sin_q_front_left_wheel;
    (*this)(0,3) = ( tz_front_left_wheel * sin_q_front_left_wheel)-( tx_front_left_wheel * cos_q_front_left_wheel);
    (*this)(1,0) = -sin_q_front_left_wheel;
    (*this)(1,2) = -cos_q_front_left_wheel;
    (*this)(1,3) = ( tx_front_left_wheel * sin_q_front_left_wheel)+( tz_front_left_wheel * cos_q_front_left_wheel);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel_link::Type_fr_base_link_X_fr_front_right_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_front_right_wheel;    // Maxima DSL: _k__ty_front_right_wheel
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_front_right_wheel;    // Maxima DSL: _k__tz_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel_link& HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,1) = -sin_q_front_right_wheel;
    (*this)(2,0) = -sin_q_front_right_wheel;
    (*this)(2,1) = -cos_q_front_right_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_front_right_wheel_link_X_fr_base_link::Type_fr_front_right_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_front_right_wheel;    // Maxima DSL: -_k__ty_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_front_right_wheel_link_X_fr_base_link& HomogeneousTransforms::Type_fr_front_right_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = -sin_q_front_right_wheel;
    (*this)(0,3) = ( tz_front_right_wheel * sin_q_front_right_wheel)-( tx_front_right_wheel * cos_q_front_right_wheel);
    (*this)(1,0) = -sin_q_front_right_wheel;
    (*this)(1,2) = -cos_q_front_right_wheel;
    (*this)(1,3) = ( tx_front_right_wheel * sin_q_front_right_wheel)+( tz_front_right_wheel * cos_q_front_right_wheel);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link::Type_fr_base_link_X_fr_rear_left_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_rear_left_wheel;    // Maxima DSL: _k__ty_rear_left_wheel
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_rear_left_wheel;    // Maxima DSL: _k__tz_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,1) = -sin_q_rear_left_wheel;
    (*this)(2,0) = -sin_q_rear_left_wheel;
    (*this)(2,1) = -cos_q_rear_left_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link::Type_fr_rear_left_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_rear_left_wheel;    // Maxima DSL: -_k__ty_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link& HomogeneousTransforms::Type_fr_rear_left_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = -sin_q_rear_left_wheel;
    (*this)(0,3) = ( tz_rear_left_wheel * sin_q_rear_left_wheel)-( tx_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(1,0) = -sin_q_rear_left_wheel;
    (*this)(1,2) = -cos_q_rear_left_wheel;
    (*this)(1,3) = ( tx_rear_left_wheel * sin_q_rear_left_wheel)+( tz_rear_left_wheel * cos_q_rear_left_wheel);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link::Type_fr_base_link_X_fr_rear_right_wheel_link()
{
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_rear_right_wheel;    // Maxima DSL: _k__ty_rear_right_wheel
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_rear_right_wheel;    // Maxima DSL: _k__tz_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,1) = -sin_q_rear_right_wheel;
    (*this)(2,0) = -sin_q_rear_right_wheel;
    (*this)(2,1) = -cos_q_rear_right_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link::Type_fr_rear_right_wheel_link_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) = - ty_rear_right_wheel;    // Maxima DSL: -_k__ty_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link& HomogeneousTransforms::Type_fr_rear_right_wheel_link_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = -sin_q_rear_right_wheel;
    (*this)(0,3) = ( tz_rear_right_wheel * sin_q_rear_right_wheel)-( tx_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(1,0) = -sin_q_rear_right_wheel;
    (*this)(1,2) = -cos_q_rear_right_wheel;
    (*this)(1,3) = ( tx_rear_right_wheel * sin_q_rear_right_wheel)+( tz_rear_right_wheel * cos_q_rear_right_wheel);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_base_link_COM::Type_fr_base_link_X_fr_base_link_COM()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_fr_base_link_COM;    // Maxima DSL: _k__tx_fr_base_link_COM
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_fr_base_link_COM;    // Maxima DSL: _k__ty_fr_base_link_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_fr_base_link_COM;    // Maxima DSL: _k__tz_fr_base_link_COM
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_base_link_COM& HomogeneousTransforms::Type_fr_base_link_X_fr_base_link_COM::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_COM_X_fr_base_link::Type_fr_base_link_COM_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_fr_base_link_COM;    // Maxima DSL: -_k__tx_fr_base_link_COM
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_fr_base_link_COM;    // Maxima DSL: -_k__ty_fr_base_link_COM
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_fr_base_link_COM;    // Maxima DSL: -_k__tz_fr_base_link_COM
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_COM_X_fr_base_link& HomogeneousTransforms::Type_fr_base_link_COM_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_chassis_link::Type_fr_base_link_X_fr_chassis_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_chassis_link& HomogeneousTransforms::Type_fr_base_link_X_fr_chassis_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_chassis_link_X_fr_base_link::Type_fr_chassis_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_chassis_link_X_fr_base_link& HomogeneousTransforms::Type_fr_chassis_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_front_fender_link::Type_fr_base_link_X_fr_front_fender_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_fender_link& HomogeneousTransforms::Type_fr_base_link_X_fr_front_fender_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_front_fender_link_X_fr_base_link::Type_fr_front_fender_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_front_fender_link_X_fr_base_link& HomogeneousTransforms::Type_fr_front_fender_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM::Type_fr_base_link_X_fr_front_left_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_front_left_wheel;    // Maxima DSL: _k__ty_front_left_wheel
    (*this)(2,1) = 0.0;
    (*this)(2,3) =  tz_front_left_wheel;    // Maxima DSL: _k__tz_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM& HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = sin_q_front_left_wheel;
    (*this)(2,0) = -sin_q_front_left_wheel;
    (*this)(2,2) = cos_q_front_left_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link::Type_fr_front_left_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_front_left_wheel;    // Maxima DSL: -_k__ty_front_left_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link& HomogeneousTransforms::Type_fr_front_left_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_left_wheel  = ScalarTraits::sin( q(FRONT_LEFT_WHEEL) );
    Scalar cos_q_front_left_wheel  = ScalarTraits::cos( q(FRONT_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_front_left_wheel;
    (*this)(0,2) = -sin_q_front_left_wheel;
    (*this)(0,3) = ( tz_front_left_wheel * sin_q_front_left_wheel)-( tx_front_left_wheel * cos_q_front_left_wheel);
    (*this)(2,0) = sin_q_front_left_wheel;
    (*this)(2,2) = cos_q_front_left_wheel;
    (*this)(2,3) = (- tx_front_left_wheel * sin_q_front_left_wheel)-( tz_front_left_wheel * cos_q_front_left_wheel);
    return *this;
}

HomogeneousTransforms::Type_fr_base_link_X_fr_front_mount::Type_fr_base_link_X_fr_front_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_fr_front_mount;    // Maxima DSL: _k__tx_fr_front_mount
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_fr_front_mount;    // Maxima DSL: _k__tz_fr_front_mount
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_mount& HomogeneousTransforms::Type_fr_base_link_X_fr_front_mount::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_front_mount_X_fr_base_link::Type_fr_front_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_fr_front_mount;    // Maxima DSL: -_k__tx_fr_front_mount
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_fr_front_mount;    // Maxima DSL: -_k__tz_fr_front_mount
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_front_mount_X_fr_base_link& HomogeneousTransforms::Type_fr_front_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM::Type_fr_base_link_X_fr_front_right_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_front_right_wheel;    // Maxima DSL: _k__ty_front_right_wheel
    (*this)(2,1) = 0.0;
    (*this)(2,3) =  tz_front_right_wheel;    // Maxima DSL: _k__tz_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM& HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = sin_q_front_right_wheel;
    (*this)(2,0) = -sin_q_front_right_wheel;
    (*this)(2,2) = cos_q_front_right_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link::Type_fr_front_right_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_front_right_wheel;    // Maxima DSL: -_k__ty_front_right_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link& HomogeneousTransforms::Type_fr_front_right_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_front_right_wheel  = ScalarTraits::sin( q(FRONT_RIGHT_WHEEL) );
    Scalar cos_q_front_right_wheel  = ScalarTraits::cos( q(FRONT_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_front_right_wheel;
    (*this)(0,2) = -sin_q_front_right_wheel;
    (*this)(0,3) = ( tz_front_right_wheel * sin_q_front_right_wheel)-( tx_front_right_wheel * cos_q_front_right_wheel);
    (*this)(2,0) = sin_q_front_right_wheel;
    (*this)(2,2) = cos_q_front_right_wheel;
    (*this)(2,3) = (- tx_front_right_wheel * sin_q_front_right_wheel)-( tz_front_right_wheel * cos_q_front_right_wheel);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_imu_link::Type_fr_base_link_X_fr_imu_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_imu_link& HomogeneousTransforms::Type_fr_base_link_X_fr_imu_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_imu_link_X_fr_base_link::Type_fr_imu_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_imu_link_X_fr_base_link& HomogeneousTransforms::Type_fr_imu_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_mid_mount::Type_fr_base_link_X_fr_mid_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_fr_mid_mount;    // Maxima DSL: _k__tz_fr_mid_mount
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_mid_mount& HomogeneousTransforms::Type_fr_base_link_X_fr_mid_mount::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_mid_mount_X_fr_base_link::Type_fr_mid_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_fr_mid_mount;    // Maxima DSL: -_k__tz_fr_mid_mount
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_mid_mount_X_fr_base_link& HomogeneousTransforms::Type_fr_mid_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_navsat_link::Type_fr_base_link_X_fr_navsat_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_fr_navsat_link;    // Maxima DSL: _k__tx_fr_navsat_link
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_fr_navsat_link;    // Maxima DSL: _k__ty_fr_navsat_link
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_fr_navsat_link;    // Maxima DSL: _k__tz_fr_navsat_link
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_navsat_link& HomogeneousTransforms::Type_fr_base_link_X_fr_navsat_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_navsat_link_X_fr_base_link::Type_fr_navsat_link_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_fr_navsat_link;    // Maxima DSL: -_k__tx_fr_navsat_link
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_fr_navsat_link;    // Maxima DSL: -_k__ty_fr_navsat_link
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_fr_navsat_link;    // Maxima DSL: -_k__tz_fr_navsat_link
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_navsat_link_X_fr_base_link& HomogeneousTransforms::Type_fr_navsat_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_fender_link::Type_fr_base_link_X_fr_rear_fender_link()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = -1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_fender_link& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_fender_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_rear_fender_link_X_fr_base_link::Type_fr_rear_fender_link_X_fr_base_link()
{
    (*this)(0,0) = -1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = -1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_rear_fender_link_X_fr_base_link& HomogeneousTransforms::Type_fr_rear_fender_link_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM::Type_fr_base_link_X_fr_rear_left_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_rear_left_wheel;    // Maxima DSL: _k__ty_rear_left_wheel
    (*this)(2,1) = 0.0;
    (*this)(2,3) =  tz_rear_left_wheel;    // Maxima DSL: _k__tz_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = sin_q_rear_left_wheel;
    (*this)(2,0) = -sin_q_rear_left_wheel;
    (*this)(2,2) = cos_q_rear_left_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link::Type_fr_rear_left_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_rear_left_wheel;    // Maxima DSL: -_k__ty_rear_left_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link& HomogeneousTransforms::Type_fr_rear_left_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_left_wheel  = ScalarTraits::sin( q(REAR_LEFT_WHEEL) );
    Scalar cos_q_rear_left_wheel  = ScalarTraits::cos( q(REAR_LEFT_WHEEL) );
    (*this)(0,0) = cos_q_rear_left_wheel;
    (*this)(0,2) = -sin_q_rear_left_wheel;
    (*this)(0,3) = ( tz_rear_left_wheel * sin_q_rear_left_wheel)-( tx_rear_left_wheel * cos_q_rear_left_wheel);
    (*this)(2,0) = sin_q_rear_left_wheel;
    (*this)(2,2) = cos_q_rear_left_wheel;
    (*this)(2,3) = (- tx_rear_left_wheel * sin_q_rear_left_wheel)-( tz_rear_left_wheel * cos_q_rear_left_wheel);
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_mount::Type_fr_base_link_X_fr_rear_mount()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_fr_rear_mount;    // Maxima DSL: _k__tx_fr_rear_mount
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) =  tz_fr_rear_mount;    // Maxima DSL: _k__tz_fr_rear_mount
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_mount& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_mount::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_rear_mount_X_fr_base_link::Type_fr_rear_mount_X_fr_base_link()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) = - tx_fr_rear_mount;    // Maxima DSL: -_k__tx_fr_rear_mount
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = 0.0;
    (*this)(2,0) = 0.0;
    (*this)(2,1) = 0.0;
    (*this)(2,2) = 1.0;
    (*this)(2,3) = - tz_fr_rear_mount;    // Maxima DSL: -_k__tz_fr_rear_mount
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_rear_mount_X_fr_base_link& HomogeneousTransforms::Type_fr_rear_mount_X_fr_base_link::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM::Type_fr_base_link_X_fr_rear_right_wheel_link_COM()
{
    (*this)(0,1) = 0.0;
    (*this)(0,3) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) =  ty_rear_right_wheel;    // Maxima DSL: _k__ty_rear_right_wheel
    (*this)(2,1) = 0.0;
    (*this)(2,3) =  tz_rear_right_wheel;    // Maxima DSL: _k__tz_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel_link_COM::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = sin_q_rear_right_wheel;
    (*this)(2,0) = -sin_q_rear_right_wheel;
    (*this)(2,2) = cos_q_rear_right_wheel;
    return *this;
}
HomogeneousTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link::Type_fr_rear_right_wheel_link_COM_X_fr_base_link()
{
    (*this)(0,1) = 0.0;
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 1.0;
    (*this)(1,2) = 0.0;
    (*this)(1,3) = - ty_rear_right_wheel;    // Maxima DSL: -_k__ty_rear_right_wheel
    (*this)(2,1) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link& HomogeneousTransforms::Type_fr_rear_right_wheel_link_COM_X_fr_base_link::update(const state_t& q)
{
    Scalar sin_q_rear_right_wheel  = ScalarTraits::sin( q(REAR_RIGHT_WHEEL) );
    Scalar cos_q_rear_right_wheel  = ScalarTraits::cos( q(REAR_RIGHT_WHEEL) );
    (*this)(0,0) = cos_q_rear_right_wheel;
    (*this)(0,2) = -sin_q_rear_right_wheel;
    (*this)(0,3) = ( tz_rear_right_wheel * sin_q_rear_right_wheel)-( tx_rear_right_wheel * cos_q_rear_right_wheel);
    (*this)(2,0) = sin_q_rear_right_wheel;
    (*this)(2,2) = cos_q_rear_right_wheel;
    (*this)(2,3) = (- tx_rear_right_wheel * sin_q_rear_right_wheel)-( tz_rear_right_wheel * cos_q_rear_right_wheel);
    return *this;
}

HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel::Type_fr_base_link_X_fr_front_left_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_front_left_wheel;    // Maxima DSL: _k__tx_front_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_front_left_wheel;    // Maxima DSL: _k__ty_front_left_wheel
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_front_left_wheel;    // Maxima DSL: _k__tz_front_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel& HomogeneousTransforms::Type_fr_base_link_X_fr_front_left_wheel::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel::Type_fr_base_link_X_fr_front_right_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_front_right_wheel;    // Maxima DSL: _k__tx_front_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_front_right_wheel;    // Maxima DSL: _k__ty_front_right_wheel
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_front_right_wheel;    // Maxima DSL: _k__tz_front_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel& HomogeneousTransforms::Type_fr_base_link_X_fr_front_right_wheel::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel::Type_fr_base_link_X_fr_rear_left_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_rear_left_wheel;    // Maxima DSL: _k__tx_rear_left_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_rear_left_wheel;    // Maxima DSL: _k__ty_rear_left_wheel
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_rear_left_wheel;    // Maxima DSL: _k__tz_rear_left_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_left_wheel::update(const state_t& q)
{
    return *this;
}
HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel::Type_fr_base_link_X_fr_rear_right_wheel()
{
    (*this)(0,0) = 1.0;
    (*this)(0,1) = 0.0;
    (*this)(0,2) = 0.0;
    (*this)(0,3) =  tx_rear_right_wheel;    // Maxima DSL: _k__tx_rear_right_wheel
    (*this)(1,0) = 0.0;
    (*this)(1,1) = 0.0;
    (*this)(1,2) = 1.0;
    (*this)(1,3) =  ty_rear_right_wheel;    // Maxima DSL: _k__ty_rear_right_wheel
    (*this)(2,0) = 0.0;
    (*this)(2,1) = -1.0;
    (*this)(2,2) = 0.0;
    (*this)(2,3) =  tz_rear_right_wheel;    // Maxima DSL: _k__tz_rear_right_wheel
    (*this)(3,0) = 0.0;
    (*this)(3,1) = 0.0;
    (*this)(3,2) = 0.0;
    (*this)(3,3) = 1.0;
}

const HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel& HomogeneousTransforms::Type_fr_base_link_X_fr_rear_right_wheel::update(const state_t& q)
{
    return *this;
}


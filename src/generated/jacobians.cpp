#include "jacobians.h"

Jackal::rcg::Jacobians::Jacobians()
:    fr_base_link_J_fr_front_left_wheel_link(), 
    fr_base_link_J_fr_front_right_wheel_link(), 
    fr_base_link_J_fr_rear_left_wheel_link(), 
    fr_base_link_J_fr_rear_right_wheel_link(), 
    fr_base_link_J_fr_front_left_wheel_link_COM(), 
    fr_base_link_J_fr_front_right_wheel_link_COM(), 
    fr_base_link_J_fr_rear_left_wheel_link_COM(), 
    fr_base_link_J_fr_rear_right_wheel_link_COM()
{}

void Jackal::rcg::Jacobians::updateParameters(const Params_lengths& _lengths, const Params_angles& _angles)
{
    params.lengths = _lengths;
    params.angles = _angles;
    params.trig.update();
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_left_wheel_link::Type_fr_base_link_J_fr_front_left_wheel_link()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_left_wheel_link& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_left_wheel_link::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_right_wheel_link::Type_fr_base_link_J_fr_front_right_wheel_link()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_right_wheel_link& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_right_wheel_link::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_left_wheel_link::Type_fr_base_link_J_fr_rear_left_wheel_link()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_left_wheel_link& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_left_wheel_link::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_right_wheel_link::Type_fr_base_link_J_fr_rear_right_wheel_link()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_right_wheel_link& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_right_wheel_link::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_left_wheel_link_COM::Type_fr_base_link_J_fr_front_left_wheel_link_COM()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_left_wheel_link_COM& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_left_wheel_link_COM::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_right_wheel_link_COM::Type_fr_base_link_J_fr_front_right_wheel_link_COM()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_right_wheel_link_COM& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_front_right_wheel_link_COM::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_left_wheel_link_COM::Type_fr_base_link_J_fr_rear_left_wheel_link_COM()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_left_wheel_link_COM& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_left_wheel_link_COM::update(const JointState& q)
{
    return *this;
}

Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_right_wheel_link_COM::Type_fr_base_link_J_fr_rear_right_wheel_link_COM()
{
    (*this)(0,0) = 0.0;
    (*this)(1,0) = 1.0;
    (*this)(2,0) = 0.0;
    (*this)(3,0) = 0.0;
    (*this)(4,0) = 0.0;
    (*this)(5,0) = 0.0;
}

const Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_right_wheel_link_COM& Jackal::rcg::Jacobians::Type_fr_base_link_J_fr_rear_right_wheel_link_COM::update(const JointState& q)
{
    return *this;
}


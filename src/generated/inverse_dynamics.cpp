#include <iit/rbd/robcogen_commons.h>

#include "inverse_dynamics.h"
#include "inertia_properties.h"
#ifndef EIGEN_NO_DEBUG
    #include <iostream>
#endif
using namespace std;
using namespace iit::rbd;
using namespace Jackal::rcg;

// Initialization of static-const data
const Jackal::rcg::InverseDynamics::ExtForces
Jackal::rcg::InverseDynamics::zeroExtForces(Force::Zero());

Jackal::rcg::InverseDynamics::InverseDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( & inertia ),
    xm( & transforms ),
    front_left_wheel_link_I(inertiaProps->getTensor_front_left_wheel_link() ),
    front_right_wheel_link_I(inertiaProps->getTensor_front_right_wheel_link() ),
    rear_left_wheel_link_I(inertiaProps->getTensor_rear_left_wheel_link() ),
    rear_right_wheel_link_I(inertiaProps->getTensor_rear_right_wheel_link() )
    ,
        base_link_I( inertiaProps->getTensor_base_link() ),
        front_left_wheel_link_Ic(front_left_wheel_link_I),
        front_right_wheel_link_Ic(front_right_wheel_link_I),
        rear_left_wheel_link_Ic(rear_left_wheel_link_I),
        rear_right_wheel_link_Ic(rear_right_wheel_link_I)
{
#ifndef EIGEN_NO_DEBUG
    std::cout << "Robot Jackal, InverseDynamics::InverseDynamics()" << std::endl;
    std::cout << "Compiled with Eigen debug active" << std::endl;
#endif
    front_left_wheel_link_v.setZero();
    front_right_wheel_link_v.setZero();
    rear_left_wheel_link_v.setZero();
    rear_right_wheel_link_v.setZero();

    vcross.setZero();
}

void Jackal::rcg::InverseDynamics::id(
    JointState& jForces, Acceleration& base_link_a,
    const Acceleration& g, const Velocity& base_link_v,
    const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    base_link_Ic = base_link_I;

    // First pass, link 'front_left_wheel_link'
    front_left_wheel_link_v = ((xm->fr_front_left_wheel_link_X_fr_base_link) * base_link_v);
    front_left_wheel_link_v(iit::rbd::AZ) += qd(FRONT_LEFT_WHEEL);
    
    motionCrossProductMx<Scalar>(front_left_wheel_link_v, vcross);
    
    front_left_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(FRONT_LEFT_WHEEL));
    front_left_wheel_link_a(iit::rbd::AZ) += qdd(FRONT_LEFT_WHEEL);
    
    front_left_wheel_link_f = front_left_wheel_link_I * front_left_wheel_link_a + vxIv(front_left_wheel_link_v, front_left_wheel_link_I);
    
    // First pass, link 'front_right_wheel_link'
    front_right_wheel_link_v = ((xm->fr_front_right_wheel_link_X_fr_base_link) * base_link_v);
    front_right_wheel_link_v(iit::rbd::AZ) += qd(FRONT_RIGHT_WHEEL);
    
    motionCrossProductMx<Scalar>(front_right_wheel_link_v, vcross);
    
    front_right_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(FRONT_RIGHT_WHEEL));
    front_right_wheel_link_a(iit::rbd::AZ) += qdd(FRONT_RIGHT_WHEEL);
    
    front_right_wheel_link_f = front_right_wheel_link_I * front_right_wheel_link_a + vxIv(front_right_wheel_link_v, front_right_wheel_link_I);
    
    // First pass, link 'rear_left_wheel_link'
    rear_left_wheel_link_v = ((xm->fr_rear_left_wheel_link_X_fr_base_link) * base_link_v);
    rear_left_wheel_link_v(iit::rbd::AZ) += qd(REAR_LEFT_WHEEL);
    
    motionCrossProductMx<Scalar>(rear_left_wheel_link_v, vcross);
    
    rear_left_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(REAR_LEFT_WHEEL));
    rear_left_wheel_link_a(iit::rbd::AZ) += qdd(REAR_LEFT_WHEEL);
    
    rear_left_wheel_link_f = rear_left_wheel_link_I * rear_left_wheel_link_a + vxIv(rear_left_wheel_link_v, rear_left_wheel_link_I);
    
    // First pass, link 'rear_right_wheel_link'
    rear_right_wheel_link_v = ((xm->fr_rear_right_wheel_link_X_fr_base_link) * base_link_v);
    rear_right_wheel_link_v(iit::rbd::AZ) += qd(REAR_RIGHT_WHEEL);
    
    motionCrossProductMx<Scalar>(rear_right_wheel_link_v, vcross);
    
    rear_right_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(REAR_RIGHT_WHEEL));
    rear_right_wheel_link_a(iit::rbd::AZ) += qdd(REAR_RIGHT_WHEEL);
    
    rear_right_wheel_link_f = rear_right_wheel_link_I * rear_right_wheel_link_a + vxIv(rear_right_wheel_link_v, rear_right_wheel_link_I);
    
    // The force exerted on the floating base by the links
    base_link_f = vxIv(base_link_v, base_link_I);
    

    // Add the external forces:
    base_link_f -= fext[BASE_LINK];
    front_left_wheel_link_f -= fext[FRONT_LEFT_WHEEL_LINK];
    front_right_wheel_link_f -= fext[FRONT_RIGHT_WHEEL_LINK];
    rear_left_wheel_link_f -= fext[REAR_LEFT_WHEEL_LINK];
    rear_right_wheel_link_f -= fext[REAR_RIGHT_WHEEL_LINK];

    InertiaMatrix Ic_spare;
    iit::rbd::transformInertia<Scalar>(rear_right_wheel_link_Ic, (xm->fr_rear_right_wheel_link_X_fr_base_link).transpose(), Ic_spare);
    base_link_Ic += Ic_spare;
    base_link_f = base_link_f + (xm->fr_rear_right_wheel_link_X_fr_base_link).transpose() * rear_right_wheel_link_f;
    
    iit::rbd::transformInertia<Scalar>(rear_left_wheel_link_Ic, (xm->fr_rear_left_wheel_link_X_fr_base_link).transpose(), Ic_spare);
    base_link_Ic += Ic_spare;
    base_link_f = base_link_f + (xm->fr_rear_left_wheel_link_X_fr_base_link).transpose() * rear_left_wheel_link_f;
    
    iit::rbd::transformInertia<Scalar>(front_right_wheel_link_Ic, (xm->fr_front_right_wheel_link_X_fr_base_link).transpose(), Ic_spare);
    base_link_Ic += Ic_spare;
    base_link_f = base_link_f + (xm->fr_front_right_wheel_link_X_fr_base_link).transpose() * front_right_wheel_link_f;
    
    iit::rbd::transformInertia<Scalar>(front_left_wheel_link_Ic, (xm->fr_front_left_wheel_link_X_fr_base_link).transpose(), Ic_spare);
    base_link_Ic += Ic_spare;
    base_link_f = base_link_f + (xm->fr_front_left_wheel_link_X_fr_base_link).transpose() * front_left_wheel_link_f;
    

    // The base acceleration due to the force due to the movement of the links
    base_link_a = - base_link_Ic.inverse() * base_link_f;
    
    front_left_wheel_link_a = xm->fr_front_left_wheel_link_X_fr_base_link * base_link_a;
    jForces(FRONT_LEFT_WHEEL) = (front_left_wheel_link_Ic.row(iit::rbd::AZ) * front_left_wheel_link_a + front_left_wheel_link_f(iit::rbd::AZ));
    
    front_right_wheel_link_a = xm->fr_front_right_wheel_link_X_fr_base_link * base_link_a;
    jForces(FRONT_RIGHT_WHEEL) = (front_right_wheel_link_Ic.row(iit::rbd::AZ) * front_right_wheel_link_a + front_right_wheel_link_f(iit::rbd::AZ));
    
    rear_left_wheel_link_a = xm->fr_rear_left_wheel_link_X_fr_base_link * base_link_a;
    jForces(REAR_LEFT_WHEEL) = (rear_left_wheel_link_Ic.row(iit::rbd::AZ) * rear_left_wheel_link_a + rear_left_wheel_link_f(iit::rbd::AZ));
    
    rear_right_wheel_link_a = xm->fr_rear_right_wheel_link_X_fr_base_link * base_link_a;
    jForces(REAR_RIGHT_WHEEL) = (rear_right_wheel_link_Ic.row(iit::rbd::AZ) * rear_right_wheel_link_a + rear_right_wheel_link_f(iit::rbd::AZ));
    

    base_link_a += g;
}


void Jackal::rcg::InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g)
{
    const Acceleration& base_link_a = -g;

    // Link 'front_left_wheel_link'
    front_left_wheel_link_a = (xm->fr_front_left_wheel_link_X_fr_base_link) * base_link_a;
    front_left_wheel_link_f = front_left_wheel_link_I * front_left_wheel_link_a;
    // Link 'front_right_wheel_link'
    front_right_wheel_link_a = (xm->fr_front_right_wheel_link_X_fr_base_link) * base_link_a;
    front_right_wheel_link_f = front_right_wheel_link_I * front_right_wheel_link_a;
    // Link 'rear_left_wheel_link'
    rear_left_wheel_link_a = (xm->fr_rear_left_wheel_link_X_fr_base_link) * base_link_a;
    rear_left_wheel_link_f = rear_left_wheel_link_I * rear_left_wheel_link_a;
    // Link 'rear_right_wheel_link'
    rear_right_wheel_link_a = (xm->fr_rear_right_wheel_link_X_fr_base_link) * base_link_a;
    rear_right_wheel_link_f = rear_right_wheel_link_I * rear_right_wheel_link_a;

    base_link_f = base_link_I * base_link_a;

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

void Jackal::rcg::InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_v, const JointState& qd)
{
    // Link 'front_left_wheel_link'
    front_left_wheel_link_v = ((xm->fr_front_left_wheel_link_X_fr_base_link) * base_link_v);
    front_left_wheel_link_v(iit::rbd::AZ) += qd(FRONT_LEFT_WHEEL);
    motionCrossProductMx<Scalar>(front_left_wheel_link_v, vcross);
    front_left_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(FRONT_LEFT_WHEEL));
    front_left_wheel_link_f = front_left_wheel_link_I * front_left_wheel_link_a + vxIv(front_left_wheel_link_v, front_left_wheel_link_I);
    
    // Link 'front_right_wheel_link'
    front_right_wheel_link_v = ((xm->fr_front_right_wheel_link_X_fr_base_link) * base_link_v);
    front_right_wheel_link_v(iit::rbd::AZ) += qd(FRONT_RIGHT_WHEEL);
    motionCrossProductMx<Scalar>(front_right_wheel_link_v, vcross);
    front_right_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(FRONT_RIGHT_WHEEL));
    front_right_wheel_link_f = front_right_wheel_link_I * front_right_wheel_link_a + vxIv(front_right_wheel_link_v, front_right_wheel_link_I);
    
    // Link 'rear_left_wheel_link'
    rear_left_wheel_link_v = ((xm->fr_rear_left_wheel_link_X_fr_base_link) * base_link_v);
    rear_left_wheel_link_v(iit::rbd::AZ) += qd(REAR_LEFT_WHEEL);
    motionCrossProductMx<Scalar>(rear_left_wheel_link_v, vcross);
    rear_left_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(REAR_LEFT_WHEEL));
    rear_left_wheel_link_f = rear_left_wheel_link_I * rear_left_wheel_link_a + vxIv(rear_left_wheel_link_v, rear_left_wheel_link_I);
    
    // Link 'rear_right_wheel_link'
    rear_right_wheel_link_v = ((xm->fr_rear_right_wheel_link_X_fr_base_link) * base_link_v);
    rear_right_wheel_link_v(iit::rbd::AZ) += qd(REAR_RIGHT_WHEEL);
    motionCrossProductMx<Scalar>(rear_right_wheel_link_v, vcross);
    rear_right_wheel_link_a = (vcross.col(iit::rbd::AZ) * qd(REAR_RIGHT_WHEEL));
    rear_right_wheel_link_f = rear_right_wheel_link_I * rear_right_wheel_link_a + vxIv(rear_right_wheel_link_v, rear_right_wheel_link_I);
    

    base_link_f = vxIv(base_link_v, base_link_I);

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}

void Jackal::rcg::InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    Acceleration base_link_a = baseAccel -g;

    // First pass, link 'front_left_wheel_link'
    front_left_wheel_link_v = ((xm->fr_front_left_wheel_link_X_fr_base_link) * base_link_v);
    front_left_wheel_link_v(iit::rbd::AZ) += qd(FRONT_LEFT_WHEEL);
    
    motionCrossProductMx<Scalar>(front_left_wheel_link_v, vcross);
    
    front_left_wheel_link_a = (xm->fr_front_left_wheel_link_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(FRONT_LEFT_WHEEL);
    front_left_wheel_link_a(iit::rbd::AZ) += qdd(FRONT_LEFT_WHEEL);
    
    front_left_wheel_link_f = front_left_wheel_link_I * front_left_wheel_link_a + vxIv(front_left_wheel_link_v, front_left_wheel_link_I) - fext[FRONT_LEFT_WHEEL_LINK];
    
    // First pass, link 'front_right_wheel_link'
    front_right_wheel_link_v = ((xm->fr_front_right_wheel_link_X_fr_base_link) * base_link_v);
    front_right_wheel_link_v(iit::rbd::AZ) += qd(FRONT_RIGHT_WHEEL);
    
    motionCrossProductMx<Scalar>(front_right_wheel_link_v, vcross);
    
    front_right_wheel_link_a = (xm->fr_front_right_wheel_link_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(FRONT_RIGHT_WHEEL);
    front_right_wheel_link_a(iit::rbd::AZ) += qdd(FRONT_RIGHT_WHEEL);
    
    front_right_wheel_link_f = front_right_wheel_link_I * front_right_wheel_link_a + vxIv(front_right_wheel_link_v, front_right_wheel_link_I) - fext[FRONT_RIGHT_WHEEL_LINK];
    
    // First pass, link 'rear_left_wheel_link'
    rear_left_wheel_link_v = ((xm->fr_rear_left_wheel_link_X_fr_base_link) * base_link_v);
    rear_left_wheel_link_v(iit::rbd::AZ) += qd(REAR_LEFT_WHEEL);
    
    motionCrossProductMx<Scalar>(rear_left_wheel_link_v, vcross);
    
    rear_left_wheel_link_a = (xm->fr_rear_left_wheel_link_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(REAR_LEFT_WHEEL);
    rear_left_wheel_link_a(iit::rbd::AZ) += qdd(REAR_LEFT_WHEEL);
    
    rear_left_wheel_link_f = rear_left_wheel_link_I * rear_left_wheel_link_a + vxIv(rear_left_wheel_link_v, rear_left_wheel_link_I) - fext[REAR_LEFT_WHEEL_LINK];
    
    // First pass, link 'rear_right_wheel_link'
    rear_right_wheel_link_v = ((xm->fr_rear_right_wheel_link_X_fr_base_link) * base_link_v);
    rear_right_wheel_link_v(iit::rbd::AZ) += qd(REAR_RIGHT_WHEEL);
    
    motionCrossProductMx<Scalar>(rear_right_wheel_link_v, vcross);
    
    rear_right_wheel_link_a = (xm->fr_rear_right_wheel_link_X_fr_base_link) * base_link_a + vcross.col(iit::rbd::AZ) * qd(REAR_RIGHT_WHEEL);
    rear_right_wheel_link_a(iit::rbd::AZ) += qdd(REAR_RIGHT_WHEEL);
    
    rear_right_wheel_link_f = rear_right_wheel_link_I * rear_right_wheel_link_a + vxIv(rear_right_wheel_link_v, rear_right_wheel_link_I) - fext[REAR_RIGHT_WHEEL_LINK];
    

    // The base
    base_link_f = base_link_I * base_link_a + vxIv(base_link_v, base_link_I) - fext[BASE_LINK];

    secondPass_fullyActuated(jForces);

    baseWrench = base_link_f;
}


void Jackal::rcg::InverseDynamics::secondPass_fullyActuated(JointState& jForces)
{
    // Link 'rear_right_wheel_link'
    jForces(REAR_RIGHT_WHEEL) = rear_right_wheel_link_f(iit::rbd::AZ);
    base_link_f += xm->fr_rear_right_wheel_link_X_fr_base_link.transpose() * rear_right_wheel_link_f;
    // Link 'rear_left_wheel_link'
    jForces(REAR_LEFT_WHEEL) = rear_left_wheel_link_f(iit::rbd::AZ);
    base_link_f += xm->fr_rear_left_wheel_link_X_fr_base_link.transpose() * rear_left_wheel_link_f;
    // Link 'front_right_wheel_link'
    jForces(FRONT_RIGHT_WHEEL) = front_right_wheel_link_f(iit::rbd::AZ);
    base_link_f += xm->fr_front_right_wheel_link_X_fr_base_link.transpose() * front_right_wheel_link_f;
    // Link 'front_left_wheel_link'
    jForces(FRONT_LEFT_WHEEL) = front_left_wheel_link_f(iit::rbd::AZ);
    base_link_f += xm->fr_front_left_wheel_link_X_fr_base_link.transpose() * front_left_wheel_link_f;
}

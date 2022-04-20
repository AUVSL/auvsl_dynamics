#include "forward_dynamics.h"

#include <Eigen/Cholesky>
#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

// Initialization of static-const data
const Jackal::rcg::ForwardDynamics::ExtForces
    Jackal::rcg::ForwardDynamics::zeroExtForces(Force::Zero());

Jackal::rcg::ForwardDynamics::ForwardDynamics(InertiaProperties& inertia, MotionTransforms& transforms) :
    inertiaProps( &inertia ),
    motionTransforms( &transforms )
{
    front_left_wheel_link_v.setZero();
    front_left_wheel_link_c.setZero();
    front_right_wheel_link_v.setZero();
    front_right_wheel_link_c.setZero();
    rear_left_wheel_link_v.setZero();
    rear_left_wheel_link_c.setZero();
    rear_right_wheel_link_v.setZero();
    rear_right_wheel_link_c.setZero();

    vcross.setZero();
    Ia_r.setZero();

}

void Jackal::rcg::ForwardDynamics::fd(
    JointState& qdd,
    Acceleration& base_link_a,
    const Velocity& base_link_v,
    const Acceleration& g,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    
    base_link_AI = inertiaProps->getTensor_base_link();
    base_link_p = - fext[BASE_LINK];
    front_left_wheel_link_AI = inertiaProps->getTensor_front_left_wheel_link();
    front_left_wheel_link_p = - fext[FRONT_LEFT_WHEEL_LINK];
    front_right_wheel_link_AI = inertiaProps->getTensor_front_right_wheel_link();
    front_right_wheel_link_p = - fext[FRONT_RIGHT_WHEEL_LINK];
    rear_left_wheel_link_AI = inertiaProps->getTensor_rear_left_wheel_link();
    rear_left_wheel_link_p = - fext[REAR_LEFT_WHEEL_LINK];
    rear_right_wheel_link_AI = inertiaProps->getTensor_rear_right_wheel_link();
    rear_right_wheel_link_p = - fext[REAR_RIGHT_WHEEL_LINK];
    // ---------------------- FIRST PASS ---------------------- //
    // Note that, during the first pass, the articulated inertias are really
    //  just the spatial inertia of the links (see assignments above).
    //  Afterwards things change, and articulated inertias shall not be used
    //  in functions which work specifically with spatial inertias.
    
    // + Link front_left_wheel_link
    //  - The spatial velocity:
    front_left_wheel_link_v = (motionTransforms-> fr_front_left_wheel_link_X_fr_base_link) * base_link_v;
    front_left_wheel_link_v(AZ) += qd(FRONT_LEFT_WHEEL);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(front_left_wheel_link_v, vcross);
    front_left_wheel_link_c = vcross.col(AZ) * qd(FRONT_LEFT_WHEEL);
    
    //  - The bias force term:
    front_left_wheel_link_p += vxIv(front_left_wheel_link_v, front_left_wheel_link_AI);
    
    
    
    // + Link front_right_wheel_link
    //  - The spatial velocity:
    front_right_wheel_link_v = (motionTransforms-> fr_front_right_wheel_link_X_fr_base_link) * base_link_v;
    front_right_wheel_link_v(AZ) += qd(FRONT_RIGHT_WHEEL);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(front_right_wheel_link_v, vcross);
    front_right_wheel_link_c = vcross.col(AZ) * qd(FRONT_RIGHT_WHEEL);
    
    //  - The bias force term:
    front_right_wheel_link_p += vxIv(front_right_wheel_link_v, front_right_wheel_link_AI);
    
    // + Link rear_left_wheel_link
    //  - The spatial velocity:
    rear_left_wheel_link_v = (motionTransforms-> fr_rear_left_wheel_link_X_fr_base_link) * base_link_v;
    rear_left_wheel_link_v(AZ) += qd(REAR_LEFT_WHEEL);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(rear_left_wheel_link_v, vcross);
    rear_left_wheel_link_c = vcross.col(AZ) * qd(REAR_LEFT_WHEEL);
    
    //  - The bias force term:
    rear_left_wheel_link_p += vxIv(rear_left_wheel_link_v, rear_left_wheel_link_AI);
    
    // + Link rear_right_wheel_link
    //  - The spatial velocity:
    rear_right_wheel_link_v = (motionTransforms-> fr_rear_right_wheel_link_X_fr_base_link) * base_link_v;
    rear_right_wheel_link_v(AZ) += qd(REAR_RIGHT_WHEEL);
    
    //  - The velocity-product acceleration term:
    motionCrossProductMx<Scalar>(rear_right_wheel_link_v, vcross);
    rear_right_wheel_link_c = vcross.col(AZ) * qd(REAR_RIGHT_WHEEL);
    
    //  - The bias force term:
    rear_right_wheel_link_p += vxIv(rear_right_wheel_link_v, rear_right_wheel_link_AI);
    
    // + The floating base body
    base_link_p += vxIv(base_link_v, base_link_AI);
    
    // ---------------------- SECOND PASS ---------------------- //
    Matrix66 IaB;
    Force pa;
    
    // + Link rear_right_wheel_link
    rear_right_wheel_link_u = tau(REAR_RIGHT_WHEEL) - rear_right_wheel_link_p(AZ);
    rear_right_wheel_link_U = rear_right_wheel_link_AI.col(AZ);
    rear_right_wheel_link_D = rear_right_wheel_link_U(AZ);
    
    compute_Ia_revolute(rear_right_wheel_link_AI, rear_right_wheel_link_U, rear_right_wheel_link_D, Ia_r);  // same as: Ia_r = rear_right_wheel_link_AI - rear_right_wheel_link_U/rear_right_wheel_link_D * rear_right_wheel_link_U.transpose();
    pa = rear_right_wheel_link_p + Ia_r * rear_right_wheel_link_c + rear_right_wheel_link_U * rear_right_wheel_link_u/rear_right_wheel_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_rear_right_wheel_link_X_fr_base_link, IaB);
    base_link_AI += IaB;
    base_link_p += (motionTransforms-> fr_rear_right_wheel_link_X_fr_base_link).transpose() * pa;
    
    // + Link rear_left_wheel_link
    rear_left_wheel_link_u = tau(REAR_LEFT_WHEEL) - rear_left_wheel_link_p(AZ);
    rear_left_wheel_link_U = rear_left_wheel_link_AI.col(AZ);
    rear_left_wheel_link_D = rear_left_wheel_link_U(AZ);
    
    compute_Ia_revolute(rear_left_wheel_link_AI, rear_left_wheel_link_U, rear_left_wheel_link_D, Ia_r);  // same as: Ia_r = rear_left_wheel_link_AI - rear_left_wheel_link_U/rear_left_wheel_link_D * rear_left_wheel_link_U.transpose();
    pa = rear_left_wheel_link_p + Ia_r * rear_left_wheel_link_c + rear_left_wheel_link_U * rear_left_wheel_link_u/rear_left_wheel_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_rear_left_wheel_link_X_fr_base_link, IaB);
    base_link_AI += IaB;
    base_link_p += (motionTransforms-> fr_rear_left_wheel_link_X_fr_base_link).transpose() * pa;
    
    // + Link front_right_wheel_link
    front_right_wheel_link_u = tau(FRONT_RIGHT_WHEEL) - front_right_wheel_link_p(AZ);
    front_right_wheel_link_U = front_right_wheel_link_AI.col(AZ);
    front_right_wheel_link_D = front_right_wheel_link_U(AZ);
    
    compute_Ia_revolute(front_right_wheel_link_AI, front_right_wheel_link_U, front_right_wheel_link_D, Ia_r);  // same as: Ia_r = front_right_wheel_link_AI - front_right_wheel_link_U/front_right_wheel_link_D * front_right_wheel_link_U.transpose();
    pa = front_right_wheel_link_p + Ia_r * front_right_wheel_link_c + front_right_wheel_link_U * front_right_wheel_link_u/front_right_wheel_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_front_right_wheel_link_X_fr_base_link, IaB);
    base_link_AI += IaB;
    base_link_p += (motionTransforms-> fr_front_right_wheel_link_X_fr_base_link).transpose() * pa;
    
    // + Link front_left_wheel_link
    front_left_wheel_link_u = tau(FRONT_LEFT_WHEEL) - front_left_wheel_link_p(AZ);
    front_left_wheel_link_U = front_left_wheel_link_AI.col(AZ);
    front_left_wheel_link_D = front_left_wheel_link_U(AZ);
    
    compute_Ia_revolute(front_left_wheel_link_AI, front_left_wheel_link_U, front_left_wheel_link_D, Ia_r);  // same as: Ia_r = front_left_wheel_link_AI - front_left_wheel_link_U/front_left_wheel_link_D * front_left_wheel_link_U.transpose();
    pa = front_left_wheel_link_p + Ia_r * front_left_wheel_link_c + front_left_wheel_link_U * front_left_wheel_link_u/front_left_wheel_link_D;
    ctransform_Ia_revolute(Ia_r, motionTransforms-> fr_front_left_wheel_link_X_fr_base_link, IaB);
    base_link_AI += IaB;
    base_link_p += (motionTransforms-> fr_front_left_wheel_link_X_fr_base_link).transpose() * pa;
    
    // + The acceleration of the floating base base_link, without gravity
    base_link_a = - base_link_AI.llt().solve(base_link_p);  // base_link_a = - IA^-1 * base_link_p
    
    // ---------------------- THIRD PASS ---------------------- //
    front_left_wheel_link_a = (motionTransforms-> fr_front_left_wheel_link_X_fr_base_link) * base_link_a + front_left_wheel_link_c;
    qdd(FRONT_LEFT_WHEEL) = (front_left_wheel_link_u - front_left_wheel_link_U.dot(front_left_wheel_link_a)) / front_left_wheel_link_D;
    front_left_wheel_link_a(AZ) += qdd(FRONT_LEFT_WHEEL);
    
    front_right_wheel_link_a = (motionTransforms-> fr_front_right_wheel_link_X_fr_base_link) * base_link_a + front_right_wheel_link_c;
    qdd(FRONT_RIGHT_WHEEL) = (front_right_wheel_link_u - front_right_wheel_link_U.dot(front_right_wheel_link_a)) / front_right_wheel_link_D;
    front_right_wheel_link_a(AZ) += qdd(FRONT_RIGHT_WHEEL);
    
    rear_left_wheel_link_a = (motionTransforms-> fr_rear_left_wheel_link_X_fr_base_link) * base_link_a + rear_left_wheel_link_c;
    qdd(REAR_LEFT_WHEEL) = (rear_left_wheel_link_u - rear_left_wheel_link_U.dot(rear_left_wheel_link_a)) / rear_left_wheel_link_D;
    rear_left_wheel_link_a(AZ) += qdd(REAR_LEFT_WHEEL);
    
    rear_right_wheel_link_a = (motionTransforms-> fr_rear_right_wheel_link_X_fr_base_link) * base_link_a + rear_right_wheel_link_c;
    qdd(REAR_RIGHT_WHEEL) = (rear_right_wheel_link_u - rear_right_wheel_link_U.dot(rear_right_wheel_link_a)) / rear_right_wheel_link_D;
    rear_right_wheel_link_a(AZ) += qdd(REAR_RIGHT_WHEEL);
    
    
    // + Add gravity to the acceleration of the floating base
    base_link_a += g;
}

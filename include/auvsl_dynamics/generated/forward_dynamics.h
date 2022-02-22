#ifndef RCG_JACKAL_FORWARD_DYNAMICS_H_
#define RCG_JACKAL_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace Jackal {
namespace rcg {

/**
 * The Forward Dynamics routine for the robot Jackal.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */
class ForwardDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;
    
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot Jackal, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties& in, MotionTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_link_a
     * \param base_link_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_link_a, // output parameters,
       const Velocity& base_link_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_link_a, // output parameters,
        const Velocity& base_link_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* motionTransforms;

    Matrix66 vcross; // support variable
    Matrix66 Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base_link'
    Matrix66 base_link_AI;
    Force base_link_p;

    // Link 'front_left_wheel_link' :
    Matrix66 front_left_wheel_link_AI;
    Velocity front_left_wheel_link_a;
    Velocity front_left_wheel_link_v;
    Velocity front_left_wheel_link_c;
    Force    front_left_wheel_link_p;

    Column6 front_left_wheel_link_U;
    Scalar front_left_wheel_link_D;
    Scalar front_left_wheel_link_u;
    // Link 'front_right_wheel_link' :
    Matrix66 front_right_wheel_link_AI;
    Velocity front_right_wheel_link_a;
    Velocity front_right_wheel_link_v;
    Velocity front_right_wheel_link_c;
    Force    front_right_wheel_link_p;

    Column6 front_right_wheel_link_U;
    Scalar front_right_wheel_link_D;
    Scalar front_right_wheel_link_u;
    // Link 'rear_left_wheel_link' :
    Matrix66 rear_left_wheel_link_AI;
    Velocity rear_left_wheel_link_a;
    Velocity rear_left_wheel_link_v;
    Velocity rear_left_wheel_link_c;
    Force    rear_left_wheel_link_p;

    Column6 rear_left_wheel_link_U;
    Scalar rear_left_wheel_link_D;
    Scalar rear_left_wheel_link_u;
    // Link 'rear_right_wheel_link' :
    Matrix66 rear_right_wheel_link_AI;
    Velocity rear_right_wheel_link_a;
    Velocity rear_right_wheel_link_v;
    Velocity rear_right_wheel_link_c;
    Force    rear_right_wheel_link_p;

    Column6 rear_right_wheel_link_U;
    Scalar rear_right_wheel_link_D;
    Scalar rear_right_wheel_link_u;
private:
    static const ExtForces zeroExtForces;
};

inline void ForwardDynamics::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_front_left_wheel_link_X_fr_base_link)(q);
    (motionTransforms-> fr_front_right_wheel_link_X_fr_base_link)(q);
    (motionTransforms-> fr_rear_left_wheel_link_X_fr_base_link)(q);
    (motionTransforms-> fr_rear_right_wheel_link_X_fr_base_link)(q);
}

inline void ForwardDynamics::fd(
    JointState& qdd, Acceleration& base_link_a, // output parameters,
    const Velocity& base_link_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_link_a, base_link_v, g, qd, tau, fext);
}

}
}

#endif

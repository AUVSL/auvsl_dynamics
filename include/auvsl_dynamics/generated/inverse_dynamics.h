#ifndef RCG_JACKAL_INVERSE_DYNAMICS_H_
#define RCG_JACKAL_INVERSE_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "inertia_properties.h"
#include "transforms.h"
#include "link_data_map.h"

namespace Jackal {
namespace rcg {

/**
 * The Inverse Dynamics routine for the robot Jackal.
 *
 * In addition to the full Newton-Euler algorithm, specialized versions to compute
 * only certain terms are provided.
 * The parameters common to most of the methods are the joint status vector \c q, the
 * joint velocity vector \c qd and the acceleration vector \c qdd.
 *
 * Additional overloaded methods are provided without the \c q parameter. These
 * methods use the current configuration of the robot; they are provided for the
 * sake of efficiency, in case the motion transforms of the robot have already
 * been updated elsewhere with the most recent configuration (eg by a call to
 * setJointStatus()), so that it is useless to compute them again.
 *
 * Whenever present, the external forces parameter is a set of external
 * wrenches acting on the robot links. Each wrench must be expressed in
 * the reference frame of the link it is excerted on.
 */
class InverseDynamics {
public:
    typedef LinkDataMap<Force> ExtForces;

    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot Jackal, which will be used by this instance
     *     to compute inverse-dynamics.
     */
    InverseDynamics(InertiaProperties& in, MotionTransforms& tr);

    /** \name Inverse dynamics
     * The full algorithm for the inverse dynamics of this robot.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[out] baseAccel the spatial acceleration of the robot base
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_v the spatial velocity of the base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id(
        JointState& jForces, Acceleration& base_link_a,
        const Acceleration& g, const Velocity& base_link_v,
        const JointState& q, const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    void id(
        JointState& jForces, Acceleration& base_link_a,
        const Acceleration& g, const Velocity& base_link_v,
        const JointState& qd, const JointState& qdd,
        const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Inverse dynamics, fully actuated base
     * The inverse dynamics algorithm for the floating base robot,
     * in the assumption of a fully actuated base.
     *
     * All the spatial vectors in the parameters are expressed in base coordinates,
     * besides the external forces: each force must be expressed in the reference
     * frame of the link it is acting on.
     * \param[out] baseWrench the spatial force to be applied to the robot base to achieve
     *             the desired accelerations
     * \param[out] jForces the joint force vector required to achieve the desired accelerations
     * \param[in] g the gravity acceleration, as a spatial vector;
     *              gravity implicitly specifies the orientation of the base in space
     * \param[in] base_link_v the spatial velocity of the base
     * \param[in] baseAccel the desired spatial acceleration of the robot base
     * \param[in] q the joint position vector
     * \param[in] qd the joint velocity vector
     * \param[in] qdd the desired joint acceleration vector
     * \param[in] fext the external forces acting on the links; this parameters
     *            defaults to zero
     */ ///@{
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    void id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& qd, const JointState& qdd, const ExtForces& fext = zeroExtForces);
    ///@}
    /** \name Gravity terms, fully actuated base
     */
    ///@{
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const JointState& q);
    void G_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g);
    ///@}
    /** \name Centrifugal and Coriolis terms, fully actuated base
     *
     * These functions take only velocity inputs, that is, they assume
     * a zero spatial acceleration of the base (in addition to zero acceleration
     * at the actuated joints).
     * Note that this is NOT the same as imposing zero acceleration
     * at the virtual 6-dof-floting-base joint, which would result, in general,
     * in a non-zero spatial acceleration of the base, due to velocity
     * product terms.
     */
    ///@{
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_v, const JointState& q, const JointState& qd);
    void C_terms_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Velocity& base_link_v, const JointState& qd);
    ///@}
    /** Updates all the kinematics transforms used by the inverse dynamics routine. */
    void setJointStatus(const JointState& q) const;

public:
    /** \name Getters
     * These functions return various spatial quantities used internally
     * by the inverse dynamics routines, like the spatial acceleration
     * of the links.
     *
     * The getters can be useful to retrieve the additional data that is not
     * returned explicitly by the inverse dynamics routines even though it
     * is computed. For example, after a call to the inverse dynamics,
     * the spatial velocity of all the links has been determined and
     * can be accessed.
     *
     * However, beware that certain routines might not use some of the
     * spatial quantities, which therefore would retain their last value
     * without being updated nor reset (for example, the spatial velocity
     * of the links is unaffected by the computation of the gravity terms).
     */
    ///@{
    const Force& getForce_base_link() const { return base_link_f; }
    const Velocity& getVelocity_front_left_wheel_link() const { return front_left_wheel_link_v; }
    const Acceleration& getAcceleration_front_left_wheel_link() const { return front_left_wheel_link_a; }
    const Force& getForce_front_left_wheel_link() const { return front_left_wheel_link_f; }
    const Velocity& getVelocity_front_right_wheel_link() const { return front_right_wheel_link_v; }
    const Acceleration& getAcceleration_front_right_wheel_link() const { return front_right_wheel_link_a; }
    const Force& getForce_front_right_wheel_link() const { return front_right_wheel_link_f; }
    const Velocity& getVelocity_rear_left_wheel_link() const { return rear_left_wheel_link_v; }
    const Acceleration& getAcceleration_rear_left_wheel_link() const { return rear_left_wheel_link_a; }
    const Force& getForce_rear_left_wheel_link() const { return rear_left_wheel_link_f; }
    const Velocity& getVelocity_rear_right_wheel_link() const { return rear_right_wheel_link_v; }
    const Acceleration& getAcceleration_rear_right_wheel_link() const { return rear_right_wheel_link_a; }
    const Force& getForce_rear_right_wheel_link() const { return rear_right_wheel_link_f; }
    ///@}
protected:
    void secondPass_fullyActuated(JointState& jForces);

private:
    InertiaProperties* inertiaProps;
    MotionTransforms* xm;
private:
    Matrix66 vcross; // support variable
    // Link 'front_left_wheel_link' :
    const InertiaMatrix& front_left_wheel_link_I;
    Velocity      front_left_wheel_link_v;
    Acceleration  front_left_wheel_link_a;
    Force         front_left_wheel_link_f;
    // Link 'front_right_wheel_link' :
    const InertiaMatrix& front_right_wheel_link_I;
    Velocity      front_right_wheel_link_v;
    Acceleration  front_right_wheel_link_a;
    Force         front_right_wheel_link_f;
    // Link 'rear_left_wheel_link' :
    const InertiaMatrix& rear_left_wheel_link_I;
    Velocity      rear_left_wheel_link_v;
    Acceleration  rear_left_wheel_link_a;
    Force         rear_left_wheel_link_f;
    // Link 'rear_right_wheel_link' :
    const InertiaMatrix& rear_right_wheel_link_I;
    Velocity      rear_right_wheel_link_v;
    Acceleration  rear_right_wheel_link_a;
    Force         rear_right_wheel_link_f;

    // The robot base
    const InertiaMatrix& base_link_I;
    InertiaMatrix base_link_Ic;
    Force         base_link_f;
    // The composite inertia tensors
    const InertiaMatrix& front_left_wheel_link_Ic;
    const InertiaMatrix& front_right_wheel_link_Ic;
    const InertiaMatrix& rear_left_wheel_link_Ic;
    const InertiaMatrix& rear_right_wheel_link_Ic;

private:
    static const ExtForces zeroExtForces;
};

inline void InverseDynamics::setJointStatus(const JointState& q) const
{
    (xm->fr_front_left_wheel_link_X_fr_base_link)(q);
    (xm->fr_front_right_wheel_link_X_fr_base_link)(q);
    (xm->fr_rear_left_wheel_link_X_fr_base_link)(q);
    (xm->fr_rear_right_wheel_link_X_fr_base_link)(q);
}

inline void InverseDynamics::id(
    JointState& jForces, Acceleration& base_link_a,
    const Acceleration& g, const Velocity& base_link_v,
    const JointState& q, const JointState& qd, const JointState& qdd,
    const ExtForces& fext)
{
    setJointStatus(q);
    id(jForces, base_link_a, g, base_link_v,
       qd, qdd, fext);
}

inline void InverseDynamics::G_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Acceleration& g, const JointState& q)
{
    setJointStatus(q);
    G_terms_fully_actuated(baseWrench, jForces, g);
}

inline void InverseDynamics::C_terms_fully_actuated(
    Force& baseWrench, JointState& jForces,
    const Velocity& base_link_v, const JointState& q, const JointState& qd)
{
    setJointStatus(q);
    C_terms_fully_actuated(baseWrench, jForces, base_link_v, qd);
}

inline void InverseDynamics::id_fully_actuated(
        Force& baseWrench, JointState& jForces,
        const Acceleration& g, const Velocity& base_link_v, const Acceleration& baseAccel,
        const JointState& q, const JointState& qd, const JointState& qdd, const ExtForces& fext)
{
    setJointStatus(q);
    id_fully_actuated(baseWrench, jForces, g, base_link_v,
        baseAccel, qd, qdd, fext);
}

}
}

#endif

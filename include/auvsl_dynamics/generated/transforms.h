#ifndef JACKAL_TRANSFORMS_H_
#define JACKAL_TRANSFORMS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "model_constants.h"
#include "kinematics_parameters.h"

namespace Jackal {
namespace rcg {

struct Parameters
{
    struct AngleFuncValues {
        AngleFuncValues() {
            update();
        }

        void update()
        {
        }
    };

    Params_lengths lengths;
    Params_angles angles;
    AngleFuncValues trig = AngleFuncValues();
};

// The type of the "vector" with the status of the variables
typedef JointState state_t;

template<class M>
using TransformMotion = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformForce = iit::rbd::SpatialTransformBase<state_t, M>;

template<class M>
using TransformHomogeneous = iit::rbd::HomogeneousTransformBase<state_t, M>;

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial motion vectors.
 */
class MotionTransforms
{
public:
    class Dummy {};
    typedef TransformMotion<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_link_X_fr_front_left_wheel_link : public TransformMotion<Type_fr_base_link_X_fr_front_left_wheel_link>
    {
        Type_fr_base_link_X_fr_front_left_wheel_link();
        const Type_fr_base_link_X_fr_front_left_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_front_left_wheel_link_X_fr_base_link : public TransformMotion<Type_fr_front_left_wheel_link_X_fr_base_link>
    {
        Type_fr_front_left_wheel_link_X_fr_base_link();
        const Type_fr_front_left_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel_link : public TransformMotion<Type_fr_base_link_X_fr_front_right_wheel_link>
    {
        Type_fr_base_link_X_fr_front_right_wheel_link();
        const Type_fr_base_link_X_fr_front_right_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_front_right_wheel_link_X_fr_base_link : public TransformMotion<Type_fr_front_right_wheel_link_X_fr_base_link>
    {
        Type_fr_front_right_wheel_link_X_fr_base_link();
        const Type_fr_front_right_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel_link : public TransformMotion<Type_fr_base_link_X_fr_rear_left_wheel_link>
    {
        Type_fr_base_link_X_fr_rear_left_wheel_link();
        const Type_fr_base_link_X_fr_rear_left_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_rear_left_wheel_link_X_fr_base_link : public TransformMotion<Type_fr_rear_left_wheel_link_X_fr_base_link>
    {
        Type_fr_rear_left_wheel_link_X_fr_base_link();
        const Type_fr_rear_left_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel_link : public TransformMotion<Type_fr_base_link_X_fr_rear_right_wheel_link>
    {
        Type_fr_base_link_X_fr_rear_right_wheel_link();
        const Type_fr_base_link_X_fr_rear_right_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_rear_right_wheel_link_X_fr_base_link : public TransformMotion<Type_fr_rear_right_wheel_link_X_fr_base_link>
    {
        Type_fr_rear_right_wheel_link_X_fr_base_link();
        const Type_fr_rear_right_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_base_link_COM : public TransformMotion<Type_fr_base_link_X_fr_base_link_COM>
    {
        Type_fr_base_link_X_fr_base_link_COM();
        const Type_fr_base_link_X_fr_base_link_COM& update(const state_t&);
    };
    
    struct Type_fr_base_link_COM_X_fr_base_link : public TransformMotion<Type_fr_base_link_COM_X_fr_base_link>
    {
        Type_fr_base_link_COM_X_fr_base_link();
        const Type_fr_base_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_chassis_link : public TransformMotion<Type_fr_base_link_X_fr_chassis_link>
    {
        Type_fr_base_link_X_fr_chassis_link();
        const Type_fr_base_link_X_fr_chassis_link& update(const state_t&);
    };
    
    struct Type_fr_chassis_link_X_fr_base_link : public TransformMotion<Type_fr_chassis_link_X_fr_base_link>
    {
        Type_fr_chassis_link_X_fr_base_link();
        const Type_fr_chassis_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_fender_link : public TransformMotion<Type_fr_base_link_X_fr_front_fender_link>
    {
        Type_fr_base_link_X_fr_front_fender_link();
        const Type_fr_base_link_X_fr_front_fender_link& update(const state_t&);
    };
    
    struct Type_fr_front_fender_link_X_fr_base_link : public TransformMotion<Type_fr_front_fender_link_X_fr_base_link>
    {
        Type_fr_front_fender_link_X_fr_base_link();
        const Type_fr_front_fender_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_left_wheel_link_COM : public TransformMotion<Type_fr_base_link_X_fr_front_left_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_front_left_wheel_link_COM();
        const Type_fr_base_link_X_fr_front_left_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_front_left_wheel_link_COM_X_fr_base_link : public TransformMotion<Type_fr_front_left_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_front_left_wheel_link_COM_X_fr_base_link();
        const Type_fr_front_left_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_mount : public TransformMotion<Type_fr_base_link_X_fr_front_mount>
    {
        Type_fr_base_link_X_fr_front_mount();
        const Type_fr_base_link_X_fr_front_mount& update(const state_t&);
    };
    
    struct Type_fr_front_mount_X_fr_base_link : public TransformMotion<Type_fr_front_mount_X_fr_base_link>
    {
        Type_fr_front_mount_X_fr_base_link();
        const Type_fr_front_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel_link_COM : public TransformMotion<Type_fr_base_link_X_fr_front_right_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_front_right_wheel_link_COM();
        const Type_fr_base_link_X_fr_front_right_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_front_right_wheel_link_COM_X_fr_base_link : public TransformMotion<Type_fr_front_right_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_front_right_wheel_link_COM_X_fr_base_link();
        const Type_fr_front_right_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_imu_link : public TransformMotion<Type_fr_base_link_X_fr_imu_link>
    {
        Type_fr_base_link_X_fr_imu_link();
        const Type_fr_base_link_X_fr_imu_link& update(const state_t&);
    };
    
    struct Type_fr_imu_link_X_fr_base_link : public TransformMotion<Type_fr_imu_link_X_fr_base_link>
    {
        Type_fr_imu_link_X_fr_base_link();
        const Type_fr_imu_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_mid_mount : public TransformMotion<Type_fr_base_link_X_fr_mid_mount>
    {
        Type_fr_base_link_X_fr_mid_mount();
        const Type_fr_base_link_X_fr_mid_mount& update(const state_t&);
    };
    
    struct Type_fr_mid_mount_X_fr_base_link : public TransformMotion<Type_fr_mid_mount_X_fr_base_link>
    {
        Type_fr_mid_mount_X_fr_base_link();
        const Type_fr_mid_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_navsat_link : public TransformMotion<Type_fr_base_link_X_fr_navsat_link>
    {
        Type_fr_base_link_X_fr_navsat_link();
        const Type_fr_base_link_X_fr_navsat_link& update(const state_t&);
    };
    
    struct Type_fr_navsat_link_X_fr_base_link : public TransformMotion<Type_fr_navsat_link_X_fr_base_link>
    {
        Type_fr_navsat_link_X_fr_base_link();
        const Type_fr_navsat_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_fender_link : public TransformMotion<Type_fr_base_link_X_fr_rear_fender_link>
    {
        Type_fr_base_link_X_fr_rear_fender_link();
        const Type_fr_base_link_X_fr_rear_fender_link& update(const state_t&);
    };
    
    struct Type_fr_rear_fender_link_X_fr_base_link : public TransformMotion<Type_fr_rear_fender_link_X_fr_base_link>
    {
        Type_fr_rear_fender_link_X_fr_base_link();
        const Type_fr_rear_fender_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel_link_COM : public TransformMotion<Type_fr_base_link_X_fr_rear_left_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_rear_left_wheel_link_COM();
        const Type_fr_base_link_X_fr_rear_left_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_rear_left_wheel_link_COM_X_fr_base_link : public TransformMotion<Type_fr_rear_left_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_rear_left_wheel_link_COM_X_fr_base_link();
        const Type_fr_rear_left_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_mount : public TransformMotion<Type_fr_base_link_X_fr_rear_mount>
    {
        Type_fr_base_link_X_fr_rear_mount();
        const Type_fr_base_link_X_fr_rear_mount& update(const state_t&);
    };
    
    struct Type_fr_rear_mount_X_fr_base_link : public TransformMotion<Type_fr_rear_mount_X_fr_base_link>
    {
        Type_fr_rear_mount_X_fr_base_link();
        const Type_fr_rear_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel_link_COM : public TransformMotion<Type_fr_base_link_X_fr_rear_right_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_rear_right_wheel_link_COM();
        const Type_fr_base_link_X_fr_rear_right_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_rear_right_wheel_link_COM_X_fr_base_link : public TransformMotion<Type_fr_rear_right_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_rear_right_wheel_link_COM_X_fr_base_link();
        const Type_fr_rear_right_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_left_wheel : public TransformMotion<Type_fr_base_link_X_fr_front_left_wheel>
    {
        Type_fr_base_link_X_fr_front_left_wheel();
        const Type_fr_base_link_X_fr_front_left_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel : public TransformMotion<Type_fr_base_link_X_fr_front_right_wheel>
    {
        Type_fr_base_link_X_fr_front_right_wheel();
        const Type_fr_base_link_X_fr_front_right_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel : public TransformMotion<Type_fr_base_link_X_fr_rear_left_wheel>
    {
        Type_fr_base_link_X_fr_rear_left_wheel();
        const Type_fr_base_link_X_fr_rear_left_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel : public TransformMotion<Type_fr_base_link_X_fr_rear_right_wheel>
    {
        Type_fr_base_link_X_fr_rear_right_wheel();
        const Type_fr_base_link_X_fr_rear_right_wheel& update(const state_t&);
    };
    
public:
    MotionTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_link_X_fr_front_left_wheel_link fr_base_link_X_fr_front_left_wheel_link;
    Type_fr_front_left_wheel_link_X_fr_base_link fr_front_left_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_right_wheel_link fr_base_link_X_fr_front_right_wheel_link;
    Type_fr_front_right_wheel_link_X_fr_base_link fr_front_right_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_left_wheel_link fr_base_link_X_fr_rear_left_wheel_link;
    Type_fr_rear_left_wheel_link_X_fr_base_link fr_rear_left_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_right_wheel_link fr_base_link_X_fr_rear_right_wheel_link;
    Type_fr_rear_right_wheel_link_X_fr_base_link fr_rear_right_wheel_link_X_fr_base_link;
  
    Type_fr_base_link_X_fr_base_link_COM fr_base_link_X_fr_base_link_COM;
  
    Type_fr_base_link_COM_X_fr_base_link fr_base_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_chassis_link fr_base_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_base_link fr_chassis_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_fender_link fr_base_link_X_fr_front_fender_link;
    Type_fr_front_fender_link_X_fr_base_link fr_front_fender_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_left_wheel_link_COM fr_base_link_X_fr_front_left_wheel_link_COM;
    Type_fr_front_left_wheel_link_COM_X_fr_base_link fr_front_left_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_front_mount fr_base_link_X_fr_front_mount;
    Type_fr_front_mount_X_fr_base_link fr_front_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_front_right_wheel_link_COM fr_base_link_X_fr_front_right_wheel_link_COM;
    Type_fr_front_right_wheel_link_COM_X_fr_base_link fr_front_right_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_imu_link fr_base_link_X_fr_imu_link;
    Type_fr_imu_link_X_fr_base_link fr_imu_link_X_fr_base_link;
    Type_fr_base_link_X_fr_mid_mount fr_base_link_X_fr_mid_mount;
    Type_fr_mid_mount_X_fr_base_link fr_mid_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_navsat_link fr_base_link_X_fr_navsat_link;
    Type_fr_navsat_link_X_fr_base_link fr_navsat_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_fender_link fr_base_link_X_fr_rear_fender_link;
    Type_fr_rear_fender_link_X_fr_base_link fr_rear_fender_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_left_wheel_link_COM fr_base_link_X_fr_rear_left_wheel_link_COM;
    Type_fr_rear_left_wheel_link_COM_X_fr_base_link fr_rear_left_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_mount fr_base_link_X_fr_rear_mount;
    Type_fr_rear_mount_X_fr_base_link fr_rear_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_right_wheel_link_COM fr_base_link_X_fr_rear_right_wheel_link_COM;
    Type_fr_rear_right_wheel_link_COM_X_fr_base_link fr_rear_right_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_front_left_wheel fr_base_link_X_fr_front_left_wheel;
    Type_fr_base_link_X_fr_front_right_wheel fr_base_link_X_fr_front_right_wheel;
    Type_fr_base_link_X_fr_rear_left_wheel fr_base_link_X_fr_rear_left_wheel;
    Type_fr_base_link_X_fr_rear_right_wheel fr_base_link_X_fr_rear_right_wheel;

protected:
    Parameters params;

}; //class 'MotionTransforms'

/**
 * The class for the 6-by-6 coordinates transformation matrices for
 * spatial force vectors.
 */
class ForceTransforms
{
public:
    class Dummy {};
    typedef TransformForce<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_link_X_fr_front_left_wheel_link : public TransformForce<Type_fr_base_link_X_fr_front_left_wheel_link>
    {
        Type_fr_base_link_X_fr_front_left_wheel_link();
        const Type_fr_base_link_X_fr_front_left_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_front_left_wheel_link_X_fr_base_link : public TransformForce<Type_fr_front_left_wheel_link_X_fr_base_link>
    {
        Type_fr_front_left_wheel_link_X_fr_base_link();
        const Type_fr_front_left_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel_link : public TransformForce<Type_fr_base_link_X_fr_front_right_wheel_link>
    {
        Type_fr_base_link_X_fr_front_right_wheel_link();
        const Type_fr_base_link_X_fr_front_right_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_front_right_wheel_link_X_fr_base_link : public TransformForce<Type_fr_front_right_wheel_link_X_fr_base_link>
    {
        Type_fr_front_right_wheel_link_X_fr_base_link();
        const Type_fr_front_right_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel_link : public TransformForce<Type_fr_base_link_X_fr_rear_left_wheel_link>
    {
        Type_fr_base_link_X_fr_rear_left_wheel_link();
        const Type_fr_base_link_X_fr_rear_left_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_rear_left_wheel_link_X_fr_base_link : public TransformForce<Type_fr_rear_left_wheel_link_X_fr_base_link>
    {
        Type_fr_rear_left_wheel_link_X_fr_base_link();
        const Type_fr_rear_left_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel_link : public TransformForce<Type_fr_base_link_X_fr_rear_right_wheel_link>
    {
        Type_fr_base_link_X_fr_rear_right_wheel_link();
        const Type_fr_base_link_X_fr_rear_right_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_rear_right_wheel_link_X_fr_base_link : public TransformForce<Type_fr_rear_right_wheel_link_X_fr_base_link>
    {
        Type_fr_rear_right_wheel_link_X_fr_base_link();
        const Type_fr_rear_right_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_base_link_COM : public TransformForce<Type_fr_base_link_X_fr_base_link_COM>
    {
        Type_fr_base_link_X_fr_base_link_COM();
        const Type_fr_base_link_X_fr_base_link_COM& update(const state_t&);
    };
    
    struct Type_fr_base_link_COM_X_fr_base_link : public TransformForce<Type_fr_base_link_COM_X_fr_base_link>
    {
        Type_fr_base_link_COM_X_fr_base_link();
        const Type_fr_base_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_chassis_link : public TransformForce<Type_fr_base_link_X_fr_chassis_link>
    {
        Type_fr_base_link_X_fr_chassis_link();
        const Type_fr_base_link_X_fr_chassis_link& update(const state_t&);
    };
    
    struct Type_fr_chassis_link_X_fr_base_link : public TransformForce<Type_fr_chassis_link_X_fr_base_link>
    {
        Type_fr_chassis_link_X_fr_base_link();
        const Type_fr_chassis_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_fender_link : public TransformForce<Type_fr_base_link_X_fr_front_fender_link>
    {
        Type_fr_base_link_X_fr_front_fender_link();
        const Type_fr_base_link_X_fr_front_fender_link& update(const state_t&);
    };
    
    struct Type_fr_front_fender_link_X_fr_base_link : public TransformForce<Type_fr_front_fender_link_X_fr_base_link>
    {
        Type_fr_front_fender_link_X_fr_base_link();
        const Type_fr_front_fender_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_left_wheel_link_COM : public TransformForce<Type_fr_base_link_X_fr_front_left_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_front_left_wheel_link_COM();
        const Type_fr_base_link_X_fr_front_left_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_front_left_wheel_link_COM_X_fr_base_link : public TransformForce<Type_fr_front_left_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_front_left_wheel_link_COM_X_fr_base_link();
        const Type_fr_front_left_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_mount : public TransformForce<Type_fr_base_link_X_fr_front_mount>
    {
        Type_fr_base_link_X_fr_front_mount();
        const Type_fr_base_link_X_fr_front_mount& update(const state_t&);
    };
    
    struct Type_fr_front_mount_X_fr_base_link : public TransformForce<Type_fr_front_mount_X_fr_base_link>
    {
        Type_fr_front_mount_X_fr_base_link();
        const Type_fr_front_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel_link_COM : public TransformForce<Type_fr_base_link_X_fr_front_right_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_front_right_wheel_link_COM();
        const Type_fr_base_link_X_fr_front_right_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_front_right_wheel_link_COM_X_fr_base_link : public TransformForce<Type_fr_front_right_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_front_right_wheel_link_COM_X_fr_base_link();
        const Type_fr_front_right_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_imu_link : public TransformForce<Type_fr_base_link_X_fr_imu_link>
    {
        Type_fr_base_link_X_fr_imu_link();
        const Type_fr_base_link_X_fr_imu_link& update(const state_t&);
    };
    
    struct Type_fr_imu_link_X_fr_base_link : public TransformForce<Type_fr_imu_link_X_fr_base_link>
    {
        Type_fr_imu_link_X_fr_base_link();
        const Type_fr_imu_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_mid_mount : public TransformForce<Type_fr_base_link_X_fr_mid_mount>
    {
        Type_fr_base_link_X_fr_mid_mount();
        const Type_fr_base_link_X_fr_mid_mount& update(const state_t&);
    };
    
    struct Type_fr_mid_mount_X_fr_base_link : public TransformForce<Type_fr_mid_mount_X_fr_base_link>
    {
        Type_fr_mid_mount_X_fr_base_link();
        const Type_fr_mid_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_navsat_link : public TransformForce<Type_fr_base_link_X_fr_navsat_link>
    {
        Type_fr_base_link_X_fr_navsat_link();
        const Type_fr_base_link_X_fr_navsat_link& update(const state_t&);
    };
    
    struct Type_fr_navsat_link_X_fr_base_link : public TransformForce<Type_fr_navsat_link_X_fr_base_link>
    {
        Type_fr_navsat_link_X_fr_base_link();
        const Type_fr_navsat_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_fender_link : public TransformForce<Type_fr_base_link_X_fr_rear_fender_link>
    {
        Type_fr_base_link_X_fr_rear_fender_link();
        const Type_fr_base_link_X_fr_rear_fender_link& update(const state_t&);
    };
    
    struct Type_fr_rear_fender_link_X_fr_base_link : public TransformForce<Type_fr_rear_fender_link_X_fr_base_link>
    {
        Type_fr_rear_fender_link_X_fr_base_link();
        const Type_fr_rear_fender_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel_link_COM : public TransformForce<Type_fr_base_link_X_fr_rear_left_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_rear_left_wheel_link_COM();
        const Type_fr_base_link_X_fr_rear_left_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_rear_left_wheel_link_COM_X_fr_base_link : public TransformForce<Type_fr_rear_left_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_rear_left_wheel_link_COM_X_fr_base_link();
        const Type_fr_rear_left_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_mount : public TransformForce<Type_fr_base_link_X_fr_rear_mount>
    {
        Type_fr_base_link_X_fr_rear_mount();
        const Type_fr_base_link_X_fr_rear_mount& update(const state_t&);
    };
    
    struct Type_fr_rear_mount_X_fr_base_link : public TransformForce<Type_fr_rear_mount_X_fr_base_link>
    {
        Type_fr_rear_mount_X_fr_base_link();
        const Type_fr_rear_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel_link_COM : public TransformForce<Type_fr_base_link_X_fr_rear_right_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_rear_right_wheel_link_COM();
        const Type_fr_base_link_X_fr_rear_right_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_rear_right_wheel_link_COM_X_fr_base_link : public TransformForce<Type_fr_rear_right_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_rear_right_wheel_link_COM_X_fr_base_link();
        const Type_fr_rear_right_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_left_wheel : public TransformForce<Type_fr_base_link_X_fr_front_left_wheel>
    {
        Type_fr_base_link_X_fr_front_left_wheel();
        const Type_fr_base_link_X_fr_front_left_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel : public TransformForce<Type_fr_base_link_X_fr_front_right_wheel>
    {
        Type_fr_base_link_X_fr_front_right_wheel();
        const Type_fr_base_link_X_fr_front_right_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel : public TransformForce<Type_fr_base_link_X_fr_rear_left_wheel>
    {
        Type_fr_base_link_X_fr_rear_left_wheel();
        const Type_fr_base_link_X_fr_rear_left_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel : public TransformForce<Type_fr_base_link_X_fr_rear_right_wheel>
    {
        Type_fr_base_link_X_fr_rear_right_wheel();
        const Type_fr_base_link_X_fr_rear_right_wheel& update(const state_t&);
    };
    
public:
    ForceTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);

    Type_fr_base_link_X_fr_front_left_wheel_link fr_base_link_X_fr_front_left_wheel_link;
    Type_fr_front_left_wheel_link_X_fr_base_link fr_front_left_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_right_wheel_link fr_base_link_X_fr_front_right_wheel_link;
    Type_fr_front_right_wheel_link_X_fr_base_link fr_front_right_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_left_wheel_link fr_base_link_X_fr_rear_left_wheel_link;
    Type_fr_rear_left_wheel_link_X_fr_base_link fr_rear_left_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_right_wheel_link fr_base_link_X_fr_rear_right_wheel_link;
    Type_fr_rear_right_wheel_link_X_fr_base_link fr_rear_right_wheel_link_X_fr_base_link;
    
    Type_fr_base_link_X_fr_base_link_COM fr_base_link_X_fr_base_link_COM;
    
    Type_fr_base_link_COM_X_fr_base_link fr_base_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_chassis_link fr_base_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_base_link fr_chassis_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_fender_link fr_base_link_X_fr_front_fender_link;
    Type_fr_front_fender_link_X_fr_base_link fr_front_fender_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_left_wheel_link_COM fr_base_link_X_fr_front_left_wheel_link_COM;
    Type_fr_front_left_wheel_link_COM_X_fr_base_link fr_front_left_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_front_mount fr_base_link_X_fr_front_mount;
    Type_fr_front_mount_X_fr_base_link fr_front_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_front_right_wheel_link_COM fr_base_link_X_fr_front_right_wheel_link_COM;
    Type_fr_front_right_wheel_link_COM_X_fr_base_link fr_front_right_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_imu_link fr_base_link_X_fr_imu_link;
    Type_fr_imu_link_X_fr_base_link fr_imu_link_X_fr_base_link;
    Type_fr_base_link_X_fr_mid_mount fr_base_link_X_fr_mid_mount;
    Type_fr_mid_mount_X_fr_base_link fr_mid_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_navsat_link fr_base_link_X_fr_navsat_link;
    Type_fr_navsat_link_X_fr_base_link fr_navsat_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_fender_link fr_base_link_X_fr_rear_fender_link;
    Type_fr_rear_fender_link_X_fr_base_link fr_rear_fender_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_left_wheel_link_COM fr_base_link_X_fr_rear_left_wheel_link_COM;
    Type_fr_rear_left_wheel_link_COM_X_fr_base_link fr_rear_left_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_mount fr_base_link_X_fr_rear_mount;
    Type_fr_rear_mount_X_fr_base_link fr_rear_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_right_wheel_link_COM fr_base_link_X_fr_rear_right_wheel_link_COM;
    Type_fr_rear_right_wheel_link_COM_X_fr_base_link fr_rear_right_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_front_left_wheel fr_base_link_X_fr_front_left_wheel;
    Type_fr_base_link_X_fr_front_right_wheel fr_base_link_X_fr_front_right_wheel;
    Type_fr_base_link_X_fr_rear_left_wheel fr_base_link_X_fr_rear_left_wheel;
    Type_fr_base_link_X_fr_rear_right_wheel fr_base_link_X_fr_rear_right_wheel;

protected:
    Parameters params;

}; //class 'ForceTransforms'

/**
 * The class with the homogeneous (4x4) coordinates transformation
 * matrices.
 */
class HomogeneousTransforms
{
public:
    class Dummy {};
    typedef TransformHomogeneous<Dummy>::MatrixType MatrixType;

    struct Type_fr_base_link_X_fr_front_left_wheel_link : public TransformHomogeneous<Type_fr_base_link_X_fr_front_left_wheel_link>
    {
        Type_fr_base_link_X_fr_front_left_wheel_link();
        const Type_fr_base_link_X_fr_front_left_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_front_left_wheel_link_X_fr_base_link : public TransformHomogeneous<Type_fr_front_left_wheel_link_X_fr_base_link>
    {
        Type_fr_front_left_wheel_link_X_fr_base_link();
        const Type_fr_front_left_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel_link : public TransformHomogeneous<Type_fr_base_link_X_fr_front_right_wheel_link>
    {
        Type_fr_base_link_X_fr_front_right_wheel_link();
        const Type_fr_base_link_X_fr_front_right_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_front_right_wheel_link_X_fr_base_link : public TransformHomogeneous<Type_fr_front_right_wheel_link_X_fr_base_link>
    {
        Type_fr_front_right_wheel_link_X_fr_base_link();
        const Type_fr_front_right_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel_link : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_left_wheel_link>
    {
        Type_fr_base_link_X_fr_rear_left_wheel_link();
        const Type_fr_base_link_X_fr_rear_left_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_rear_left_wheel_link_X_fr_base_link : public TransformHomogeneous<Type_fr_rear_left_wheel_link_X_fr_base_link>
    {
        Type_fr_rear_left_wheel_link_X_fr_base_link();
        const Type_fr_rear_left_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel_link : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_right_wheel_link>
    {
        Type_fr_base_link_X_fr_rear_right_wheel_link();
        const Type_fr_base_link_X_fr_rear_right_wheel_link& update(const state_t&);
    };
    
    struct Type_fr_rear_right_wheel_link_X_fr_base_link : public TransformHomogeneous<Type_fr_rear_right_wheel_link_X_fr_base_link>
    {
        Type_fr_rear_right_wheel_link_X_fr_base_link();
        const Type_fr_rear_right_wheel_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_base_link_COM : public TransformHomogeneous<Type_fr_base_link_X_fr_base_link_COM>
    {
        Type_fr_base_link_X_fr_base_link_COM();
        const Type_fr_base_link_X_fr_base_link_COM& update(const state_t&);
    };
    
    struct Type_fr_base_link_COM_X_fr_base_link : public TransformHomogeneous<Type_fr_base_link_COM_X_fr_base_link>
    {
        Type_fr_base_link_COM_X_fr_base_link();
        const Type_fr_base_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_chassis_link : public TransformHomogeneous<Type_fr_base_link_X_fr_chassis_link>
    {
        Type_fr_base_link_X_fr_chassis_link();
        const Type_fr_base_link_X_fr_chassis_link& update(const state_t&);
    };
    
    struct Type_fr_chassis_link_X_fr_base_link : public TransformHomogeneous<Type_fr_chassis_link_X_fr_base_link>
    {
        Type_fr_chassis_link_X_fr_base_link();
        const Type_fr_chassis_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_fender_link : public TransformHomogeneous<Type_fr_base_link_X_fr_front_fender_link>
    {
        Type_fr_base_link_X_fr_front_fender_link();
        const Type_fr_base_link_X_fr_front_fender_link& update(const state_t&);
    };
    
    struct Type_fr_front_fender_link_X_fr_base_link : public TransformHomogeneous<Type_fr_front_fender_link_X_fr_base_link>
    {
        Type_fr_front_fender_link_X_fr_base_link();
        const Type_fr_front_fender_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_left_wheel_link_COM : public TransformHomogeneous<Type_fr_base_link_X_fr_front_left_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_front_left_wheel_link_COM();
        const Type_fr_base_link_X_fr_front_left_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_front_left_wheel_link_COM_X_fr_base_link : public TransformHomogeneous<Type_fr_front_left_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_front_left_wheel_link_COM_X_fr_base_link();
        const Type_fr_front_left_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_mount : public TransformHomogeneous<Type_fr_base_link_X_fr_front_mount>
    {
        Type_fr_base_link_X_fr_front_mount();
        const Type_fr_base_link_X_fr_front_mount& update(const state_t&);
    };
    
    struct Type_fr_front_mount_X_fr_base_link : public TransformHomogeneous<Type_fr_front_mount_X_fr_base_link>
    {
        Type_fr_front_mount_X_fr_base_link();
        const Type_fr_front_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel_link_COM : public TransformHomogeneous<Type_fr_base_link_X_fr_front_right_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_front_right_wheel_link_COM();
        const Type_fr_base_link_X_fr_front_right_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_front_right_wheel_link_COM_X_fr_base_link : public TransformHomogeneous<Type_fr_front_right_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_front_right_wheel_link_COM_X_fr_base_link();
        const Type_fr_front_right_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_imu_link : public TransformHomogeneous<Type_fr_base_link_X_fr_imu_link>
    {
        Type_fr_base_link_X_fr_imu_link();
        const Type_fr_base_link_X_fr_imu_link& update(const state_t&);
    };
    
    struct Type_fr_imu_link_X_fr_base_link : public TransformHomogeneous<Type_fr_imu_link_X_fr_base_link>
    {
        Type_fr_imu_link_X_fr_base_link();
        const Type_fr_imu_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_mid_mount : public TransformHomogeneous<Type_fr_base_link_X_fr_mid_mount>
    {
        Type_fr_base_link_X_fr_mid_mount();
        const Type_fr_base_link_X_fr_mid_mount& update(const state_t&);
    };
    
    struct Type_fr_mid_mount_X_fr_base_link : public TransformHomogeneous<Type_fr_mid_mount_X_fr_base_link>
    {
        Type_fr_mid_mount_X_fr_base_link();
        const Type_fr_mid_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_navsat_link : public TransformHomogeneous<Type_fr_base_link_X_fr_navsat_link>
    {
        Type_fr_base_link_X_fr_navsat_link();
        const Type_fr_base_link_X_fr_navsat_link& update(const state_t&);
    };
    
    struct Type_fr_navsat_link_X_fr_base_link : public TransformHomogeneous<Type_fr_navsat_link_X_fr_base_link>
    {
        Type_fr_navsat_link_X_fr_base_link();
        const Type_fr_navsat_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_fender_link : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_fender_link>
    {
        Type_fr_base_link_X_fr_rear_fender_link();
        const Type_fr_base_link_X_fr_rear_fender_link& update(const state_t&);
    };
    
    struct Type_fr_rear_fender_link_X_fr_base_link : public TransformHomogeneous<Type_fr_rear_fender_link_X_fr_base_link>
    {
        Type_fr_rear_fender_link_X_fr_base_link();
        const Type_fr_rear_fender_link_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel_link_COM : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_left_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_rear_left_wheel_link_COM();
        const Type_fr_base_link_X_fr_rear_left_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_rear_left_wheel_link_COM_X_fr_base_link : public TransformHomogeneous<Type_fr_rear_left_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_rear_left_wheel_link_COM_X_fr_base_link();
        const Type_fr_rear_left_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_mount : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_mount>
    {
        Type_fr_base_link_X_fr_rear_mount();
        const Type_fr_base_link_X_fr_rear_mount& update(const state_t&);
    };
    
    struct Type_fr_rear_mount_X_fr_base_link : public TransformHomogeneous<Type_fr_rear_mount_X_fr_base_link>
    {
        Type_fr_rear_mount_X_fr_base_link();
        const Type_fr_rear_mount_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel_link_COM : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_right_wheel_link_COM>
    {
        Type_fr_base_link_X_fr_rear_right_wheel_link_COM();
        const Type_fr_base_link_X_fr_rear_right_wheel_link_COM& update(const state_t&);
    };
    
    struct Type_fr_rear_right_wheel_link_COM_X_fr_base_link : public TransformHomogeneous<Type_fr_rear_right_wheel_link_COM_X_fr_base_link>
    {
        Type_fr_rear_right_wheel_link_COM_X_fr_base_link();
        const Type_fr_rear_right_wheel_link_COM_X_fr_base_link& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_left_wheel : public TransformHomogeneous<Type_fr_base_link_X_fr_front_left_wheel>
    {
        Type_fr_base_link_X_fr_front_left_wheel();
        const Type_fr_base_link_X_fr_front_left_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_front_right_wheel : public TransformHomogeneous<Type_fr_base_link_X_fr_front_right_wheel>
    {
        Type_fr_base_link_X_fr_front_right_wheel();
        const Type_fr_base_link_X_fr_front_right_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_left_wheel : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_left_wheel>
    {
        Type_fr_base_link_X_fr_rear_left_wheel();
        const Type_fr_base_link_X_fr_rear_left_wheel& update(const state_t&);
    };
    
    struct Type_fr_base_link_X_fr_rear_right_wheel : public TransformHomogeneous<Type_fr_base_link_X_fr_rear_right_wheel>
    {
        Type_fr_base_link_X_fr_rear_right_wheel();
        const Type_fr_base_link_X_fr_rear_right_wheel& update(const state_t&);
    };
    
public:
    HomogeneousTransforms();
    void updateParams(const Params_lengths&, const Params_angles&);
  
    Type_fr_base_link_X_fr_front_left_wheel_link fr_base_link_X_fr_front_left_wheel_link;
    Type_fr_front_left_wheel_link_X_fr_base_link fr_front_left_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_right_wheel_link fr_base_link_X_fr_front_right_wheel_link;
    Type_fr_front_right_wheel_link_X_fr_base_link fr_front_right_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_left_wheel_link fr_base_link_X_fr_rear_left_wheel_link;
    Type_fr_rear_left_wheel_link_X_fr_base_link fr_rear_left_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_right_wheel_link fr_base_link_X_fr_rear_right_wheel_link;
    Type_fr_rear_right_wheel_link_X_fr_base_link fr_rear_right_wheel_link_X_fr_base_link;
    Type_fr_base_link_X_fr_base_link_COM fr_base_link_X_fr_base_link_COM;
    Type_fr_base_link_COM_X_fr_base_link fr_base_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_chassis_link fr_base_link_X_fr_chassis_link;
    Type_fr_chassis_link_X_fr_base_link fr_chassis_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_fender_link fr_base_link_X_fr_front_fender_link;
    Type_fr_front_fender_link_X_fr_base_link fr_front_fender_link_X_fr_base_link;
    Type_fr_base_link_X_fr_front_left_wheel_link_COM fr_base_link_X_fr_front_left_wheel_link_COM;
    Type_fr_front_left_wheel_link_COM_X_fr_base_link fr_front_left_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_front_mount fr_base_link_X_fr_front_mount;
    Type_fr_front_mount_X_fr_base_link fr_front_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_front_right_wheel_link_COM fr_base_link_X_fr_front_right_wheel_link_COM;
    Type_fr_front_right_wheel_link_COM_X_fr_base_link fr_front_right_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_imu_link fr_base_link_X_fr_imu_link;
    Type_fr_imu_link_X_fr_base_link fr_imu_link_X_fr_base_link;
    Type_fr_base_link_X_fr_mid_mount fr_base_link_X_fr_mid_mount;
    Type_fr_mid_mount_X_fr_base_link fr_mid_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_navsat_link fr_base_link_X_fr_navsat_link;
    Type_fr_navsat_link_X_fr_base_link fr_navsat_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_fender_link fr_base_link_X_fr_rear_fender_link;
    Type_fr_rear_fender_link_X_fr_base_link fr_rear_fender_link_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_left_wheel_link_COM fr_base_link_X_fr_rear_left_wheel_link_COM;
    Type_fr_rear_left_wheel_link_COM_X_fr_base_link fr_rear_left_wheel_link_COM_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_mount fr_base_link_X_fr_rear_mount;
    Type_fr_rear_mount_X_fr_base_link fr_rear_mount_X_fr_base_link;
    Type_fr_base_link_X_fr_rear_right_wheel_link_COM fr_base_link_X_fr_rear_right_wheel_link_COM;
    Type_fr_rear_right_wheel_link_COM_X_fr_base_link fr_rear_right_wheel_link_COM_X_fr_base_link;
  
    Type_fr_base_link_X_fr_front_left_wheel fr_base_link_X_fr_front_left_wheel;
    Type_fr_base_link_X_fr_front_right_wheel fr_base_link_X_fr_front_right_wheel;
    Type_fr_base_link_X_fr_rear_left_wheel fr_base_link_X_fr_rear_left_wheel;
    Type_fr_base_link_X_fr_rear_right_wheel fr_base_link_X_fr_rear_right_wheel;

protected:
    Parameters params;

}; //class 'HomogeneousTransforms'

}
}

#endif

#ifndef JACKAL_JACOBIANS_H_
#define JACKAL_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace Jackal {
namespace rcg {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
        
        struct Type_fr_base_link_J_fr_front_left_wheel_link : public JacobianT<1, Type_fr_base_link_J_fr_front_left_wheel_link>
        {
            Type_fr_base_link_J_fr_front_left_wheel_link();
            const Type_fr_base_link_J_fr_front_left_wheel_link& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_front_right_wheel_link : public JacobianT<1, Type_fr_base_link_J_fr_front_right_wheel_link>
        {
            Type_fr_base_link_J_fr_front_right_wheel_link();
            const Type_fr_base_link_J_fr_front_right_wheel_link& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_rear_left_wheel_link : public JacobianT<1, Type_fr_base_link_J_fr_rear_left_wheel_link>
        {
            Type_fr_base_link_J_fr_rear_left_wheel_link();
            const Type_fr_base_link_J_fr_rear_left_wheel_link& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_rear_right_wheel_link : public JacobianT<1, Type_fr_base_link_J_fr_rear_right_wheel_link>
        {
            Type_fr_base_link_J_fr_rear_right_wheel_link();
            const Type_fr_base_link_J_fr_rear_right_wheel_link& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_front_left_wheel_link_COM : public JacobianT<1, Type_fr_base_link_J_fr_front_left_wheel_link_COM>
        {
            Type_fr_base_link_J_fr_front_left_wheel_link_COM();
            const Type_fr_base_link_J_fr_front_left_wheel_link_COM& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_front_right_wheel_link_COM : public JacobianT<1, Type_fr_base_link_J_fr_front_right_wheel_link_COM>
        {
            Type_fr_base_link_J_fr_front_right_wheel_link_COM();
            const Type_fr_base_link_J_fr_front_right_wheel_link_COM& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_rear_left_wheel_link_COM : public JacobianT<1, Type_fr_base_link_J_fr_rear_left_wheel_link_COM>
        {
            Type_fr_base_link_J_fr_rear_left_wheel_link_COM();
            const Type_fr_base_link_J_fr_rear_left_wheel_link_COM& update(const JointState&);
        };
        
        
        struct Type_fr_base_link_J_fr_rear_right_wheel_link_COM : public JacobianT<1, Type_fr_base_link_J_fr_rear_right_wheel_link_COM>
        {
            Type_fr_base_link_J_fr_rear_right_wheel_link_COM();
            const Type_fr_base_link_J_fr_rear_right_wheel_link_COM& update(const JointState&);
        };
        
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_base_link_J_fr_front_left_wheel_link fr_base_link_J_fr_front_left_wheel_link;
        Type_fr_base_link_J_fr_front_right_wheel_link fr_base_link_J_fr_front_right_wheel_link;
        Type_fr_base_link_J_fr_rear_left_wheel_link fr_base_link_J_fr_rear_left_wheel_link;
        Type_fr_base_link_J_fr_rear_right_wheel_link fr_base_link_J_fr_rear_right_wheel_link;
        Type_fr_base_link_J_fr_front_left_wheel_link_COM fr_base_link_J_fr_front_left_wheel_link_COM;
        Type_fr_base_link_J_fr_front_right_wheel_link_COM fr_base_link_J_fr_front_right_wheel_link_COM;
        Type_fr_base_link_J_fr_rear_left_wheel_link_COM fr_base_link_J_fr_rear_left_wheel_link_COM;
        Type_fr_base_link_J_fr_rear_right_wheel_link_COM fr_base_link_J_fr_rear_right_wheel_link_COM;

    protected:
        Parameters params;

};


}
}

#endif

#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace Jackal::rcg;

Vector3 Jackal::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    Vector3 tmpSum(Vector3::Zero());

    tmpSum += inertiaProps.getCOM_base_link() * inertiaProps.getMass_base_link();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_front_left_wheel_chain;
    HomogeneousTransforms::MatrixType base_X_front_right_wheel_chain;
    HomogeneousTransforms::MatrixType base_X_rear_left_wheel_chain;
    HomogeneousTransforms::MatrixType base_X_rear_right_wheel_chain;
    
    
    base_X_front_left_wheel_chain = tmpX * ht.fr_base_link_X_fr_front_left_wheel_link;
    tmpSum += inertiaProps.getMass_front_left_wheel_link() *
            ( iit::rbd::Utils::transform(base_X_front_left_wheel_chain, inertiaProps.getCOM_front_left_wheel_link()));
    
    base_X_front_right_wheel_chain = tmpX * ht.fr_base_link_X_fr_front_right_wheel_link;
    tmpSum += inertiaProps.getMass_front_right_wheel_link() *
            ( iit::rbd::Utils::transform(base_X_front_right_wheel_chain, inertiaProps.getCOM_front_right_wheel_link()));
    
    base_X_rear_left_wheel_chain = tmpX * ht.fr_base_link_X_fr_rear_left_wheel_link;
    tmpSum += inertiaProps.getMass_rear_left_wheel_link() *
            ( iit::rbd::Utils::transform(base_X_rear_left_wheel_chain, inertiaProps.getCOM_rear_left_wheel_link()));
    
    base_X_rear_right_wheel_chain = tmpX * ht.fr_base_link_X_fr_rear_right_wheel_link;
    tmpSum += inertiaProps.getMass_rear_right_wheel_link() *
            ( iit::rbd::Utils::transform(base_X_rear_right_wheel_chain, inertiaProps.getCOM_rear_right_wheel_link()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

Vector3 Jackal::rcg::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_link_X_fr_front_left_wheel_link(q);
    ht.fr_base_link_X_fr_front_right_wheel_link(q);
    ht.fr_base_link_X_fr_rear_left_wheel_link(q);
    ht.fr_base_link_X_fr_rear_right_wheel_link(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}

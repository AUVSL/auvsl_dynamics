#include "transforms.h"
#include "jsim.h"

#include <iit/rbd/robcogen_commons.h>

using namespace iit::rbd;

//Implementation of default constructor
Jackal::rcg::JSIM::JSIM(InertiaProperties& inertiaProperties, ForceTransforms& forceTransforms) :
    linkInertias(inertiaProperties),
    frcTransf( &forceTransforms ),
    front_left_wheel_link_Ic(linkInertias.getTensor_front_left_wheel_link()),
    front_right_wheel_link_Ic(linkInertias.getTensor_front_right_wheel_link()),
    rear_left_wheel_link_Ic(linkInertias.getTensor_rear_left_wheel_link()),
    rear_right_wheel_link_Ic(linkInertias.getTensor_rear_right_wheel_link())
{
    //Initialize the matrix itself
    this->setZero();
}

#define DATA operator()
#define Fcol(j) (block<6,1>(0,(j)+6))
#define F(i,j) DATA((i),(j)+6)
const Jackal::rcg::JSIM& Jackal::rcg::JSIM::update(const JointState& state) {

    // Precomputes only once the coordinate transforms:
    frcTransf -> fr_base_link_X_fr_rear_right_wheel_link(state);
    frcTransf -> fr_base_link_X_fr_rear_left_wheel_link(state);
    frcTransf -> fr_base_link_X_fr_front_right_wheel_link(state);
    frcTransf -> fr_base_link_X_fr_front_left_wheel_link(state);

    // Initializes the composite inertia tensors
    base_link_Ic = linkInertias.getTensor_base_link();

    // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration

    // Link rear_right_wheel_link:
    iit::rbd::transformInertia<Scalar>(rear_right_wheel_link_Ic, frcTransf -> fr_base_link_X_fr_rear_right_wheel_link, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(REAR_RIGHT_WHEEL) = rear_right_wheel_link_Ic.col(AZ);
    DATA(REAR_RIGHT_WHEEL+6, REAR_RIGHT_WHEEL+6) = Fcol(REAR_RIGHT_WHEEL)(AZ);

    Fcol(REAR_RIGHT_WHEEL) = frcTransf -> fr_base_link_X_fr_rear_right_wheel_link * Fcol(REAR_RIGHT_WHEEL);

    // Link rear_left_wheel_link:
    iit::rbd::transformInertia<Scalar>(rear_left_wheel_link_Ic, frcTransf -> fr_base_link_X_fr_rear_left_wheel_link, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(REAR_LEFT_WHEEL) = rear_left_wheel_link_Ic.col(AZ);
    DATA(REAR_LEFT_WHEEL+6, REAR_LEFT_WHEEL+6) = Fcol(REAR_LEFT_WHEEL)(AZ);

    Fcol(REAR_LEFT_WHEEL) = frcTransf -> fr_base_link_X_fr_rear_left_wheel_link * Fcol(REAR_LEFT_WHEEL);

    // Link front_right_wheel_link:
    iit::rbd::transformInertia<Scalar>(front_right_wheel_link_Ic, frcTransf -> fr_base_link_X_fr_front_right_wheel_link, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(FRONT_RIGHT_WHEEL) = front_right_wheel_link_Ic.col(AZ);
    DATA(FRONT_RIGHT_WHEEL+6, FRONT_RIGHT_WHEEL+6) = Fcol(FRONT_RIGHT_WHEEL)(AZ);

    Fcol(FRONT_RIGHT_WHEEL) = frcTransf -> fr_base_link_X_fr_front_right_wheel_link * Fcol(FRONT_RIGHT_WHEEL);

    // Link front_left_wheel_link:
    iit::rbd::transformInertia<Scalar>(front_left_wheel_link_Ic, frcTransf -> fr_base_link_X_fr_front_left_wheel_link, Ic_spare);
    base_link_Ic += Ic_spare;

    Fcol(FRONT_LEFT_WHEEL) = front_left_wheel_link_Ic.col(AZ);
    DATA(FRONT_LEFT_WHEEL+6, FRONT_LEFT_WHEEL+6) = Fcol(FRONT_LEFT_WHEEL)(AZ);

    Fcol(FRONT_LEFT_WHEEL) = frcTransf -> fr_base_link_X_fr_front_left_wheel_link * Fcol(FRONT_LEFT_WHEEL);

    // Copies the upper-right block into the lower-left block, after transposing
    block<4, 6>(6,0) = (block<6, 4>(0,6)).transpose();
    // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
    block<6,6>(0,0) = base_link_Ic;
    return *this;
}

#undef DATA
#undef F

void Jackal::rcg::JSIM::computeL() {
    L = this -> triangularView<Eigen::Lower>();
    // Joint rear_right_wheel, index 3 :
    L(3, 3) = ScalarTraits::sqrt(L(3, 3));
    
    // Joint rear_left_wheel, index 2 :
    L(2, 2) = ScalarTraits::sqrt(L(2, 2));
    
    // Joint front_right_wheel, index 1 :
    L(1, 1) = ScalarTraits::sqrt(L(1, 1));
    
    // Joint front_left_wheel, index 0 :
    L(0, 0) = ScalarTraits::sqrt(L(0, 0));
    
}

void Jackal::rcg::JSIM::computeInverse() {
    computeLInverse();

    inverse(0, 0) =  + (Linv(0, 0) * Linv(0, 0));
    inverse(1, 1) =  + (Linv(1, 1) * Linv(1, 1));
    inverse(2, 2) =  + (Linv(2, 2) * Linv(2, 2));
    inverse(3, 3) =  + (Linv(3, 3) * Linv(3, 3));
}

void Jackal::rcg::JSIM::computeLInverse() {
    //assumes L has been computed already
    Linv(0, 0) = 1 / L(0, 0);
    Linv(1, 1) = 1 / L(1, 1);
    Linv(2, 2) = 1 / L(2, 2);
    Linv(3, 3) = 1 / L(3, 3);
}

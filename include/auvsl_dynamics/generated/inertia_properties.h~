#ifndef RCG_JACKAL_INERTIA_PROPERTIES_H_
#define RCG_JACKAL_INERTIA_PROPERTIES_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/InertiaMatrix.h>
#include <iit/rbd/utils.h>

#include "declarations.h"
#include "model_constants.h"
#include "dynamics_parameters.h"

namespace Jackal {
namespace rcg {

class InertiaProperties {
    public:
        InertiaProperties();
        ~InertiaProperties();
        const InertiaMatrix& getTensor_base_link() const;
        const InertiaMatrix& getTensor_front_left_wheel_link() const;
        const InertiaMatrix& getTensor_front_right_wheel_link() const;
        const InertiaMatrix& getTensor_rear_left_wheel_link() const;
        const InertiaMatrix& getTensor_rear_right_wheel_link() const;
        Scalar getMass_base_link() const;
        Scalar getMass_front_left_wheel_link() const;
        Scalar getMass_front_right_wheel_link() const;
        Scalar getMass_rear_left_wheel_link() const;
        Scalar getMass_rear_right_wheel_link() const;
        const Vector3& getCOM_base_link() const;
        const Vector3& getCOM_front_left_wheel_link() const;
        const Vector3& getCOM_front_right_wheel_link() const;
        const Vector3& getCOM_rear_left_wheel_link() const;
        const Vector3& getCOM_rear_right_wheel_link() const;
        Scalar getTotalMass() const;


        /*!
         * Fresh values for the runtime parameters of the robot Jackal,
         * causing the update of the inertia properties modeled by this
         * instance.
         */
        void updateParameters(const RuntimeInertiaParams&);

    private:
        RuntimeInertiaParams params;

        InertiaMatrix tensor_base_link;
        InertiaMatrix tensor_front_left_wheel_link;
        InertiaMatrix tensor_front_right_wheel_link;
        InertiaMatrix tensor_rear_left_wheel_link;
        InertiaMatrix tensor_rear_right_wheel_link;
        Vector3 com_base_link;
        Vector3 com_front_left_wheel_link;
        Vector3 com_front_right_wheel_link;
        Vector3 com_rear_left_wheel_link;
        Vector3 com_rear_right_wheel_link;
};


inline InertiaProperties::~InertiaProperties() {}

inline const InertiaMatrix& InertiaProperties::getTensor_base_link() const {
    return this->tensor_base_link;
}
inline const InertiaMatrix& InertiaProperties::getTensor_front_left_wheel_link() const {
    return this->tensor_front_left_wheel_link;
}
inline const InertiaMatrix& InertiaProperties::getTensor_front_right_wheel_link() const {
    return this->tensor_front_right_wheel_link;
}
inline const InertiaMatrix& InertiaProperties::getTensor_rear_left_wheel_link() const {
    return this->tensor_rear_left_wheel_link;
}
inline const InertiaMatrix& InertiaProperties::getTensor_rear_right_wheel_link() const {
    return this->tensor_rear_right_wheel_link;
}
inline Scalar InertiaProperties::getMass_base_link() const {
    return this->tensor_base_link.getMass();
}
inline Scalar InertiaProperties::getMass_front_left_wheel_link() const {
    return this->tensor_front_left_wheel_link.getMass();
}
inline Scalar InertiaProperties::getMass_front_right_wheel_link() const {
    return this->tensor_front_right_wheel_link.getMass();
}
inline Scalar InertiaProperties::getMass_rear_left_wheel_link() const {
    return this->tensor_rear_left_wheel_link.getMass();
}
inline Scalar InertiaProperties::getMass_rear_right_wheel_link() const {
    return this->tensor_rear_right_wheel_link.getMass();
}
inline const Vector3& InertiaProperties::getCOM_base_link() const {
    return this->com_base_link;
}
inline const Vector3& InertiaProperties::getCOM_front_left_wheel_link() const {
    return this->com_front_left_wheel_link;
}
inline const Vector3& InertiaProperties::getCOM_front_right_wheel_link() const {
    return this->com_front_right_wheel_link;
}
inline const Vector3& InertiaProperties::getCOM_rear_left_wheel_link() const {
    return this->com_rear_left_wheel_link;
}
inline const Vector3& InertiaProperties::getCOM_rear_right_wheel_link() const {
    return this->com_rear_right_wheel_link;
}

inline Scalar InertiaProperties::getTotalMass() const {
    return m_base_link + m_front_left_wheel_link + m_front_right_wheel_link + m_rear_left_wheel_link + m_rear_right_wheel_link;
}

}
}

#endif

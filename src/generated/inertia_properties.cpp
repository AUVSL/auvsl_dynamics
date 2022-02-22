#include "inertia_properties.h"

using namespace std;
using namespace iit::rbd;

Jackal::rcg::InertiaProperties::InertiaProperties()
{
    com_base_link = Vector3(comx_base_link,comy_base_link,comz_base_link);
    tensor_base_link.fill(
        m_base_link,
        com_base_link,
        Utils::buildInertiaTensor<Scalar>(ix_base_link,iy_base_link,iz_base_link,ixy_base_link,ixz_base_link,iyz_base_link) );

    com_front_left_wheel_link = Vector3(0.0,0.0,0.0);
    tensor_front_left_wheel_link.fill(
        m_front_left_wheel_link,
        com_front_left_wheel_link,
        Utils::buildInertiaTensor<Scalar>(ix_front_left_wheel_link,iy_front_left_wheel_link,iz_front_left_wheel_link,0.0,0.0,iyz_front_left_wheel_link) );

    com_front_right_wheel_link = Vector3(0.0,0.0,0.0);
    tensor_front_right_wheel_link.fill(
        m_front_right_wheel_link,
        com_front_right_wheel_link,
        Utils::buildInertiaTensor<Scalar>(ix_front_right_wheel_link,iy_front_right_wheel_link,iz_front_right_wheel_link,0.0,0.0,iyz_front_right_wheel_link) );

    com_rear_left_wheel_link = Vector3(0.0,0.0,0.0);
    tensor_rear_left_wheel_link.fill(
        m_rear_left_wheel_link,
        com_rear_left_wheel_link,
        Utils::buildInertiaTensor<Scalar>(ix_rear_left_wheel_link,iy_rear_left_wheel_link,iz_rear_left_wheel_link,0.0,0.0,iyz_rear_left_wheel_link) );

    com_rear_right_wheel_link = Vector3(0.0,0.0,0.0);
    tensor_rear_right_wheel_link.fill(
        m_rear_right_wheel_link,
        com_rear_right_wheel_link,
        Utils::buildInertiaTensor<Scalar>(ix_rear_right_wheel_link,iy_rear_right_wheel_link,iz_rear_right_wheel_link,0.0,0.0,iyz_rear_right_wheel_link) );

}


void Jackal::rcg::InertiaProperties::updateParameters(const RuntimeInertiaParams& fresh)
{
    this-> params = fresh;
}

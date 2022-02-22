#ifndef RCG__JACKAL_TRAITS_H_
#define RCG__JACKAL_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace Jackal {
namespace rcg {
struct Traits {
    typedef typename Jackal::rcg::ScalarTraits ScalarTraits;

    typedef typename Jackal::rcg::JointState JointState;

    typedef typename Jackal::rcg::JointIdentifiers JointID;
    typedef typename Jackal::rcg::LinkIdentifiers  LinkID;

    typedef typename Jackal::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename Jackal::rcg::MotionTransforms MotionTransforms;
    typedef typename Jackal::rcg::ForceTransforms ForceTransforms;

    typedef typename Jackal::rcg::InertiaProperties InertiaProperties;
    typedef typename Jackal::rcg::ForwardDynamics FwdDynEngine;
    typedef typename Jackal::rcg::InverseDynamics InvDynEngine;
    typedef typename Jackal::rcg::JSIM JSIM;

    static const int joints_count = Jackal::rcg::jointsCount;
    static const int links_count  = Jackal::rcg::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return Jackal::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return Jackal::rcg::orderedLinkIDs;
}

}
}

#endif

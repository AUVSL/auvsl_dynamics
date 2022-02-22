#ifndef RCG_JACKAL_DECLARATIONS_H_
#define RCG_JACKAL_DECLARATIONS_H_

#include "rbd_types.h"

namespace Jackal {
namespace rcg {

static constexpr int JointSpaceDimension = 4;
static constexpr int jointsCount = 4;
/** The total number of rigid bodies of this robot, including the base */
static constexpr int linksCount  = 5;

typedef Matrix<4, 1> Column4d;
typedef Column4d JointState;

enum JointIdentifiers {
    FRONT_LEFT_WHEEL = 0
    , FRONT_RIGHT_WHEEL
    , REAR_LEFT_WHEEL
    , REAR_RIGHT_WHEEL
};

enum LinkIdentifiers {
    BASE_LINK = 0
    , FRONT_LEFT_WHEEL_LINK
    , FRONT_RIGHT_WHEEL_LINK
    , REAR_LEFT_WHEEL_LINK
    , REAR_RIGHT_WHEEL_LINK
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {FRONT_LEFT_WHEEL,FRONT_RIGHT_WHEEL,REAR_LEFT_WHEEL,REAR_RIGHT_WHEEL};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE_LINK,FRONT_LEFT_WHEEL_LINK,FRONT_RIGHT_WHEEL_LINK,REAR_LEFT_WHEEL_LINK,REAR_RIGHT_WHEEL_LINK};

}
}
#endif

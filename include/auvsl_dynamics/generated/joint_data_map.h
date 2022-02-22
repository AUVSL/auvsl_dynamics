#ifndef RCG_JACKAL_JOINT_DATA_MAP_H_
#define RCG_JACKAL_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace Jackal {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[FRONT_LEFT_WHEEL] = rhs[FRONT_LEFT_WHEEL];
    data[FRONT_RIGHT_WHEEL] = rhs[FRONT_RIGHT_WHEEL];
    data[REAR_LEFT_WHEEL] = rhs[REAR_LEFT_WHEEL];
    data[REAR_RIGHT_WHEEL] = rhs[REAR_RIGHT_WHEEL];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[FRONT_LEFT_WHEEL] = value;
    data[FRONT_RIGHT_WHEEL] = value;
    data[REAR_LEFT_WHEEL] = value;
    data[REAR_RIGHT_WHEEL] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   front_left_wheel = "
    << map[FRONT_LEFT_WHEEL]
    << "   front_right_wheel = "
    << map[FRONT_RIGHT_WHEEL]
    << "   rear_left_wheel = "
    << map[REAR_LEFT_WHEEL]
    << "   rear_right_wheel = "
    << map[REAR_RIGHT_WHEEL]
    ;
    return out;
}

}
}
#endif

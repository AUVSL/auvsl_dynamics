#ifndef RCG_JACKAL_LINK_DATA_MAP_H_
#define RCG_JACKAL_LINK_DATA_MAP_H_

#include "declarations.h"

namespace Jackal {
namespace rcg {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE_LINK] = rhs[BASE_LINK];
    data[FRONT_LEFT_WHEEL_LINK] = rhs[FRONT_LEFT_WHEEL_LINK];
    data[FRONT_RIGHT_WHEEL_LINK] = rhs[FRONT_RIGHT_WHEEL_LINK];
    data[REAR_LEFT_WHEEL_LINK] = rhs[REAR_LEFT_WHEEL_LINK];
    data[REAR_RIGHT_WHEEL_LINK] = rhs[REAR_RIGHT_WHEEL_LINK];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE_LINK] = value;
    data[FRONT_LEFT_WHEEL_LINK] = value;
    data[FRONT_RIGHT_WHEEL_LINK] = value;
    data[REAR_LEFT_WHEEL_LINK] = value;
    data[REAR_RIGHT_WHEEL_LINK] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base_link = "
    << map[BASE_LINK]
    << "   front_left_wheel_link = "
    << map[FRONT_LEFT_WHEEL_LINK]
    << "   front_right_wheel_link = "
    << map[FRONT_RIGHT_WHEEL_LINK]
    << "   rear_left_wheel_link = "
    << map[REAR_LEFT_WHEEL_LINK]
    << "   rear_right_wheel_link = "
    << map[REAR_RIGHT_WHEEL_LINK]
    ;
    return out;
}

}
}
#endif

#ifndef _JACKAL_RUNTIME_INERTIA_PARAMETERS_
#define _JACKAL_RUNTIME_INERTIA_PARAMETERS_

namespace Jackal {
namespace rcg {

/**
 * A container for the set of runtime inertia parameters of the robot
 * Jackal.
 *
 * Inertia parameters are non-constant inertia-properties, symbolically
 * defined in the robot model.
 * As the value of the parameters must be resolved at runtime, we might refer
 * to them as "runtime parameters", "runtime dynamics parameters",
 * "runtime inertia parameters", etc.
 *
 * Unfortunately, the literature commonly refers to the inertia-properties
 * as "inertia parameters". Do not confuse them. In RobCoGen, the parameters
 * are the non-constant values of the properties.
 */
struct RuntimeInertiaParams {
    RuntimeInertiaParams() {
        defaults();
    }
    void defaults() {
    }
};

}
}
#endif

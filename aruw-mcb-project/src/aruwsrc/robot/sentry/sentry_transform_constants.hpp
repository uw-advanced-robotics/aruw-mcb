#ifndef SENTRY_TRANSFORM_CONSTANTS_HPP_
#define SENTRY_TRANSFORM_CONSTANTS_HPP_
#include "sentry_transforms.hpp"

namespace aruwsrc::sentry
{

SentryTransforms::TransformConfig SENTRY_TRANSFORM_CONFIG{
    .minorToMajorRadius = 0.145f,  // from sentry mk 1 CAD
    .minorToCameraXOffset = 0.143f,
    .minorToCameraYOffset = 0.035f,
    .minorToCameraZOffset = 0.55f
};

}
#endif // SENTRY_TRANSFORMS_CONSTANTS_HPP_
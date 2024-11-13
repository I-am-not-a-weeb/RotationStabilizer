#pragma once

#ifndef _ROTATION_STABILIZER_H_
#define _ROTATION_STABILIZER_H_

#include "helper_3dmath.h"

VectorFloat computeOmegaV(
    const Quaternion& firstQ,
    const Quaternion& secondQ,
    const float dt
);


VectorFloat computeStabilizingVector(
    const Quaternion& currentQ,
    const Quaternion& targetQ,
    Quaternion& previousQ,
    const float deltaTime,
    const float dampingThreshold,
    const float Kp,
    const float Kd
);


Quaternion applyAngularVelocityToQuaternion(const Quaternion& qCurrent, const VectorFloat& angularVelocity, const float dt);

#endif /* _ROTATION_STABILIZER_H_ */
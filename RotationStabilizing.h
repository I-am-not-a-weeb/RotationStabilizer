#pragma once
#include "helper_3dmath.h"
#include <cmath>

VectorFloat computeOmegaV(
    const Quaternion& firstQ,
    const Quaternion& secondQ,
    const float dt
)
{
    return VectorFloat{
        (2 / dt) * (secondQ.w * firstQ.x - secondQ.x * firstQ.w - secondQ.y * firstQ.z + secondQ.z * firstQ.y),
        (2 / dt) * (secondQ.w * firstQ.y + secondQ.x * firstQ.z - secondQ.y * firstQ.w - secondQ.z * firstQ.x),
        (2 / dt) * (secondQ.w * firstQ.z - secondQ.x * firstQ.y + secondQ.y * firstQ.x - secondQ.z * firstQ.w)
    };
}

VectorFloat computeStabilizingVector(
    const Quaternion& currentQ,
    const Quaternion& targetQ,
    Quaternion& previousQ,
    const float deltaTime,
    const float dampingThreshold,
    const float Kp,
    const float Kd
)
{
    const VectorFloat trueAV{ computeOmegaV(currentQ, previousQ, deltaTime) };
    const VectorFloat targetAV{ computeOmegaV(targetQ, currentQ, deltaTime * 2) };

    VectorFloat angularVelocityAdjustment = {
        Kp * (targetAV.x - trueAV.x) + Kd * (0 - trueAV.x),
        Kp * (targetAV.y - trueAV.y) + Kd * (0 - trueAV.y),
        Kp * (targetAV.z - trueAV.z) + Kd * (0 - trueAV.z)
    };

    // ugly but C++17 thing
    if (
        const float angleToTarget{
            std::acos(targetQ.w * currentQ.w +
                targetQ.x * currentQ.x +
                targetQ.y * currentQ.y +
                targetQ.z * currentQ.z)
        };
        angleToTarget < dampingThreshold
        ) {
        // P factor
        const float dampingFactor = angleToTarget / dampingThreshold;
        angularVelocityAdjustment.x *= dampingFactor;
        angularVelocityAdjustment.y *= dampingFactor;
        angularVelocityAdjustment.z *= dampingFactor;
    }

    previousQ = currentQ;

    return angularVelocityAdjustment;

}

Quaternion applyAngularVelocityToQuaternion(const Quaternion& qCurrent, const VectorFloat& angularVelocity, const float dt)
{
    const float omega = angularVelocity.getMagnitude();

    const float halfAngle = omega * dt / 2.0f;
    const float sinHalfAngle = std::sin(halfAngle);
    const float cosHalfAngle = std::cos(halfAngle);

    Quaternion omegaQuat = {
        cosHalfAngle,
        sinHalfAngle * (angularVelocity.x / omega),
        sinHalfAngle * (angularVelocity.y / omega),
        sinHalfAngle * (angularVelocity.z / omega)
    };

    const Quaternion updatedQuaternion = omegaQuat.getProduct(qCurrent);

    return updatedQuaternion.getNormalized();
}
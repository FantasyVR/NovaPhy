#pragma once

#include "novaphy/math/math_types.h"

namespace novaphy {

// demo3d-style AVBD forces (constraints) integrated into VBD solver loop.

struct AvbdIgnoreCollision {
    int body_a = -1;
    int body_b = -1;
};

struct AvbdJoint {
    int body_a = -1;  // -1 means world anchor/orientation
    int body_b = -1;

    Vec3f rA = Vec3f::Zero();  // local anchor in A, or world position if body_a == -1
    Vec3f rB = Vec3f::Zero();  // local anchor in B

    Vec3f C0Lin = Vec3f::Zero();
    Vec3f C0Ang = Vec3f::Zero();

    Vec3f penaltyLin = Vec3f::Zero();
    Vec3f penaltyAng = Vec3f::Zero();
    Vec3f lambdaLin = Vec3f::Zero();
    Vec3f lambdaAng = Vec3f::Zero();

    float stiffnessLin = std::numeric_limits<float>::infinity();
    float stiffnessAng = 0.0f;
    float fracture = std::numeric_limits<float>::infinity();

    float torqueArm = 1.0f;
    bool broken = false;
};

struct AvbdSpring {
    int body_a = -1;
    int body_b = -1;
    Vec3f rA = Vec3f::Zero();  // local anchor in A
    Vec3f rB = Vec3f::Zero();  // local anchor in B
    float rest = -1.0f;
    float stiffness = 0.0f;
};

}  // namespace novaphy


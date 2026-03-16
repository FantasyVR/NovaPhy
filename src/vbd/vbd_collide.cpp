/**
 * @file vbd_collide.cpp
 * @brief Box-box and box-plane collision ported from avbd-demo3d (SAT, feature key, manifold).
 */
#include "novaphy/vbd/vbd_collide.h"
#include <cmath>
#include <cfloat>
#include <algorithm>

namespace novaphy {

namespace {

constexpr int MAX_CONTACTS = 8;
constexpr int MAX_POLY_VERTS = 16;
constexpr float SAT_AXIS_EPSILON = 1.0e-6f;
constexpr float PLANE_EPSILON = 1.0e-5f;
constexpr float CONTACT_MERGE_DIST_SQ = 1.0e-6f;

enum AxisType { AXIS_FACE_A, AXIS_FACE_B, AXIS_EDGE };

struct OBB {
    Vec3f center;
    Quatf rotation;
    Vec3f half;
    Vec3f axis[3];
};

static OBB makeOBB(const Vec3f& pos, const Quatf& q, const Vec3f& half) {
    OBB box{};
    box.center = pos;
    box.rotation = q;
    box.half = half;
    box.axis[0] = q * Vec3f(1.f, 0.f, 0.f);
    box.axis[1] = q * Vec3f(0.f, 1.f, 0.f);
    box.axis[2] = q * Vec3f(0.f, 0.f, 1.f);
    return box;
}

static float absDot(const Vec3f& a, const Vec3f& b) {
    return std::abs(a.dot(b));
}

static Vec3f supportPoint(const OBB& box, const Vec3f& dir) {
    float sx = dir.dot(box.axis[0]) >= 0.f ? 1.f : -1.f;
    float sy = dir.dot(box.axis[1]) >= 0.f ? 1.f : -1.f;
    float sz = dir.dot(box.axis[2]) >= 0.f ? 1.f : -1.f;
    return box.center + box.axis[0] * (box.half.x() * sx) + box.axis[1] * (box.half.y() * sy) + box.axis[2] * (box.half.z() * sz);
}

static void getFaceAxes(const OBB& box, int axisIndex, Vec3f& u, Vec3f& v, float& extentU, float& extentV) {
    if (axisIndex == 0) {
        u = box.axis[1]; v = box.axis[2];
        extentU = box.half.y(); extentV = box.half.z();
    } else if (axisIndex == 1) {
        u = box.axis[0]; v = box.axis[2];
        extentU = box.half.x(); extentV = box.half.z();
    } else {
        u = box.axis[0]; v = box.axis[1];
        extentU = box.half.x(); extentV = box.half.y();
    }
}

struct FaceFrame {
    int axisIndex;
    Vec3f normal, center, u, v;
    float extentU, extentV;
};

static void buildFaceFrame(const OBB& box, int axisIndex, const Vec3f& outwardNormal, FaceFrame& frame) {
    float s = outwardNormal.dot(box.axis[axisIndex]) >= 0.f ? 1.f : -1.f;
    frame.axisIndex = axisIndex;
    frame.normal = box.axis[axisIndex] * s;
    frame.center = box.center + frame.normal * box.half(axisIndex);
    getFaceAxes(box, axisIndex, frame.u, frame.v, frame.extentU, frame.extentV);
}

static int chooseIncidentFaceAxis(const OBB& box, const Vec3f& referenceNormal) {
    int axis = 0;
    float best = -FLT_MAX;
    for (int i = 0; i < 3; ++i) {
        float d = absDot(box.axis[i], referenceNormal);
        if (d > best) { best = d; axis = i; }
    }
    return axis;
}

static void buildIncidentFace(const OBB& box, int axisIndex, const Vec3f& referenceNormal, Vec3f outVerts[4]) {
    float s = box.axis[axisIndex].dot(referenceNormal) > 0.f ? -1.f : 1.f;
    Vec3f faceNormal = box.axis[axisIndex] * s;
    Vec3f faceCenter = box.center + faceNormal * box.half(axisIndex);
    Vec3f u, v;
    float extentU, extentV;
    getFaceAxes(box, axisIndex, u, v, extentU, extentV);
    outVerts[0] = faceCenter + u * extentU + v * extentV;
    outVerts[1] = faceCenter - u * extentU + v * extentV;
    outVerts[2] = faceCenter - u * extentU - v * extentV;
    outVerts[3] = faceCenter + u * extentU - v * extentV;
}

static float clamp(float x, float a, float b) {
    return std::max(a, std::min(b, x));
}

static int clipPolygonAgainstPlane(const Vec3f* inVerts, int inCount, const Vec3f& planeNormal, float planeOffset, Vec3f* outVerts) {
    if (inCount <= 0) return 0;
    int outCount = 0;
    Vec3f a = inVerts[inCount - 1];
    float da = planeNormal.dot(a) - planeOffset;
    for (int i = 0; i < inCount; ++i) {
        Vec3f b = inVerts[i];
        float db = planeNormal.dot(b) - planeOffset;
        bool aInside = da <= PLANE_EPSILON;
        bool bInside = db <= PLANE_EPSILON;
        if (aInside != bInside) {
            float t = 0.f;
            float denom = da - db;
            if (std::abs(denom) > SAT_AXIS_EPSILON) t = clamp(da / denom, 0.f, 1.f);
            if (outCount < MAX_POLY_VERTS) outVerts[outCount++] = a + (b - a) * t;
        }
        if (bInside && outCount < MAX_POLY_VERTS) outVerts[outCount++] = b;
        a = b; da = db;
    }
    return outCount;
}

static bool addContact(const Vec3f& pa, const Quatf& qa, const Vec3f& pb, const Quatf& qb,
                      Vec3f xA, Vec3f xB, int featureKey,
                      std::vector<VbdContactPoint>* contacts, Vec3f* contactMidpoints) {
    Vec3f midpoint = (xA + xB) * 0.5f;
    int n = static_cast<int>(contacts->size());
    for (int i = 0; i < n; ++i) {
        if ((midpoint - contactMidpoints[i]).squaredNorm() < CONTACT_MERGE_DIST_SQ)
            return false;
    }
    if (n >= MAX_CONTACTS) return false;
    VbdContactPoint c;
    c.rA = qa.conjugate() * (xA - pa);
    c.rB = qb.conjugate() * (xB - pb);
    c.feature_key = featureKey;
    contacts->push_back(c);
    contactMidpoints[n] = midpoint;
    return true;
}

struct SatAxis {
    AxisType type;
    int indexA, indexB;
    float separation;
    Vec3f normalAB;
    bool valid;
};

static bool testAxis(const OBB& boxA, const OBB& boxB, const Vec3f& delta, const Vec3f& axis,
                    AxisType type, int indexA, int indexB, SatAxis& best) {
    float lenSq = axis.squaredNorm();
    if (lenSq < SAT_AXIS_EPSILON) return true;
    float invLen = 1.f / std::sqrt(lenSq);
    Vec3f n = axis * invLen;
    if (n.dot(delta) < 0.f) n = -n;
    float distance = std::abs(delta.dot(n));
    float rA = boxA.half.x() * absDot(n, boxA.axis[0]) + boxA.half.y() * absDot(n, boxA.axis[1]) + boxA.half.z() * absDot(n, boxA.axis[2]);
    float rB = boxB.half.x() * absDot(n, boxB.axis[0]) + boxB.half.y() * absDot(n, boxB.axis[1]) + boxB.half.z() * absDot(n, boxB.axis[2]);
    float separation = distance - (rA + rB);
    if (separation > 0.f) return false;
    if (!best.valid || separation > best.separation) {
        best.valid = true;
        best.type = type;
        best.indexA = indexA;
        best.indexB = indexB;
        best.separation = separation;
        best.normalAB = n;
    }
    return true;
}

static void supportEdge(const OBB& box, int axisIndex, const Vec3f& dir, Vec3f& edgeA, Vec3f& edgeB) {
    int axis1 = (axisIndex + 1) % 3;
    int axis2 = (axisIndex + 2) % 3;
    float s1 = dir.dot(box.axis[axis1]) >= 0.f ? 1.f : -1.f;
    float s2 = dir.dot(box.axis[axis2]) >= 0.f ? 1.f : -1.f;
    Vec3f edgeCenter = box.center + box.axis[axis1] * (box.half(axis1) * s1) + box.axis[axis2] * (box.half(axis2) * s2);
    edgeA = edgeCenter - box.axis[axisIndex] * box.half(axisIndex);
    edgeB = edgeCenter + box.axis[axisIndex] * box.half(axisIndex);
}

static void closestPointsOnSegments(const Vec3f& p0, const Vec3f& p1, const Vec3f& q0, const Vec3f& q1, Vec3f& c0, Vec3f& c1) {
    Vec3f d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
    float a = d1.dot(d1), e = d2.dot(d2), f = d2.dot(r);
    float s = 0.f, t = 0.f;
    if (a <= SAT_AXIS_EPSILON && e <= SAT_AXIS_EPSILON) {
        c0 = p0; c1 = q0;
        return;
    }
    if (a <= SAT_AXIS_EPSILON) {
        t = clamp(f / e, 0.f, 1.f);
    } else {
        float c = d1.dot(r);
        if (e <= SAT_AXIS_EPSILON) {
            s = clamp(-c / a, 0.f, 1.f);
        } else {
            float b = d1.dot(d2);
            float denom = a * e - b * b;
            if (std::abs(denom) > SAT_AXIS_EPSILON) s = clamp((b * f - c * e) / denom, 0.f, 1.f);
            t = (b * s + f) / e;
            if (t < 0.f) { t = 0.f; s = clamp(-c / a, 0.f, 1.f); }
            else if (t > 1.f) { t = 1.f; s = clamp((b - c) / a, 0.f, 1.f); }
        }
    }
    c0 = p0 + d1 * s;
    c1 = q0 + d2 * t;
}

static Mat3f orthonormalBasis(const Vec3f& normal) {
    Vec3f n = normal.normalized();
    Vec3f t1 = (std::abs(n.x()) > std::abs(n.z())) ? Vec3f(-n.y(), n.x(), 0.f) : Vec3f(0.f, -n.z(), n.y());
    t1.normalize();
    Vec3f t2 = n.cross(t1);
    t2.normalize();
    Mat3f basis;
    basis.row(0) = n;
    basis.row(1) = t1;
    basis.row(2) = t2;
    return basis;
}

static int buildFaceManifold(const Vec3f& pa, const Quatf& qa, const Vec3f& pb, const Quatf& qb,
                             const OBB& boxA, const OBB& boxB, bool referenceIsA, int referenceAxis,
                             const Vec3f& normalAB, std::vector<VbdContactPoint>* out, Vec3f* midpoints) {
    const OBB& refBox = referenceIsA ? boxA : boxB;
    const OBB& incBox = referenceIsA ? boxB : boxA;
    Vec3f refOutward = referenceIsA ? normalAB : -normalAB;
    FaceFrame refFace{};
    buildFaceFrame(refBox, referenceAxis, refOutward, refFace);
    int incAxis = chooseIncidentFaceAxis(incBox, refFace.normal);
    Vec3f clip0[MAX_POLY_VERTS], clip1[MAX_POLY_VERTS];
    buildIncidentFace(incBox, incAxis, refFace.normal, clip0);
    int count = 4;
    Vec3f n0 = refFace.u;
    float o0 = n0.dot(refFace.center) + refFace.extentU;
    count = clipPolygonAgainstPlane(clip0, count, n0, o0, clip1);
    if (!count) return 0;
    Vec3f n1 = -refFace.u;
    float o1 = n1.dot(refFace.center) + refFace.extentU;
    count = clipPolygonAgainstPlane(clip1, count, n1, o1, clip0);
    if (!count) return 0;
    Vec3f n2 = refFace.v;
    float o2 = n2.dot(refFace.center) + refFace.extentV;
    count = clipPolygonAgainstPlane(clip0, count, n2, o2, clip1);
    if (!count) return 0;
    Vec3f n3 = -refFace.v;
    float o3 = n3.dot(refFace.center) + refFace.extentV;
    count = clipPolygonAgainstPlane(clip1, count, n3, o3, clip0);
    if (!count) return 0;
    int featurePrefix = (referenceIsA ? AXIS_FACE_A : AXIS_FACE_B) << 24;
    featurePrefix |= (referenceAxis & 0xFF) << 16;
    featurePrefix |= (incAxis & 0xFF) << 8;
    int contactCount = 0;
    for (int i = 0; i < count && contactCount < MAX_CONTACTS; ++i) {
        Vec3f pInc = clip0[i];
        float dist = (pInc - refFace.center).dot(refFace.normal);
        if (dist > PLANE_EPSILON) continue;
        Vec3f pRef = pInc - refFace.normal * dist;
        Vec3f xA = referenceIsA ? pRef : pInc;
        Vec3f xB = referenceIsA ? pInc : pRef;
        if (addContact(pa, qa, pb, qb, xA, xB, featurePrefix | (i & 0xFF), out, midpoints))
            ++contactCount;
    }
    if (contactCount == 0) {
        Vec3f xA = supportPoint(boxA, normalAB);
        Vec3f xB = supportPoint(boxB, -normalAB);
        addContact(pa, qa, pb, qb, xA, xB, featurePrefix, out, midpoints);
        contactCount = 1;
    }
    return contactCount;
}

static int buildEdgeContact(const Vec3f& pa, const Quatf& qa, const Vec3f& pb, const Quatf& qb,
                            const OBB& boxA, const OBB& boxB, int axisA, int axisB, const Vec3f& normalAB,
                            std::vector<VbdContactPoint>* out, Vec3f* midpoints) {
    Vec3f a0, a1, b0, b1;
    supportEdge(boxA, axisA, normalAB, a0, a1);
    supportEdge(boxB, axisB, -normalAB, b0, b1);
    Vec3f xA, xB;
    closestPointsOnSegments(a0, a1, b0, b1, xA, xB);
    int featureKey = (AXIS_EDGE << 24) | ((axisA & 0xFF) << 8) | (axisB & 0xFF);
    if (addContact(pa, qa, pb, qb, xA, xB, featureKey, out, midpoints))
        return 1;
    xA = supportPoint(boxA, normalAB);
    xB = supportPoint(boxB, -normalAB);
    addContact(pa, qa, pb, qb, xA, xB, featureKey, out, midpoints);
    return 1;
}

}  // namespace

int vbd_collide_box_box(const Vec3f& pa, const Quatf& qa, const Vec3f& half_a,
                        const Vec3f& pb, const Quatf& qb, const Vec3f& half_b,
                        std::vector<VbdContactPoint>* out, Mat3f* basis_out) {
    if (!out || !basis_out) return 0;
    out->clear();
    OBB boxA = makeOBB(pa, qa, half_a);
    OBB boxB = makeOBB(pb, qb, half_b);
    Vec3f delta = pb - pa;
    SatAxis bestFace{};
    bestFace.separation = -FLT_MAX;
    bestFace.valid = false;
    SatAxis bestEdge{};
    bestEdge.separation = -FLT_MAX;
    bestEdge.valid = false;
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(boxA, boxB, delta, boxA.axis[i], AXIS_FACE_A, i, -1, bestFace)) return 0;
    }
    for (int i = 0; i < 3; ++i) {
        if (!testAxis(boxA, boxB, delta, boxB.axis[i], AXIS_FACE_B, -1, i, bestFace)) return 0;
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3f axis = boxA.axis[i].cross(boxB.axis[j]);
            if (!testAxis(boxA, boxB, delta, axis, AXIS_EDGE, i, j, bestEdge)) return 0;
        }
    }
    if (!bestFace.valid) return 0;
    SatAxis best = bestFace;
    if (bestEdge.valid) {
        const float edgeRelTol = 0.95f, edgeAbsTol = 0.01f;
        if (edgeRelTol * bestEdge.separation > bestFace.separation + edgeAbsTol)
            best = bestEdge;
    }
    *basis_out = orthonormalBasis(-best.normalAB);
    Vec3f midpoints[MAX_CONTACTS];
    if (best.type == AXIS_EDGE) {
        buildEdgeContact(pa, qa, pb, qb, boxA, boxB, best.indexA, best.indexB, best.normalAB, out, midpoints);
        return static_cast<int>(out->size());
    }
    if (best.type == AXIS_FACE_A) {
        buildFaceManifold(pa, qa, pb, qb, boxA, boxB, true, best.indexA, best.normalAB, out, midpoints);
    } else {
        buildFaceManifold(pa, qa, pb, qb, boxA, boxB, false, best.indexB, best.normalAB, out, midpoints);
    }
    return static_cast<int>(out->size());
}

int vbd_collide_box_plane(const Vec3f& n, float d,
                          const Vec3f& pb, const Quatf& qb, const Vec3f& half_b,
                          std::vector<VbdContactPoint>* out, Mat3f* basis_out) {
    if (!out || !basis_out) return 0;
    out->clear();
    Vec3f nn = n.normalized();
    *basis_out = orthonormalBasis(nn);
    Vec3f midpoints[MAX_CONTACTS];
    int featurePrefix = (AXIS_FACE_B << 24);
    int contactCount = 0;
    for (int i = 0; i < 8; ++i) {
        Vec3f corner((i & 1) ? half_b.x() : -half_b.x(),
                     (i & 2) ? half_b.y() : -half_b.y(),
                     (i & 4) ? half_b.z() : -half_b.z());
        Vec3f worldCorner = pb + qb * corner;
        float dist = nn.dot(worldCorner) - d;
        if (dist < 0.f) {
            Vec3f xB = worldCorner;
            Vec3f xA = worldCorner - nn * dist;
            VbdContactPoint c;
            c.rA = xA;
            c.rB = qb.conjugate() * (xB - pb);
            c.feature_key = featurePrefix | (i & 0xFF);
            out->push_back(c);
            midpoints[contactCount++] = (xA + xB) * 0.5f;
            if (contactCount >= MAX_CONTACTS) break;
        }
    }
    return contactCount;
}

}  // namespace novaphy

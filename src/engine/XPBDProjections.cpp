#include <reactphysics3d/engine/XPBDProjections.h>
#include <reactphysics3d/components/RigidBodyComponents.h>

using namespace reactphysics3d;
using namespace std;

XPBDProjections::XPBDProjections(RigidBodyComponents & rigidBodyComponents)
    : mRigidBodyComponents(rigidBodyComponents)
{
}

void XPBDProjections::limitAngleXPBD(uint32 componentIndexBodyA, uint32 componentIndexBodyB, const Vector3 & n, const Vector3 & n1, const Vector3 & n2, decimal minAngle, decimal maxAngle, decimal compliance, decimal dt, decimal & lambda, decimal maxCorr)
{
    // the key function to handle all angular joint limits
    Vector3 c = n1.cross(n2);

    decimal cn = c.dot(n);
    decimal phi = std::asin(std::clamp(cn, decimal(-1.0), decimal(1.0)));
    if (n1.dot(n2) < decimal(0.0))
    {
        phi = PI - phi;
    }
    if (phi > PI)
    {
        phi -= decimal(2.0) * PI;
    }
    else if (phi < -PI)
    {
        phi += decimal(2.0) * PI;
    }

    if ((phi < minAngle) || (phi > maxAngle))
    {
        phi = std::clamp(phi, minAngle, maxAngle);

        // QTR q = QTR::AngleAxisRad(phi, n);
        decimal halfPhi = phi * decimal(0.5);
        decimal cosHalfPhi = std::cos(halfPhi);
        decimal sinHalfPhi = std::sin(halfPhi);
        Quaternion q(sinHalfPhi * n.x, sinHalfPhi * n.y, sinHalfPhi * n.z, cosHalfPhi);

        Vector3 omega = q * n1;
        omega = omega.cross(n2);

        phi = omega.length();
        if (phi > maxCorr)
        {
            omega *= maxCorr / phi;
        }

        applyBodyPairCorrectionXPBD(omega, compliance, dt, lambda, componentIndexBodyA, componentIndexBodyB);
    }
}

void XPBDProjections::limitAngleXPBD(uint32 componentIndexBody1, uint32 componentIndexBody2, const Vector3 & rotationToCurrent, const Vector3 & limitsAnglesMin, const Vector3 & limitsAnglesMax,
    const Quaternion & pivot,
    void (*callbackX)(BallAndSocketJoint * joint, decimal angle, decimal velocity, decimal & outTargetAngle, decimal & outTorque), 
    void (*callbackY)(BallAndSocketJoint * joint, decimal angle, decimal velocity, decimal & outTargetAngle, decimal & outTorque), 
    void (*callbackZ)(BallAndSocketJoint * joint, decimal angle, decimal velocity, decimal & outTargetAngle, decimal & outTorque), 
    BallAndSocketJoint * joint,
    const Vector3 & angularVelocityDeltaProjected, decimal dt, Vector3 & lambdas, decimal maxCorr)
{
    Vector3 targetAngles, torques;

    callbackX(joint, rotationToCurrent.x, angularVelocityDeltaProjected.x, targetAngles.x, torques.x);
    callbackY(joint, rotationToCurrent.y, angularVelocityDeltaProjected.y, targetAngles.y, torques.y);
    callbackZ(joint, rotationToCurrent.z, angularVelocityDeltaProjected.z, targetAngles.z, torques.z);

    if (torques.x != decimal(0.0))
    {
        decimal angleDelta = rotationToCurrent.x - targetAngles.x;
        decimal compliance = angleDelta / torques.x;

        Vector3 omega = pivot.getAxisX() * std::sin(angleDelta);
        applyBodyPairCorrectionXPBD(omega, compliance, dt, lambdas.x, componentIndexBody1, componentIndexBody2);
    }
    if (torques.y != decimal(0.0))
    {
        decimal angleDelta = rotationToCurrent.y - targetAngles.y;
        decimal compliance = angleDelta / torques.y;

        Vector3 omega = pivot.getAxisY() * std::sin(angleDelta);
        applyBodyPairCorrectionXPBD(omega, compliance, dt, lambdas.y, componentIndexBody1, componentIndexBody2);
    }
    if (torques.z != decimal(0.0))
    {
        decimal angleDelta = rotationToCurrent.z - targetAngles.z;
        decimal compliance = angleDelta / torques.z;

        Vector3 omega = pivot.getAxisZ() * std::sin(angleDelta);
        applyBodyPairCorrectionXPBD(omega, compliance, dt, lambdas.z, componentIndexBody1, componentIndexBody2);
    }
}

void XPBDProjections::applyBodyPairCorrectionXPBD(const Vector3 & correction, decimal compliance, decimal dt, decimal & lambda, uint32 componentIndexBody1, uint32 componentIndexBody2)
{
    decimal c = correction.length();
    if (c < MACHINE_EPSILON)
    {
        return;
    }

    Vector3 normal = correction * (decimal(1.0) / c);
    decimal w1 = getGeneralizedInverseMassXPBD(normal, componentIndexBody1);
    decimal w2 = getGeneralizedInverseMassXPBD(normal, componentIndexBody2);
    decimal w = w1 + w2;
    if (w < MACHINE_EPSILON)
    {
        return;
    }

    decimal compliance_ = compliance / (dt * dt);
    lambda = c / (w + compliance_);
    Vector3 correction2 = normal * lambda;

    applyBodyCorrectionXPBD(correction2, componentIndexBody1);
    applyBodyCorrectionXPBD(-correction2, componentIndexBody2);
}

void XPBDProjections::applyBodyPairCorrectionXPBD(const Vector3 & correction, decimal compliance, const Vector3 & r1, const Vector3 & r2, decimal dt, decimal & lambda, uint32 componentIndexBody1, uint32 componentIndexBody2)
{
    decimal c = correction.length();
    if (c < MACHINE_EPSILON)
    {
        return;
    }

    Vector3 dir = correction * (decimal(1.0) / c);
    decimal w1 = getGeneralizedInverseMassXPBD(dir, r1, componentIndexBody1);
    decimal w2 = getGeneralizedInverseMassXPBD(dir, r2, componentIndexBody2);
    decimal w = w1 + w2;
    if (w < MACHINE_EPSILON)
    {
        return;
    }

    decimal compliance_ = compliance / (dt * dt);
    lambda = c / (w + compliance_);
    Vector3 correction2 = dir * lambda;

    applyBodyCorrectionXPBD(correction2, r1, componentIndexBody1);
    applyBodyCorrectionXPBD(-correction2, r2, componentIndexBody2);
}

void XPBDProjections::applyBodyPairCorrectionThresholdXPBD(const Vector3 & correction, decimal compliance, const Vector3 & r1, const Vector3 & r2, decimal dt, decimal & lambda, decimal threshold, uint32 componentIndexBody1, uint32 componentIndexBody2)
{
    decimal c = correction.length();
    if (c < MACHINE_EPSILON)
    {
        return;
    }

    Vector3 dir = correction * (decimal(1.0) / c);
    decimal w1 = getGeneralizedInverseMassXPBD(dir, r1, componentIndexBody1);
    decimal w2 = getGeneralizedInverseMassXPBD(dir, r2, componentIndexBody2);
    decimal w = w1 + w2;
    if (w < MACHINE_EPSILON)
    {
        return;
    }

    decimal compliance_ = compliance / (dt * dt);
    lambda = c / (w + compliance_);
    if (lambda > threshold)
    {
        return;
    }

    Vector3 correction2 = dir * lambda;
    applyBodyCorrectionXPBD(correction2, r1, componentIndexBody1);
    applyBodyCorrectionXPBD(-correction2, r2, componentIndexBody2);
}


void XPBDProjections::applyBodyPairCorrectionVelocityXPBD(const Vector3 & correction, decimal compliance, decimal dt, uint32 componentIndexBody1, uint32 componentIndexBody2)
{
    decimal c = correction.length();
    if (c < MACHINE_EPSILON)
    {
        return;
    }

    Vector3 dir = correction * (decimal(1.0) / c);
    decimal w1 = getGeneralizedInverseMassXPBD(dir, componentIndexBody1);
    decimal w2 = getGeneralizedInverseMassXPBD(dir, componentIndexBody2);
    decimal w = w1 + w2;
    if (w < MACHINE_EPSILON)
    {
        return;
    }

    decimal compliance_ = compliance / (dt * dt);
    decimal lambda = c / (w + compliance_);
    Vector3 correction2 = dir * lambda;

    applyBodyCorrectionVelocityXPBD(correction2, componentIndexBody1);
    applyBodyCorrectionVelocityXPBD(-correction2, componentIndexBody2);
}

void XPBDProjections::applyBodyPairCorrectionVelocityXPBD(const Vector3 & correction, decimal compliance, const Vector3 & r1, const Vector3 & r2, decimal dt, uint32 componentIndexBody1, uint32 componentIndexBody2)
{
    decimal c = correction.length();
    if (c < MACHINE_EPSILON)
    {
        return;
    }

    Vector3 dir = correction * (decimal(1.0) / c);
    decimal w1 = getGeneralizedInverseMassXPBD(dir, r1, componentIndexBody1);
    decimal w2 = getGeneralizedInverseMassXPBD(dir, r2, componentIndexBody2);
    decimal w = w1 + w2;
    if (w < MACHINE_EPSILON)
    {
        return;
    }

    decimal compliance_ = compliance / (dt * dt);
    decimal lambda = c / (w + compliance_);
    Vector3 correction2 = dir * lambda;

    applyBodyCorrectionVelocityXPBD(correction2, r1, componentIndexBody1);
    applyBodyCorrectionVelocityXPBD(-correction2, r2, componentIndexBody2);
}

decimal XPBDProjections::getGeneralizedInverseMassXPBD(const Vector3 & normal, const Vector3 & r, uint32 componentIndexBody)
{
    Vector3 rn = r.cross(normal);
    const Quaternion & orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    const Quaternion & inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[componentIndexBody];
    Quaternion combinedRotation = orientation * inertiaOrientation;

    Vector3 rnRotated = combinedRotation.getInverse() * rn;
    const Vector3 & inertiaInvDiag = mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody];
    Vector3 transf(rnRotated.x * inertiaInvDiag.x, rnRotated.y * inertiaInvDiag.y, rnRotated.z * inertiaInvDiag.z);
    Vector3 invIrn = combinedRotation * transf;

    decimal w = rn.dot(invIrn);
    w += mRigidBodyComponents.mInverseMasses[componentIndexBody];

    return w;
}

decimal XPBDProjections::getGeneralizedInverseMassXPBD(const Vector3 & normal, uint32 componentIndexBody)
{
    Vector3 rn = normal;
    const Quaternion & orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    const Quaternion & inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[componentIndexBody];
    Quaternion combinedRotation = orientation * inertiaOrientation;

    Vector3 rnRotated = combinedRotation.getInverse() * rn;
    const Vector3& inertiaInvDiag = mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody];
    Vector3 transf(rnRotated.x * inertiaInvDiag.x, rnRotated.y * inertiaInvDiag.y, rnRotated.z * inertiaInvDiag.z);
    Vector3 invIrn = combinedRotation * transf;

    decimal w = rn.dot(invIrn);

    return w;
}

void XPBDProjections::applyBodyCorrectionXPBD(const Vector3 & correction, const Vector3 & r, uint32 componentIndexBody)
{
    Vector3 & position = mRigidBodyComponents.mXPBDPositions[componentIndexBody];
    position += correction * mRigidBodyComponents.mInverseMasses[componentIndexBody];
    Vector3 dq = r.cross(correction);

    const Quaternion & orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    const Quaternion & inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[componentIndexBody];
    Quaternion combinedRotation = orientation * inertiaOrientation; // TODO: consider some cashing for combinedRotation and combinedRotation_Inv ^

    Vector3 dqRotated = combinedRotation.getInverse() * dq;
    const Vector3 & inertiaInvDiag = mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody];
    Vector3 transf(dqRotated.x * inertiaInvDiag.x, dqRotated.y * inertiaInvDiag.y, dqRotated.z * inertiaInvDiag.z);

    dq = combinedRotation * transf;
    applyBodyRotationXPBD(dq, componentIndexBody);
}

void XPBDProjections::applyBodyCorrectionXPBD(const Vector3 & correction, uint32 componentIndexBody)
{
    Vector3 dq = correction;

    const Quaternion & orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    const Quaternion & inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[componentIndexBody];
    Quaternion combinedRotation = orientation * inertiaOrientation; // TODO: consider some cashing for combinedRotation and combinedRotation_Inv ^

    Vector3 dqRotated = combinedRotation.getInverse() * dq;
    const Vector3 & inertiaInvDiag = mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody];
    Vector3 transf(dqRotated.x * inertiaInvDiag.x, dqRotated.y * inertiaInvDiag.y, dqRotated.z * inertiaInvDiag.z);
    dq = combinedRotation * transf;

    applyBodyRotationXPBD(dq, componentIndexBody);
}

void XPBDProjections::applyBodyCorrectionVelocityXPBD(const Vector3 & correction, uint32 componentIndexBody)
{
    Vector3 dq = correction;

    const Quaternion& orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    const Quaternion& inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[componentIndexBody];
    Quaternion combinedRotation = orientation * inertiaOrientation; // TODO: consider some cashing for combinedRotation and combinedRotation_Inv ^

    Vector3 dqRotated = combinedRotation.getInverse() * dq;
    const Vector3 & inertiaInvDiag = mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody];
    Vector3 transf(dqRotated.x * inertiaInvDiag.x, dqRotated.y * inertiaInvDiag.y, dqRotated.z * inertiaInvDiag.z);
    dq = combinedRotation * transf;

    mRigidBodyComponents.mAngularVelocities[componentIndexBody] += dq;
}

void XPBDProjections::applyBodyCorrectionVelocityXPBD(const Vector3 & correction, const Vector3 & r, uint32 componentIndexBody)
{
    Vector3& velocity = mRigidBodyComponents.mLinearVelocities[componentIndexBody];
    velocity += correction * mRigidBodyComponents.mInverseMasses[componentIndexBody];
    Vector3 dq = r.cross(correction);

    const Quaternion& orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    const Quaternion& inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[componentIndexBody];
    Quaternion combinedRotation = orientation * inertiaOrientation; // TODO: consider some cashing for combinedRotation and combinedRotation_Inv ^

    Vector3 dqRotated = combinedRotation.getInverse() * dq;
    const Vector3& inertiaInvDiag = mRigidBodyComponents.mInverseInertiaTensorsLocal[componentIndexBody];
    Vector3 transf(dqRotated.x * inertiaInvDiag.x, dqRotated.y * inertiaInvDiag.y, dqRotated.z * inertiaInvDiag.z);
    dq = combinedRotation * transf;

    mRigidBodyComponents.mAngularVelocities[componentIndexBody] += dq;
}

void XPBDProjections::applyBodyRotationXPBD(const Vector3 & rot, uint32 componentIndexBody)
{
    decimal phi = rot.length();

    Quaternion rotQ(rot.x, rot.y, rot.z, 0.0);
    static const decimal MAX_ROTATION_PER_SUBSTEP(0.5); // Safety clamping. This happens very rarely if the solver wants to turn the body by more than 30 degrees in the orders of milliseconds
    if (phi > MAX_ROTATION_PER_SUBSTEP)
    {
        decimal scale = MAX_ROTATION_PER_SUBSTEP / phi;
        rotQ.x *= scale;
        rotQ.y *= scale;
        rotQ.z *= scale;
    }

    Quaternion & orientation = mRigidBodyComponents.mXPBDOrientations[componentIndexBody];
    Quaternion dq = rotQ * orientation;

    orientation.x += decimal(0.5) * dq.x;
    orientation.y += decimal(0.5) * dq.y;
    orientation.z += decimal(0.5) * dq.z;
    orientation.w += decimal(0.5) * dq.w;
    orientation.normalize();
}
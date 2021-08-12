/********************************************************************************
* ReactPhysics3D physics library, http://www.reactphysics3d.com                 *
* Copyright (c) 2010-2020 Daniel Chappuis                                       *
*********************************************************************************
*                                                                               *
* This software is provided 'as-is', without any express or implied warranty.   *
* In no event will the authors be held liable for any damages arising from the  *
* use of this software.                                                         *
*                                                                               *
* Permission is granted to anyone to use this software for any purpose,         *
* including commercial applications, and to alter it and redistribute it        *
* freely, subject to the following restrictions:                                *
*                                                                               *
* 1. The origin of this software must not be misrepresented; you must not claim *
*    that you wrote the original software. If you use this software in a        *
*    product, an acknowledgment in the product documentation would be           *
*    appreciated but is not required.                                           *
*                                                                               *
* 2. Altered source versions must be plainly marked as such, and must not be    *
*    misrepresented as being the original software.                             *
*                                                                               *
* 3. This notice may not be removed or altered from any source distribution.    *
*                                                                               *
********************************************************************************/

// Libraries
#include <reactphysics3d/systems/SolveHingeJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Constructor
SolveHingeJointSystem::SolveHingeJointSystem(PhysicsWorld & world, RigidBodyComponents & rigidBodyComponents, 
    TransformComponents & transformComponents, JointComponents & jointComponents, HingeJointComponents & hingeJointComponents)
    : mWorld(world)
    , mRigidBodyComponents(rigidBodyComponents)
    , mTransformComponents(transformComponents)
    , mJointComponents(jointComponents)
    , mHingeJointComponents(hingeJointComponents)
    , mTimeStep(0)
{
}

// Given an angle in radian, this method returns the corresponding angle in the range [-pi; pi]
decimal SolveHingeJointSystem::computeNormalizedAngle(decimal angle) const 
{
    // Convert it into the range [-2*pi; 2*pi]
    angle = std::fmod(angle, PI_TIMES_2);

    // Convert it into the range [-pi; pi]
    if (angle < -PI) {
        return angle + PI_TIMES_2;
    }
    else if (angle > PI) {
        return angle - PI_TIMES_2;
    }
    else {
        return angle;
    }
}

// Given an "inputAngle" in the range [-pi, pi], this method returns an
// angle (modulo 2*pi) in the range [-2*pi; 2*pi] that is closest to one of the
// two angle limits in arguments.
decimal SolveHingeJointSystem::computeCorrespondingAngleNearLimits(decimal inputAngle, decimal lowerLimitAngle, decimal upperLimitAngle) const 
{
    if (upperLimitAngle <= lowerLimitAngle) {
        return inputAngle;
    }
    else if (inputAngle > upperLimitAngle) {
        decimal diffToUpperLimit = std::fabs(computeNormalizedAngle(inputAngle - upperLimitAngle));
        decimal diffToLowerLimit = std::fabs(computeNormalizedAngle(inputAngle - lowerLimitAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? (inputAngle - PI_TIMES_2) : inputAngle;
    }
    else if (inputAngle < lowerLimitAngle) {
        decimal diffToUpperLimit = std::fabs(computeNormalizedAngle(upperLimitAngle - inputAngle));
        decimal diffToLowerLimit = std::fabs(computeNormalizedAngle(lowerLimitAngle - inputAngle));
        return (diffToUpperLimit > diffToLowerLimit) ? inputAngle : (inputAngle + PI_TIMES_2);
    }
    else {
        return inputAngle;
    }
}

// Compute the current angle around the hinge axis
decimal SolveHingeJointSystem::computeCurrentHingeAngle(Entity jointEntity, const Quaternion & orientationBody1, const Quaternion & orientationBody2) 
{
    decimal hingeAngle;

    // Compute the current orientation difference between the two bodies
    Quaternion currentOrientationDiff = orientationBody2 * orientationBody1.getInverse();
    currentOrientationDiff.normalize();

    // Compute the relative rotation considering the initial orientation difference
    Quaternion relativeRotation = currentOrientationDiff * mHingeJointComponents.getInitOrientationDifferenceInv(jointEntity);
    relativeRotation.normalize();

    // A quaternion q = [cos(theta/2); sin(theta/2) * rotAxis] where rotAxis is a unit
    // length vector. We can extract cos(theta/2) with q.w and we can extract |sin(theta/2)| with :
    // |sin(theta/2)| = q.getVectorV().length() since rotAxis is unit length. Note that any
    // rotation can be represented by a quaternion q and -q. Therefore, if the relative rotation
    // axis is not pointing in the same direction as the hinge axis, we use the rotation -q which
    // has the same |sin(theta/2)| value but the value cos(theta/2) is sign inverted. Some details
    // about this trick is explained in the source code of OpenTissue (http://www.opentissue.org).
    decimal cosHalfAngle = relativeRotation.w;
    decimal sinHalfAngleAbs = relativeRotation.getVectorV().length();

    // Compute the dot product of the relative rotation axis and the hinge axis
    decimal dotProduct = relativeRotation.getVectorV().dot(mHingeJointComponents.getA1(jointEntity));

    // If the relative rotation axis and the hinge axis are pointing the same direction
    if (dotProduct >= decimal(0.0)) {
        hingeAngle = decimal(2.0) * std::atan2(sinHalfAngleAbs, cosHalfAngle);
    }
    else {
        hingeAngle = decimal(2.0) * std::atan2(sinHalfAngleAbs, -cosHalfAngle);
    }

    // Convert the angle from range [-2*pi; 2*pi] into the range [-pi; pi]
    hingeAngle = computeNormalizedAngle(hingeAngle);

    // Compute and return the corresponding angle near one the two limits
    return computeCorrespondingAngleNearLimits(hingeAngle,
                                               mHingeJointComponents.getLowerLimit(jointEntity),
                                               mHingeJointComponents.getUpperLimit(jointEntity));
}

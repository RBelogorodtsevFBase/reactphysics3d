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
#include <reactphysics3d/systems/DynamicsSystem.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/engine/PhysicsWorld.h>

using namespace reactphysics3d;

// Constructor
DynamicsSystem::DynamicsSystem(PhysicsWorld & world, CollisionBodyComponents & collisionBodyComponents, RigidBodyComponents & rigidBodyComponents, 
    TransformComponents & transformComponents, ColliderComponents & colliderComponents, bool& isGravityEnabled, Vector3 & gravity)
    : mWorld(world)
    , mCollisionBodyComponents(collisionBodyComponents)
    , mRigidBodyComponents(rigidBodyComponents)
    , mTransformComponents(transformComponents)
    , mColliderComponents(colliderComponents)
    , mIsGravityEnabled(isGravityEnabled)
    , mGravity(gravity)
{
}

void DynamicsSystem::initPositionsOrientations()
{
    RP3D_PROFILE("DynamicsSystem::initPositionsOrientations()", mProfiler);

    for (uint32 i = 0; i < mRigidBodyComponents.getNbEnabledComponents(); i++)
    {
        const Vector3 & position = mRigidBodyComponents.mCentersOfMassWorld[i];
        const Quaternion & orientation = mTransformComponents.getTransform(mRigidBodyComponents.mBodiesEntities[i]).getOrientation();

        mRigidBodyComponents.mXPBDPositions[i] = position;
        mRigidBodyComponents.mXPBDOrientations[i] = orientation;
    }
}

void DynamicsSystem::backUpPositionsOrientationsXPBD()
{
    RP3D_PROFILE("DynamicsSystem::backUpPositionsOrientationsXPBD()", mProfiler);

    for (uint32 i = 0; i < mRigidBodyComponents.getNbEnabledComponents(); i++)
    {
        mRigidBodyComponents.mXPBDPositionsPrevious[i] = mRigidBodyComponents.mXPBDPositions[i];
        mRigidBodyComponents.mXPBDOrientationsPrevious[i] = mRigidBodyComponents.mXPBDOrientations[i];
    }
}

void DynamicsSystem::integrateRigidBodiesXPBD(decimal timeSubStep)
{
    RP3D_PROFILE("DynamicsSystem::integrateRigidBodiesXPBD()", mProfiler);

    for (uint32 i=0; i < mRigidBodyComponents.getNbEnabledComponents(); i++) 
    {
        // Calculate new linear acceleration and velocity
        Vector3 linearAcceleration = mRigidBodyComponents.mInverseMasses[i] * mRigidBodyComponents.mExternalForces[i];
        if (mIsGravityEnabled && mRigidBodyComponents.mIsGravityEnabled[i])
        {
            linearAcceleration += mRigidBodyComponents.mInverseMasses[i] * mRigidBodyComponents.mMasses[i] * mGravity;
        }
        Vector3 linearVelocity = mRigidBodyComponents.mLinearVelocities[i] + timeSubStep * linearAcceleration;
        // Calculate new position
        mRigidBodyComponents.mXPBDPositions[i] += linearVelocity * timeSubStep;

        // Calculate new angular aceleration and velocity
        const Quaternion & currentOrientation = mRigidBodyComponents.mXPBDOrientations[i];
        const Quaternion & inertiaOrientation = mRigidBodyComponents.mLocalInertiaOrientations[i];
        Quaternion combinedRotation = currentOrientation * inertiaOrientation;
        Quaternion combinedRotationInverse = combinedRotation.getInverse();

        const Vector3 & inertiaLocalTensor = mRigidBodyComponents.mLocalInertiaTensors[i];
        const Vector3 & inertiaLocalTensorInverse = mRigidBodyComponents.mInverseInertiaTensorsLocal[i];

        const Vector3 & currentAngularVelocity = mRigidBodyComponents.mAngularVelocities[i];
        Vector3 currentAngularVelocityRotated = combinedRotationInverse * currentAngularVelocity;
        Vector3 currentAngularVelocityRotatedTransformed(currentAngularVelocityRotated.x * inertiaLocalTensor.x, 
            currentAngularVelocityRotated.y * inertiaLocalTensor.y,
            currentAngularVelocityRotated.z * inertiaLocalTensor.z);
        Vector3 gyroscopicTorque = currentAngularVelocity.cross(combinedRotation * currentAngularVelocityRotatedTransformed);

        Vector3 torqueCombined = mRigidBodyComponents.mExternalTorques[i] - gyroscopicTorque;

        Vector3 torqueCombinedRotated = combinedRotationInverse * torqueCombined;
        Vector3 torqueCombinedRotatedTransformed(torqueCombinedRotated.x * inertiaLocalTensorInverse.x, 
            torqueCombinedRotated.y * inertiaLocalTensorInverse.y, 
            torqueCombinedRotated.z * inertiaLocalTensorInverse.z);
        Vector3 angularAcceleration = combinedRotation * torqueCombinedRotatedTransformed;

        Vector3 angularVelocity = currentAngularVelocity + timeSubStep * angularAcceleration;

        // Calculate new orientation
        decimal scale = timeSubStep;
        decimal phi = angularVelocity.length();

        // Safety clamping. This happens very rarely if the solver wants to turn the body by more than 30 degrees in the orders of milliseconds
        static const decimal MAX_ROTATION_PER_SUBSTEP(0.5);
        if (phi * scale > MAX_ROTATION_PER_SUBSTEP)
        {
            scale = MAX_ROTATION_PER_SUBSTEP / phi;
        }
        Quaternion dq = Quaternion(angularVelocity.x * scale, angularVelocity.y * scale, angularVelocity.z * scale, decimal(0.0)) * currentOrientation;
        mRigidBodyComponents.mXPBDOrientations[i].x += decimal(0.5) * dq.x;
        mRigidBodyComponents.mXPBDOrientations[i].y += decimal(0.5) * dq.y;
        mRigidBodyComponents.mXPBDOrientations[i].z += decimal(0.5) * dq.z;
        mRigidBodyComponents.mXPBDOrientations[i].w += decimal(0.5) * dq.w;
        mRigidBodyComponents.mXPBDOrientations[i].normalize();
    }
}

void DynamicsSystem::infereVelocitiesXPBD(decimal timeSubStepInv, decimal doubleTimeSubStepInv)
{
    RP3D_PROFILE("DynamicsSystem::infereVelocitiesXPBD()", mProfiler);

    for (uint32 i = 0; i < mRigidBodyComponents.getNbEnabledComponents(); i++)
    {
        // Update the linear and angular velocity of the body
        Vector3 positionDelta = mRigidBodyComponents.mXPBDPositions[i] - mRigidBodyComponents.mXPBDPositionsPrevious[i];
        mRigidBodyComponents.mLinearVelocities[i] = positionDelta * timeSubStepInv;

        Quaternion dq = mRigidBodyComponents.mXPBDOrientations[i] * mRigidBodyComponents.mXPBDOrientationsPrevious[i].getInverse(); // TODO : there can be non-identity dq even if orientation == previousOrientation
        if (dq.w >= decimal(0.0))
        {
            mRigidBodyComponents.mAngularVelocities[i] = Vector3(dq.x * doubleTimeSubStepInv, dq.y * doubleTimeSubStepInv, dq.z * doubleTimeSubStepInv);
        }
        else
        {
            mRigidBodyComponents.mAngularVelocities[i] = Vector3(-dq.x * doubleTimeSubStepInv, -dq.y * doubleTimeSubStepInv, -dq.z * doubleTimeSubStepInv);
        }
    }
}


void DynamicsSystem::updateBodiesStatesXPBD()
{
    RP3D_PROFILE("DynamicsSystem::updateBodiesStatesXPBD()", mProfiler);

    for (uint32 i = 0; i < mRigidBodyComponents.getNbEnabledComponents(); i++)
    {
        // Update the position of the center of mass of the body
        mRigidBodyComponents.mCentersOfMassWorld[i] = mRigidBodyComponents.mXPBDPositions[i];

        // Update the orientation of the body
        mTransformComponents.getTransform(mRigidBodyComponents.mBodiesEntities[i]).setOrientation(mRigidBodyComponents.mXPBDOrientations[i]);
    }

    // Update the position of the body (using the new center of mass and new orientation)
    for (uint32 i = 0; i < mRigidBodyComponents.getNbEnabledComponents(); i++) 
    {
        Transform& transform = mTransformComponents.getTransform(mRigidBodyComponents.mBodiesEntities[i]);
        const Vector3& centerOfMassWorld = mRigidBodyComponents.mCentersOfMassWorld[i];
        const Vector3& centerOfMassLocal = mRigidBodyComponents.mCentersOfMassLocal[i];
        transform.setPosition(centerOfMassWorld - transform.getOrientation() * centerOfMassLocal);
    }

    // Update the local-to-world transform of the colliders
    for (uint32 i = 0; i < mColliderComponents.getNbEnabledComponents(); i++) 
    {
        // Update the local-to-world transform of the collider
        mColliderComponents.mLocalToWorldTransforms[i] = mTransformComponents.getTransform(mColliderComponents.mBodiesEntities[i]) * mColliderComponents.mLocalToBodyTransforms[i];
    }
}

// Reset the external force and torque applied to the bodies
void DynamicsSystem::resetBodiesForceAndTorque() 
{
    // For each body of the world
    for (uint32 i=0; i < mRigidBodyComponents.getNbComponents(); i++) 
    {
        mRigidBodyComponents.mExternalForces[i].setToZero();
        mRigidBodyComponents.mExternalTorques[i].setToZero();
    }
}

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
#include <reactphysics3d/systems/SolveBallAndSocketJointSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>

using namespace reactphysics3d;

// Constructor
SolveBallAndSocketJointSystem::SolveBallAndSocketJointSystem(PhysicsWorld & world, RigidBodyComponents & rigidBodyComponents, 
    TransformComponents & transformComponents, JointComponents & jointComponents, BallAndSocketJointComponents & ballAndSocketJointComponents)
    : mWorld(world)
    , mRigidBodyComponents(rigidBodyComponents)
    , mTransformComponents(transformComponents)
    , mJointComponents(jointComponents)
    , mBallAndSocketJointComponents(ballAndSocketJointComponents)
    , mTimeStep(0)
    , mXPBDProjections(rigidBodyComponents)
{
}

void SolveBallAndSocketJointSystem::solvePositionXPBD(decimal timeSubStep)
{
    // For each joint component
    for (uint32 i = 0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++) 
    {
        const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

        const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
        const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

        const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
        const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

        const Vector3 & positionBody1 = mRigidBodyComponents.mXPBDPositions[componentIndexBody1];
        const Vector3 & positionBody2 = mRigidBodyComponents.mXPBDPositions[componentIndexBody2];

        const Quaternion & orientationBody1 = mRigidBodyComponents.mXPBDOrientations[componentIndexBody1];
        const Quaternion & orientationBody2 = mRigidBodyComponents.mXPBDOrientations[componentIndexBody2];

        const Quaternion & localOrientationTarget = mBallAndSocketJointComponents.mTargetLocalInBody1[i];
        const Quaternion & localOrientation2 = mBallAndSocketJointComponents.mReferenceLocalInBody2[i];

        const Vector3 & limitsAnglesMin = mBallAndSocketJointComponents.mLimitsAnglesMin[i];
        const Vector3 & limitsAnglesMax = mBallAndSocketJointComponents.mLimitsAnglesMax[i];

        Vector3 angularVelocityDelta = mRigidBodyComponents.mAngularVelocities[componentIndexBody2] - mRigidBodyComponents.mAngularVelocities[componentIndexBody1];

        BallAndSocketJoint * joint = mBallAndSocketJointComponents.mJoints[i];

        // Swing X
        {
            Quaternion globalOrientationTarget = orientationBody1 * localOrientationTarget;
            Quaternion globalOrientation2 = orientationBody2 * localOrientation2;

            Vector3 z1 = globalOrientationTarget.getAxisZ();
            Vector3 z2 = globalOrientation2.getAxisZ();

            Vector3 x1 = globalOrientationTarget.getAxisX();
            Vector3 z2_ = z2 - x1 * z2.dot(x1);
            z2_.normalize();

            decimal & lambda = mBallAndSocketJointComponents.mSwingXLambda[i];
            decimal angularVelocityDeltaProjected = angularVelocityDelta.dot(x1);

            mXPBDProjections.limitAngleXPBD(componentIndexBody1, componentIndexBody2, x1, z1, z2_, limitsAnglesMin.x, limitsAnglesMax.x, 
                mBallAndSocketJointComponents.mCallbacksX[i], joint, angularVelocityDeltaProjected, timeSubStep, lambda);
        }

        // Swing Y
        {
            Quaternion globalOrientationTarget = orientationBody1 * localOrientationTarget;
            Quaternion globalOrientation2 = orientationBody2 * localOrientation2;

            Vector3 z1 = globalOrientationTarget.getAxisZ();
            Vector3 z2 = globalOrientation2.getAxisZ();

            Vector3 y1 = globalOrientationTarget.getAxisY();
            Vector3 z2_ = z2 - y1 * z2.dot(y1);
            z2_.normalize();

            decimal & lambda = mBallAndSocketJointComponents.mSwingYLambda[i];
            decimal angularVelocityDeltaProjected = angularVelocityDelta.dot(y1);

            mXPBDProjections.limitAngleXPBD(componentIndexBody1, componentIndexBody2, y1, z1, z2_, limitsAnglesMin.y, limitsAnglesMax.y, 
                mBallAndSocketJointComponents.mCallbacksY[i], joint, angularVelocityDeltaProjected, timeSubStep, lambda);
        }

        // Twist
        {
            Quaternion globalOrientationTarget = orientationBody1 * localOrientationTarget;
            Quaternion globalOrientation2 = orientationBody2 * localOrientation2;

            Vector3 n1 = globalOrientationTarget.getAxisZ();
            Vector3 n2 = globalOrientation2.getAxisZ();
            Vector3 n = (n1 + n2);
            n.normalize();

            Vector3 a1 = globalOrientationTarget.getAxisX();
            a1 -= n * n.dot(a1);
            a1.normalize();

            Vector3 a2 = globalOrientation2.getAxisX();
            a2 -= n * n.dot(a2);
            a2.normalize();

            // handling gimbal lock problem
            decimal maxCorr;
            if (n1.dot(n2) > decimal(-0.5))
            {
                maxCorr = decimal(2.0) * PI;
            }
            else
            {
                maxCorr = timeSubStep;
            }

            decimal & lambda = mBallAndSocketJointComponents.mTwistLambda[i];
            decimal angularVelocityDeltaProjected = angularVelocityDelta.dot(n);

            mXPBDProjections.limitAngleXPBD(componentIndexBody1, componentIndexBody2, n, a1, a2, limitsAnglesMin.z, limitsAnglesMax.z, 
                mBallAndSocketJointComponents.mCallbacksZ[i], joint, angularVelocityDeltaProjected, timeSubStep, lambda, maxCorr);
        }
        
        // Simple attachement
        {
            Vector3 r1 = orientationBody1 * mBallAndSocketJointComponents.mLocalAnchorPointBody1[i];
            Vector3 r2 = orientationBody2 * mBallAndSocketJointComponents.mLocalAnchorPointBody2[i];
            Vector3 corr = r2 - r1;
            corr += positionBody2 - positionBody1;
            decimal lambda(0.0);
            mXPBDProjections.applyBodyPairCorrectionXPBD(corr, decimal(0.0), r1, r2, timeSubStep, lambda, componentIndexBody1, componentIndexBody2);
        }
    }
}

void SolveBallAndSocketJointSystem::solveVelocityXPBD(decimal timeSubStep)
{
    // For each joint component
    for (uint32 i = 0; i < mBallAndSocketJointComponents.getNbEnabledComponents(); i++)
    {
        decimal dampingRotation = 550.0f;
        if (dampingRotation > 0.0)
        {
            const Entity jointEntity = mBallAndSocketJointComponents.mJointEntities[i];

            const Entity body1Entity = mJointComponents.getBody1Entity(jointEntity);
            const Entity body2Entity = mJointComponents.getBody2Entity(jointEntity);

            const uint32 componentIndexBody1 = mRigidBodyComponents.getEntityIndex(body1Entity);
            const uint32 componentIndexBody2 = mRigidBodyComponents.getEntityIndex(body2Entity);

            const Vector3& angularVelocityBody1 = mRigidBodyComponents.mAngularVelocities[componentIndexBody1];
            const Vector3& angularVelocityBody2 = mRigidBodyComponents.mAngularVelocities[componentIndexBody2];

            Vector3 angularVelocityDelta = angularVelocityBody2 - angularVelocityBody1;
            Vector3 corr = angularVelocityDelta * std::min(decimal(1.0), dampingRotation * timeSubStep);

            mXPBDProjections.applyBodyPairCorrectionVelocityXPBD(corr, 0.0, timeSubStep, componentIndexBody1, componentIndexBody2);
        }
    }
}

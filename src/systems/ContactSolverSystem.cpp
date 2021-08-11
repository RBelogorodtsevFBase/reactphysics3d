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
#include <reactphysics3d/systems/ContactSolverSystem.h>
#include <reactphysics3d/engine/PhysicsWorld.h>
#include <reactphysics3d/body/RigidBody.h>
#include <reactphysics3d/constraint/ContactPoint.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/engine/Island.h>
#include <reactphysics3d/collision/Collider.h>
#include <reactphysics3d/components/CollisionBodyComponents.h>
#include <reactphysics3d/components/ColliderComponents.h>
#include <reactphysics3d/collision/ContactManifold.h>

using namespace reactphysics3d;
using namespace std;

// Constructor
ContactSolverSystem::ContactSolverSystem(MemoryManager & memoryManager, PhysicsWorld & world, Islands & islands, CollisionBodyComponents & bodyComponents, RigidBodyComponents & rigidBodyComponents, ColliderComponents & colliderComponents)
    : mMemoryManager(memoryManager)
    , mWorld(world)
    , mContactConstraints(nullptr)
    , mContactPoints(nullptr)
    , mIslands(islands)
    , mAllContactManifolds(nullptr)
    , mAllContactPoints(nullptr)
    , mBodyComponents(bodyComponents)
    , mRigidBodyComponents(rigidBodyComponents)
    , mColliderComponents(colliderComponents)
    , mXPBDProjections(rigidBodyComponents)
{
#ifdef IS_RP3D_PROFILING_ENABLED

        mProfiler = nullptr;
#endif
}

// Initialize the contact constraints
void ContactSolverSystem::init(List<ContactManifold>* contactManifolds, List<ContactPoint>* contactPoints, decimal timeStep) 
{
    mAllContactManifolds = contactManifolds;
    mAllContactPoints = contactPoints;

    RP3D_PROFILE("ContactSolver::init()", mProfiler);

    mTimeStep = timeStep;

    uint nbContactManifolds = mAllContactManifolds->size();
    uint nbContactPoints = mAllContactPoints->size();

    mNbContactManifolds = 0;
    mNbContactPoints = 0;

    mContactConstraints = nullptr;
    mContactPoints = nullptr;

    if (nbContactManifolds == 0 || nbContactPoints == 0) return;

    mContactPoints = static_cast<ContactPointSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                              sizeof(ContactPointSolver) * nbContactPoints));
    assert(mContactPoints != nullptr);

    mContactConstraints = static_cast<ContactManifoldSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame,
                                                                                      sizeof(ContactManifoldSolver) * nbContactManifolds));
    assert(mContactConstraints != nullptr);

    // For each island of the world
    for (uint i = 0; i < mIslands.getNbIslands(); i++) {

        if (mIslands.nbContactManifolds[i] > 0) {
            initializeForIsland(i);
        }
    }
}

void ContactSolverSystem::initXPBD(List<ContactManifold> * contactManifolds, List<ContactPoint> * contactPoints, decimal timeStep) 
{
    mAllContactManifolds = contactManifolds;
    mAllContactPoints = contactPoints;

    RP3D_PROFILE("ContactSolver::initXPBD()", mProfiler);

    mTimeStep = timeStep;

    uint nbContactManifolds = mAllContactManifolds->size();
    uint nbContactPoints = mAllContactPoints->size();

    mNbContactManifolds = 0;
    mNbContactPoints = 0;

    mContactConstraints = nullptr;
    mContactPoints = nullptr;

    if (nbContactManifolds == 0 || nbContactPoints == 0) return;

    mContactPoints = static_cast<ContactPointSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame, sizeof(ContactPointSolver) * nbContactPoints));
    assert(mContactPoints != nullptr);

    mContactConstraints = static_cast<ContactManifoldSolver*>(mMemoryManager.allocate(MemoryManager::AllocationType::Frame, sizeof(ContactManifoldSolver) * nbContactManifolds));
    assert(mContactConstraints != nullptr);

    // For each island of the world
    for (uint i = 0; i < mIslands.getNbIslands(); i++) 
    {
        if (mIslands.nbContactManifolds[i] > 0) 
        {
            initializeForIsland(i);
        }
    }
}

// Release allocated memory
void ContactSolverSystem::reset() {

    if (mAllContactPoints->size() > 0)
    {
        mMemoryManager.release(MemoryManager::AllocationType::Frame, mContactPoints, sizeof(ContactPointSolver) * mAllContactPoints->size());
    }
    if (mAllContactManifolds->size() > 0)
    {
        mMemoryManager.release(MemoryManager::AllocationType::Frame, mContactConstraints, sizeof(ContactManifoldSolver) * mAllContactManifolds->size());
    }
}

// Initialize the constraint solver for a given island
void ContactSolverSystem::initializeForIsland(uint islandIndex) 
{
    RP3D_PROFILE("ContactSolver::initializeForIsland()", mProfiler);

    assert(mIslands.bodyEntities[islandIndex].size() > 0);
    assert(mIslands.nbContactManifolds[islandIndex] > 0);

    // For each contact manifold of the island
    uint contactManifoldsIndex = mIslands.contactManifoldsIndices[islandIndex];
    uint nbContactManifolds = mIslands.nbContactManifolds[islandIndex];
    for (uint m=contactManifoldsIndex; m < contactManifoldsIndex + nbContactManifolds; m++) {

        ContactManifold& externalManifold = (*mAllContactManifolds)[m];

        assert(externalManifold.nbContactPoints > 0);

        // Get the two bodies of the contact
        RigidBody* body1 = static_cast<RigidBody*>(mBodyComponents.getBody(externalManifold.bodyEntity1));
        RigidBody* body2 = static_cast<RigidBody*>(mBodyComponents.getBody(externalManifold.bodyEntity2));
        assert(body1 != nullptr);
        assert(body2 != nullptr);
        assert(!mBodyComponents.getIsEntityDisabled(externalManifold.bodyEntity1));
        assert(!mBodyComponents.getIsEntityDisabled(externalManifold.bodyEntity2));

        const uint rigidBodyIndex1 = mRigidBodyComponents.getEntityIndex(externalManifold.bodyEntity1);
        const uint rigidBodyIndex2 = mRigidBodyComponents.getEntityIndex(externalManifold.bodyEntity2);

        Collider* collider1 = mColliderComponents.getCollider(externalManifold.colliderEntity1);
        Collider* collider2 = mColliderComponents.getCollider(externalManifold.colliderEntity2);

        // Get the position of the two bodies
        const Vector3& x1 = mRigidBodyComponents.mCentersOfMassWorld[rigidBodyIndex1];
        const Vector3& x2 = mRigidBodyComponents.mCentersOfMassWorld[rigidBodyIndex2];

        // Initialize the internal contact manifold structure using the external contact manifold
        new (mContactConstraints + mNbContactManifolds) ContactManifoldSolver();
        mContactConstraints[mNbContactManifolds].rigidBodyComponentIndexBody1 = rigidBodyIndex1;
        mContactConstraints[mNbContactManifolds].rigidBodyComponentIndexBody2 = rigidBodyIndex2;
        mContactConstraints[mNbContactManifolds].nbContacts = externalManifold.nbContactPoints;
        mContactConstraints[mNbContactManifolds].frictionCoefficient = computeMixedFrictionCoefficient(collider1, collider2);
        mContactConstraints[mNbContactManifolds].externalContactManifold = &externalManifold;
        mContactConstraints[mNbContactManifolds].normal.setToZero();

        // For each  contact point of the contact manifold
        assert(externalManifold.nbContactPoints > 0);
        uint contactPointsStartIndex = externalManifold.contactPointsIndex;
        uint nbContactPoints = static_cast<uint>(externalManifold.nbContactPoints);
        for (uint c=contactPointsStartIndex; c < contactPointsStartIndex + nbContactPoints; c++) {

            ContactPoint& externalContact = (*mAllContactPoints)[c];

            // Get the contact point on the two bodies
            Vector3 p1 = mColliderComponents.getLocalToWorldTransform(externalManifold.colliderEntity1) * externalContact.getLocalPointOnShape1();
            Vector3 p2 = mColliderComponents.getLocalToWorldTransform(externalManifold.colliderEntity2) * externalContact.getLocalPointOnShape2();

            new (mContactPoints + mNbContactPoints) ContactPointSolver();
            mContactPoints[mNbContactPoints].externalContact = &externalContact;
            mContactPoints[mNbContactPoints].normal = externalContact.getNormal();
            mContactPoints[mNbContactPoints].r1 = mRigidBodyComponents.mXPBDOrientationsPrevious[rigidBodyIndex1].getInverse() * (p1 - x1);
            mContactPoints[mNbContactPoints].r2 = mRigidBodyComponents.mXPBDOrientationsPrevious[rigidBodyIndex2].getInverse() * (p2 - x2);

            mContactPoints[mNbContactPoints].isRestingContact = externalContact.getIsRestingContact();
            externalContact.setIsRestingContact(true);

            mContactConstraints[mNbContactManifolds].normal += mContactPoints[mNbContactPoints].normal;

            mNbContactPoints++;
        }
        mContactConstraints[mNbContactManifolds].normal.normalize();

        mNbContactManifolds++;
    }
}

void ContactSolverSystem::solvePositionXPBD()
{
    RP3D_PROFILE("ContactSolverSystem::solvePositionXPBD()", mProfiler);

    // contact penetration handling
    uint contactPointIndex = 0;
    for (uint c = 0; c < mNbContactManifolds; c++)
    {
        uint32 indexBody1 = mContactConstraints[c].rigidBodyComponentIndexBody1;
        uint32 indexBody2 = mContactConstraints[c].rigidBodyComponentIndexBody2;

        for (short int i = 0; i < mContactConstraints[c].nbContacts; i++, contactPointIndex++)
        {
            if (mRigidBodyComponents.mInverseMasses[indexBody1] != 0.0 && mRigidBodyComponents.mInverseMasses[indexBody2] != 0.0) // TODO : this is makeshift to avoid self-collisions !!
            {
                mContactPoints[contactPointIndex].contactHappened = false;
                continue;
            }

            const Vector3 & n = mContactPoints[contactPointIndex].normal;

            Vector3 r1 = mRigidBodyComponents.mXPBDOrientations[indexBody1] * mContactPoints[contactPointIndex].r1;
            Vector3 r2 = mRigidBodyComponents.mXPBDOrientations[indexBody2] * mContactPoints[contactPointIndex].r2;

            Vector3 p1 = mRigidBodyComponents.mXPBDPositions[indexBody1] + r1;
            Vector3 p2 = mRigidBodyComponents.mXPBDPositions[indexBody2] + r2;

            decimal d = (p1 - p2).dot(n);
            if (d <= decimal(0.0))
            {
                mContactPoints[contactPointIndex].contactHappened = false;
                continue;
            }
            mContactPoints[contactPointIndex].contactHappened = true;
            mXPBDProjections.applyBodyPairCorrectionXPBD(n * -d, 0.0, r1, r2, mTimeStep, mContactPoints[contactPointIndex].lambdaN, indexBody1, indexBody2);
        }
    }

    // static friction
    contactPointIndex = 0;
    for (uint c = 0; c < mNbContactManifolds; c++)
    {
        uint32 indexBody1 = mContactConstraints[c].rigidBodyComponentIndexBody1;
        uint32 indexBody2 = mContactConstraints[c].rigidBodyComponentIndexBody2;

        for (short int i = 0; i < mContactConstraints[c].nbContacts; i++, contactPointIndex++)
        {
            if (!mContactPoints[contactPointIndex].contactHappened)
            {
                continue;
            }

            const Vector3 & n = mContactPoints[contactPointIndex].normal;

            Vector3 r1 = mRigidBodyComponents.mXPBDOrientations[indexBody1] * mContactPoints[contactPointIndex].r1;
            Vector3 r2 = mRigidBodyComponents.mXPBDOrientations[indexBody2] * mContactPoints[contactPointIndex].r2;

            Vector3 p1 = mRigidBodyComponents.mXPBDPositions[indexBody1] + r1;
            Vector3 p2 = mRigidBodyComponents.mXPBDPositions[indexBody2] + r2;

            Vector3 p1Previous = mRigidBodyComponents.mXPBDPositionsPrevious[indexBody1] + mRigidBodyComponents.mXPBDOrientationsPrevious[indexBody1] * mContactPoints[contactPointIndex].r1;
            Vector3 p2Previous = mRigidBodyComponents.mXPBDPositionsPrevious[indexBody2] + mRigidBodyComponents.mXPBDOrientationsPrevious[indexBody2] * mContactPoints[contactPointIndex].r2;

            Vector3 pDelta = (p1 - p1Previous) - (p2 - p2Previous);
            Vector3 pDeltaT = pDelta - n * (pDelta.dot(n));

            decimal frictionStatic(0.80);
            decimal thresholdLambda = frictionStatic * mContactPoints[contactPointIndex].lambdaN;

            mXPBDProjections.applyBodyPairCorrectionThresholdXPBD(-pDeltaT, 0.0, r1, r2, mTimeStep, mContactPoints[contactPointIndex].lambdaT, thresholdLambda, indexBody1, indexBody2);
        }
    }
}

void ContactSolverSystem::solveVelocityXPBD()
{
    RP3D_PROFILE("ContactSolverSystem::solvePositionXPBD()", mProfiler);

    uint contactPointIndex = 0;

    // dynamic friction
    for (uint c = 0; c < mNbContactManifolds; c++)
    {
        uint32 indexBody1 = mContactConstraints[c].rigidBodyComponentIndexBody1;
        uint32 indexBody2 = mContactConstraints[c].rigidBodyComponentIndexBody2;

        for (short int i = 0; i < mContactConstraints[c].nbContacts; i++, contactPointIndex++)
        {
            if (!mContactPoints[contactPointIndex].contactHappened)
            {
                continue;
            }

            Vector3 r1 = mRigidBodyComponents.mXPBDOrientations[indexBody1] * mContactPoints[contactPointIndex].r1;
            Vector3 r2 = mRigidBodyComponents.mXPBDOrientations[indexBody2] * mContactPoints[contactPointIndex].r2;

            Vector3 v1 = mRigidBodyComponents.mLinearVelocities[indexBody1] + mRigidBodyComponents.mAngularVelocities[indexBody1].cross(r1);
            Vector3 v2 = mRigidBodyComponents.mLinearVelocities[indexBody2] + mRigidBodyComponents.mAngularVelocities[indexBody2].cross(r2);
            const Vector3 & n = mContactPoints[contactPointIndex].normal;

            Vector3 v = v1 - v2;
            decimal vN_ = n.dot(v);
            Vector3 vT = v - n * vN_;
            decimal vT_ = vT.length();

            if (vT_ == decimal(0.0))
            {
                continue;
            }

            decimal frictionDynamic(0.10);
            decimal dvFriction = frictionDynamic * mContactPoints[contactPointIndex].lambdaN / mTimeStep;
            Vector3 corr = (dvFriction < vT_) ? vT * (-1.0 / vT_) * dvFriction : -vT;

            mXPBDProjections.applyBodyPairCorrectionVelocityXPBD(corr, 0.0, r1, r2, mTimeStep, indexBody1, indexBody2);
        }
    }

    // restitution
    contactPointIndex = 0;
    for (uint c = 0; c < mNbContactManifolds; c++)
    {
        uint32 indexBody1 = mContactConstraints[c].rigidBodyComponentIndexBody1;
        uint32 indexBody2 = mContactConstraints[c].rigidBodyComponentIndexBody2;

        for (short int i = 0; i < mContactConstraints[c].nbContacts; i++, contactPointIndex++)
        {
            if (!mContactPoints[contactPointIndex].contactHappened)
            {
                continue;
            }

            if (mContactPoints[contactPointIndex].isRestingContact)
            {
                continue;
            }

            Vector3 r1 = mRigidBodyComponents.mXPBDOrientations[indexBody1] * mContactPoints[contactPointIndex].r1;
            Vector3 r2 = mRigidBodyComponents.mXPBDOrientations[indexBody2] * mContactPoints[contactPointIndex].r2;

            Vector3 v1 = mRigidBodyComponents.mLinearVelocities[indexBody1] + mRigidBodyComponents.mAngularVelocities[indexBody1].cross(r1);
            Vector3 v2 = mRigidBodyComponents.mLinearVelocities[indexBody2] + mRigidBodyComponents.mAngularVelocities[indexBody2].cross(r2);
            const Vector3 & n = mContactPoints[contactPointIndex].normal;

            decimal vN = n.dot(v1 - v2);
            decimal vNPreUpdate = mContactPoints[contactPointIndex].vNPreUpdate;
            decimal restitution(0.5); // TODO : to avoid jittering we set e = 0 if vN is small ...

            Vector3 corr = n * (std::min(-restitution * vNPreUpdate, decimal(0.0)) - vN);

            mXPBDProjections.applyBodyPairCorrectionVelocityXPBD(corr, 0.0, r1, r2, mTimeStep, indexBody1, indexBody2);
        }
    }
}

void ContactSolverSystem::cacheVnXPBD()
{
    RP3D_PROFILE("ContactSolverSystem::cacheVnXPBD()", mProfiler);

    uint contactPointIndex = 0;

    for (uint c = 0; c < mNbContactManifolds; c++)
    {
        uint32 indexBody1 = mContactConstraints[c].rigidBodyComponentIndexBody1;
        uint32 indexBody2 = mContactConstraints[c].rigidBodyComponentIndexBody2;

        for (short int i = 0; i < mContactConstraints[c].nbContacts; i++, contactPointIndex++)
        {
            //if (!mContactPoints[contactPointIndex].contactHappened)
            //{
            //    continue;
            //}

            Vector3 r1 = mRigidBodyComponents.mXPBDOrientations[indexBody1] * mContactPoints[contactPointIndex].r1;
            Vector3 r2 = mRigidBodyComponents.mXPBDOrientations[indexBody2] * mContactPoints[contactPointIndex].r2;

            Vector3 v1 = mRigidBodyComponents.mLinearVelocities[indexBody1] + mRigidBodyComponents.mAngularVelocities[indexBody1].cross(r1);
            Vector3 v2 = mRigidBodyComponents.mLinearVelocities[indexBody2] + mRigidBodyComponents.mAngularVelocities[indexBody2].cross(r2);
            const Vector3 & n = mContactPoints[contactPointIndex].normal;

            mContactPoints[contactPointIndex].vNPreUpdate = n.dot(v1 - v2);
        }
    }
}

// Compute the collision restitution factor from the restitution factor of each collider
decimal ContactSolverSystem::computeMixedRestitutionFactor(Collider* collider1, Collider* collider2) const {
    decimal restitution1 = collider1->getMaterial().getBounciness();
    decimal restitution2 = collider2->getMaterial().getBounciness();

    // Return the largest restitution factor
    return (restitution1 > restitution2) ? restitution1 : restitution2;
}

// Compute the mixed friction coefficient from the friction coefficient of each collider
decimal ContactSolverSystem::computeMixedFrictionCoefficient(Collider* collider1, Collider* collider2) const {

    // Use the geometric mean to compute the mixed friction coefficient
    return std::sqrt(collider1->getMaterial().getFrictionCoefficient() *
                collider2->getMaterial().getFrictionCoefficient());
}

// Compute th mixed rolling resistance factor between two colliders
inline decimal ContactSolverSystem::computeMixedRollingResistance(Collider* collider1, Collider* collider2) const {
    return decimal(0.5f) * (collider1->getMaterial().getRollingResistance() + collider2->getMaterial().getRollingResistance());
}
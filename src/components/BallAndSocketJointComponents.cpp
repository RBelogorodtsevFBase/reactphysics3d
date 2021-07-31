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
#include <reactphysics3d/components/BallAndSocketJointComponents.h>
#include <reactphysics3d/engine/EntityManager.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <cassert>

// We want to use the ReactPhysics3D namespace
using namespace reactphysics3d;

// Constructor
BallAndSocketJointComponents::BallAndSocketJointComponents(MemoryAllocator& allocator)
                    :Components(allocator, sizeof(Entity) + sizeof(BallAndSocketJoint*) + sizeof(Vector3) +
                                sizeof(bool) + sizeof(bool) + sizeof(bool) +
                                sizeof(Quaternion) + sizeof(Quaternion) +
                                sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Matrix3x3) + sizeof(Vector3) +
                                sizeof(Matrix3x3) + sizeof(Vector3)) {

    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void BallAndSocketJointComponents::allocate(uint32 nbComponentsToAllocate) {

    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newJointEntities = static_cast<Entity*>(newBuffer);
    BallAndSocketJoint** newJoints = reinterpret_cast<BallAndSocketJoint**>(newJointEntities + nbComponentsToAllocate);
    bool * newLimitSwingXEnabled = reinterpret_cast<bool*>(newJoints + nbComponentsToAllocate);
    bool * newLimitSwingYEnabled = reinterpret_cast<bool*>(newLimitSwingXEnabled + nbComponentsToAllocate);
    bool * newLimitTwistEnabled = reinterpret_cast<bool*>(newLimitSwingYEnabled + nbComponentsToAllocate);
    Quaternion * newTargetLocalInBody1 = reinterpret_cast<Quaternion *>(newLimitTwistEnabled + nbComponentsToAllocate);
    Quaternion * newReferenceLocalInBody2 = reinterpret_cast<Quaternion *>(newTargetLocalInBody1 + nbComponentsToAllocate);
    Vector3 * newLimitsAnglesMin = reinterpret_cast<Vector3 *>(newReferenceLocalInBody2 + nbComponentsToAllocate);
    Vector3 * newLimitsAnglesMax = reinterpret_cast<Vector3 *>(newLimitsAnglesMin + nbComponentsToAllocate);
    Vector3 * newStiffnessesPositive = reinterpret_cast<Vector3 *>(newLimitsAnglesMax + nbComponentsToAllocate);
    Vector3 * newStiffnessesNegative = reinterpret_cast<Vector3 *>(newStiffnessesPositive + nbComponentsToAllocate);
    Vector3 * newDampings = reinterpret_cast<Vector3 *>(newStiffnessesNegative + nbComponentsToAllocate);
    Vector3 * newSpringTargets = reinterpret_cast<Vector3 *>(newDampings + nbComponentsToAllocate);
    Vector3* newLocalAnchorPointBody1 = reinterpret_cast<Vector3*>(newSpringTargets + nbComponentsToAllocate);
    Vector3* newLocalAnchorPointBody2 = reinterpret_cast<Vector3*>(newLocalAnchorPointBody1 + nbComponentsToAllocate);
    Vector3* newR1World = reinterpret_cast<Vector3*>(newLocalAnchorPointBody2 + nbComponentsToAllocate);
    Vector3* newR2World = reinterpret_cast<Vector3*>(newR1World + nbComponentsToAllocate);
    Matrix3x3* newI1 = reinterpret_cast<Matrix3x3*>(newR2World + nbComponentsToAllocate);
    Matrix3x3* newI2 = reinterpret_cast<Matrix3x3*>(newI1 + nbComponentsToAllocate);
    Vector3* newBiasVector = reinterpret_cast<Vector3*>(newI2 + nbComponentsToAllocate);
    Matrix3x3* newInverseMassMatrix = reinterpret_cast<Matrix3x3*>(newBiasVector + nbComponentsToAllocate);
    Vector3* newImpulse = reinterpret_cast<Vector3*>(newInverseMassMatrix + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) {

        // Copy component data from the previous buffer to the new one
        memcpy(newJointEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(BallAndSocketJoint*));
        memcpy(newLimitSwingXEnabled, mLimitSwingXEnabled, mNbComponents * sizeof(bool));
        memcpy(newLimitSwingYEnabled, mLimitSwingYEnabled, mNbComponents * sizeof(bool));
        memcpy(newLimitTwistEnabled, mLimitTwistEnabled, mNbComponents * sizeof(bool));
        memcpy(newTargetLocalInBody1, mTargetLocalInBody1, mNbComponents * sizeof(Quaternion));
        memcpy(newReferenceLocalInBody2, mReferenceLocalInBody2, mNbComponents * sizeof(Quaternion));
        memcpy(newLimitsAnglesMin, mLimitsAnglesMin, mNbComponents * sizeof(Vector3));
        memcpy(newLimitsAnglesMax, mLimitsAnglesMax, mNbComponents * sizeof(Vector3));
        memcpy(newStiffnessesPositive, mStiffnessesPositive, mNbComponents * sizeof(Vector3));
        memcpy(newStiffnessesNegative, mStiffnessesNegative, mNbComponents * sizeof(Vector3));
        memcpy(newDampings, mDampings, mNbComponents * sizeof(Vector3));
        memcpy(newSpringTargets, mSpringTargets, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody1, mLocalAnchorPointBody1, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody2, mLocalAnchorPointBody2, mNbComponents * sizeof(Vector3));
        memcpy(newR1World, mR1World, mNbComponents * sizeof(Vector3));
        memcpy(newR2World, mR2World, mNbComponents * sizeof(Vector3));
        memcpy(newI1, mI1, mNbComponents * sizeof(Matrix3x3));
        memcpy(newI2, mI2, mNbComponents * sizeof(Matrix3x3));
        memcpy(newBiasVector, mBiasVector, mNbComponents * sizeof(Vector3));
        memcpy(newInverseMassMatrix, mInverseMassMatrix, mNbComponents * sizeof(Matrix3x3));
        memcpy(newImpulse, mImpulse, mNbComponents * sizeof(Vector3));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mJointEntities = newJointEntities;
    mJoints = newJoints;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mLimitSwingXEnabled = newLimitSwingXEnabled;
    mLimitSwingYEnabled = newLimitSwingYEnabled;
    mLimitTwistEnabled = newLimitTwistEnabled;
    mTargetLocalInBody1 = newTargetLocalInBody1;
    mReferenceLocalInBody2 = newReferenceLocalInBody2;
    mLimitsAnglesMin = newLimitsAnglesMin;
    mLimitsAnglesMax = newLimitsAnglesMax;
    mStiffnessesPositive = newStiffnessesPositive;
    mStiffnessesNegative = newStiffnessesNegative;
    mDampings = newDampings;
    mSpringTargets = newSpringTargets;
    mLocalAnchorPointBody1 = newLocalAnchorPointBody1;
    mLocalAnchorPointBody2 = newLocalAnchorPointBody2;
    mR1World = newR1World;
    mR2World = newR2World;
    mI1 = newI1;
    mI2 = newI2;
    mBiasVector = newBiasVector;
    mInverseMassMatrix = newInverseMassMatrix;
    mImpulse = newImpulse;
}

// Add a component
void BallAndSocketJointComponents::addComponent(Entity jointEntity, bool isSleeping, const BallAndSocketJointComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    mJoints[index] = nullptr;
    new (mLimitSwingXEnabled + index) bool(false);
    new (mLimitSwingYEnabled + index) bool(false);
    new (mLimitTwistEnabled + index) bool(false);
    new (mTargetLocalInBody1 + index) Quaternion(0, 0, 0, 1);
    new (mReferenceLocalInBody2 + index) Quaternion(0, 0, 0, 1);
    new (mLimitsAnglesMin + index) Vector3(0, 0, 0);
    new (mLimitsAnglesMax + index) Vector3(0, 0, 0);
    new (mStiffnessesPositive + index) Vector3(0, 0, 0);
    new (mStiffnessesNegative + index) Vector3(0, 0, 0);
    new (mDampings + index) Vector3(0, 0, 0);
    new (mSpringTargets + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody1 + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody2 + index) Vector3(0, 0, 0);
    new (mR1World + index) Vector3(0, 0, 0);
    new (mR2World + index) Vector3(0, 0, 0);
    new (mI1 + index) Matrix3x3();
    new (mI2 + index) Matrix3x3();
    new (mBiasVector + index) Vector3(0, 0, 0);
    new (mInverseMassMatrix + index) Matrix3x3();
    new (mImpulse + index) Vector3(0, 0, 0);

    // Map the entity with the new component lookup index
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity, index));

    mNbComponents++;

    assert(mDisabledStartIndex <= mNbComponents);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Move a component from a source to a destination index in the components array
// The destination location must contain a constructed object
void BallAndSocketJointComponents::moveComponentToIndex(uint32 srcIndex, uint32 destIndex) {

    const Entity entity = mJointEntities[srcIndex];

    // Copy the data of the source component to the destination location
    new (mJointEntities + destIndex) Entity(mJointEntities[srcIndex]);
    mJoints[destIndex] = mJoints[srcIndex];
    new (mLimitSwingXEnabled + destIndex) bool(mLimitSwingXEnabled[srcIndex]);
    new (mLimitSwingYEnabled + destIndex) bool(mLimitSwingYEnabled[srcIndex]);
    new (mLimitTwistEnabled + destIndex) bool(mLimitTwistEnabled[srcIndex]);
    new (mTargetLocalInBody1 + destIndex) Quaternion(mTargetLocalInBody1[srcIndex]);
    new (mReferenceLocalInBody2 + destIndex) Quaternion(mReferenceLocalInBody2[srcIndex]);
    new (mLimitsAnglesMin + destIndex) Vector3(mLimitsAnglesMin[srcIndex]);
    new (mLimitsAnglesMax + destIndex) Vector3(mLimitsAnglesMax[srcIndex]);
    new (mStiffnessesPositive + destIndex) Vector3(mStiffnessesPositive[srcIndex]);
    new (mStiffnessesNegative + destIndex) Vector3(mStiffnessesNegative[srcIndex]);
    new (mDampings + destIndex) Vector3(mDampings[srcIndex]);
    new (mSpringTargets + destIndex) Vector3(mSpringTargets[srcIndex]);
    new (mLocalAnchorPointBody1 + destIndex) Vector3(mLocalAnchorPointBody1[srcIndex]);
    new (mLocalAnchorPointBody2 + destIndex) Vector3(mLocalAnchorPointBody2[srcIndex]);
    new (mR1World + destIndex) Vector3(mR1World[srcIndex]);
    new (mR2World + destIndex) Vector3(mR2World[srcIndex]);
    new (mI1 + destIndex) Matrix3x3(mI1[srcIndex]);
    new (mI2 + destIndex) Matrix3x3(mI2[srcIndex]);
    new (mBiasVector + destIndex) Vector3(mBiasVector[srcIndex]);
    new (mInverseMassMatrix + destIndex) Matrix3x3(mInverseMassMatrix[srcIndex]);
    new (mImpulse + destIndex) Vector3(mImpulse[srcIndex]);

    // Destroy the source component
    destroyComponent(srcIndex);

    assert(!mMapEntityToComponentIndex.containsKey(entity));

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(entity, destIndex));

    assert(mMapEntityToComponentIndex[mJointEntities[destIndex]] == destIndex);
}

// Swap two components in the array
void BallAndSocketJointComponents::swapComponents(uint32 index1, uint32 index2) {

    // Copy component 1 data
    Entity jointEntity1(mJointEntities[index1]);
    BallAndSocketJoint* joint1 = mJoints[index1];
    bool limitSwingXEnabled1(mLimitSwingXEnabled[index1]);
    bool limitSwingYEnabled1(mLimitSwingYEnabled[index1]);
    bool limitTwistEnabled1(mLimitTwistEnabled[index1]);
    Quaternion targetLocalInBody1(mTargetLocalInBody1[index1]);
    Quaternion referenceLocalInBody2(mReferenceLocalInBody2[index1]);
    Vector3 limitsAnglesMin1(mLimitsAnglesMin[index1]);
    Vector3 limitsAnglesMax1(mLimitsAnglesMax[index1]);
    Vector3 stiffnessesPositive1(mStiffnessesPositive[index1]);
    Vector3 stiffnessesNegative1(mStiffnessesNegative[index1]);
    Vector3 dampings1(mDampings[index1]);
    Vector3 springTarget1(mSpringTargets[index1]);
    Vector3 localAnchorPointBody1(mLocalAnchorPointBody1[index1]);
    Vector3 localAnchorPointBody2(mLocalAnchorPointBody2[index1]);
    Vector3 r1World1(mR1World[index1]);
    Vector3 r2World1(mR2World[index1]);
    Matrix3x3 i11(mI1[index1]);
    Matrix3x3 i21(mI2[index1]);
    Vector3 biasVector1(mBiasVector[index1]);
    Matrix3x3 inverseMassMatrix1(mInverseMassMatrix[index1]);
    Vector3 impulse1(mImpulse[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mJointEntities + index2) Entity(jointEntity1);
    mJoints[index2] = joint1;
    new (mLimitSwingXEnabled + index2) bool(limitSwingXEnabled1);
    new (mLimitSwingYEnabled + index2) bool(limitSwingYEnabled1);
    new (mLimitTwistEnabled + index2) bool(limitTwistEnabled1);
    new (mTargetLocalInBody1 + index2) Quaternion(targetLocalInBody1);
    new (mReferenceLocalInBody2 + index2) Quaternion(referenceLocalInBody2);
    new (mLimitsAnglesMin + index2) Vector3(limitsAnglesMin1);
    new (mLimitsAnglesMax + index2) Vector3(limitsAnglesMax1);
    new (mStiffnessesPositive + index2) Vector3(stiffnessesPositive1);
    new (mStiffnessesNegative + index2) Vector3(stiffnessesNegative1);
    new (mDampings + index2) Vector3(dampings1);
    new (mSpringTargets + index2) Vector3(springTarget1);
    new (mLocalAnchorPointBody1 + index2) Vector3(localAnchorPointBody1);
    new (mLocalAnchorPointBody2 + index2) Vector3(localAnchorPointBody2);
    new (mR1World + index2) Vector3(r1World1);
    new (mR2World + index2) Vector3(r2World1);
    new (mI1 + index2) Matrix3x3(i11);
    new (mI2 + index2) Matrix3x3(i21);
    new (mBiasVector + index2) Vector3(biasVector1);
    new (mInverseMassMatrix + index2) Matrix3x3(inverseMassMatrix1);
    new (mImpulse + index2) Vector3(impulse1);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void BallAndSocketJointComponents::destroyComponent(uint32 index) {

    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mJointEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mJointEntities[index]);

    mJointEntities[index].~Entity();
    mJoints[index] = nullptr;
    mTargetLocalInBody1[index].~Quaternion();
    mReferenceLocalInBody2[index].~Quaternion();
    mLimitsAnglesMin[index].~Vector3();
    mLimitsAnglesMax[index].~Vector3();
    mStiffnessesPositive[index].~Vector3();
    mStiffnessesNegative[index].~Vector3();
    mDampings[index].~Vector3();
    mSpringTargets[index].~Vector3();
    mLocalAnchorPointBody1[index].~Vector3();
    mLocalAnchorPointBody2[index].~Vector3();
    mR1World[index].~Vector3();
    mR2World[index].~Vector3();
    mI1[index].~Matrix3x3();
    mI2[index].~Matrix3x3();
    mBiasVector[index].~Vector3();
    mInverseMassMatrix[index].~Matrix3x3();
    mImpulse[index].~Vector3();
}

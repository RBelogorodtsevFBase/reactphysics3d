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

typedef void (*CallbackType)(BallAndSocketJoint *, decimal, decimal, decimal &, decimal &);

// Constructor
BallAndSocketJointComponents::BallAndSocketJointComponents(MemoryAllocator& allocator)
    : Components(allocator, sizeof(Entity) + sizeof(BallAndSocketJoint *) + 
        sizeof(Quaternion) + sizeof(Quaternion) + sizeof(decimal) + 
        sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) + sizeof(Vector3) +
        sizeof(CallbackType) + sizeof(CallbackType) + sizeof(CallbackType) + sizeof(void *))
{
    // Allocate memory for the components data
    allocate(INIT_NB_ALLOCATED_COMPONENTS);
}

// Allocate memory for a given number of components
void BallAndSocketJointComponents::allocate(uint32 nbComponentsToAllocate)
{
    assert(nbComponentsToAllocate > mNbAllocatedComponents);

    // Size for the data of a single component (in bytes)
    const size_t totalSizeBytes = nbComponentsToAllocate * mComponentDataSize;

    // Allocate memory
    void* newBuffer = mMemoryAllocator.allocate(totalSizeBytes);
    assert(newBuffer != nullptr);

    // New pointers to components data
    Entity* newJointEntities = static_cast<Entity*>(newBuffer);
    BallAndSocketJoint ** newJoints = reinterpret_cast<BallAndSocketJoint **>(newJointEntities + nbComponentsToAllocate);
    void ** newUserData = reinterpret_cast<void **>(newJoints + nbComponentsToAllocate);
    Quaternion * newTargetLocalInBody1 = reinterpret_cast<Quaternion *>(newUserData + nbComponentsToAllocate);
    Quaternion * newReferenceLocalInBody2 = reinterpret_cast<Quaternion *>(newTargetLocalInBody1 + nbComponentsToAllocate);
    CallbackType * newCallbacksX = reinterpret_cast<CallbackType *>(newReferenceLocalInBody2 + nbComponentsToAllocate);
    CallbackType * newCallbacksY = reinterpret_cast<CallbackType *>(newCallbacksX + nbComponentsToAllocate);
    CallbackType * newCallbacksZ = reinterpret_cast<CallbackType *>(newCallbacksY + nbComponentsToAllocate);
    Vector3 * newLimitsAnglesMin = reinterpret_cast<Vector3 *>(newCallbacksZ + nbComponentsToAllocate);
    Vector3 * newLimitsAnglesMax = reinterpret_cast<Vector3 *>(newLimitsAnglesMin + nbComponentsToAllocate);
    decimal * newDampings = reinterpret_cast<decimal *>(newLimitsAnglesMax + nbComponentsToAllocate);
    Vector3 * newLambda = reinterpret_cast<Vector3 *>(newDampings + nbComponentsToAllocate);
    Vector3 * newLocalAnchorPointBody1 = reinterpret_cast<Vector3*>(newLambda + nbComponentsToAllocate);
    Vector3 * newLocalAnchorPointBody2 = reinterpret_cast<Vector3*>(newLocalAnchorPointBody1 + nbComponentsToAllocate);

    // If there was already components before
    if (mNbComponents > 0) 
    {
        // Copy component data from the previous buffer to the new one
        memcpy(newJointEntities, mJointEntities, mNbComponents * sizeof(Entity));
        memcpy(newJoints, mJoints, mNbComponents * sizeof(BallAndSocketJoint *));
        memcpy(newUserData, mUserData, mNbComponents * sizeof(void *));
        memcpy(newTargetLocalInBody1, mTargetLocalInBody1, mNbComponents * sizeof(Quaternion));
        memcpy(newReferenceLocalInBody2, mReferenceLocalInBody2, mNbComponents * sizeof(Quaternion));
        memcpy(newCallbacksX, mCallbacksX, mNbComponents * sizeof(CallbackType));
        memcpy(newCallbacksY, mCallbacksY, mNbComponents * sizeof(CallbackType));
        memcpy(newCallbacksZ, mCallbacksZ, mNbComponents * sizeof(CallbackType));
        memcpy(newLimitsAnglesMin, mLimitsAnglesMin, mNbComponents * sizeof(Vector3));
        memcpy(newLimitsAnglesMax, mLimitsAnglesMax, mNbComponents * sizeof(Vector3));
        memcpy(newDampings, mDampings, mNbComponents * sizeof(decimal));
        memcpy(newLambda, mLambda, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody1, mLocalAnchorPointBody1, mNbComponents * sizeof(Vector3));
        memcpy(newLocalAnchorPointBody2, mLocalAnchorPointBody2, mNbComponents * sizeof(Vector3));

        // Deallocate previous memory
        mMemoryAllocator.release(mBuffer, mNbAllocatedComponents * mComponentDataSize);
    }

    mBuffer = newBuffer;
    mJointEntities = newJointEntities;
    mJoints = newJoints;
    mUserData = newUserData;
    mNbAllocatedComponents = nbComponentsToAllocate;
    mTargetLocalInBody1 = newTargetLocalInBody1;
    mReferenceLocalInBody2 = newReferenceLocalInBody2;
    mCallbacksX = newCallbacksX;
    mCallbacksY = newCallbacksY;
    mCallbacksZ = newCallbacksZ;
    mLimitsAnglesMin = newLimitsAnglesMin;
    mLimitsAnglesMax = newLimitsAnglesMax;
    mDampings = newDampings;
    mLambda = newLambda;
    mLocalAnchorPointBody1 = newLocalAnchorPointBody1;
    mLocalAnchorPointBody2 = newLocalAnchorPointBody2;
}

// Add a component
void BallAndSocketJointComponents::addComponent(Entity jointEntity, bool isSleeping, const BallAndSocketJointComponent& component) {

    // Prepare to add new component (allocate memory if necessary and compute insertion index)
    uint32 index = prepareAddComponent(isSleeping);

    // Insert the new component data
    new (mJointEntities + index) Entity(jointEntity);
    mJoints[index] = nullptr;
    mUserData[index] = nullptr;
    new (mTargetLocalInBody1 + index) Quaternion(0, 0, 0, 1);
    new (mReferenceLocalInBody2 + index) Quaternion(0, 0, 0, 1);
    new (mCallbacksX + index) CallbackType(nullptr);
    new (mCallbacksY + index) CallbackType(nullptr);
    new (mCallbacksZ + index) CallbackType(nullptr);
    new (mLimitsAnglesMin + index) Vector3(-PI, -PI, -PI);
    new (mLimitsAnglesMax + index) Vector3(+PI, +PI, +PI);
    new (mDampings + index) decimal(0);
    new (mLambda + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody1 + index) Vector3(0, 0, 0);
    new (mLocalAnchorPointBody2 + index) Vector3(0, 0, 0);

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
    mUserData[destIndex] = mUserData[srcIndex];
    new (mTargetLocalInBody1 + destIndex) Quaternion(mTargetLocalInBody1[srcIndex]);
    new (mReferenceLocalInBody2 + destIndex) Quaternion(mReferenceLocalInBody2[srcIndex]);
    new (mCallbacksX + destIndex) CallbackType(mCallbacksX[srcIndex]);
    new (mCallbacksY + destIndex) CallbackType(mCallbacksY[srcIndex]);
    new (mCallbacksZ + destIndex) CallbackType(mCallbacksZ[srcIndex]);
    new (mLimitsAnglesMin + destIndex) Vector3(mLimitsAnglesMin[srcIndex]);
    new (mLimitsAnglesMax + destIndex) Vector3(mLimitsAnglesMax[srcIndex]);
    new (mDampings + destIndex) decimal(mDampings[srcIndex]);
    new (mLambda + destIndex) Vector3(mLambda[srcIndex]);
    new (mLocalAnchorPointBody1 + destIndex) Vector3(mLocalAnchorPointBody1[srcIndex]);
    new (mLocalAnchorPointBody2 + destIndex) Vector3(mLocalAnchorPointBody2[srcIndex]);

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
    BallAndSocketJoint * joint1 = mJoints[index1];
    void * userData1 = mUserData[index1];
    Quaternion targetLocalInBody1(mTargetLocalInBody1[index1]);
    Quaternion referenceLocalInBody2(mReferenceLocalInBody2[index1]);
    CallbackType callbackX1(mCallbacksX[index1]);
    CallbackType callbackY1(mCallbacksY[index1]);
    CallbackType callbackZ1(mCallbacksZ[index1]);
    Vector3 limitsAnglesMin1(mLimitsAnglesMin[index1]);
    Vector3 limitsAnglesMax1(mLimitsAnglesMax[index1]);
    decimal dampings1(mDampings[index1]);
    Vector3 lambda1(mLambda[index1]);
    Vector3 localAnchorPointBody1(mLocalAnchorPointBody1[index1]);
    Vector3 localAnchorPointBody2(mLocalAnchorPointBody2[index1]);

    // Destroy component 1
    destroyComponent(index1);

    moveComponentToIndex(index2, index1);

    // Reconstruct component 1 at component 2 location
    new (mJointEntities + index2) Entity(jointEntity1);
    mJoints[index2] = joint1;
    mUserData[index2] = userData1;
    new (mTargetLocalInBody1 + index2) Quaternion(targetLocalInBody1);
    new (mReferenceLocalInBody2 + index2) Quaternion(referenceLocalInBody2);
    new (mCallbacksX + index2) CallbackType(callbackX1);
    new (mCallbacksY + index2) CallbackType(callbackY1);
    new (mCallbacksZ + index2) CallbackType(callbackZ1);
    new (mLimitsAnglesMin + index2) Vector3(limitsAnglesMin1);
    new (mLimitsAnglesMax + index2) Vector3(limitsAnglesMax1);
    new (mDampings + index2) decimal(dampings1);
    new (mLambda + index2) Vector3(lambda1);
    new (mLocalAnchorPointBody1 + index2) Vector3(localAnchorPointBody1);
    new (mLocalAnchorPointBody2 + index2) Vector3(localAnchorPointBody2);

    // Update the entity to component index mapping
    mMapEntityToComponentIndex.add(Pair<Entity, uint32>(jointEntity1, index2));

    assert(mMapEntityToComponentIndex[mJointEntities[index1]] == index1);
    assert(mMapEntityToComponentIndex[mJointEntities[index2]] == index2);
    assert(mNbComponents == static_cast<uint32>(mMapEntityToComponentIndex.size()));
}

// Destroy a component at a given index
void BallAndSocketJointComponents::destroyComponent(uint32 index)
{
    Components::destroyComponent(index);

    assert(mMapEntityToComponentIndex[mJointEntities[index]] == index);

    mMapEntityToComponentIndex.remove(mJointEntities[index]);

    mJointEntities[index].~Entity();
    mJoints[index] = nullptr;
    mUserData[index] = nullptr;
    mTargetLocalInBody1[index].~Quaternion();
    mReferenceLocalInBody2[index].~Quaternion();
    mCallbacksX[index] = nullptr;
    mCallbacksY[index] = nullptr;
    mCallbacksZ[index] = nullptr;
    mLimitsAnglesMin[index].~Vector3();
    mLimitsAnglesMax[index].~Vector3();
    mDampings[index].~decimal();
    mLambda[index].~Vector3();
    mLocalAnchorPointBody1[index].~Vector3();
    mLocalAnchorPointBody2[index].~Vector3();
}

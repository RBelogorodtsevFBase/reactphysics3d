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

#ifndef REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_COMPONENTS_H
#define REACTPHYSICS3D_BALL_AND_SOCKET_JOINT_COMPONENTS_H

// Libraries
#include <reactphysics3d/mathematics/Transform.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>
#include <reactphysics3d/engine/Entity.h>
#include <reactphysics3d/components/Components.h>
#include <reactphysics3d/containers/Map.h>

// ReactPhysics3D namespace
namespace reactphysics3d {

// Class declarations
class MemoryAllocator;
class EntityManager;
class BallAndSocketJoint;
enum class JointType;

// Class BallAndSocketJointComponents
/**
 * This class represent the component of the ECS with data for the BallAndSocketJoint.
 */
class BallAndSocketJointComponents : public Components 
{
private:
        /// Array of joint entities
        Entity * mJointEntities;

        /// Array of pointers to the joints
        BallAndSocketJoint ** mJoints;

        void ** mUserData;

        Quaternion * mTargetLocalInBody1;

        Quaternion * mReferenceLocalInBody2;

        void (** mCallbacks)(BallAndSocketJoint *, const Vector3 &, const Vector3 &, Vector3 &, Vector3 &);

        Vector3 * mLimitsAnglesMin;

        Vector3 * mLimitsAnglesMax;

        decimal * mDampings;

        Vector3 * mLambda;

        /// Anchor point of body 1 (in local-space coordinates of body 1)
        Vector3 * mLocalAnchorPointBody1;

        /// Anchor point of body 2 (in local-space coordinates of body 2)
        Vector3 * mLocalAnchorPointBody2;

        // -------------------- Methods -------------------- //

        /// Allocate memory for a given number of components
        virtual void allocate(uint32 nbComponentsToAllocate) override;

        /// Destroy a component at a given index
        virtual void destroyComponent(uint32 index) override;

        /// Move a component from a source to a destination index in the components array
        virtual void moveComponentToIndex(uint32 srcIndex, uint32 destIndex) override;

        /// Swap two components in the array
        virtual void swapComponents(uint32 index1, uint32 index2) override;

    public:

        /// Structure for the data of a transform component
        struct BallAndSocketJointComponent 
        {
            /// Constructor
            BallAndSocketJointComponent() 
            {
            }
        };

        // -------------------- Methods -------------------- //

        /// Constructor
        BallAndSocketJointComponents(MemoryAllocator& allocator);

        /// Destructor
        virtual ~BallAndSocketJointComponents() override = default;

        /// Add a component
        void addComponent(Entity jointEntity, bool isSleeping, const BallAndSocketJointComponent& component);

        /// Return a pointer to a given joint
        BallAndSocketJoint* getJoint(Entity jointEntity) const;

        /// Set the joint pointer to a given joint
        void setJoint(Entity jointEntity, BallAndSocketJoint* joint) const;

        /// Return the local anchor point of body 1 for a given joint
        const Vector3& getLocalAnchorPointBody1(Entity jointEntity) const;

        void setTargetLocalInBody1(Entity jointEntity, const Quaternion & targetLocalInBody1);

        void setReferenceLocalInBody2(Entity jointEntity, const Quaternion & referenceLocalInBody2);

        void setSpringCallback(Entity jointEntity, void (*callback)(BallAndSocketJoint * joint, const Vector3 & angle, const Vector3 & velocity, Vector3 & outTargetAngle, Vector3 & outTorque));

        void setLimitsAnglesMin(Entity jointEntity, const Vector3 & angles);

        void setLimitsAnglesMax(Entity jointEntity, const Vector3 & angles);

        void setDamping(Entity jointEntity, decimal dampings);

        void * getUserData(Entity jointEntity) const;

        void setUserData(Entity jointEntity, void * userData);

        /// Set the local anchor point of body 1 for a given joint
        void setLocalAnchorPointBody1(Entity jointEntity, const Vector3& localAnchoirPointBody1);

        /// Return the local anchor point of body 2 for a given joint
        const Vector3 & getLocalAnchorPointBody2(Entity jointEntity) const;

        /// Set the local anchor point of body 2 for a given joint
        void setLocalAnchorPointBody2(Entity jointEntity, const Vector3& localAnchoirPointBody2);

        // -------------------- Friendship -------------------- //

        friend class BroadPhaseSystem;
        friend class SolveBallAndSocketJointSystem;
};

// Return a pointer to a given joint
inline BallAndSocketJoint* BallAndSocketJointComponents::getJoint(Entity jointEntity) const 
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mJoints[mMapEntityToComponentIndex[jointEntity]];
}

// Set the joint pointer to a given joint
inline void BallAndSocketJointComponents::setJoint(Entity jointEntity, BallAndSocketJoint* joint) const 
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mJoints[mMapEntityToComponentIndex[jointEntity]] = joint;
}

inline void BallAndSocketJointComponents::setTargetLocalInBody1(Entity jointEntity, const Quaternion& targetLocalInBody1)
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mTargetLocalInBody1[mMapEntityToComponentIndex[jointEntity]] = targetLocalInBody1;
}

inline void BallAndSocketJointComponents::setReferenceLocalInBody2(Entity jointEntity, const Quaternion& referenceLocalInBody2)
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mReferenceLocalInBody2[mMapEntityToComponentIndex[jointEntity]] = referenceLocalInBody2;
}

inline void BallAndSocketJointComponents::setSpringCallback(Entity jointEntity, void (*callback)(BallAndSocketJoint * joint, const Vector3 & angle, const Vector3 & velocity, Vector3 & outTargetAngle, Vector3 & outTorque))
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mCallbacks[mMapEntityToComponentIndex[jointEntity]] = callback;
}

inline void BallAndSocketJointComponents::setLimitsAnglesMin(Entity jointEntity, const Vector3 & angles)
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLimitsAnglesMin[mMapEntityToComponentIndex[jointEntity]] = angles;
}

inline void BallAndSocketJointComponents::setLimitsAnglesMax(Entity jointEntity, const Vector3 & angles)
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLimitsAnglesMax[mMapEntityToComponentIndex[jointEntity]] = angles;
}

inline void BallAndSocketJointComponents::setDamping(Entity jointEntity, decimal dampings)
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mDampings[mMapEntityToComponentIndex[jointEntity]] = dampings;
}

inline void * BallAndSocketJointComponents::getUserData(Entity jointEntity) const
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mUserData[mMapEntityToComponentIndex[jointEntity]];
}

inline void BallAndSocketJointComponents::setUserData(Entity jointEntity, void * userData)
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mUserData[mMapEntityToComponentIndex[jointEntity]] = userData;
}

// Return the local anchor point of body 1 for a given joint
inline const Vector3 & BallAndSocketJointComponents::getLocalAnchorPointBody1(Entity jointEntity) const 
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 1 for a given joint
inline void BallAndSocketJointComponents::setLocalAnchorPointBody1(Entity jointEntity, const Vector3 & localAnchoirPointBody1) 
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody1[mMapEntityToComponentIndex[jointEntity]] = localAnchoirPointBody1;
}

// Return the local anchor point of body 2 for a given joint
inline const Vector3 & BallAndSocketJointComponents::getLocalAnchorPointBody2(Entity jointEntity) const 
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    return mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]];
}

// Set the local anchor point of body 2 for a given joint
inline void BallAndSocketJointComponents::setLocalAnchorPointBody2(Entity jointEntity, const Vector3 & localAnchoirPointBody2) 
{
    assert(mMapEntityToComponentIndex.containsKey(jointEntity));
    mLocalAnchorPointBody2[mMapEntityToComponentIndex[jointEntity]] = localAnchoirPointBody2;
}

}

#endif

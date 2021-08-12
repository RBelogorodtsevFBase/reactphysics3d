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

#ifndef REACTPHYSICS3D_SOLVE_BALL_SOCKET_JOINT_SYSTEM_H
#define REACTPHYSICS3D_SOLVE_BALL_SOCKET_JOINT_SYSTEM_H

// Libraries
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/components/JointComponents.h>
#include <reactphysics3d/components/BallAndSocketJointComponents.h>
#include <reactphysics3d/components/TransformComponents.h>
#include <reactphysics3d/engine/XPBDProjections.h>

namespace reactphysics3d {

// Forward declarations
class PhysicsWorld;

// Class SolveBallAndSocketJointSystem
/**
 * This class is responsible to solve the BallAndSocketJoint constraints
 */
class SolveBallAndSocketJointSystem {

    private :

        // -------------------- Attributes -------------------- //

        /// Physics world
        PhysicsWorld& mWorld;

        /// Reference to the rigid body components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to transform components
        TransformComponents& mTransformComponents;

        /// Reference to the joint components
        JointComponents& mJointComponents;

        /// Reference to the ball-and-socket joint components
        BallAndSocketJointComponents& mBallAndSocketJointComponents;

        /// Current time step of the simulation
        decimal mTimeStep;

        XPBDProjections mXPBDProjections;

#ifdef IS_RP3D_PROFILING_ENABLED

        /// Pointer to the profiler
        Profiler* mProfiler;
#endif

    public :

        // -------------------- Methods -------------------- //

        /// Constructor
        SolveBallAndSocketJointSystem(PhysicsWorld & world, RigidBodyComponents & rigidBodyComponents,
                                      TransformComponents & transformComponents,
                                      JointComponents & jointComponents,
                                      BallAndSocketJointComponents & ballAndSocketJointComponents);

        /// Destructor
        ~SolveBallAndSocketJointSystem() = default;

        void solvePositionXPBD(decimal timeSubStep);

        void solveVelocityXPBD(decimal timeSubStep);

        /// Set the time step
        void setTimeStep(decimal timeStep);

#ifdef IS_RP3D_PROFILING_ENABLED
        /// Set the profiler
        void setProfiler(Profiler* profiler);
#endif

};

#ifdef IS_RP3D_PROFILING_ENABLED

// Set the profiler
inline void SolveBallAndSocketJointSystem::setProfiler(Profiler* profiler) {
    mProfiler = profiler;
}

#endif

// Set the time step
inline void SolveBallAndSocketJointSystem::setTimeStep(decimal timeStep) {
    assert(timeStep > decimal(0.0));
    mTimeStep = timeStep;
}

}

#endif

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

#ifndef REACTPHYSICS3D_SOLVE_FIXED_JOINT_SYSTEM_H
#define REACTPHYSICS3D_SOLVE_FIXED_JOINT_SYSTEM_H

// Libraries
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/components/RigidBodyComponents.h>
#include <reactphysics3d/components/JointComponents.h>
#include <reactphysics3d/components/FixedJointComponents.h>
#include <reactphysics3d/components/TransformComponents.h>

namespace reactphysics3d {

class PhysicsWorld;

// Class SolveFixedJointSystem
/**
 * This class is responsible to solve the FixedJoint constraints
 */
class SolveFixedJointSystem 
{
private:

        // -------------------- Attributes -------------------- //

        /// Physics world
        PhysicsWorld& mWorld;

        /// Reference to the rigid body components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to transform components
        TransformComponents& mTransformComponents;

        /// Reference to the joint components
        JointComponents& mJointComponents;

        /// Reference to the fixed joint components
        FixedJointComponents& mFixedJointComponents;

        /// Current time step of the simulation
        decimal mTimeStep;

#ifdef IS_RP3D_PROFILING_ENABLED
        /// Pointer to the profiler
        Profiler* mProfiler;
#endif
public:
        /// Constructor
        SolveFixedJointSystem(PhysicsWorld& world, RigidBodyComponents& rigidBodyComponents, TransformComponents& transformComponents,
                              JointComponents& jointComponents, FixedJointComponents& fixedJointComponents);

        /// Destructor
        ~SolveFixedJointSystem() = default;

        /// Set the time step
        void setTimeStep(decimal timeStep);

#ifdef IS_RP3D_PROFILING_ENABLED
        /// Set the profiler
        void setProfiler(Profiler* profiler);

#endif

};

#ifdef IS_RP3D_PROFILING_ENABLED
// Set the profiler
inline void SolveFixedJointSystem::setProfiler(Profiler * profiler) 
{
    mProfiler = profiler;
}
#endif

// Set the time step
inline void SolveFixedJointSystem::setTimeStep(decimal timeStep) 
{
    assert(timeStep > decimal(0.0));
    mTimeStep = timeStep;
}

}

#endif

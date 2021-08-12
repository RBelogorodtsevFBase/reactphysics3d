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
#include <reactphysics3d/systems/ConstraintSolverSystem.h>
#include <reactphysics3d/components/JointComponents.h>
#include <reactphysics3d/components/BallAndSocketJointComponents.h>
#include <reactphysics3d/utils/Profiler.h>
#include <reactphysics3d/engine/Island.h>

using namespace reactphysics3d;

// Constructor
ConstraintSolverSystem::ConstraintSolverSystem(PhysicsWorld& world, Islands& islands, RigidBodyComponents& rigidBodyComponents,
                                               TransformComponents& transformComponents,
                                               JointComponents& jointComponents,
                                               BallAndSocketJointComponents& ballAndSocketJointComponents,
                                               FixedJointComponents& fixedJointComponents,
                                               HingeJointComponents& hingeJointComponents,
                                               SliderJointComponents& sliderJointComponents)
                 : mIsWarmStartingActive(true), mIslands(islands),
                   mConstraintSolverData(rigidBodyComponents, jointComponents),
                   mSolveBallAndSocketJointSystem(world, rigidBodyComponents, transformComponents, jointComponents, ballAndSocketJointComponents),
                   mSolveFixedJointSystem(world, rigidBodyComponents, transformComponents, jointComponents, fixedJointComponents),
                   mSolveHingeJointSystem(world, rigidBodyComponents, transformComponents, jointComponents, hingeJointComponents),
                   mSolveSliderJointSystem(world, rigidBodyComponents, transformComponents, jointComponents, sliderJointComponents) {

#ifdef IS_RP3D_PROFILING_ENABLED


	mProfiler = nullptr;

#endif

}

void ConstraintSolverSystem::solvePositionXPBD(decimal timeSubStep)
{
    RP3D_PROFILE("ConstraintSolverSystem::solvePositionXPBD()", mProfiler);

    mSolveBallAndSocketJointSystem.solvePositionXPBD(timeSubStep);
    //mSolveFixedJointSystem.solvePositionXPBD(timeSubStep);
    //mSolveHingeJointSystem.solvePositionXPBD(timeSubStep);
    //mSolveSliderJointSystem.solvePositionXPBD(timeSubStep);
}

void ConstraintSolverSystem::solveVelocityXPBD(decimal timeSubStep)
{
    RP3D_PROFILE("ConstraintSolverSystem::solveVelocityXPBD()", mProfiler);

    mSolveBallAndSocketJointSystem.solveVelocityXPBD(timeSubStep);
    //mSolveFixedJointSystem.solveVelocityXPBD(timeSubStep);
    //mSolveHingeJointSystem.solveVelocityXPBD(timeSubStep);
    //mSolveSliderJointSystem.solveVelocityXPBD(timeSubStep);
}

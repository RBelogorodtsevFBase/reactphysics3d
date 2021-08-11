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

#ifndef REACTPHYSICS3D_CONTACT_SOLVER_SYSTEM_H
#define REACTPHYSICS3D_CONTACT_SOLVER_SYSTEM_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/engine/XPBDProjections.h>
#include <reactphysics3d/mathematics/Vector3.h>
#include <reactphysics3d/mathematics/Matrix3x3.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

// Declarations
class ContactPoint;
class Joint;
class ContactManifold;
class MemoryManager;
class Profiler;
class Island;
struct Islands;
class RigidBody;
class Collider;
class PhysicsWorld;
class CollisionBodyComponents;
class DynamicsComponents;
class RigidBodyComponents;
class ColliderComponents;

// Class ContactSolverSystem
/**
 * This class represents the contact solver system that is used to solve rigid bodies contacts.
 * The constraint solver is based on the "Sequential Impulse" technique described by
 * Erin Catto in his GDC slides (http://code.google.com/p/box2d/downloads/list).
 *
 * A constraint between two bodies is represented by a function C(x) which is equal to zero
 * when the constraint is satisfied. The condition C(x)=0 describes a valid position and the
 * condition dC(x)/dt=0 describes a valid velocity. We have dC(x)/dt = Jv + b = 0 where J is
 * the Jacobian matrix of the constraint, v is a vector that contains the velocity of both
 * bodies and b is the constraint bias. We are looking for a force F_c that will act on the
 * bodies to keep the constraint satisfied. Note that from the virtual work principle, we have
 * F_c = J^t * lambda where J^t is the transpose of the Jacobian matrix and lambda is a
 * Lagrange multiplier. Therefore, finding the force F_c is equivalent to finding the Lagrange
 * multiplier lambda.
 *
 * An impulse P = F * dt where F is a force and dt is the timestep. We can apply impulses a
 * body to change its velocity. The idea of the Sequential Impulse technique is to apply
 * impulses to bodies of each constraints in order to keep the constraint satisfied.
 *
 * --- Step 1 ---
 *
 * First, we integrate the applied force F_a acting of each rigid body (like gravity, ...) and
 * we obtain some new velocities v2' that tends to violate the constraints.
 *
 * v2' = v1 + dt * M^-1 * F_a
 *
 * where M is a matrix that contains mass and inertia tensor information.
 *
 * --- Step 2 ---
 *
 * During the second step, we iterate over all the constraints for a certain number of
 * iterations and for each constraint we compute the impulse to apply to the bodies needed
 * so that the new velocity of the bodies satisfy Jv + b = 0. From the Newton law, we know that
 * M * deltaV = P_c where M is the mass of the body, deltaV is the difference of velocity and
 * P_c is the constraint impulse to apply to the body. Therefore, we have
 * v2 = v2' + M^-1 * P_c. For each constraint, we can compute the Lagrange multiplier lambda
 * using : lambda = -m_c (Jv2' + b) where m_c = 1 / (J * M^-1 * J^t). Now that we have the
 * Lagrange multiplier lambda, we can compute the impulse P_c = J^t * lambda * dt to apply to
 * the bodies to satisfy the constraint.
 *
 * --- Step 3 ---
 *
 * In the third step, we integrate the new position x2 of the bodies using the new velocities
 * v2 computed in the second step with : x2 = x1 + dt * v2.
 *
 * Note that in the following code (as it is also explained in the slides from Erin Catto),
 * the value lambda is not only the lagrange multiplier but is the multiplication of the
 * Lagrange multiplier with the timestep dt. Therefore, in the following code, when we use
 * lambda, we mean (lambda * dt).
 *
 * We are using the accumulated impulse technique that is also described in the slides from
 * Erin Catto.
 *
 * We are also using warm starting. The idea is to warm start the solver at the beginning of
 * each step by applying the last impulstes for the constraints that we already existing at the
 * previous step. This allows the iterative solver to converge faster towards the solution.
 *
 * For contact constraints, we are also using split impulses so that the position correction
 * that uses Baumgarte stabilization does not change the momentum of the bodies.
 *
 * There are two ways to apply the friction constraints. Either the friction constraints are
 * applied at each contact point or they are applied only at the center of the contact manifold
 * between two bodies. If we solve the friction constraints at each contact point, we need
 * two constraints (two tangential friction directions) and if we solve the friction
 * constraints at the center of the contact manifold, we need two constraints for tangential
 * friction but also another twist friction constraint to prevent spin of the body around the
 * contact manifold center.
 */
class ContactSolverSystem
{
private:
        // Structure ContactPointSolver
        /**
         * Contact solver internal data structure that to store all the
         * information relative to a contact point
         */
        struct ContactPointSolver
        {
            /// Pointer to the external contact
            ContactPoint* externalContact;

            /// Normal vector of the contact
            Vector3 normal;

            /// Local body 1 contact point
            Vector3 r1;

            /// Local body 1 contact point
            Vector3 r2;

            /// Normal velocity before the XPBD velocity update
            decimal vNPreUpdate;

            bool contactHappened;

            decimal lambdaN;

            decimal lambdaT;

            /// True if the contact was existing last time step
            bool isRestingContact;
        };

        // Structure ContactManifoldSolver
        /**
         * Contact solver internal data structure to store all the
         * information relative to a contact manifold.
         */
        struct ContactManifoldSolver 
        {
            /// Pointer to the external contact manifold
            ContactManifold* externalContactManifold;

            /// Index of body 1 in the dynamics components arrays
            uint32 rigidBodyComponentIndexBody1;

            /// Index of body 2 in the dynamics components arrays
            uint32 rigidBodyComponentIndexBody2;

            /// Mix friction coefficient for the two bodies
            decimal frictionCoefficient;

            /// Average normal vector of the contact manifold
            Vector3 normal;

            /// Number of contact points
            int8 nbContacts;
        };

        // -------------------- Attributes -------------------- //

        /// Memory manager
        MemoryManager& mMemoryManager;

        /// Physics world
        PhysicsWorld& mWorld;

        /// Current time step
        decimal mTimeStep;

        /// Contact constraints
        ContactManifoldSolver* mContactConstraints;

        /// Contact points
        ContactPointSolver* mContactPoints;

        /// Number of contact point constraints
        uint mNbContactPoints;

        /// Number of contact constraints
        uint mNbContactManifolds;

        /// Reference to the islands
        Islands& mIslands;

        /// Pointer to the list of contact manifolds from narrow-phase
        List<ContactManifold>* mAllContactManifolds;

        /// Pointer to the list of contact points from narrow-phase
        List<ContactPoint>* mAllContactPoints;

        /// Reference to the body components
        CollisionBodyComponents& mBodyComponents;

        /// Reference to the dynamics components
        RigidBodyComponents& mRigidBodyComponents;

        /// Reference to the colliders components
        ColliderComponents& mColliderComponents;

        XPBDProjections mXPBDProjections;

#ifdef IS_RP3D_PROFILING_ENABLED
		/// Pointer to the profiler
		Profiler* mProfiler;
#endif

        // -------------------- Methods -------------------- //

        /// Compute the collision restitution factor from the restitution factor of each collider
        decimal computeMixedRestitutionFactor(Collider* collider1, Collider* collider2) const;

        /// Compute the mixed friction coefficient from the friction coefficient of each collider
        decimal computeMixedFrictionCoefficient(Collider* collider1, Collider* collider2) const;

        /// Compute th mixed rolling resistance factor between two colliders
        decimal computeMixedRollingResistance(Collider* collider1, Collider* collider2) const;

   public:

        // -------------------- Methods -------------------- //

        /// Constructor
        ContactSolverSystem(MemoryManager& memoryManager, PhysicsWorld& world, Islands& islands, CollisionBodyComponents& bodyComponents, RigidBodyComponents& rigidBodyComponents, ColliderComponents& colliderComponents);

        /// Destructor
        ~ContactSolverSystem() = default;

        void initXPBD(List<ContactManifold> * contactManifolds, List<ContactPoint> * contactPoints, decimal timeStep);

        void solvePositionXPBD();

        void solveVelocityXPBD();

        void cacheVnXPBD();

        /// Initialize the contact constraints
        void init(List<ContactManifold>* contactManifolds, List<ContactPoint>* contactPoints, decimal timeStep);

        /// Initialize the constraint solver for a given island
        void initializeForIsland(uint islandIndex);

        /// Release allocated memory
        void reset();

#ifdef IS_RP3D_PROFILING_ENABLED
		/// Set the profiler
		void setProfiler(Profiler* profiler);
#endif
};

#ifdef IS_RP3D_PROFILING_ENABLED
// Set the profiler
inline void ContactSolverSystem::setProfiler(Profiler* profiler) 
{
	mProfiler = profiler;
}
#endif

}

#endif

#ifndef REACTPHYSICS3D_XPBD_PROJECTIONS_H
#define REACTPHYSICS3D_XPBD_PROJECTIONS_H

// Libraries
#include <reactphysics3d/configuration.h>
#include <reactphysics3d/mathematics/Vector3.h>

/// ReactPhysics3D namespace
namespace reactphysics3d {

class RigidBodyComponents;

// Class XPBDProjections
/**
 * This class contains XPBD core projection operations.
 */
class XPBDProjections {

    private:

        RigidBodyComponents & mRigidBodyComponents;

        // -------------------- Methods -------------------- //

        decimal getGeneralizedInverseMassXPBD(const Vector3 & normal, const Vector3 & r, uint32 componentIndexBody);

        decimal getGeneralizedInverseMassXPBD(const Vector3 & normal, uint32 componentIndexBody);

        void applyBodyCorrectionXPBD(const Vector3 & corr, const Vector3 & r, uint32 componentIndexBody);

        void applyBodyCorrectionXPBD(const Vector3 & corr, uint32 componentIndexBody);

        void applyBodyCorrectionVelocityXPBD(const Vector3 & corr, uint32 componentIndexBody);

        void applyBodyCorrectionVelocityXPBD(const Vector3 & corr, const Vector3 & r, uint32 componentIndexBody);

        void applyBodyRotationXPBD(const Vector3 & rot, uint32 componentIndexBody);

   public:

        // -------------------- Methods -------------------- //

        /// Constructor
        XPBDProjections(RigidBodyComponents & rigidBodyComponents);

        /// Destructor
        ~XPBDProjections() = default;

        void applyBodyPairCorrectionXPBD(const Vector3 & corr, decimal compliance, decimal dt, decimal & lambda, uint32 componentIndexBody1, uint32 componentIndexBody2);

        void applyBodyPairCorrectionXPBD(const Vector3 & corr, decimal compliance, const Vector3 & r1, const Vector3 & r2, decimal dt, decimal & lambda, uint32 componentIndexBody1, uint32 componentIndexBody2);

        void applyBodyPairCorrectionVelocityXPBD(const Vector3 & corr, decimal compliance, decimal dt, uint32 componentIndexBody1, uint32 componentIndexBody2);

        void applyBodyPairCorrectionVelocityXPBD(const Vector3 & corr, decimal compliance, const Vector3 & r1, const Vector3 & r2, decimal dt, uint32 componentIndexBody1, uint32 componentIndexBody2);

        void limitAngleXPBD(uint32 componentIndexBodyA, uint32 componentIndexBodyB, const Vector3 & n, const Vector3 & n1, const Vector3 & n2, decimal minAngle, decimal maxAngle, decimal compliance, decimal dt, decimal & lambda, decimal maxCorr = PI);
};

}

#endif

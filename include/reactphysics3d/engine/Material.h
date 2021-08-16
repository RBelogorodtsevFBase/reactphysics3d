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

#ifndef REACTPHYSICS3D_MATERIAL_H
#define REACTPHYSICS3D_MATERIAL_H

// Libraries
#include <cassert>
#include <reactphysics3d/configuration.h>

namespace reactphysics3d {

// Class Material
/**
 * This class contains the material properties of a collider that will be use for
 * the dynamics simulation like the friction coefficient or the bounciness of the rigid
 * body.
 */
class Material 
{
private:

        /// Static friction coefficient
        decimal mFrictionStatic;

        /// Dynamic friction coefficient
        decimal mFrictionDynamic;

        /// Coefficient of restitution
        decimal mRestitution;

        /// Density of mass used to compute the mass of the collider
        decimal mMassDensity;

        // -------------------- Methods -------------------- //

        /// Constructor
        Material(decimal frictionStatic, decimal frictionDynamic, decimal restitution, decimal massDensity = decimal(1.0));

        /// Copy-constructor
        Material(const Material & material);

        /// Destructor
        ~Material() = default;

public:
        /// Return the restitution
        decimal getRestitution() const;

        /// Set the restitution.
        void setRestitution(decimal restitution);

        /// Return the static friction coefficient
        decimal getFrictionStatic() const;

        /// Set the static friction coefficient.
        void setFrictionStatic(decimal frictionStatic);

        /// Return the dynamic friction coefficient
        decimal getFrictionDynamic() const;

        /// Set the dynamic friction coefficient.
        void setFrictionDynamic(decimal frictionDynamic);

        /// Return the mass density of the collider
        decimal getMassDensity() const;

        /// Set the mass density of the collider
        void setMassDensity(decimal massDensity);

        /// Return a string representation for the material
        std::string to_string() const;

        /// Overloaded assignment operator
        Material & operator=(const Material & material);

        // ---------- Friendship ---------- //

        friend class Collider;
};

// Return the bounciness
/**
 * @return Bounciness factor (between 0 and 1) where 1 is very bouncy
 */
inline decimal Material::getRestitution() const 
{
    return mRestitution;
}

// Set the bounciness.
/**
 * @param restitution Coefficient of Restitution
 */
inline void Material::setRestitution(decimal restitution) 
{
    assert(restitution >= decimal(0.0));
    mRestitution = restitution;
}

// Return the static friction coefficient
/**
 * @return Static friction coefficient
 */
inline decimal Material::getFrictionStatic() const
{
    return mFrictionStatic;
}

// Set the static friction coefficient.
/// The static friction coefficient has to be a positive value
/**
 * @param frictionStatic Friction coefficient (positive value)
 */
inline void Material::setFrictionStatic(decimal frictionStatic)
{
    assert(frictionStatic >= decimal(0.0));
    mFrictionStatic = frictionStatic;
}

// Return the dynamic friction coefficient
/**
 * @return Dynamic friction coefficient
 */
inline decimal Material::getFrictionDynamic() const
{
    return mFrictionDynamic;
}

// Set the dynamic friction coefficient.
/// The dynamic friction coefficient has to be a positive value
/**
 * @param frictionDynamic Friction coefficient
 */
inline void Material::setFrictionDynamic(decimal frictionDynamic)
{
    assert(frictionDynamic >= decimal(0.0));
    mFrictionDynamic = frictionDynamic;
}

// Return the mass density of the collider
inline decimal Material::getMassDensity() const 
{
   return mMassDensity;
}

// Set the mass density of the collider
/**
 * @param massDensity The mass density of the collider
 */
inline void Material::setMassDensity(decimal massDensity) 
{
   mMassDensity = massDensity;
}

// Return a string representation for the material
inline std::string Material::to_string() const 
{
    std::stringstream ss;

    ss << "frictionStatic=" << mFrictionStatic << std::endl;
    ss << "frictionDynamic=" << mFrictionDynamic << std::endl;
    ss << "restitution=" << mRestitution << std::endl;

    return ss.str();
}

// Overloaded assignment operator
inline Material & Material::operator=(const Material & material)
{
    // Check for self-assignment
    if (this != &material)
    {
        mFrictionStatic = material.mFrictionStatic;
        mFrictionDynamic = material.mFrictionDynamic;
        mRestitution = material.mRestitution;
    }

    // Return this material
    return *this;
}

}

#endif

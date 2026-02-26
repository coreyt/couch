#ifndef SOFA_COMPAT_H
#define SOFA_COMPAT_H

// Compatibility shim for SOFA API changes between v24.06 and v25.12.
// v25.12 deleted ConstVecCoordId::position() and ConstVecDerivId::velocity()
// in favor of constexpr variables in sofa::core::vec_id::read_access.

#include <sofa/version.h>
#include <sofa/core/VecId.h>

#if SOFA_VERSION >= 250600
    // v25.06+ uses constexpr variables instead of static functions
    #define SOFA_CONST_POSITION  sofa::core::vec_id::read_access::position
    #define SOFA_CONST_VELOCITY  sofa::core::vec_id::read_access::velocity
#else
    // v24.06 uses static member functions
    #define SOFA_CONST_POSITION  sofa::core::ConstVecCoordId::position()
    #define SOFA_CONST_VELOCITY  sofa::core::ConstVecDerivId::velocity()
#endif

#endif // SOFA_COMPAT_H

#pragma once

#include <memory>

#include <ct/hydro/common/HydroTypes.h>
#include <ct/hydro/parameters/HydroParameters.h>

namespace ct {
namespace hydro {

template <typename SCALAR = double>
class HydroForceBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Kinematics = HydroKinematics<SCALAR>;
    using Parameters = HydroParameters<SCALAR>;
    using Output = HydroForceOutput<SCALAR>;

    virtual ~HydroForceBase() = default;

    virtual Output compute(const Kinematics& kinematics, const Parameters& params) const = 0;
};

template <typename SCALAR>
using HydroForceBasePtr = std::shared_ptr<HydroForceBase<SCALAR>>;

}  // namespace hydro
}  // namespace ct

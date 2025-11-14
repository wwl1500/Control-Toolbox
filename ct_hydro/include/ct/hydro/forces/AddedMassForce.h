#pragma once

#include <ct/hydro/forces/HydroForceBase.h>
#include <cmath>

namespace ct {
namespace hydro {

template <typename SCALAR = double>
class AddedMassForce : public HydroForceBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using typename HydroForceBase<SCALAR>::Kinematics;
    using typename HydroForceBase<SCALAR>::Parameters;
    using typename HydroForceBase<SCALAR>::Output;

    Output compute(const Kinematics& kinematics, const Parameters& params) const override
    {
        const size_t n = static_cast<size_t>(kinematics.theta.size());
        Output result(n);
        result.setZero(n);

        if (n == 0)
            return result;

        for (size_t i = 0; i < n; ++i)
        {
            const SCALAR s = std::sin(kinematics.theta(i));
            const SCALAR c = std::cos(kinematics.theta(i));

            const SCALAR acc_x = kinematics.accelerationX(i);
            const SCALAR acc_y = kinematics.accelerationY(i);

            const SCALAR fx = -(params.added_mass_normal * s * s * acc_x -
                params.added_mass_normal * s * c * acc_y);
            const SCALAR fy = -(-params.added_mass_normal * s * c * acc_x +
                params.added_mass_normal * c * c * acc_y);

            result.fx(i) = fx;
            result.fy(i) = fy;
        }

        return result;
    }
};

}  // namespace hydro
}  // namespace ct

#pragma once

#include <ct/hydro/forces/HydroForceBase.h>
#include <cmath>

namespace ct {
namespace hydro {

template <typename SCALAR = double>
class DragForce : public HydroForceBase<SCALAR>
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

            const SCALAR v_body_x = c * kinematics.velocityX(i) + s * kinematics.velocityY(i);
            const SCALAR v_body_y = -s * kinematics.velocityX(i) + c * kinematics.velocityY(i);

            const SCALAR vc_body_x = c * params.current_velocity(0) + s * params.current_velocity(1);
            const SCALAR vc_body_y = -s * params.current_velocity(0) + c * params.current_velocity(1);

            const SCALAR rel_body_x = v_body_x - vc_body_x;
            const SCALAR rel_body_y = v_body_y - vc_body_y;

            SCALAR fx_body = -params.drag_tangential * rel_body_x;
            SCALAR fy_body = -params.drag_normal * rel_body_y;

            if (params.include_nonlinear_drag)
            {
                fx_body -= params.drag_tangential * rel_body_x * std::abs(rel_body_x);
                fy_body -= params.drag_normal * rel_body_y * std::abs(rel_body_y);
            }

            const SCALAR fx_world = c * fx_body - s * fy_body;
            const SCALAR fy_world = s * fx_body + c * fy_body;

            result.fx(i) = fx_world;
            result.fy(i) = fy_world;
        }

        return result;
    }
};

}  // namespace hydro
}  // namespace ct

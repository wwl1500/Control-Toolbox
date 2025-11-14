#pragma once

#include <Eigen/Dense>

namespace ct {
namespace hydro {

template <typename SCALAR = double>
struct HydroParameters
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Vector2s = Eigen::Matrix<SCALAR, 2, 1>;

    SCALAR fluid_density = static_cast<SCALAR>(1000.0);
    SCALAR added_mass_normal = static_cast<SCALAR>(0.0);
    SCALAR drag_tangential = static_cast<SCALAR>(0.0);
    SCALAR drag_normal = static_cast<SCALAR>(0.0);
    SCALAR torque_added_mass = static_cast<SCALAR>(0.0);
    SCALAR torque_linear_damping = static_cast<SCALAR>(0.0);
    SCALAR torque_nonlinear_damping = static_cast<SCALAR>(0.0);
    bool include_nonlinear_drag = true;
    Vector2s current_velocity = Vector2s::Zero();
};

}  // namespace hydro
}  // namespace ct

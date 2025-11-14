#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace ct {
namespace hydro {

template <typename SCALAR = double>
class FluidTorque
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

    FluidTorque() = default;

    FluidTorque(SCALAR lambda1, SCALAR lambda2, SCALAR lambda3)
        : lambda1_(lambda1), lambda2_(lambda2), lambda3_(lambda3)
    {
    }

    SCALAR addedMassGain() const { return lambda1_; }

    SCALAR linearDampingGain() const { return lambda2_; }

    VectorXs nonlinearDamping(const VectorXs& thetaDot) const
    {
        VectorXs result(thetaDot.size());
        for (int i = 0; i < thetaDot.size(); ++i)
        {
            result(i) = lambda3_ * std::abs(thetaDot(i)) * thetaDot(i);
        }
        return result;
    }

    void setCoefficients(SCALAR lambda1, SCALAR lambda2, SCALAR lambda3)
    {
        lambda1_ = lambda1;
        lambda2_ = lambda2;
        lambda3_ = lambda3;
    }

private:
    SCALAR lambda1_ = static_cast<SCALAR>(0.0);
    SCALAR lambda2_ = static_cast<SCALAR>(0.0);
    SCALAR lambda3_ = static_cast<SCALAR>(0.0);
};

}  // namespace hydro
}  // namespace ct

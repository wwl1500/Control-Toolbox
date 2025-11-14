#pragma once

#include <Eigen/Dense>

namespace ct {
namespace hydro {

template <typename SCALAR>
struct HydroKinematics
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

    VectorXs theta;
    VectorXs thetaDot;
    VectorXs velocityX;
    VectorXs velocityY;
    VectorXs accelerationX;
    VectorXs accelerationY;

    void resize(size_t n)
    {
        theta.resize(n);
        thetaDot.resize(n);
        velocityX.resize(n);
        velocityY.resize(n);
        accelerationX.resize(n);
        accelerationY.resize(n);
    }
};

template <typename SCALAR>
struct HydroForceOutput
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using VectorXs = Eigen::Matrix<SCALAR, Eigen::Dynamic, 1>;

    VectorXs fx;
    VectorXs fy;

    HydroForceOutput() = default;

    explicit HydroForceOutput(size_t n) { setZero(n); }

    void setZero(size_t n)
    {
        fx = VectorXs::Zero(n);
        fy = VectorXs::Zero(n);
    }

    size_t size() const { return static_cast<size_t>(fx.size()); }

    HydroForceOutput& operator+=(const HydroForceOutput& other)
    {
        if (fx.size() == 0)
        {
            fx = other.fx;
            fy = other.fy;
            return *this;
        }

        fx += other.fx;
        fy += other.fy;
        return *this;
    }
};

}  // namespace hydro
}  // namespace ct

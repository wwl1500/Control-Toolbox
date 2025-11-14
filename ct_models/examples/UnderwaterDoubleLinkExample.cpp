#include <ct/core/core.h>
#include <ct/hydro/hydro.h>

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <cmath>

namespace
{
static constexpr size_t STATE_DIM = 8;
static constexpr size_t CONTROL_DIM = 1;
}  // namespace

class UnderwaterDoubleLinkSystem : public ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, double>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using state_vector_t = ct::core::StateVector<STATE_DIM, double>;
    using control_vector_t = ct::core::ControlVector<CONTROL_DIM, double>;
    using time_t = double;

    UnderwaterDoubleLinkSystem()
        : ct::core::ControlledSystem<STATE_DIM, CONTROL_DIM, double>(ct::core::SYSTEM_TYPE::GENERAL)
    {
        // 计算转动惯量
        J_ = (1.0 / 3.0) * m_ * l_ * l_;

        hydro_params_.added_mass_normal = 0.3958;
        hydro_params_.drag_tangential = 0.2639;
        hydro_params_.drag_normal = 8.4;
        hydro_params_.torque_added_mass = 4.3103e-4;
        hydro_params_.torque_linear_damping = 2.2629e-5;
        hydro_params_.torque_nonlinear_damping = 2.2988e-7;
        hydro_params_.current_velocity << 0.1, 0.1;

        hydro_force_ = std::make_shared<ct::hydro::HydroForceComposite<double>>();
        hydro_force_->addForce(std::make_shared<ct::hydro::AddedMassForce<double>>());
        hydro_force_->addForce(std::make_shared<ct::hydro::DragForce<double>>());
        hydro_torque_.setCoefficients(
            hydro_params_.torque_added_mass,
            hydro_params_.torque_linear_damping,
            hydro_params_.torque_nonlinear_damping);

        D_ << 1.0, -1.0;
        A_ << 1.0, 1.0;
        e_ << 1.0, 1.0;

        const double invDDT = 1.0 / (D_ * D_.transpose())(0, 0);
        K_ = A_.transpose() * invDDT * D_;

        Eigen::Matrix2d projector = Eigen::Matrix2d::Identity() - 0.5 * e_ * e_.transpose();
        Eigen::CompleteOrthogonalDecomposition<Eigen::Matrix2d> cod(projector);
        Eigen::Matrix2d projector_pinv = cod.pseudoInverse();

        H_ = projector_pinv * K_.transpose();
        V_ = H_ * K_;
    }

    UnderwaterDoubleLinkSystem(const UnderwaterDoubleLinkSystem& other) = default;

    UnderwaterDoubleLinkSystem* clone() const override { return new UnderwaterDoubleLinkSystem(*this); }

    void computeControlledDynamics(const state_vector_t& state,
        const time_t& /*t*/,
        const control_vector_t& control,
        state_vector_t& derivative) override
    {
        (void)control;

        const Eigen::Vector2d theta = state.template segment<2>(0);
        const Eigen::Vector2d thetaDot = state.template segment<2>(4);
        const Eigen::Vector2d pDot = state.template segment<2>(6);

        Eigen::Array2d sinTheta = theta.array().sin();
        Eigen::Array2d cosTheta = theta.array().cos();

        Eigen::Matrix2d S_theta = sinTheta.matrix().asDiagonal();
        Eigen::Matrix2d C_theta = cosTheta.matrix().asDiagonal();
        Eigen::Matrix2d S_theta_sq = S_theta * S_theta;
        Eigen::Matrix2d C_theta_sq = C_theta * C_theta;
        Eigen::Matrix2d S_thetaC_theta = S_theta * C_theta;

        Eigen::Vector2d thetaDotSq = thetaDot.array().square().matrix();

        Eigen::Matrix2d Mtheta = J_ * Eigen::Matrix2d::Identity();
        Mtheta += m_ * l_ * l_ * (S_theta * V_ * S_theta + C_theta * V_ * C_theta);
        Mtheta += hydro_torque_.addedMassGain() * Eigen::Matrix2d::Identity();
        Mtheta += l_ * l_ * (S_theta * K_ * (hydro_params_.added_mass_normal * S_theta_sq) * H_ * S_theta +
                             S_theta * K_ * (hydro_params_.added_mass_normal * S_thetaC_theta) * H_ * C_theta +
                             C_theta * K_ * (hydro_params_.added_mass_normal * S_thetaC_theta) * H_ * S_theta +
                             C_theta * K_ * (hydro_params_.added_mass_normal * C_theta_sq) * H_ * C_theta);

        Eigen::Matrix2d Wtheta = m_ * l_ * l_ * (S_theta * V_ * C_theta - C_theta * V_ * S_theta);
        Wtheta += l_ * l_ * (S_theta * K_ * (hydro_params_.added_mass_normal * S_theta_sq) * H_ * C_theta -
                             S_theta * K_ * (hydro_params_.added_mass_normal * S_thetaC_theta) * H_ * S_theta +
                             C_theta * K_ * (hydro_params_.added_mass_normal * S_thetaC_theta) * H_ * C_theta -
                             C_theta * K_ * (hydro_params_.added_mass_normal * C_theta_sq) * H_ * S_theta);

        Eigen::Matrix2d Vtheta = hydro_torque_.linearDampingGain() * Eigen::Matrix2d::Identity();

        Eigen::Vector2d dotX = computeDotX(theta, thetaDot);
        Eigen::Vector2d dotY = computeDotY(theta, thetaDot);

        ct::hydro::HydroKinematics<double> hydro_state;
        hydro_state.theta = theta;
        hydro_state.thetaDot = thetaDot;
        hydro_state.velocityX = dotX;
        hydro_state.velocityY = dotY;
        hydro_state.accelerationX = Eigen::Vector2d::Zero();
        hydro_state.accelerationY = Eigen::Vector2d::Zero();

        const auto hydro_drag = hydro_force_->compute(hydro_state, hydro_params_);
        Eigen::Vector2d lambda3Term = hydro_torque_.nonlinearDamping(thetaDot);

        Eigen::Vector2d rhs = -(Wtheta * thetaDotSq) - (Vtheta * thetaDot) - lambda3Term +
                              l_ * (S_theta * K_ * hydro_drag.fx) - l_ * (C_theta * K_ * hydro_drag.fy);

        Eigen::Vector2d thetaAcc = Mtheta.fullPivLu().solve(rhs);

        Eigen::Vector2d ddotX = l_ * H_ * (C_theta * thetaDotSq + S_theta * thetaAcc);
        Eigen::Vector2d ddotY = l_ * H_ * (S_theta * thetaDotSq - C_theta * thetaAcc);

        hydro_state.accelerationX = ddotX;
        hydro_state.accelerationY = ddotY;
        const auto hydro_total = hydro_force_->compute(hydro_state, hydro_params_);

        Eigen::Vector2d pAcc;
        pAcc(0) = hydro_total.fx.sum() / (2.0 * m_);
        pAcc(1) = hydro_total.fy.sum() / (2.0 * m_);

        derivative.template segment<2>(0) = thetaDot;
        derivative.template segment<2>(2) = pDot;
        derivative.template segment<2>(4) = thetaAcc;
        derivative.template segment<2>(6) = pAcc;
    }

private:
    double m_ = 0.6597;  // 每个连杆质量 (kg)
    double l_ = 0.14;    // 半长 (m)
    double J_;            // 转动惯量，将在构造函数中计算

    ct::hydro::HydroParameters<double> hydro_params_{};
    std::shared_ptr<ct::hydro::HydroForceComposite<double>> hydro_force_{};
    ct::hydro::FluidTorque<double> hydro_torque_{};

    Eigen::RowVector2d D_;
    Eigen::RowVector2d A_;
    Eigen::Vector2d e_;
    Eigen::Matrix2d K_;
    Eigen::Matrix2d H_;
    Eigen::Matrix2d V_;

    Eigen::Vector2d computeDotX(const Eigen::Vector2d& theta, const Eigen::Vector2d& thetaDot) const
    {
        double s1 = std::sin(theta(0));
        double s2 = std::sin(theta(1));

        Eigen::Vector2d dx;
        dx(0) = -l_ * s1 * thetaDot(0);
        dx(1) = -2.0 * l_ * s1 * thetaDot(0) - l_ * s2 * thetaDot(1);
        return dx;
    }

    Eigen::Vector2d computeDotY(const Eigen::Vector2d& theta, const Eigen::Vector2d& thetaDot) const
    {
        double c1 = std::cos(theta(0));
        double c2 = std::cos(theta(1));

        Eigen::Vector2d dy;
        dy(0) = l_ * c1 * thetaDot(0);
        dy(1) = 2.0 * l_ * c1 * thetaDot(0) + l_ * c2 * thetaDot(1);
        return dy;
    }
};

int main()
{
    auto system = std::make_shared<UnderwaterDoubleLinkSystem>();

    ct::core::Integrator<STATE_DIM> integrator(system, ct::core::IntegrationType::RK4);

    ct::core::StateVector<STATE_DIM> state;
    state.setZero();
    // 初始角度（弧度）
    state(0) = 0.1;   // 第一个连杆角度
    state(1) = -0.1;  // 第二个连杆角度
    // 初始质心位置（m）
    state(2) = 0.0;   // x坐标
    state(3) = 0.0;   // y坐标
    // 初始角速度（rad/s）
    state(4) = 0.0;
    state(5) = 0.0;
    // 初始质心速度（m/s）
    state(6) = 0.0;
    state(7) = 0.0;

    const double dt = 0.01;
    const double t0 = 0.0;
    const double tf = 5.0;  // 延长仿真时间以观察动态

    integrator.integrate_const(state, t0, tf, dt);

    std::cout << "Final state after " << tf << " seconds:\n" << state.transpose() << std::endl;

    return 0;
}

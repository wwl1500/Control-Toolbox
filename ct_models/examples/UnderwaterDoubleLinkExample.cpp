#include <ct/core/core.h>

#include <Eigen/Dense>
#include <iostream>
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
        
        // 设置环境流速
        v_c_ << 0.1, 0.1;  // [0.1, 0.1]^T m/s
        
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

        Eigen::Matrix2d Lambda1 = lambda1_ * Eigen::Matrix2d::Identity();
        Eigen::Matrix2d Lambda2 = lambda2_ * Eigen::Matrix2d::Identity();

        Eigen::Matrix2d Mtheta = J_ * Eigen::Matrix2d::Identity();
        Mtheta += m_ * l_ * l_ * (S_theta * V_ * S_theta + C_theta * V_ * C_theta);
        Mtheta += Lambda1;
        Mtheta += l_ * l_ * (S_theta * K_ * (mu_n_ * S_theta_sq) * H_ * S_theta +
                             S_theta * K_ * (mu_n_ * S_thetaC_theta) * H_ * C_theta +
                             C_theta * K_ * (mu_n_ * S_thetaC_theta) * H_ * S_theta +
                             C_theta * K_ * (mu_n_ * C_theta_sq) * H_ * C_theta);

        Eigen::Matrix2d Wtheta = m_ * l_ * l_ * (S_theta * V_ * C_theta - C_theta * V_ * S_theta);
        Wtheta += l_ * l_ * (S_theta * K_ * (mu_n_ * S_theta_sq) * H_ * C_theta -
                             S_theta * K_ * (mu_n_ * S_thetaC_theta) * H_ * S_theta +
                             C_theta * K_ * (mu_n_ * S_thetaC_theta) * H_ * C_theta -
                             C_theta * K_ * (mu_n_ * C_theta_sq) * H_ * S_theta);

        Eigen::Matrix2d Vtheta = Lambda2;

        Eigen::Vector2d dotX = computeDotX(theta, thetaDot);
        Eigen::Vector2d dotY = computeDotY(theta, thetaDot);

        // 计算相对速度（考虑环境流速）
        // 将环境流速从惯性系转换到各连杆坐标系
        Eigen::Vector2d v_c1, v_c2;  // 环境流速在各连杆坐标系中的表示
        v_c1(0) = std::cos(theta(0)) * v_c_(0) + std::sin(theta(0)) * v_c_(1);
        v_c1(1) = -std::sin(theta(0)) * v_c_(0) + std::cos(theta(0)) * v_c_(1);
        v_c2(0) = std::cos(theta(1)) * v_c_(0) + std::sin(theta(1)) * v_c_(1);
        v_c2(1) = -std::sin(theta(1)) * v_c_(0) + std::cos(theta(1)) * v_c_(1);

        // 相对速度 = 绝对速度 - 环境流速
        Eigen::Vector2d dotX_rel, dotY_rel;
        dotX_rel(0) = dotX(0) - v_c1(0);
        dotY_rel(0) = dotY(0) - v_c1(1);
        dotX_rel(1) = dotX(1) - v_c2(0);
        dotY_rel(1) = dotY(1) - v_c2(1);

        Eigen::Vector2d fDxI;
        Eigen::Vector2d fDyI;
        Eigen::Vector2d fDxII;
        Eigen::Vector2d fDyII;

        for (int i = 0; i < 2; ++i)
        {
            double s = sinTheta(i);
            double c = cosTheta(i);

            double diagXX = c_t_ * c * c + c_n_ * s * s;
            double diagYY = c_t_ * s * s + c_n_ * c * c;
            double offDiag = (c_t_ - c_n_) * s * c;

            // 使用相对速度计算阻尼力
            fDxI(i) = -(diagXX * dotX_rel(i) + offDiag * dotY_rel(i));
            fDyI(i) = -(offDiag * dotX_rel(i) + diagYY * dotY_rel(i));

            double vRx = c * dotX_rel(i) + s * dotY_rel(i);
            double vRy = -s * dotX_rel(i) + c * dotY_rel(i);

            double signVrx = (vRx > 0.0) - (vRx < 0.0);
            double signVry = (vRy > 0.0) - (vRy < 0.0);

            fDxII(i) = -(c_t_ * c * signVrx * vRx * vRx - c_n_ * s * signVry * vRy * vRy);
            fDyII(i) = -(c_t_ * s * signVrx * vRx * vRx + c_n_ * c * signVry * vRy * vRy);
        }

        Eigen::Vector2d fDx = fDxI + fDxII;
        Eigen::Vector2d fDy = fDyI + fDyII;

        Eigen::Vector2d lambda3Term = lambda3_ * thetaDot.cwiseAbs().cwiseProduct(thetaDot);

        Eigen::Vector2d rhs = -(Wtheta * thetaDotSq) - (Vtheta * thetaDot) - lambda3Term +
                              l_ * (S_theta * K_ * fDx) - l_ * (C_theta * K_ * fDy);

        Eigen::Vector2d thetaAcc = Mtheta.fullPivLu().solve(rhs);

        Eigen::Vector2d ddotX = l_ * H_ * (C_theta * thetaDotSq + S_theta * thetaAcc);
        Eigen::Vector2d ddotY = l_ * H_ * (S_theta * thetaDotSq - C_theta * thetaAcc);

        Eigen::Vector2d fAx;
        Eigen::Vector2d fAy;
        for (int i = 0; i < 2; ++i)
        {
            double s = sinTheta(i);
            double c = cosTheta(i);
            fAx(i) = -(mu_n_ * s * s * ddotX(i) - mu_n_ * s * c * ddotY(i));
            fAy(i) = -(-mu_n_ * s * c * ddotX(i) + mu_n_ * c * c * ddotY(i));
        }

        Eigen::Vector2d totalFx = fAx + fDx;
        Eigen::Vector2d totalFy = fAy + fDy;

        Eigen::Vector2d pAcc;
        pAcc(0) = totalFx.sum() / (2.0 * m_);
        pAcc(1) = totalFy.sum() / (2.0 * m_);

        derivative.template segment<2>(0) = thetaDot;
        derivative.template segment<2>(2) = pDot;
        derivative.template segment<2>(4) = thetaAcc;
        derivative.template segment<2>(6) = pAcc;
    }

private:
    // 根据论文参数设置
    double m_ = 0.6597;  // 每个连杆质量 (kg)
    double l_ = 0.14;    // 半长 (m)
    double J_;            // 转动惯量，将在构造函数中计算
    double lambda1_ = 4.3103e-4;   // 流体扭矩参数（附加质量）
    double lambda2_ = 2.2629e-5;   // 流体扭矩参数（线性阻尼）
    double lambda3_ = 2.2988e-7;   // 流体扭矩参数（非线性阻尼）
    double mu_n_ = 0.3958;          // 法向附加质量参数
    double c_t_ = 0.2639;           // 切向阻尼系数
    double c_n_ = 8.4;              // 法向阻尼系数
    
    // 环境流速 (m/s)
    Eigen::Vector2d v_c_;

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

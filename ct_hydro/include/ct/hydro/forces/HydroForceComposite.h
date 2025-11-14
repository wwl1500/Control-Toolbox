#pragma once

#include <vector>

#include <ct/hydro/forces/HydroForceBase.h>

namespace ct {
namespace hydro {

template <typename SCALAR = double>
class HydroForceComposite : public HydroForceBase<SCALAR>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using Base = HydroForceBase<SCALAR>;
    using typename Base::Kinematics;
    using typename Base::Parameters;
    using typename Base::Output;

    void addForce(const HydroForceBasePtr<SCALAR>& component) { components_.push_back(component); }

    Output compute(const Kinematics& kinematics, const Parameters& params) const override
    {
        const size_t n = static_cast<size_t>(kinematics.theta.size());
        Output output(n);
        output.setZero(n);

        for (const auto& component : components_)
        {
            if (component)
                output += component->compute(kinematics, params);
        }

        return output;
    }

private:
    std::vector<HydroForceBasePtr<SCALAR>> components_;
};

}  // namespace hydro
}  // namespace ct

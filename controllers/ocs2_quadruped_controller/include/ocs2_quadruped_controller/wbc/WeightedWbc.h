//
// Created by qiayuan on 22-12-23.
//
#pragma once

#include "WbcBase.h"

namespace ocs2::legged_robot
{
    struct WbcDiagnostics
    {
        int init_status{0};
        int solution_status{0};
        int n_wsr{0};
        size_t mode{0};
        size_t contacts{0};
        bool has_finite_solution{false};
        scalar_t max_constraint_violation{0.0};
        scalar_t max_torque{0.0};
    };

    class WeightedWbc final : public WbcBase
    {
    public:
        using WbcBase::WbcBase;

        vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured,
                        size_t mode,
                        scalar_t period) override;

        void loadTasksSetting(const std::string& taskFile, bool verbose) override;

        const WbcDiagnostics& diagnostics() const { return diagnostics_; }

    protected:
        Task formulateConstraints();

        Task formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired,
                                    scalar_t period);

    private:
        scalar_t weightSwingLeg_, weightBaseAccel_, weightContactForce_;
        vector_t last_valid_solution_;
        size_t qp_warning_count_{0};
        WbcDiagnostics diagnostics_;
    };
} // namespace legged

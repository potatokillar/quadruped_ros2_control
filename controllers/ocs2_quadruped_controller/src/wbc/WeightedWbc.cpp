//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_quadruped_controller/wbc/WeightedWbc.h"

#include <qpOASES.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                 const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period)
    {
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        // Constraints
        Task constraints = formulateConstraints();
        size_t numConstraints = constraints.b_.size() + constraints.f_.size();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
        vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
        A << constraints.a_,
                constraints.d_;

        lbA << constraints.b_,
                -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
        ubA << constraints.b_,
                constraints.f_; // clang-format on

        // Cost
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
            weighedTask.a_.transpose() * weighedTask.a_;
        vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

        // Solve
        auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        options.enableEqualities = qpOASES::BT_TRUE;
        qpProblem.setOptions(options);
        int nWsr = 20;

        const auto initStatus = qpProblem.init(
            H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
        vector_t qpSol = vector_t::Zero(getNumDecisionVars());
        const auto solutionStatus = qpProblem.getPrimalSolution(qpSol.data());
        const bool hasFiniteSolution = solutionStatus == qpOASES::SUCCESSFUL_RETURN && qpSol.allFinite();

        scalar_t maxConstraintViolation = std::numeric_limits<scalar_t>::quiet_NaN();
        scalar_t maxTorque = std::numeric_limits<scalar_t>::quiet_NaN();
        if (hasFiniteSolution)
        {
            const vector_t constraintValues = A * qpSol;
            maxConstraintViolation = 0.0;
            for (Eigen::Index i = 0; i < constraintValues.size(); ++i)
            {
                if (lbA(i) > -0.5 * qpOASES::INFTY)
                {
                    maxConstraintViolation = std::max(maxConstraintViolation, lbA(i) - constraintValues(i));
                }
                if (ubA(i) < 0.5 * qpOASES::INFTY)
                {
                    maxConstraintViolation = std::max(maxConstraintViolation, constraintValues(i) - ubA(i));
                }
            }
            maxConstraintViolation = std::max<scalar_t>(0.0, maxConstraintViolation);
            maxTorque = qpSol.tail(info_.actuatedDofNum).cwiseAbs().maxCoeff();
        }

        diagnostics_.init_status = static_cast<int>(initStatus);
        diagnostics_.solution_status = static_cast<int>(solutionStatus);
        diagnostics_.n_wsr = nWsr;
        diagnostics_.mode = mode;
        diagnostics_.contacts = num_contacts_;
        diagnostics_.has_finite_solution = hasFiniteSolution;
        diagnostics_.max_constraint_violation = maxConstraintViolation;
        diagnostics_.max_torque = maxTorque;

        if (initStatus != qpOASES::SUCCESSFUL_RETURN || !hasFiniteSolution)
        {
            ++qp_warning_count_;
            if (qp_warning_count_ == 1 || qp_warning_count_ % 200 == 0)
            {
                std::cerr << "[WeightedWbc] qpOASES warning count=" << qp_warning_count_
                          << " init_status=" << static_cast<int>(initStatus)
                          << " solution_status=" << static_cast<int>(solutionStatus)
                          << " nWSR=" << nWsr
                          << " mode=" << mode
                          << " contacts=" << num_contacts_
                          << " max_constraint_violation=" << maxConstraintViolation
                          << " max_torque=" << maxTorque << '\n';
            }
        }

        if (!hasFiniteSolution)
        {
            if (last_valid_solution_.size() == qpSol.size())
            {
                return last_valid_solution_;
            }
            return qpSol;
        }

        last_valid_solution_ = qpSol;
        return qpSol;
    }

    Task WeightedWbc::formulateConstraints()
    {
        return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
            formulateNoContactMotionTask();
    }

    Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired,
                                             scalar_t period)
    {
        return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) *
            weightBaseAccel_ +
            formulateContactForceTask(inputDesired) * weightContactForce_;
    }

    void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose)
    {
        WbcBase::loadTasksSetting(taskFile, verbose);

        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        const std::string prefix = "weight.";
        if (verbose)
        {
            std::cerr << "\n #### WBC weight:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
        loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
        loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
    }
} // namespace legged

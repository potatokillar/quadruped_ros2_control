//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>

#include <array>
#include <cmath>
#include <string>

namespace ocs2::legged_robot
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        if (!node_->has_parameter("default_kp"))
        {
            node_->declare_parameter("default_kp", default_kp_);
        }
        if (!node_->has_parameter("default_kd"))
        {
            node_->declare_parameter("default_kd", default_kd_);
        }
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();

        // selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
        //                                                                        leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                             ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                             *ctrl_component_->ee_kinematics_);
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);

        const auto& model_info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
        matrix_t state_weight(model_info.stateDim, model_info.stateDim);
        loadData::loadEigenMatrix(ctrl_component_->task_file_, "Q", state_weight);
        vcom_x_weight_ = state_weight(0, 0);
        vcom_y_weight_ = state_weight(1, 1);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());

        measured_observation_publisher_ =
            node_->create_publisher<ocs2_msgs::msg::MpcObservation>("~/measured_observation", 10);
        desired_observation_publisher_ =
            node_->create_publisher<ocs2_msgs::msg::MpcObservation>("~/desired_observation", 10);
        wbc_diagnostics_publisher_ =
            node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("~/wbc_diagnostics", 10);
    }

    void StateOCS2::enter()
    {
        ctrl_component_->init();
    }

    void StateOCS2::run(const rclcpp::Time& time,
                        const rclcpp::Duration& period)
    {
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // Load the latest MPC policy
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);
        // 保存本周期 MRT 规划模式，供下一控制周期的状态估计使用。
        ctrl_component_->planned_mode_ = planned_mode;

        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();

        vector_t torque = x.tail(12);

        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());

        for (int i = 0; i < 12; i++)
        {
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        if (ctrl_component_->observation_.time - last_telemetry_time_ >= 0.02)
        {
            measured_observation_publisher_->publish(
                ros_msg_conversions::createObservationMsg(ctrl_component_->observation_));

            SystemObservation desired_observation;
            desired_observation.time = ctrl_component_->observation_.time;
            desired_observation.mode = planned_mode;
            desired_observation.state = optimized_state_;
            desired_observation.input = optimized_input_;
            desired_observation_publisher_->publish(
                ros_msg_conversions::createObservationMsg(desired_observation));

            const auto& wbc_diagnostics = wbc_->diagnostics();
            diagnostic_msgs::msg::DiagnosticArray diagnostics_message;
            diagnostics_message.header.stamp = time;
            diagnostics_message.status.resize(1);
            auto& status = diagnostics_message.status.front();
            const bool solver_ok = wbc_diagnostics.init_status == 0 &&
                                   wbc_diagnostics.solution_status == 0 &&
                                   wbc_diagnostics.has_finite_solution;
            status.level = solver_ok
                               ? diagnostic_msgs::msg::DiagnosticStatus::OK
                               : diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.name = "ocs2_quadruped_controller/weighted_wbc";
            status.message = solver_ok ? "solver_ok" : "solver_failed";
            status.hardware_id = "sd05";

            const auto append_value = [&status](const std::string& key, const auto value)
            {
                diagnostic_msgs::msg::KeyValue item;
                item.key = key;
                item.value = std::to_string(value);
                status.values.push_back(std::move(item));
            };
            append_value("init_status", wbc_diagnostics.init_status);
            append_value("solution_status", wbc_diagnostics.solution_status);
            append_value("n_wsr", wbc_diagnostics.n_wsr);
            append_value("mode", wbc_diagnostics.mode);
            append_value("contacts", wbc_diagnostics.contacts);
            append_value("has_finite_solution", wbc_diagnostics.has_finite_solution ? 1 : 0);
            append_value("max_constraint_violation", wbc_diagnostics.max_constraint_violation);
            append_value("max_torque", wbc_diagnostics.max_torque);

            const auto& policy = ctrl_component_->mpc_mrt_interface_->getPolicy();
            const auto& command = ctrl_component_->mpc_mrt_interface_->getCommand();
            const auto& performance = ctrl_component_->mpc_mrt_interface_->getPerformanceIndices();
            append_value("mpc_merit", performance.merit);
            append_value("mpc_total_cost", performance.cost);
            append_value("mpc_dynamics_violation_sse", performance.dynamicsViolationSSE);
            append_value("mpc_equality_constraints_sse", performance.equalityConstraintsSSE);
            append_value("mpc_inequality_constraints_sse", performance.inequalityConstraintsSSE);
            append_value("mpc_policy_nodes", policy.timeTrajectory_.size());

            if (policy.timeTrajectory_.size() == policy.stateTrajectory_.size() &&
                policy.timeTrajectory_.size() >= 2 &&
                policy.stateTrajectory_.front().size() >= 2 &&
                policy.stateTrajectory_.back().size() >= 2)
            {
                const scalar_t horizon = policy.timeTrajectory_.back() - policy.timeTrajectory_.front();
                append_value("mpc_horizon", horizon);
                append_value("mpc_predicted_vcom_x_first", policy.stateTrajectory_.front()(0));
                append_value("mpc_predicted_vcom_y_first", policy.stateTrajectory_.front()(1));
                append_value("mpc_predicted_vcom_x_last", policy.stateTrajectory_.back()(0));
                append_value("mpc_predicted_vcom_y_last", policy.stateTrajectory_.back()(1));

                const vector_t target_first = command.mpcTargetTrajectories_.getDesiredState(
                    policy.timeTrajectory_.front());
                const vector_t target_last = command.mpcTargetTrajectories_.getDesiredState(
                    policy.timeTrajectory_.back());
                if (policy.stateTrajectory_.front().size() >= 8 &&
                    policy.stateTrajectory_.back().size() >= 8 &&
                    target_first.size() >= 8 && target_last.size() >= 8)
                {
                    append_value("mpc_predicted_base_x_first", policy.stateTrajectory_.front()(6));
                    append_value("mpc_predicted_base_y_first", policy.stateTrajectory_.front()(7));
                    append_value("mpc_predicted_base_x_last", policy.stateTrajectory_.back()(6));
                    append_value("mpc_predicted_base_y_last", policy.stateTrajectory_.back()(7));
                    append_value("mpc_predicted_base_x_delta",
                                 policy.stateTrajectory_.back()(6) - policy.stateTrajectory_.front()(6));
                    append_value("mpc_predicted_base_y_delta",
                                 policy.stateTrajectory_.back()(7) - policy.stateTrajectory_.front()(7));
                    append_value("mpc_target_base_x_delta", target_last(6) - target_first(6));
                    append_value("mpc_target_base_y_delta", target_last(7) - target_first(7));
                    append_value("mpc_base_x_error_first", policy.stateTrajectory_.front()(6) - target_first(6));
                    append_value("mpc_base_y_error_first", policy.stateTrajectory_.front()(7) - target_first(7));
                    append_value("mpc_base_x_error_last", policy.stateTrajectory_.back()(6) - target_last(6));
                    append_value("mpc_base_y_error_last", policy.stateTrajectory_.back()(7) - target_last(7));
                }

                scalar_t predicted_x_integral = 0.0;
                scalar_t predicted_y_integral = 0.0;
                scalar_t predicted_x_squared_integral = 0.0;
                scalar_t predicted_y_squared_integral = 0.0;
                scalar_t target_x_squared_integral = 0.0;
                scalar_t target_y_squared_integral = 0.0;
                scalar_t error_x_squared_integral = 0.0;
                scalar_t error_y_squared_integral = 0.0;

                for (size_t index = 0; index + 1 < policy.timeTrajectory_.size(); ++index)
                {
                    const scalar_t dt = policy.timeTrajectory_[index + 1] - policy.timeTrajectory_[index];
                    const auto& state0 = policy.stateTrajectory_[index];
                    const auto& state1 = policy.stateTrajectory_[index + 1];
                    if (dt <= 0.0 || state0.size() < 2 || state1.size() < 2)
                    {
                        continue;
                    }

                    const vector_t target0 = command.mpcTargetTrajectories_.getDesiredState(
                        policy.timeTrajectory_[index]);
                    const vector_t target1 = command.mpcTargetTrajectories_.getDesiredState(
                        policy.timeTrajectory_[index + 1]);
                    if (target0.size() < 2 || target1.size() < 2)
                    {
                        continue;
                    }

                    const scalar_t error_x0 = state0(0) - target0(0);
                    const scalar_t error_x1 = state1(0) - target1(0);
                    const scalar_t error_y0 = state0(1) - target0(1);
                    const scalar_t error_y1 = state1(1) - target1(1);
                    predicted_x_integral += 0.5 * dt * (state0(0) + state1(0));
                    predicted_y_integral += 0.5 * dt * (state0(1) + state1(1));
                    predicted_x_squared_integral += 0.5 * dt * (std::pow(state0(0), 2) + std::pow(state1(0), 2));
                    predicted_y_squared_integral += 0.5 * dt * (std::pow(state0(1), 2) + std::pow(state1(1), 2));
                    target_x_squared_integral += 0.5 * dt * (std::pow(target0(0), 2) + std::pow(target1(0), 2));
                    target_y_squared_integral += 0.5 * dt * (std::pow(target0(1), 2) + std::pow(target1(1), 2));
                    error_x_squared_integral += 0.5 * dt * (std::pow(error_x0, 2) + std::pow(error_x1, 2));
                    error_y_squared_integral += 0.5 * dt * (std::pow(error_y0, 2) + std::pow(error_y1, 2));
                }

                if (horizon > 0.0)
                {
                    append_value("mpc_predicted_vcom_x_mean", predicted_x_integral / horizon);
                    append_value("mpc_predicted_vcom_y_mean", predicted_y_integral / horizon);
                    append_value("mpc_predicted_vcom_x_rms", std::sqrt(predicted_x_squared_integral / horizon));
                    append_value("mpc_predicted_vcom_y_rms", std::sqrt(predicted_y_squared_integral / horizon));
                    append_value("mpc_target_vcom_x_rms", std::sqrt(target_x_squared_integral / horizon));
                    append_value("mpc_target_vcom_y_rms", std::sqrt(target_y_squared_integral / horizon));
                    append_value("mpc_vcom_x_error_rms", std::sqrt(error_x_squared_integral / horizon));
                    append_value("mpc_vcom_y_error_rms", std::sqrt(error_y_squared_integral / horizon));
                    append_value("mpc_vcom_x_tracking_cost", 0.5 * vcom_x_weight_ * error_x_squared_integral);
                    append_value("mpc_vcom_y_tracking_cost", 0.5 * vcom_y_weight_ * error_y_squared_integral);
                }
            }

            const auto& info = ctrl_component_->legged_interface_->getCentroidalModelInfo();
            const auto& contact_names = ctrl_component_->legged_interface_->modelSettings().contactNames3DoF;
            const Eigen::Index contact_force_offset = info.generalizedCoordinatesNum;
            const Eigen::Index contact_force_size = 3 * info.numThreeDofContacts;
            if (x.size() >= contact_force_offset + contact_force_size &&
                optimized_input_.size() >= contact_force_size &&
                contact_names.size() == info.numThreeDofContacts)
            {
                static constexpr std::array<const char*, 3> axes{"x", "y", "z"};
                for (size_t contact = 0; contact < info.numThreeDofContacts; ++contact)
                {
                    for (size_t axis = 0; axis < axes.size(); ++axis)
                    {
                        const Eigen::Index index = static_cast<Eigen::Index>(3 * contact + axis);
                        append_value("wbc_contact_force_" + contact_names[contact] + "_" + axes[axis],
                                     x(contact_force_offset + index));
                        append_value("mpc_contact_force_" + contact_names[contact] + "_" + axes[axis],
                                     optimized_input_(index));
                    }
                }
            }

            if (x.size() >= 6)
            {
                static constexpr std::array<const char*, 6> acceleration_names{
                    "linear_x", "linear_y", "linear_z", "zyx_z", "zyx_y", "zyx_x"
                };
                for (size_t index = 0; index < acceleration_names.size(); ++index)
                {
                    append_value("wbc_base_acceleration_" + std::string(acceleration_names[index]), x(index));
                }
            }
            wbc_diagnostics_publisher_->publish(std::move(diagnostics_message));

            last_telemetry_time_ = ctrl_component_->observation_.time;
        }
    }

    void StateOCS2::exit()
    {
        // 离开 OCS2 后按四足支撑处理，避免残留上一时刻的 trot 接触模式。
        ctrl_component_->planned_mode_ = STANCE;
    }

    FSMStateName StateOCS2::checkChange()
    {
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::PASSIVE;
        default:
            return FSMStateName::OCS2;
        }
    }
}

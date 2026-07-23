//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <SafetyChecker.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/duration.hpp>

#include "controller_common/FSM/FSMState.h"

namespace ocs2
{
    class MPC_MRT_Interface;
    class MPC_BASE;
}

namespace ocs2::legged_robot
{
    class StateOCS2 final : public FSMState
    {
    public:
        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<CtrlComponent>& ctrl_component
        );

        void enter() override;

        void run(const rclcpp::Time& time,
                 const rclcpp::Duration& period) override;

        void exit() override;

        FSMStateName checkChange() override;

    private:

        std::shared_ptr<CtrlComponent> ctrl_component_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        // Whole Body Control
        std::shared_ptr<WeightedWbc> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;
        benchmark::RepeatedTimer wbc_timer_;

        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr measured_observation_publisher_;
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr desired_observation_publisher_;
        rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr wbc_diagnostics_publisher_;
        scalar_t last_telemetry_time_{-1.0};
        scalar_t vcom_x_weight_{0.0};
        scalar_t vcom_y_weight_{0.0};

        double default_kp_ = 0;
        double default_kd_ = 6;

        vector_t optimized_state_, optimized_input_;
    };
}


#endif //STATEOCS2_H

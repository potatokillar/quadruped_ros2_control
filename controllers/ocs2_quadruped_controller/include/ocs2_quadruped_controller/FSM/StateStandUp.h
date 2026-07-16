#pragma once

#include <controller_common/FSM/BaseFixedStand.h>

namespace ocs2::legged_robot
{
    class StateStandUp final : public BaseFixedStand
    {
    public:
        StateStandUp(CtrlInterfaces& ctrlInterfaces, const std::vector<double>& targetPosition,
                     double kp, double kd);

        FSMStateName checkChange() override;
    };
} // namespace ocs2::legged_robot

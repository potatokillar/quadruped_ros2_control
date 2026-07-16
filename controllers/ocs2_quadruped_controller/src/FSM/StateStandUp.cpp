#include "ocs2_quadruped_controller/FSM/StateStandUp.h"

namespace ocs2::legged_robot
{
    StateStandUp::StateStandUp(CtrlInterfaces& ctrlInterfaces, const std::vector<double>& targetPosition,
                               const double kp, const double kd)
        : BaseFixedStand(ctrlInterfaces, targetPosition, kp, kd)
    {
    }

    FSMStateName StateStandUp::checkChange()
    {
        if (ctrl_interfaces_.control_inputs_.command == 1)
        {
            return FSMStateName::PASSIVE;
        }
        if (percent_ < 1.5)
        {
            return FSMStateName::FIXEDSTAND;
        }
        return FSMStateName::OCS2;
    }
} // namespace ocs2::legged_robot

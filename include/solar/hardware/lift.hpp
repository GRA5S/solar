#pragma once
#include "pros/motor_group.hpp"
#include "solar/auto/pid.hpp"
namespace solar {
    class Lift{
        public:
            Lift(
                pros::MotorGroup motors,
                ControllerSettings lift_controller
            );
            ControllerSettings lift_controller;
            pros::MotorGroup motors;
            
    };
}

//very wip i focus on motions for now
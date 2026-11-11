#pragma once
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include <cstdint>
namespace solar {
    class Drivetrain{
        public:
            Drivetrain(
                pros::MotorGroup* left_ports,
                pros::MotorGroup* right_ports,
                double wheel_diameter
            );
            void move_raw(int left, int right) const; // lowk kinda useless cuz the other ones dont even have drive curves atm so this is jsut the same thing as tank
            void stop() const;
            void arcade(std::int32_t throttle, std::int32_t turn) const;
            void tank(int left, int right) const;
            void set_brake_mode(pros::motor_brake_mode_e_t mode) const;
        private:
            // hardware
			pros::MotorGroup* _left_motors;
			pros::MotorGroup* _right_motors;
			double _wheel_diameter;

    };
}
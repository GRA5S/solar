#include "drivetrain.hpp"
#include "pros/motor_group.hpp"
namespace solar {
    Drivetrain::Drivetrain(
        pros::MotorGroup* left_ports,
        pros::MotorGroup* right_ports,
        double wheel_diameter
    ):
    _left_motors(left_ports),
    _right_motors(right_ports),
    _wheel_diameter(wheel_diameter)
    {

    };
    void Drivetrain::arcade(std::int32_t throttle, std::int32_t turn) const {
		_left_motors->move(throttle - turn);
		_right_motors->move(throttle + turn);
	}
	void Drivetrain::move_raw(int left, int right) const { // lowk kinda useless cuz the other ones dont even have drive curves atm so this is jsut the same thing as tank but ima implement uit ajnywhay for the future
		_left_motors->move(left);
        _right_motors->move(right);
	}
	void Drivetrain::tank(int left, int right) const {
		_left_motors->move(left);
        _right_motors->move(right);
	}
	void Drivetrain::stop() const {
		_left_motors->brake();
		_right_motors->brake();
	}
	void Drivetrain::set_brake_mode(pros::motor_brake_mode_e_t mode) const {
		_left_motors->set_brake_mode_all(mode);
		_right_motors->set_brake_mode_all(mode);
	}
}
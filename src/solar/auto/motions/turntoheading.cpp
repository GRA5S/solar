#include "solar/auto/exitconditions.hpp"
#include "pros/rtos.hpp"
#include "solar/chassis.hpp"
#include "solar/util.hpp"
#include <algorithm>
#include <cstdint>
namespace solar {
    void Chassis::turnToHeading(double theta, int timeout) {
        _newMotion([=, this] {
            const uint32_t start = pros::millis();
            const double inital_theta = get_pose().theta;

            ExitCondition large_exit(
                _angular_controller.large_error,
                _angular_controller.large_error_timeout
            );
            ExitCondition small_exit(
                _angular_controller.small_error,
                _angular_controller.small_error_timeout
            );
            double prev_power = 0;
            _angular_pid.reset();
            while (_motion_running) {
                Pose pose = get_pose();
                double error = normalizeAngle(theta - pose.theta);

                if (timeout > 0 && pros::millis() - start >= timeout) break;
                if (large_exit.update(error) || small_exit.update(error)) break;
                
                _motion_progress = std::fabs(normalizeAngle(pose.theta - inital_theta));

                double power = _angular_pid.update(error);
                if (_angular_controller.slew && std::fabs(error) > 20) { // magic number cuz im just doin what the lemlib do but i shoudl fic that later
                    power = slew(power, prev_power, _angular_controller.slew.value());
                };
                power = std::clamp(power, -127.0, 127.0);
                _drivetrain.move_raw(power, -power);
                prev_power = power;
                pros::delay(10);

            }
        });
    }
}
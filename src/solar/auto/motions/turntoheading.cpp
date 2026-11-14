#include "solar/auto/exitconditions.hpp"
#include "pros/rtos.hpp"
#include "solar/auto/pid.hpp"
#include "solar/chassis.hpp"
#include "solar/util.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include "solar/auto/motion.hpp"
namespace solar {
    void Chassis::turnToHeading(double theta, int timeout, TurnToHeadingParams params) {
        assert(!params.minSpeed || 0 < *params.minSpeed && *params.minSpeed <= 127);
        assert(!params.maxSpeed || 0 < *params.maxSpeed && *params.maxSpeed <= 127);
        assert(!params.earlyExitRange || 0 < *params.earlyExitRange);
        assert(!(params.minSpeed && params.maxSpeed) || *params.minSpeed < *params.maxSpeed);

        _newMotion([=, this] {
            const uint32_t start = pros::millis();
            const double initial_theta = get_pose().theta;

            ControllerSettings controller = params.controller ? *params.controller : _angular_controller;
            PID pid(controller);
            ExitCondition large_exit(
                controller.large_error,
                controller.large_error_timeout
            );
            ExitCondition small_exit(
                controller.small_error,
                controller.small_error_timeout
            );
            
            bool settling = false;
            std::optional<double> prev_raw;
            std::optional<double> prev_error;
            double prev_power = 0;

            while (_motion_running) {
                // update variables
                Pose pose = get_pose();
                _motion_progress = std::fabs(pose.theta - initial_theta);

                const double raw_error = angleError(theta, pose.theta);
                if (!prev_raw) prev_raw = raw_error; // this just protects it from reading the variable in the next line before it is written on the first run of the loop
                if (sign(raw_error) != sign(prev_raw.value())) settling = true; // settling is true if it overshoots atleast once
                prev_raw = raw_error;

                const double error = settling ? angleError(theta, pose.theta) : angleError(theta, pose.theta, params.direction);
                if (!prev_error) prev_error = error;

                // check exit conditions
                if (timeout > 0 && pros::millis() - start >= timeout) break;
                if (large_exit.update(error) || small_exit.update(error)) break;
                if (params.earlyExitRange.has_value() && std::fabs(error) <= params.earlyExitRange.value()) break;
                if (params.motionChaining && sign(error) != sign(prev_error.value_or(error))) break;

                // calculate power
                double power = pid.update(error);
                if (controller.slew && std::fabs(error) > 20) { // magic number cuz im just doin what the lemlib do but i shoudl fic that later
                    power = slew(power, prev_power, controller.slew.value());
                }
                power = std::clamp(power, -params.maxSpeed.value_or(127), params.maxSpeed.value_or(127));
                power = floorMagnitude(power, params.minSpeed.value_or(0));

                //run dt
                _drivetrain.move_raw(power, -power);
                prev_power = power;

                //delay
                pros::delay(10);
            }
        });
    }
}
#include "solar/auto/exitconditions.hpp"
#include "pros/rtos.hpp"
#include "solar/auto/pid.hpp"
#include "solar/chassis.hpp"
#include "solar/util.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <variant>
#include "solar/auto/motion.hpp"
namespace solar {
    void Chassis::turn(std::variant<double, Point> target, int timeout, TurnParams params) {
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

                double target_theta;
                if (std::holds_alternative<double>(target)) {
                    target_theta = std::get<double>(target);
                } else {
                    const Point& p = std::get<Point>(target);
                    const double dx = p.x - pose.x, dy = p.y - pose.y;
                    target_theta = radToDeg(std::atan2(dx, dy));
                }   

                const double raw_error = angleError(target_theta, pose.theta);
                if (!prev_raw) prev_raw = raw_error; // this just protects it from reading the variable in the next line before it is written on the first run of the loop
                if (sign(raw_error) != sign(prev_raw.value())) settling = true; // settling is true if it overshoots atleast once
                prev_raw = raw_error;

                const double error = settling ? angleError(target_theta, pose.theta) : angleError(target_theta, pose.theta, params.direction);
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
                if (params.lockedSide == LEFT){
                    _drivetrain.move_raw(0, -power * params.rightMultiplier);
                    _drivetrain.stop(LEFT);
                }
                else if (params.lockedSide == RIGHT){
                    _drivetrain.move_raw(power * params.leftMultiplier, 0);
                    _drivetrain.stop(RIGHT);
                }
                else _drivetrain.move_raw(power * params.leftMultiplier, -power * params.rightMultiplier);
                prev_power = power;

                //delay
                pros::delay(10);
            }
        });
    }
    void Chassis::turnToPoint(double x, double y, int timeout, TurnParams params) {
        turn(Point(x,y), timeout, params);
    }
    void Chassis::turnToHeading(double theta, int timeout, TurnParams params){
        turn(theta, timeout, params);
    }
    void Chassis::swingToHeading(double theta, DriveSide lockedside, int timeout, TurnParams params){
        params.lockedSide = lockedside;
        turn(theta, timeout, params);
    }
    void Chassis::swingToPoint(double x, double y, DriveSide lockedside, int timeout, TurnParams params){
        params.lockedSide = lockedside;
        turn(Point(x,y), timeout, params);
    }
}


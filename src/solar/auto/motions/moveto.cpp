#include "pros/rtos.hpp"
#include "solar/auto/exitconditions.hpp"
#include "solar/auto/motion.hpp"
#include "solar/chassis.hpp"
#include "solar/util.hpp"
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <optional>
#include <variant>

namespace solar {
    void Chassis::moveTo(std::variant<Point, Pose> target, int timeout, MoveParams params) {
        assert(!params.minSpeed || 0 < *params.minSpeed && *params.minSpeed <= 127);
        assert(!params.maxSpeed || 0 < *params.maxSpeed && *params.maxSpeed <= 127);
        assert(!params.earlyExitRange || 0 < *params.earlyExitRange);
        assert(!(params.minSpeed && params.maxSpeed) || *params.minSpeed < *params.maxSpeed);
        

        _newMotion([=, this] {
            const uint32_t start = pros::millis();
            const Pose inital_pose = get_pose(); 

            ControllerSettings linear_controller = params.linearController ? *params.linearController : _linear_controller;
            ControllerSettings angular_controller = params.angularController ? *params.angularController : _angular_controller;

            PID linear_pid(linear_controller);
            PID angular_pid(angular_controller);

            ExitCondition angular_large_exit(
                angular_controller.large_error,
                angular_controller.large_error_timeout
            );
            ExitCondition angular_small_exit(
                angular_controller.small_error,
                angular_controller.small_error_timeout
            );

            ExitCondition linear_large_exit(
                linear_controller.large_error,
                linear_controller.large_error_timeout
            );
            ExitCondition linear_small_exit(
                linear_controller.small_error,
                linear_controller.small_error_timeout
            );

            bool close = false;
            Pose prev_pose = get_pose();
            double prev_linear_out = 0;
            double prev_angular_out = 0;
            double max_speed = params.maxSpeed.value_or(127);
            bool boomerang = std::holds_alternative<Pose>(target);
            Pose target_pose; Point target_point;
            if (boomerang) target_pose = std::get<Pose>(target);
            else target_point = std::get<Point>(target);

            while (_motion_running) {
                Pose pose = get_pose();
                
                

                if (_motion_progress) *_motion_progress += distance(pose, prev_pose);
                prev_pose = pose;

                const double angularError = angleError(
                    angleto(pose, target_point) + (params.forwards ? 0 : 180), 
                    pose.theta
                );
                const double linearError = distance(pose, target_point) * cos(degToRad(angleError(pose.theta, angleto(pose, target_point))));

                if (distance(pose, target_point) < 7.5 && close == false) {
                    close = true;
                    max_speed = fmax(fabs(prev_linear_out), 60); // magic numbers!!! i love magic numbers  that i steal from lemlib!!!!!!
                    // basically says do not speed up anymore but also dont go below 60 ples
                    // also ts lowk just stolen from lemlib
                }

                if (close && (linear_large_exit.update(linearError) || linear_small_exit.update(linearError))) break;
                if (timeout > 0 && pros::millis() - start >= timeout) break;

                double linear_out = linear_pid.update(linearError);
                double angular_out = angular_pid.update(angularError);

                
                if (close) angular_out = 0;

                angular_out = std::clamp(angular_out, -max_speed, max_speed);
                if (angular_controller.slew.has_value()) {
                    angular_out = slew(angular_out, prev_angular_out, angular_controller.slew.value());
                }
                linear_out = std::clamp(linear_out, -max_speed, max_speed);
                if (!close && linear_controller.slew.has_value()) {
                    linear_out = slew(linear_out, prev_linear_out, linear_controller.slew.value());
                }
                if (!close) linear_out = params.forwards ? std::fmax(linear_out, 0) : std::fmin(linear_out, 0);
                const double min_speed = params.minSpeed.value_or(0);
                if (params.forwards && linear_out > 0 && linear_out < min_speed) linear_out = min_speed;
                else if (!params.forwards && linear_out < 0 && linear_out > -min_speed) linear_out = -min_speed;
                
                

                prev_linear_out = linear_out;
                prev_angular_out = angular_out;
                
                float left_voltage = linear_out + angular_out;
                float right_voltage = linear_out - angular_out;


                const float ratio = std::max(std::fabs(left_voltage), std::fabs(right_voltage)) / max_speed;
                if (ratio > 1) {
                    left_voltage /= ratio;
                    right_voltage /= ratio;
                }
                
                _drivetrain.move_raw(left_voltage, right_voltage);

                pros::delay(10);


            }

        });
    }
    void Chassis::moveToPoint(double x, double y, int timeout, MoveParams params) {
        moveTo(Point(x,y), timeout, params);
    }
    void Chassis::moveToPose(double x, double y, double theta, int timeout, MoveParams params){
        moveTo(Pose(x,y,theta), timeout, params);
    }
    void Chassis::moveToPoint(Point point, int timeout, MoveParams params) {
        moveTo(point, timeout, params);
    }
    void Chassis::moveToPose(Pose pose, int timeout, MoveParams params){
        moveTo(pose, timeout, params);
    }
}


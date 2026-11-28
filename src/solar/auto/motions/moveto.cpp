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
        assert(!params.lateralController || !std::holds_alternative<Pose>(target));
        

        _newMotion([=, this] {
            const uint32_t start = pros::millis();
            const Pose inital_pose = get_pose(); 

            ControllerSettings linear_controller = params.linearController ? *params.linearController : _linear_controller;
            ControllerSettings angular_controller = params.angularController ? *params.angularController : _angular_controller;
            const bool cte = params.lateralController.has_value();
            ControllerSettings lateral_settings = cte ? *params.lateralController : ControllerSettings{};

            PID linear_pid(linear_controller);
            PID angular_pid(angular_controller);
            PID lateral_pid(lateral_settings);

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

            const double approach = angleto(inital_pose, target_point);
            const double travel_heading = approach + (params.forwards ? 0 : 180);
            const Point goal {
                target_point.x - params.goal_offset * std::sin(degToRad(travel_heading)),
                target_point.y - params.goal_offset * std::cos(degToRad(travel_heading))
            };
            const double plane_heading = approach;
            std::optional<bool> first_side = std::nullopt;

            while (_motion_running) {

                // update variables

                Pose pose = get_pose();
                if (_motion_progress) *_motion_progress += distance(pose, prev_pose);
                prev_pose = pose;
                // motion chaining stuff, if it has gone past the goal at all then exit
                const double fwd = (pose.x - goal.x) * std::sin(degToRad(plane_heading))
                                 + (pose.y - goal.y) * std::cos(degToRad(plane_heading));
                const bool side = fwd >= 0;
                if (!first_side) first_side = side;
                else if (params.motionChaining && side != *first_side) break; 
                const double d = distance(pose, goal);
                

                // cte stuff

                double angularError;
                double linearError;
                double projection_angle = 0;
                bool reverse = false;

                if (cte) {
                    const double bearing = angleto(pose, goal);
                    const double cte_angle_error = normalizeAngle(bearing - pose.theta);
                    reverse = std::fabs(cte_angle_error) > 90.0;
                    projection_angle = cte_angle_error;
                    angularError = (reverse ? -1.0 : 1.0) * d * std::sin(degToRad(cte_angle_error));
                    linearError = reverse ? -d : d;
                } else {
                    angularError = angleError(
                        angleto(pose, goal) + (params.forwards ? 0 : 180), 
                        pose.theta
                    );
                    linearError = d * cos(degToRad(angleError(pose.theta, angleto(pose, goal))));
                }

                if (d < 4 && close == false) {
                    close = true;
                    max_speed = fmax(fabs(prev_linear_out), 60); // magic numbers!!! i love magic numbers  that i steal from lemlib!!!!!!
                    // basically says do not speed up anymore but also dont go below 60 ples
                    // also ts lowk just stolen from lemlib
                }

                if (close && (linear_large_exit.update(cte ? d : linearError) || linear_small_exit.update(cte ? d : linearError))) break;
                if (params.earlyExitRange && d <= *params.earlyExitRange) break;
                if (timeout > 0 && pros::millis() - start >= timeout) break;


                // calc linear and angular out

                double linear_out;
                double angular_out;
                // if cte use lateral pid for angular output
                // otherwise jsut use the angular pid
                if (cte) {
                    linear_out = linear_pid.update(linearError) * std::fabs(std::cos(degToRad(projection_angle)));
                    angular_out = lateral_pid.update(angularError);
                } else {
                    linear_out = linear_pid.update(linearError);
                    angular_out = angular_pid.update(angularError);
                }
                // set angular out to zero to avoid huge changes of angular error and therefore angular output when really clsoe to target
                if (!cte && close) angular_out = 0;


                // constrain linear and angular out

                const ControllerSettings& steering_controller = cte ? lateral_settings : angular_controller;
                // apply max speed and slew
                angular_out = std::clamp(angular_out, -max_speed, max_speed);
                if (steering_controller.slew.has_value()) {
                    angular_out = slew(angular_out, prev_angular_out, steering_controller.slew.value());
                }
                linear_out = std::clamp(linear_out, -max_speed, max_speed);
                if (!close && linear_controller.slew.has_value()) {
                    linear_out = slew(linear_out, prev_linear_out, linear_controller.slew.value());
                }
                // it should never go in the wrong direction unless its oscillating at the end
                if (!close) {
                    if (cte) linear_out = reverse ? std::fmin(linear_out, 0) : std::fmax(linear_out, 0);
                    else linear_out = params.forwards ? std::fmax(linear_out, 0) : std::fmin(linear_out, 0);
                }
                // apply min speed
                const double min_speed = params.minSpeed.value_or(0);
                // if using cte then the forwards var doesnt matter. otherwise, apply min speed based on whether the motion is supposed to be forward or not
                if (cte) linear_out = floorMagnitude(linear_out, min_speed);
                else if (params.forwards && linear_out > 0 && linear_out < min_speed) linear_out = min_speed;
                else if (!params.forwards && linear_out < 0 && linear_out > -min_speed) linear_out = -min_speed;
                // apply new outputs to the prev vars
                prev_linear_out = linear_out;
                prev_angular_out = angular_out;

                
                // apply voltages

                float left_voltage = linear_out + angular_out;
                float right_voltage = linear_out - angular_out;
                // make sure its not trying to go above max voltage yk
                const float ratio = std::max(std::fabs(left_voltage), std::fabs(right_voltage)) / max_speed;
                if (ratio > 1) {
                    left_voltage /= ratio;
                    right_voltage /= ratio;
                }
                // move the drive
                _drivetrain.move_raw(left_voltage, right_voltage);
                // delay
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


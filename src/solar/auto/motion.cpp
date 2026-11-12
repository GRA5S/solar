#include "pros/rtos.hpp"
#include "solar/chassis.hpp"
#include <functional>
#include <mutex>
#include <optional>
namespace solar {
    void Chassis::waitUntil(double dist) {
        while (_motion_progress && *_motion_progress <= dist) {
            pros::delay(10);
        }
    }
    void Chassis::waitUntilDone() { 
        while (isInMotion()) {
            pros::delay(10); 
        }
    }
    void Chassis::cancelMotion() {
        std::lock_guard<pros::Mutex> lock(_motion_mutex);
        _motion_running = false;
    }
    void Chassis::cancelAllMotions() {
        std::lock_guard<pros::Mutex> lock(_motion_mutex);
        _motion_queue = {};
        _motion_running = false;
    }
    void Chassis::_newMotion(std::function<void()> motion) {
        std::lock_guard<pros::Mutex> lock(_motion_mutex);
        _motion_queue.push(motion);
    }
    void Chassis::_motionLoop() {
        while (true) {
            std::optional<std::function<void()>> motion;
            {
                std::lock_guard<pros::Mutex> lock(_motion_mutex); // lock mutex until out of scope and destructor called
                if (!_motion_queue.empty()) {
                    motion = _motion_queue.front();
                    _motion_queue.pop();
                    _motion_running = true;
                }
            }
            if (motion) {
                _motion_progress = 0.0;
                (*motion)(); // remember motion is an optional function so u have to make it a pointer to get the value
                _drivetrain.stop();
                _motion_progress = std::nullopt;
                
                std::lock_guard<pros::Mutex> lock(_motion_mutex); // same lock guard thing as before but this time its for the motion running variable
                _motion_running = false;
            }
            pros::delay(10);
        }
    }
    bool Chassis::isInMotion() const {
        std::lock_guard<pros::Mutex> lock(_motion_mutex);
        return _motion_running || !_motion_queue.empty();
    }

}
#include "solar/auto/pid.hpp"
#include "pros/rtos.hpp"
#include "solar/util.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <sys/types.h>
#include <cmath>
namespace solar {
    PID::PID(ControllerSettings& settings) : _settings(settings) {}
    // integral range is range of error where integral applies
    // integral max is the max that the _integral can get to
    // there is both reset and clear sum error because sometimes u js want to get rid of the sum error if I be doing too much but you arent resetting everything yk
    void PID::clearSumError() {_sum_error = 0;}
    void PID::reset() {_sum_error = 0; _previous_error = 0; _previous_time = std::nullopt;}
    void PID::setTarget(double target) {_target = target; reset();}
    double PID::getI() {return _integral;}
    double PID::getOutput() {return _output;}
    double PID::update(double input){
        if (!_target) return 0; 
        // time stuff
        const uint32_t now = pros::millis();
        const uint32_t dt = (_previous_time == std::nullopt) ? 0 : (now - *_previous_time); // this is a ternary operator, if previous time is null then set to 0. if u made it an if else then dt would fall out of scope
        _previous_time = now;
        // error stuff
        _error = _target.value() - input; // .value becuase optional
        _sum_error += _error * dt / 1000; // idk if it has to be in s instead of ms thats just what lemlib does and i want the pid values to be interchangeable betwwen     
        if (_settings.integral_max){
            _sum_error = std::clamp(
                _sum_error, 
                -_settings.integral_max.value(), 
                _settings.integral_max.value()
            );
        }
        const double derror = _error - _previous_error;
        // if sign flips and flag is true then reset sum error
        if (
            sign(_error) != 
            sign(_previous_error) && 
            _settings.reset_integral_on_sign_flip
        ) { clearSumError(); }
        // if outside of integral range then set sum error to zero
        if (
            _settings.integral_range &&
            fabs(_error) > _settings.integral_range.value() // float absolute value
        ) { clearSumError(); } 
        // pid stuff
        _proportional = _settings.kp ? _settings.kp.value() * _error : 0.0;
        _integral = _settings.ki ? _settings.ki.value() * _sum_error : 0.0;
        _derivative = _settings.kd ? _settings.kd.value() * ((dt != 0) ? (derror/(dt/1000.0)) : 0) : 0.0;
        _output = _proportional + _integral + _derivative;
        _previous_error = _error;
        return _output;
    }
}
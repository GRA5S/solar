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
    PID::PID(const ControllerSettings& settings) : _settings(settings) {}
    // integral range is range of error where integral applies
    // integral max is the max that the _integral can get to
    // there is both reset and clear sum error because sometimes u js want to get rid of the sum error if I be doing too much but you arent resetting everything yk
    void PID::clearSumError() {_sum_error = 0;}
    void PID::reset() {_sum_error = 0; _previous_error = 0;}
    double PID::getI() {return _integral;}
    double PID::getOutput() {return _output;}
    double PID::update(double error){
        // error stuff
        _sum_error += error;
        if (_settings.integral_max){
            _sum_error = std::clamp(
                _sum_error, 
                -_settings.integral_max.value(), 
                _settings.integral_max.value()
            );
        }
        const double derror = error - _previous_error;
        // if sign flips and flag is true then reset sum error
        if (
            sign(error) != 
            sign(_previous_error) && 
            _settings.reset_integral_on_sign_flip
        ) { clearSumError(); }
        // if outside of integral range then set sum error to zero
        if (
            _settings.integral_range &&
            fabs(error) > _settings.integral_range.value() // float absolute value
        ) { clearSumError(); } 
        // pid stuff
        _proportional = _settings.kp ? _settings.kp.value() * error : 0.0;
        _integral = _settings.ki ? _settings.ki.value() * _sum_error : 0.0;
        _derivative = _settings.kd ? _settings.kd.value() * derror : 0.0;
        _output = _proportional + _integral + _derivative;
        _previous_error = error;
        return _output;
    }
}
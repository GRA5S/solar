#include "solar/motions/pid.hpp"
#include "pros/rtos.hpp"
#include "solar/util.hpp"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <sys/types.h>
#include <cmath>
namespace solar {
    PID::PID(
        double kp,
        double ki,
        double kd,
        double target
    ):
        _kp(kp),
        _ki(ki),
        _kd(kd),
        _target(target) 
    {}
    std::array<double, 3> PID::getCoefficients() {return {_kp, _ki, _kd};}
    void PID::setCoefficients(double kp, double ki, double kd) {_kp = kp; _ki = ki; _kd = kd;}
    void PID::setkp(double kp) {_kp = kp;}
    void PID::setki(double ki) {_ki = ki;}
    void PID::setkd(double kd) {_kd = kd;}
    double PID::getkp() {return _kp;}
    double PID::getki() {return _ki;}
    double PID::getkd() {return _kd;}
    void PID::setIntegralMax(double integral_max) {_integral_max = integral_max;}
    // there is both reset and clear sum error because sometimes u js want to get rid of the sum error if I be doing too much but you arent resetting everything yk
    void PID::clearSumError() {_sum_error = 0;}
    void PID::reset() {_sum_error = 0; _previous_error = 0; _previous_time = std::nullopt;}
    void PID::setTarget(double target) {_target = target; reset();}
    double PID::getI() {return _integral;}
    double PID::getOutput() {return _output;}
    double PID::update(double input){
        // time stuff
        const uint32_t now = pros::millis();
        const uint32_t dt = (_previous_time == std::nullopt) ? 0 : (now - *_previous_time); // this is a ternary operator, if previous time is null then set to 0. if u made it an if else then dt would fall out of scope
        _previous_time = now;
        // error stuff
        _error = _target - input;
        _sum_error += _error * dt / 1000; // idk if it has to be in s instead of ms thats just what lemlib does and i want the pid values to be interchangeable betwwen     
        _sum_error = std::clamp(_sum_error, -_integral_max, _integral_max);
        const double derror = _error - _previous_error;
        // if sign flips and flag is true then reset sum error
        if (
            sign(_error) != 
            sign(_previous_error) && 
            _sign_flip_integral_reset
        ) { clearSumError(); }
        // if outside of integral range then set sum error to zero
        if (
            fabs(_error) > // float absolute value
            _integral_range &&
            _integral_range != 0
        ) { clearSumError(); } 
        // pid stuff
        _proportional = _kp * _error;
        _integral = _ki * _sum_error;
        _derivative = _kd * ((dt != 0) ? (derror/(dt/1000.0)) : 0);
        _output = _proportional + _integral + _derivative;
        _previous_error = _error;
        return _output;
    }
}

/*

Number PID::update(Number error) {
    // find time delta
    const Time now = from_msec(pros::millis());
    // if this is the first iteration, previousTime won't be set
    // if it is not set, then assume dt is 0
    Time dt = (m_previousTime == std::nullopt) ? 0_msec : now - *m_previousTime;
    m_previousTime = now;

    // calculate the derivative (change in error / time passed)
    const Number derivative = (dt != 0_sec) ? (error - m_previousError) / to_sec(dt) : 0;

    // calculate the integral (change in error * time passed)
    m_integral += error * to_sec(dt);
    // sign flip reset. If the sign of error changes, set the integral to 0
    if (sgn(error) != sgn((m_previousError)) && m_signFlipReset) m_integral = 0;
    // anti windup range. Unless error is small enough, set the integral to 0
    if (abs(error) > m_windupRange && m_windupRange != 0) m_integral = 0;

    m_previousError = error;

    // output. error * kP + integral * kP + derivative * kD
    return error * m_gains.kP + m_integral * m_gains.kI + derivative * m_gains.kD;
}

double calculatePidValues(double input) {
    this->error = target - input;

    if (turnPid) {
        this->error = angleDifference(target, input).getValue();
    }

    this->derivitive = -(input - this->prevInput);

    if (abs(error) < integralBound) {
        totalError += error;
    } else {
        totalError = 0;
    }

    totalError = abs(totalError) > maxIntegral ? copysign(1.0, totalError) * maxIntegral : totalError;

    this->power = (error * kP) + (derivitive * kD) + (totalError * kI);

    this->prevInput = input;

    return this->power;
}

*/
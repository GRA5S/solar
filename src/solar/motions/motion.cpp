#include "solar/motions/motion.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
namespace solar {
    ExitCondition::ExitCondition(
        double range, 
        uint32_t timeout
    ):
        _range(range),
        _timeout(timeout) {}
    bool ExitCondition::update(double error){
        if (_arrived) return true;
        const uint32_t now = pros::millis();
        if (!_start_time) _start_time = now;

        if (std::fabs(error) >= _range) _start_time.reset(); // remember start time is when it started being within the error range. also u cant just set it to zero cuz optional. 
        else if (now >= (_start_time.value() + _timeout)) _arrived = true; // start time is an optional so u have to use .value() 

        return _arrived;
    }
    void ExitCondition::reset(){
        _arrived = false;
        _start_time.reset();
    }
}
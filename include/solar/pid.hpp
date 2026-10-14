#pragma once
#include <array>
#include <cstdint>
#include <optional>
namespace solar {
    class PID {
        public:
            PID(double kp, double ki, double kd);
            std::array<double, 3> getCoefficients();
            void setCoefficients(double kp, double ki, double kd);
            void setTarget(double target);
            /*
            integral stuff
            max is the max that integral can accumulate to
            range is the p range where integral applies
            so if the proportional term is not within ±(integral range) then integral term does not apply
            */
            void setIntegralMax(double integral_max);
            void setIntegralRange(double integral_range);
            /*
            tolerences
            will only settle if error is within that tolerences and has been for the duration
            and if derivitive is small enough
            */
            void setErrorTolerance(double small_error_tolerance, double big_error_tolerance);
            void setErrorDuration(double small_error_duration, double big_error_duration);
            void setDerivativeTolerance(double derivative_tolerance);
            
            void clearSumError(); // sum error for integral
            void reset();

            void setArrive(bool arrive); // set whether it has arrived
            bool targetArrived(); // get whether it has arrived
            double getI();
            double update(double input); // calculate output 
            double getOutput();
            
        protected:
            //time stuff
            std::optional<std::uint32_t> _previous_time = std::nullopt;
            // target stuff
            double _target;
            bool _arrived = false, _arrive = true;
            // tolerences
            double _small_error_tolerance = 1;
            double _big_error_tolerance = 3;
            double _small_error_duration = 100;
            double _big_error_duration = 500;
            double _small_check_time = 0, _big_check_time = 0;
            // bool _first_time = true;

            // coefficients and things
            double _kp, _ki, _kd;
            double _integral_range = 0;
            double _integral_max = 500;
            bool _sign_flip_integral_reset = false; // idc if this name is long and wordy
            double _derivative_tolerance, _error_tolerance;
            // error and output stuff
            double _current_error, _previous_error, _sum_error;
            double _proportional, _integral, _derivative;
            double _output;
    };
}
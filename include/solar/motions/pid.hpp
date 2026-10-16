#pragma once
#include <array>
#include <cstdint>
#include <optional>
namespace solar {
    class ControllerSettings {
        public:
            ControllerSettings(
                std::array<double, 3> constants, 
                double windupRange, 
                double smallError, 
                double smallErrorTimeout, 
                double largeError, 
                double largeErrorTimeout, 
                double slew
            ): 
                _constants(constants),
                _windupRange(windupRange),
                _smallError(smallError),
                _smallErrorTimeout(smallErrorTimeout),
                _largeError(largeError),
                _largeErrorTimeout(largeErrorTimeout),
                _slew(slew) 
            {}

            double _windupRange;
            double _smallError;
            double _smallErrorTimeout;
            double _largeError;
            double _largeErrorTimeout;
            double _slew;
            std::array<double, 3> _constants;
            
    };
    class PID {
        public:
            PID(double kp, double ki, double kd, double target);
            std::array<double, 3> getCoefficients();
            void setCoefficients(double kp, double ki, double kd);
            void setTarget(double target);
            double getkp();
            double getki();
            double getkd();
            void setkp(double);
            void setki(double);
            void setkd(double);
            /*
            integral stuff
            max is the max that integral can accumulate to
            range is the p range where integral applies
            so if the proportional term is not within ±(integral range) then integral term does not apply
            also getI is NOT the same as getki >:(
            */
            void setIntegralMax(double integral_max);
            void setIntegralRange(double integral_range);
            double getI();

            void clearSumError(); // sum error for integral
            void reset();

            double update(double input); // calculate output 
            double getOutput();
            
        protected:
            std::optional<std::uint32_t> _previous_time = std::nullopt;
            double _target;

            // coefficients and things
            double _kp, _ki, _kd;
            std::optional<double> _integral_range = std::nullopt;
            double _integral_max = 500;
            bool _sign_flip_integral_reset = false; // idc if this name is long and wordy

            // error and output stuff
            double _error, _previous_error = 0, _sum_error = 0;
            double _proportional, _integral, _derivative;
            double _output;
    };
}
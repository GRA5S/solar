#pragma once
#include <cstdint>
#include <optional>
namespace solar {
    struct ControllerSettings {
        std::optional<double> kp = std::nullopt;
        std::optional<double> ki = std::nullopt;
        std::optional<double> kd = std::nullopt;
        /*
            integral stuff
            max is the max that integral can accumulate to
            range is the p range where integral applies
            so if the proportional term is not within ±(integral range) then integral term does not apply
        */
        std::optional<double> integral_range = std::nullopt;
        std::optional<double> integral_max = std::nullopt;
        bool reset_integral_on_sign_flip = false;
        std::optional<double> small_error = std::nullopt;
        std::optional<double> small_error_timeout = std::nullopt;
        std::optional<double> large_error = std::nullopt;
        std::optional<double> large_error_timeout = std::nullopt;
        std::optional<double> slew = std::nullopt;
    };
    class PID {
        public:
            PID(ControllerSettings& settings);

            void setTarget(double target);

            double getI(); // ts not for getting ki btw its i

            void clearSumError(); // sum error for integral
            void reset();

            double update(double input); // calculate output 
            double getOutput();
            
        private:
            ControllerSettings& _settings;

            std::optional<std::uint32_t> _previous_time = std::nullopt;
            std::optional<double> _target = std::nullopt;
            // error and output stuff
            double _error = 0, _previous_error = 0, _sum_error = 0;
            double _proportional = 0, _integral = 0, _derivative = 0;
            double _output = 0;
    };
}
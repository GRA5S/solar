#pragma once
#include <cstdint>
#include <optional>
namespace solar {
    class ExitCondition{
        public:
            ExitCondition(std::optional<double> range, std::optional<double> timeout);
            bool update(double error);
            bool getExit();
            void reset();
        private:
            std::optional<uint32_t> _start_time;
            bool _arrived = false;
            std::optional<double> _range;
            std::optional<uint32_t> _timeout;
    }; 
}
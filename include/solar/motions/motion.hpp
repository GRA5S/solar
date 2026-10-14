#pragma once
#include <cstdint>
#include <optional>
namespace solar {
    class ExitCondition{
        public:
            ExitCondition(double range, uint32_t timeout);
            bool update(double error);
            void reset();
        private:
            std::optional<uint32_t> _start_time;
            bool _arrived = false;
            double _range;
            uint32_t _timeout;
    }; 
}
#pragma once
#include <vector>
#include <cmath>

namespace solar {
    inline double average(const std::vector<double>& values) {
        if (values.empty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (double v : values) {
            sum += v;
        }
        return sum / values.size();
    }
    inline double degToRad(double deg) {
        return (deg * std::numbers::pi / 180);
    }

    inline double radToDeg(double rad) {
        return (rad * 180 / std::numbers::pi);
    }

    inline double getRadius(double x, double y, double x1, double y1, double angle) {
        double 𝚫x = x1 - x;
        double 𝚫y = y1 - y;
        if((2 * 𝚫y * sin(degToRad(90 - angle))) == 0) {
            return 999;
        }
        return (𝚫x * 𝚫x + 𝚫y * 𝚫y) / (2 * 𝚫y * sin(degToRad(90 - angle)));
    }

    inline double normalizeAngle(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
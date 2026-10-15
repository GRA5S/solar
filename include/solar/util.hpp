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
    inline int sign(double input) {
        return (input > 0) ? 1 : ((input < 0) ? -1 : 0); // i used 2 ternary operators instead of just doing if elseif else cuz i hate my life
    }
    inline int sign(int input) {
        return (input > 0) ? 1 : ((input < 0) ? -1 : 0); // i used 2 ternary operators instead of just doing if elseif else cuz i hate my life
    }
    inline double getRadius(double x, double y, double x1, double y1, double angle) {
        double deltax = x1 - x;
        double deltay = y1 - y;
        if((2 * deltay * sin(degToRad(90 - angle))) == 0) {
            return 999;
        }
        return (deltax * deltax + deltay * deltay) / (2 * deltay * sin(degToRad(90 - angle)));
    }

    inline double normalizeAngle(double angle) {
        while (angle >= 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

}
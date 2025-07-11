//
// Created by fogoz on 08/05/2025.
//
#define _USE_MATH_DEFINES
#ifndef ANGLE_H
#define ANGLE_H
#ifdef ARDUINO
#include "Arduino.h"
#endif
#define WARP_ANGLE_DEG(angle) (fmod(fmod((angle) + 180, 360) - 360, 360) + 180)

#define WARP_ANGLE(angle) ({ \
    double _a = fmod((angle) + M_PI, 2.0 * M_PI); \
    if (_a < 0) _a += 2.0 * M_PI; \
    _a - M_PI; \
    })
#include <cmath>
#ifndef M_PI
#include <corecrt_math_defines.h>
#endif
inline double unwrapAngleDeg(double previous_unwrapped, double new_angle_wrapped) {
    double delta = new_angle_wrapped - fmod(previous_unwrapped + 180.0, 360.0) + 180.0;
    return previous_unwrapped + WARP_ANGLE_DEG(delta);
}
inline double unwrapAngleRad(double previous_unwrapped, double new_angle_wrapped) {
    double delta = new_angle_wrapped - fmod(previous_unwrapped + M_PI, M_PI * 2) + M_PI;
    return previous_unwrapped + WARP_ANGLE(delta);
}
#ifdef ARDUINO
class Angle : public Printable  {
#else
class Angle {
#endif
private:
    double rad;  // internal representation

    constexpr Angle(double rad) : rad(rad) {

    }
public:

    static constexpr Angle fromDegrees(double deg) {
        return {deg * M_PI / 180.0};
    }

    static constexpr Angle fromRadians(double rad) {
        return {rad};
    }

    static Angle fromUnwrappedDegrees(double deg, double previous_unwrapped) {
        return unwrapAngleDeg(previous_unwrapped, deg);
    }

    static Angle fromUnwrappedRadians(double rad, double previous_unwrapped) {
        return unwrapAngleRad(previous_unwrapped, rad);
    }

    Angle& toUnwrapped(Angle previous_unwrapped) {
        rad = unwrapAngleRad(previous_unwrapped.toRadians(), rad);
        return *this;
    }
    Angle& fromUnwrapped(Angle wrapped) {
        rad = unwrapAngleRad(rad, wrapped.toRadians());
        return *this;
    }

    constexpr double toDegrees() const {
        return rad * 180.0 / M_PI;
    }

    constexpr double toRadians() const {
        return rad;
    }

    Angle& warpAngle() {
        rad = WARP_ANGLE(rad);
        return *this;
    }

#ifdef ARDUINO
    size_t printTo(Print &p) const {
        size_t length = 0;
        length += p.print("Angle: ");
        length += p.print(toDegrees());
        return length;
    }
#endif

    // Comparison operators generated with the help of ia (chat.deepseek.com)
    bool operator==(const Angle& other) const {
        return std::abs(rad - other.rad) < 1e-10;  // small epsilon for floating point comparison
    }

    bool operator!=(const Angle& other) const {
        return !(*this == other);
    }

    constexpr bool operator<(const Angle& other) const {
        return rad < other.rad;
    }

    constexpr bool operator<=(const Angle& other) const {
        return rad <= other.rad;
    }

    constexpr bool operator>(const Angle& other) const {
        return rad > other.rad;
    }

    constexpr bool operator>=(const Angle& other) const {
        return rad >= other.rad;
    }

    // Arithmetic operators
    constexpr Angle operator+(const Angle& other) const {
        return {rad + other.rad};
    }

    constexpr Angle operator-(const Angle& other) const {
        return {rad - other.rad};
    }

    constexpr Angle operator-() const {
        return {-rad};
    }

    constexpr Angle& operator+=(const Angle& other) {
        rad += other.rad;
        return *this;
    }

    constexpr Angle& operator-=(const Angle& other) {
        rad -= other.rad;
        return *this;
    }

    // Scalar multiplication/division
    constexpr Angle operator*(double scalar) const {
        return {rad * scalar};
    }

    constexpr Angle operator/(double scalar) const {
        return {rad / scalar};
    }

    constexpr Angle& operator*=(double scalar) {
        rad *= scalar;
        return *this;
    }

    constexpr Angle& operator/=(double scalar) {
        rad /= scalar;
        return *this;
    }

    constexpr bool isBehind() const {
        return abs(rad) > M_PI_2;
    }
};

// User-defined literals
inline constexpr Angle operator"" _deg(long double deg) {
    return Angle::fromDegrees(static_cast<double>(deg));
}

inline constexpr Angle operator"" _rad(long double rad) {
    return Angle::fromRadians(static_cast<double>(rad));
}

inline constexpr Angle operator"" _deg(unsigned long long deg) {  // for integer degrees
    return Angle::fromDegrees(static_cast<double>(deg));
}

inline constexpr Angle operator"" _rad(unsigned long long rad) {  // for integer radians
    return Angle::fromRadians(static_cast<double>(rad));
}


// Scalar multiplication (commutative)
inline constexpr Angle operator*(double scalar, const Angle& angle) {
    return angle * scalar;
}
namespace AngleConstants {
    inline constexpr Angle ZERO = Angle::fromDegrees(0);
    inline constexpr Angle FRONT = Angle::fromDegrees(0);
    inline constexpr Angle LEFT = Angle::fromDegrees(90);
    inline constexpr Angle RIGHT = Angle::fromDegrees(-90);
    inline constexpr Angle BACK = Angle::fromDegrees(180);
    inline constexpr Angle FULL_TURN = Angle::fromDegrees(360);
    inline constexpr Angle HALF_TURN = Angle::fromDegrees(180);
}

#ifndef ARDUINO
#include <iostream>
inline std::ostream& operator<<(std::ostream& os, const Angle& angle) {
    os << angle.toDegrees();
    return os;
}
#endif

#endif //ANGLE_H

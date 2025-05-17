
#ifndef POSITION_H
#define POSITION_H
#include <optional>

#include "Angle.h"
#include "Matrix.h"
#ifdef ARDUINO
#include "Arduino.h"
#else
#include <iostream>
#endif
#ifdef ARDUINO
class Position : public Printable {
#else
class Position{
#endif
private:
    double x;
    double y;
    Angle a;
    double curvature;


    public:
    constexpr Position(double x=0.0f, double y=0.0f, Angle a=AngleConstants::ZERO, double curvature=0.0f): x(x), y(y), a(a), curvature(curvature) {
    }
    constexpr double getX() const{
        return x;
    };
    
    constexpr double getY() const{
        return y;
    }

    constexpr Angle getAngle() const{
        return a;
    }
    constexpr double getCurvature() const {
        return curvature;
    }
    constexpr void add(double x, double y, Angle a){
        this->x += x;
        this->y += y;
        this->a += a;
        this->a.warpAngle();
    }

    static constexpr Position from(Matrix<2, 1> matrix) {
        return {matrix(0,0), matrix(1,0)};
    }

    constexpr Position& warpAngle() {
        this->a.warpAngle();
        return *this;
    }

    constexpr double getDistance() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2));
    }

    constexpr double normCompleteRad(double pos_weight, double angle_weight, double curvature_weight) const {
        return sqrt(pos_weight * pow(this->x, 2) + pos_weight * pow(this->y, 2) + angle_weight * pow(this->a.toRadians(), 2) + curvature_weight * pow(this->curvature, 2));
    }

    constexpr double normDeg() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->a.toDegrees(), 2));
    }

    constexpr double normRad() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->a.toRadians(), 2));
    }

    constexpr double norm() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2));
    }

    constexpr Angle getVectorAngle() const {
        return Angle::fromRadians(atan2(this->y, this->x));
    }

    constexpr std::array<double, 2> getXY() const {
        return {this->x, this->y};
    }

    constexpr std::array<double, 3> getXYRad() const {
        return {this->x, this->y, this->a.toRadians()};
    }

    constexpr std::array<double, 4> getXYRadCurv() const {
        return {this->x, this->y, this->a.toRadians(), this->curvature};
    }


    constexpr std::array<double, 3> getXYDeg() const {
        return {this->x, this->y, this->a.toDegrees()};
    }


    constexpr Position operator+(const Position& pos) const{
        return {this->x + pos.x, this->y + pos.y, this->a+ pos.a};
    }

    constexpr Position operator-(const Position& pos) const{
        return {this->x - pos.x, this->y - pos.y, this->a - pos.a};
    }

    constexpr Position operator+=(const Position &rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        this->a += rhs.a;
        this->a.warpAngle();
        return *this;
    }

    constexpr Position operator*(const double rhs) const {
        return {this->x * rhs, this->y * rhs, this->a * rhs};
    }

    constexpr Position operator/(const double rhs) const{
        return {this->x / rhs, this->y / rhs, this->a / rhs};
    }
#ifdef ARDUINO
    size_t printTo(Print &p) const {
        size_t length = 0;
        length += p.print("x: ");
        length += p.print(x);
        length += p.print(", y: ");
        length += p.print(y);
        length += p.print(", angle: ");
        length += p.print(a.toDegrees());
        return length;
    }
#endif
    constexpr Position getSinCosAngle() const {
        return {cos(this->a.toRadians()), sin(this->a.toRadians())};
    }

    constexpr Position getNormalVector() const {
        return {-sin(this->a.toRadians()), cos(this->a.toRadians())};
    }
};

constexpr std::optional<Position> intersectLines(const Position&p1, const Position&n1, const Position&p2, const Position&n2) {
    double det = -n1.getX()*n2.getY() + n1.getY()*n2.getX();
    if (fabs(det) < 1e-6) {
        return std::nullopt;
    }
    double dx = p1.getX() - p2.getX();
    double dy = p1.getY() - p2.getY();

    double s = (dx * -n2.getY() + dy * n2.getX()) / det;
    Position center = {p1.getX() + s * n1.getX(), p1.getY() + s * n1.getY()};
    return center;
}

constexpr Position operator*(double value, const Position& pos) {
    return pos * value;
}

constexpr std::optional<Position> intersectPerpendicularLine(const Position &p1, const Position &p2) {
    Position n1 = p1.getNormalVector();
    Position n2 = p2.getNormalVector();
    return intersectLines(p1, n1, p2, n2);
}
#ifndef ARDUINO
template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& arr)
{
    os << "[";
    for (std::size_t i = 0; i < N; ++i) {
        os << arr[i];
        if (i != N - 1)
            os << ", ";
    }
    os << "]";
    return os;
}
inline std::ostream& operator<<(std::ostream& os, const Position& pos) {
    os << "x: " << pos.getX() << ", y: " << pos.getY() << ", angle: " << pos.getAngle().toDegrees() << ", curvature: " << pos.getCurvature();
    return os;
}
#endif

#endif

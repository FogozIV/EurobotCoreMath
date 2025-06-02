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

/**
 * @brief 2D position with orientation and curvature
 * 
 * This class represents a position in 2D space with:
 * - X, Y coordinates
 * - Orientation angle
 * - Path curvature
 * 
 * It provides methods for:
 * - Position calculations
 * - Vector operations
 * - Geometric transformations
 * - Distance and angle computations
 */
#ifdef ARDUINO
class Position : public Printable {
#else
class Position {
#endif
private:
    double x;         ///< X coordinate
    double y;         ///< Y coordinate
    Angle a;         ///< Orientation angle
    double curvature; ///< Path curvature

public:
    /**
     * @brief Constructs a new position
     * 
     * @param x X coordinate (default: 0)
     * @param y Y coordinate (default: 0)
     * @param a Orientation angle (default: 0)
     * @param curvature Path curvature (default: 0)
     */
    constexpr Position(double x=0.0f, double y=0.0f, Angle a=AngleConstants::ZERO, double curvature=0.0f)
        : x(x), y(y), a(a), curvature(curvature) {
    }

    /**
     * @brief Gets X coordinate
     * @return double X position
     */
    constexpr double getX() const { return x; }

    /**
     * @brief Gets Y coordinate
     * @return double Y position
     */
    constexpr double getY() const { return y; }

    /**
     * @brief Gets orientation angle
     * @return Angle Current orientation
     */
    constexpr Angle getAngle() const { return a; }

    /**
     * @brief Gets path curvature
     * @return double Current curvature
     */
    constexpr double getCurvature() const { return curvature; }

    /**
     * @brief Adds a relative position
     * 
     * Updates position by adding:
     * - X offset
     * - Y offset
     * - Angle change
     * 
     * @param x X offset
     * @param y Y offset
     * @param a Angle change
     */
    void add(double x, double y, Angle a) {
        this->x += x;
        this->y += y;
        this->a += a;
        this->a.warpAngle();
    }

    /**
     * @brief Creates position from matrix
     * @param matrix 2x1 position matrix
     * @return Position New position object
     */
    static constexpr Position from(Matrix<2, 1> matrix) {
        return {matrix(0,0), matrix(1,0)};
    }

    /**
     * @brief Normalizes angle to [-π, π]
     * @return Position& Reference to this
     */
    Position& warpAngle() {
        this->a.warpAngle();
        return *this;
    }

    /**
     * @brief Calculates distance from origin
     * @return double Distance
     */
    double getDistance() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2));
    }

    /**
     * @brief Weighted position norm
     * 
     * Computes weighted norm including:
     * - Position (x,y)
     * - Angle
     * - Curvature
     * 
     * @param pos_weight Position weight
     * @param angle_weight Angle weight
     * @param curvature_weight Curvature weight
     * @return double Weighted norm
     */
    double normCompleteRad(double pos_weight, double angle_weight, double curvature_weight) const {
        return sqrt(pos_weight * pow(this->x, 2) + pos_weight * pow(this->y, 2) + 
                   angle_weight * pow(this->a.toRadians(), 2) + 
                   curvature_weight * pow(this->curvature, 2));
    }

    /**
     * @brief Position norm with angle in degrees
     * @return double Norm value
     */
    double normDeg() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->a.toDegrees(), 2));
    }

    /**
     * @brief Position norm with angle in radians
     * @return double Norm value
     */
    double normRad() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2) + pow(this->a.toRadians(), 2));
    }

    /**
     * @brief Position norm (x,y only)
     * @return double Norm value
     */
    double norm() const {
        return sqrt(pow(this->x, 2) + pow(this->y, 2));
    }

    /**
     * @brief Gets angle of position vector
     * @return Angle Vector angle
     */
    constexpr Angle getVectorAngle() const {
        return Angle::fromRadians(atan2(this->y, this->x));
    }

    /**
     * @brief Gets X,Y coordinates
     * @return array<double,2> [x,y]
     */
    constexpr std::array<double, 2> getXY() const {
        return {this->x, this->y};
    }

    /**
     * @brief Gets X,Y,angle(rad)
     * @return array<double,3> [x,y,angle]
     */
    constexpr std::array<double, 3> getXYRad() const {
        return {this->x, this->y, this->a.toRadians()};
    }

    /**
     * @brief Gets X,Y,angle(rad),curvature
     * @return array<double,4> [x,y,angle,curvature]
     */
    constexpr std::array<double, 4> getXYRadCurv() const {
        return {this->x, this->y, this->a.toRadians(), this->curvature};
    }

    /**
     * @brief Create a Position offset
     * @param dx the variation in the local x position
     * @param dy the variation in the local y position
     * @param da the angle (can be clearly ignored)
     * @return the new position
     */
    constexpr Position offsetRelative(double dx, double dy, Angle da = AngleConstants::ZERO) const {
        double theta = a.toRadians();
        double cosA = cos(theta);
        double sinA = sin(theta);

        double newX = x + dx * cosA - dy * sinA;
        double newY = y + dx * sinA + dy * cosA;
        return Position(newX, newY, a + da);
    }
    /**
     * @brief Create a Position offset
     * @param pos the position
     * @return the new position
     */
    constexpr Position offsetRelative(Position pos) const {
        return offsetRelative(pos.getX(), pos.getY(), pos.getAngle());
    }

    /**
     * @brief Gets X,Y,angle(deg)
     * @return array<double,3> [x,y,angle]
     */
    constexpr std::array<double, 3> getXYDeg() const {
        return {this->x, this->y, this->a.toDegrees()};
    }

    /**
     * @brief Adds two positions
     * @param pos Position to add
     * @return Position Sum position
     */
    constexpr Position operator+(const Position& pos) const {
        return {this->x + pos.x, this->y + pos.y, this->a + pos.a};
    }

    /**
     * @brief Subtracts two positions
     * @param pos Position to subtract
     * @return Position Difference position
     */
    constexpr Position operator-(const Position& pos) const {
        return {this->x - pos.x, this->y - pos.y, this->a - pos.a};
    }

    /**
     * @brief Adds position in place
     * @param rhs Position to add
     * @return Position Updated position
     */
    Position operator+=(const Position &rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        this->a += rhs.a;
        this->a.warpAngle();
        return *this;
    }

    /**
     * @brief Scales position
     * @param rhs Scale factor
     * @return Position Scaled position
     */
    constexpr Position operator*(const double rhs) const {
        return {this->x * rhs, this->y * rhs, this->a * rhs};
    }

    /**
     * @brief Divides position
     * @param rhs Divisor
     * @return Position Divided position
     */
    constexpr Position operator/(const double rhs) const {
        return {this->x / rhs, this->y / rhs, this->a / rhs};
    }

#ifdef ARDUINO
    /**
     * @brief Prints position (Arduino)
     * @param p Print object
     * @return size_t Characters printed
     */
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

    /**
     * @brief Gets unit vector in current direction
     * @return Position Direction vector
     */
    constexpr Position getSinCosAngle() const {
        return {cos(this->a.toRadians()), sin(this->a.toRadians())};
    }

    /**
     * @brief Gets normal vector to current direction
     * @return Position Normal vector
     */
    constexpr Position getNormalVector() const {
        return {-sin(this->a.toRadians()), cos(this->a.toRadians())};
    }
};

/**
 * @brief Finds intersection of two lines
 * 
 * @param p1 Point on first line
 * @param n1 Direction of first line
 * @param p2 Point on second line
 * @param n2 Direction of second line
 * @return optional<Position> Intersection point if exists
 */
inline std::optional<Position> intersectLines(const Position&p1, const Position&n1, 
                                            const Position&p2, const Position&n2) {
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

/**
 * @brief Scales position
 * @param value Scale factor
 * @param pos Position to scale
 * @return Position Scaled position
 */
constexpr Position operator*(double value, const Position& pos) {
    return pos * value;
}

/**
 * @brief Finds intersection of perpendicular lines
 * 
 * @param p1 Point on first line
 * @param p2 Point on second line
 * @return optional<Position> Intersection point if exists
 */
inline std::optional<Position> intersectPerpendicularLine(const Position &p1, const Position &p2) {
    Position n1 = p1.getNormalVector();
    Position n2 = p2.getNormalVector();
    return intersectLines(p1, n1, p2, n2);
}

#ifndef ARDUINO
/**
 * @brief Array output operator
 */
template <class T, std::size_t N>
std::ostream& operator<<(std::ostream& os, const std::array<T, N>& arr) {
    os << "[";
    for (std::size_t i = 0; i < N; ++i) {
        os << arr[i];
        if (i != N - 1)
            os << ", ";
    }
    os << "]";
    return os;
}

/**
 * @brief Position output operator
 */
inline std::ostream& operator<<(std::ostream& os, const Position& pos) {
    os << "x: " << pos.getX() << ", y: " << pos.getY() 
       << ", angle: " << pos.getAngle().toDegrees() 
       << ", curvature: " << pos.getCurvature();
    return os;
}
#endif

#endif

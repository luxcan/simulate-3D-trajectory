#pragma once

// Geometry shared by the tests and the sample generator, worked out independently of
// TrajectoryFit.cpp so that it can check it.

#include <cmath>
#include <vector>

#include "TrajectoryFit.h"

namespace testgeo
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadiansPerDegree = kPi / 180.0;

inline Vector3D add(const Vector3D& a, const Vector3D& b) { return Vector3D(a.i + b.i, a.j + b.j, a.k + b.k); }
inline Vector3D subtract(const Vector3D& a, const Vector3D& b) { return Vector3D(a.i - b.i, a.j - b.j, a.k - b.k); }
inline Vector3D times(const Vector3D& v, double s) { return Vector3D(v.i * s, v.j * s, v.k * s); }
inline double dot(const Vector3D& a, const Vector3D& b) { return a.i * b.i + a.j * b.j + a.k * b.k; }
inline double length(const Vector3D& v) { return std::sqrt(dot(v, v)); }

// A z-up frame: East-North-Up at a latitude and longitude for WCS, or plain axes moved to an
// origin for local metres.
struct Frame
{
    Vector3D origin;
    Vector3D east{1, 0, 0};
    Vector3D north{0, 1, 0};
    Vector3D up{0, 0, 1};

    Vector3D toWorldDirection(const Vector3D& l) const { return add(add(times(east, l.i), times(north, l.j)), times(up, l.k)); }
    Vector3D toWorldPoint(const Vector3D& l) const { return add(origin, toWorldDirection(l)); }
    Vector3D toLocalDirection(const Vector3D& w) const { return Vector3D(dot(w, east), dot(w, north), dot(w, up)); }
    Vector3D toLocalPoint(const Vector3D& w) const { return toLocalDirection(subtract(w, origin)); }
};

inline Frame wcsSite(double latitudeDegrees, double longitudeDegrees, double height)
{
    const double semiMajorAxis = 6378137.0;
    const double flattening = 1.0 / 298.257223563;
    const double eccentricitySquared = flattening * (2.0 - flattening);
    const double lat = latitudeDegrees * kRadiansPerDegree;
    const double lon = longitudeDegrees * kRadiansPerDegree;
    const double primeVerticalRadius = semiMajorAxis / std::sqrt(1.0 - eccentricitySquared * std::sin(lat) * std::sin(lat));

    Frame frame;
    frame.origin = Vector3D((primeVerticalRadius + height) * std::cos(lat) * std::cos(lon),
                            (primeVerticalRadius + height) * std::cos(lat) * std::sin(lon),
                            (primeVerticalRadius * (1.0 - eccentricitySquared) + height) * std::sin(lat));
    frame.east = Vector3D(-std::sin(lon), std::cos(lon), 0);
    frame.north = Vector3D(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon), std::cos(lat));
    frame.up = Vector3D(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
    return frame;
}

inline Frame localOrigin(const Vector3D& origin)
{
    Frame frame;
    frame.origin = origin;
    return frame;
}

inline Vector3D atHeading(double headingDegrees, double range, double up)
{
    const double heading = headingDegrees * kRadiansPerDegree;
    return Vector3D(range * std::sin(heading), range * std::cos(heading), up);
}

// A football with air drag, kicked from (0, 0, 0) of a z-up frame and sampled every 0.1 s from
// t = 0.1 s up to the first row below the ground, the way the CSV is laid out.
inline std::vector<TrajectoryPoint> footballKick(double headingDegrees, double elevationDegrees, double speed)
{
    const double gravity = 9.80665;
    const double dragPerMetre = 0.5 * 1.2 * 0.25 * kPi * 0.11 * 0.11 / 0.43;
    const auto accelerationAt = [&](const Vector3D& v) {
        const double s = length(v);
        return Vector3D(-dragPerMetre * s * v.i, -dragPerMetre * s * v.j, -gravity - dragPerMetre * s * v.k);
    };

    const double elevation = elevationDegrees * kRadiansPerDegree;
    Vector3D position(0, 0, 0);
    Vector3D velocity = atHeading(headingDegrees, speed * std::cos(elevation), speed * std::sin(elevation));
    std::vector<TrajectoryPoint> rows;
    const int stepsPerRow = 100;
    const double step = 0.1 / stepsPerRow;
    while (true)
    {
        rows.push_back({0.1 * static_cast<double>(rows.size() + 1), position, velocity, accelerationAt(velocity)});
        if (rows.size() > 1 && position.k < 0)
            return rows;

        for (int s = 0; s < stepsPerRow; ++s)  // fourth-order Runge-Kutta
        {
            const Vector3D a1 = accelerationAt(velocity);
            const Vector3D v2 = add(velocity, times(a1, step / 2));
            const Vector3D a2 = accelerationAt(v2);
            const Vector3D v3 = add(velocity, times(a2, step / 2));
            const Vector3D a3 = accelerationAt(v3);
            const Vector3D v4 = add(velocity, times(a3, step));
            const Vector3D a4 = accelerationAt(v4);
            position = add(position, times(add(add(velocity, times(add(v2, v3), 2)), v4), step / 6));
            velocity = add(velocity, times(add(add(a1, times(add(a2, a3), 2)), a4), step / 6));
        }
    }
}

// Constant velocity from (0, 0, 0) to `end` over `duration` seconds, rows every 0.1 s from t = 0.1.
inline std::vector<TrajectoryPoint> straightLine(const Vector3D& end, double duration)
{
    const int segments = static_cast<int>(std::lround(duration / 0.1));
    const Vector3D velocity = times(end, 1.0 / duration);
    std::vector<TrajectoryPoint> rows;
    for (int row = 0; row <= segments; ++row)
        rows.push_back({0.1 * (row + 1), times(end, static_cast<double>(row) / segments), velocity, Vector3D(0, 0, 0)});
    return rows;
}

inline std::vector<TrajectoryPoint> toWorld(const std::vector<TrajectoryPoint>& local, const Frame& frame)
{
    std::vector<TrajectoryPoint> world;
    for (const auto& row : local)
        world.push_back({row.time, frame.toWorldPoint(row.position), frame.toWorldDirection(row.velocity),
                         frame.toWorldDirection(row.acceleration)});
    return world;
}
}  // namespace testgeo

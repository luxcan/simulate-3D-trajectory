#pragma once

#include <vector>

#include "Vector3D.h"

// One CSV row. If you already have a row struct, include its header instead of this one: the fit
// needs members named time (double, seconds) and position, velocity, acceleration (Vector3D).
struct TrajectoryPoint
{
    double time;            // seconds, from the CSV's time column
    Vector3D position;      // x, y, z
    Vector3D velocity;      // Vx, Vy, Vz
    Vector3D acceleration;  // Ax, Ay, Az
};

// Places the CSV trajectory between a chosen launch and impact point: turned to face the impact
// point, stretched to its distance, and shifted in height so the last row lands on it. The first
// row must be the launch and the last row the impact.
//
// The CSV and the two points may each be in WCS (geocentric metres, as the simulation uses) or in
// local metres with z up; the frame is detected from the numbers. The result is in the same frame
// as gameStart and gameEnd and keeps the CSV's times. The CSV is never modified, so every
// launch/impact choice starts from the original data.
//
// Of Vector3D it uses only the public i, j, k fields and a Vector3D(x, y, z) constructor.
//
// Throws std::invalid_argument, with a message to show the user, when the inputs cannot be fitted.
// Catch it where the points are picked rather than let it end the program:
//
//     try { path = fitTrajectory(csv, launchPoint, impactPoint); }
//     catch (const std::invalid_argument& error) { /* show error.what(), keep the previous path */ }
std::vector<TrajectoryPoint> fitTrajectory(const std::vector<TrajectoryPoint>& csv,
                                           const Vector3D& gameStart,
                                           const Vector3D& gameEnd);

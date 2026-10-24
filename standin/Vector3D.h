#pragma once

// Stand-in for your own Vector3D, used by the tests and the FitCsv app. Deliberately bare: public
// i, j, k and the two constructors, no getters or mag(), so every build proves TrajectoryFit.cpp
// needs nothing more from your class.
struct Vector3D
{
    double i = 0;
    double j = 0;
    double k = 0;

    Vector3D() = default;
    Vector3D(double x, double y, double z) : i(x), j(y), k(z) {}
};

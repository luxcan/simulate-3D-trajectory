// Fits a CSV trajectory between a launch point and an impact point and writes the result as CSV.
//
//     FitCsv <input.csv> <output.csv> <launch x> <launch y> <launch z> <impact x> <impact y> <impact z>
//
// The points are in WCS (geocentric metres) or local metres with z up, like the CSV. The output
// keeps the input's columns and their order.

#include <iostream>
#include <stdexcept>
#include <string>

#include "TrajectoryCsv.h"
#include "TrajectoryFit.h"

namespace
{
double parseCoordinate(const char* text, const char* name)
{
    const std::string value(text);
    std::size_t used = 0;
    try
    {
        const double number = std::stod(value, &used);
        if (used == value.size())
            return number;
    }
    catch (const std::exception&)
    {
    }
    throw std::invalid_argument(std::string("The ") + name + " \"" + value + "\" is not a number.");
}

Vector3D parsePoint(char* const* arguments, const std::string& name)
{
    return Vector3D(parseCoordinate(arguments[0], (name + " x").c_str()),
                    parseCoordinate(arguments[1], (name + " y").c_str()),
                    parseCoordinate(arguments[2], (name + " z").c_str()));
}
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 9)
    {
        std::cerr << "Usage: FitCsv <input.csv> <output.csv> <launch x> <launch y> <launch z> <impact x> <impact y> <impact z>\n";
        return 2;
    }

    try
    {
        const TrajectoryCsv input = readTrajectoryCsv(argv[1]);
        const Vector3D launch = parsePoint(argv + 3, "launch");
        const Vector3D impact = parsePoint(argv + 6, "impact");
        const std::vector<TrajectoryPoint> fitted = fitTrajectory(input.rows, launch, impact);
        writeTrajectoryCsv(argv[2], input.header, fitted, input.cells);
        std::cout << "Fitted " << fitted.size() << " rows: " << argv[1] << " -> " << argv[2] << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FitCsv: " << error.what() << '\n';
        return 1;
    }
}

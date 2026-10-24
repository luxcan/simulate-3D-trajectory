// Writes the sample input CSVs into a folder, with cases.txt listing, for each one, the output file
// and the launch and impact points (WCS) to fit it to. Every launch is from the Padang in Singapore,
// and every impact point is the recorded flight's own distance and drop away from it, so each output
// is its input turned and moved, with nothing stretched or raised.
//
//     MakeSamples <folder>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "TestGeometry.h"
#include "TrajectoryCsv.h"

using namespace testgeo;

namespace
{
struct Sample
{
    const char* input;
    const char* output;
    const char* description;
    std::vector<TrajectoryPoint> flight;  // z-up metres from the launch, in the place it was recorded
    Frame recordedAt;                     // that place: WCS, or local metres
    double headingDegrees;                // the way the impact point lies from the Singapore launch
};

// The impact point the flight's own distance and drop away from the launch, in the given direction.
Vector3D ownLanding(const std::vector<TrajectoryPoint>& flight, double headingDegrees)
{
    const Vector3D landing = subtract(flight.back().position, flight.front().position);
    return atHeading(headingDegrees, std::hypot(landing.i, landing.j), landing.k);
}
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: MakeSamples <folder>\n";
        return 2;
    }

    try
    {
        const std::string folder = std::string(argv[1]) + "/";
        const Frame launch = wcsSite(1.2903, 103.8520, 10.0);
        const std::vector<Sample> samples = {
            {"A_input_wcs_london.csv", "A_output_singapore.csv", "football kicked in London (51.50 N, 0.12 W), WCS",
             footballKick(75.0, 40.0, 30.0), wcsSite(51.5007, -0.1246, 20.0), 45.0},
            {"B_input_ecef_sydney.csv", "B_output_singapore.csv", "straight-line model in Sydney (33.86 S, 151.22 E), ECEF",
             straightLine(atHeading(200.0, 400.0, -30.0), 10.0), wcsSite(-33.8568, 151.2153, 5.0), 300.0},
            {"C_input_local.csv", "C_output_singapore.csv", "football kick in local metres, z up",
             footballKick(130.0, 50.0, 20.0), localOrigin(Vector3D(250.0, -80.0, 1.5)), 180.0},
        };

        std::ofstream cases(folder + "cases.txt");
        for (const auto& sample : samples)
        {
            writeTrajectoryCsv(folder + sample.input, kAssignmentHeader, toWorld(sample.flight, sample.recordedAt));
            const Vector3D landing = ownLanding(sample.flight, sample.headingDegrees);
            const Vector3D impact = launch.toWorldPoint(landing);
            char line[512];
            std::snprintf(line, sizeof line, "%s %s %.6f %.6f %.6f %.6f %.6f %.6f\n", sample.input, sample.output,
                          launch.origin.i, launch.origin.j, launch.origin.k, impact.i, impact.j, impact.k);
            cases << line;
            std::printf("Wrote %s: %s, %zu rows; impact %.2f m away at heading %.0f deg, %+.3f m in height\n", sample.input,
                        sample.description, sample.flight.size(), std::hypot(landing.i, landing.j), sample.headingDegrees,
                        landing.k);
        }
        if (!cases)
            throw std::invalid_argument("Writing cases.txt failed.");
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "MakeSamples: " << error.what() << '\n';
        return 1;
    }
}

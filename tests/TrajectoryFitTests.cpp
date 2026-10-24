// Checks fitTrajectory on synthetic CSVs in both frames. Expected values come from geometry worked
// out here, independently of TrajectoryFit.cpp, and the original snippet is run alongside for
// comparison.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "TestGeometry.h"
#include "TrajectoryFit.h"

using namespace testgeo;

namespace
{
int g_checks = 0;
int g_failures = 0;

void check(bool ok, const std::string& what)
{
    ++g_checks;
    if (!ok)
        ++g_failures;
    std::printf("  %s  %s\n", ok ? "ok  " : "FAIL", what.c_str());
}

void checkAtMost(double actual, double limit, const std::string& what)
{
    char line[256];
    std::snprintf(line, sizeof line, "%s: %.3g (limit %.0e)", what.c_str(), actual, limit);
    check(actual <= limit, line);
}

void expectRejected(const std::string& what, const std::function<void()>& action)
{
    try
    {
        action();
        check(false, what + ": was accepted");
    }
    catch (const std::invalid_argument& error)
    {
        check(true, what + ": \"" + error.what() + "\"");
    }
}

// A level line that follows the Earth (constant height above the ellipsoid, as a simulated entity
// flying level logs it) in WCS, printed to 3 decimals like a CSV, with the acceleration column
// left at zero as in a straight-line model.
std::vector<TrajectoryPoint> levelLineOverTheEarth(double latitudeDegrees, double longitudeDegrees, double height,
                                                   double headingDegrees, double distance, double duration)
{
    const double earthRadius = 6378137.0;
    const double heading = headingDegrees * kRadiansPerDegree;
    const auto printed = [](const Vector3D& v) {
        return Vector3D(std::round(v.i * 1000) / 1000, std::round(v.j * 1000) / 1000, std::round(v.k * 1000) / 1000);
    };
    const int segments = static_cast<int>(std::lround(duration / 0.1));
    std::vector<TrajectoryPoint> rows;
    for (int row = 0; row <= segments; ++row)
    {
        const double travelled = distance * row / segments;
        const double latitude = latitudeDegrees + travelled * std::cos(heading) / earthRadius / kRadiansPerDegree;
        const double longitude = longitudeDegrees + travelled * std::sin(heading) /
                                                        (earthRadius * std::cos(latitudeDegrees * kRadiansPerDegree)) /
                                                        kRadiansPerDegree;
        const Frame here = wcsSite(latitude, longitude, height);
        rows.push_back({0.1 * (row + 1), printed(here.origin),
                        printed(here.toWorldDirection(atHeading(headingDegrees, distance / duration, 0.0))), Vector3D(0, 0, 0)});
    }
    return rows;
}

// The rows from `first` on, moved so that row sits at (0, 0, 0), times kept.
std::vector<TrajectoryPoint> fromRow(const std::vector<TrajectoryPoint>& rows, std::size_t first)
{
    std::vector<TrajectoryPoint> tail(rows.begin() + static_cast<std::ptrdiff_t>(first), rows.end());
    const Vector3D origin = tail.front().position;
    for (auto& row : tail)
        row.position = subtract(row.position, origin);
    return tail;
}

// The original snippet, line for line, reading set_rotate_ccw / set_rotate_cw as counter-clockwise
// and clockwise turns in radians. Returns positions only, as the snippet does.
std::vector<Vector3D> originalSnippet(const std::vector<TrajectoryPoint>& trajectory, const Vector3D& gameStart,
                                      const Vector3D& gameEnd)
{
    const Vector3D csvStart = trajectory.front().position;
    const Vector3D csvEnd = trajectory.back().position;
    const double csvAngle = std::atan2(csvEnd.j - csvStart.j, csvEnd.i - csvStart.i);
    const double gameAngle = std::atan2(gameEnd.j - gameStart.j, gameEnd.i - gameStart.i);
    const double rotationAngle = gameAngle - csvAngle;

    std::vector<Vector3D> positions;
    for (const auto& point : trajectory)
    {
        const Vector3D relative = subtract(point.position, csvStart);
        positions.push_back(Vector3D(gameStart.i + std::cos(rotationAngle) * relative.i - std::sin(rotationAngle) * relative.j,
                                     gameStart.j + std::sin(rotationAngle) * relative.i + std::cos(rotationAngle) * relative.j,
                                     gameStart.k + relative.k));
    }
    return positions;
}

// How a placed trajectory sits in the game's own z-up frame: does it end on the impact point, and
// does its arc stand straight up over the launch-to-impact line or lean to one side?
void describeShape(const char* label, const std::vector<Vector3D>& positions, const Frame& gameFrame,
                   const Vector3D& gameEndLocal)
{
    const Vector3D end = gameFrame.toLocalPoint(positions.back());
    const Vector3D line = times(gameEndLocal, 1.0 / length(gameEndLocal));
    const Vector3D upAcrossLine = [&] {
        const Vector3D u = subtract(Vector3D(0, 0, 1), times(line, line.k));
        return times(u, 1.0 / length(u));
    }();

    double furthest = 0;
    double lean = 0;
    for (const auto& world : positions)
    {
        const Vector3D p = gameFrame.toLocalPoint(world);
        const Vector3D offLine = subtract(p, times(line, dot(p, line)));
        if (length(offLine) > furthest)
        {
            furthest = length(offLine);
            lean = std::acos(std::clamp(dot(offLine, upAcrossLine) / length(offLine), -1.0, 1.0)) / kRadiansPerDegree;
        }
    }
    char leaning[48] = "on the line";
    if (furthest > 1e-6)
        std::snprintf(leaning, sizeof leaning, "leaning %6.2f deg from vertical", lean);
    std::printf("  %-18s misses impact by %9.3f m | ends %9.3f m above/below it | furthest point off the line %8.3f m, %s\n",
                label, length(subtract(end, gameEndLocal)), end.k - gameEndLocal.k, furthest, leaning);
}

// Runs fitTrajectory and checks every row against the expected placement: along and across the
// launch-to-impact line scaled by the range ratio, height shifted linearly in time, velocity and
// acceleration turned the same way with Vz shifted by a constant and Az untouched.
std::vector<TrajectoryPoint> checkFit(const char* name, const std::vector<TrajectoryPoint>& csvLocal, const Frame& csvFrame,
                                      const Frame& gameFrame, const Vector3D& gameEndLocal)
{
    std::printf("\n%s\n", name);
    const std::vector<TrajectoryPoint> csv = toWorld(csvLocal, csvFrame);
    const Vector3D gameStart = gameFrame.origin;
    const Vector3D gameEnd = gameFrame.toWorldPoint(gameEndLocal);
    const std::vector<TrajectoryPoint> fitted = fitTrajectory(csv, gameStart, gameEnd);

    check(fitted.size() == csv.size(), "one fitted row per CSV row (" + std::to_string(csv.size()) + ")");
    checkAtMost(length(subtract(fitted.front().position, gameStart)), 1e-6, "launch row distance from the launch point, m");
    checkAtMost(length(subtract(fitted.back().position, gameEnd)), 1e-6, "impact row distance from the impact point, m");

    const Vector3D csvEnd = csvLocal.back().position;
    const double csvRange = std::hypot(csvEnd.i, csvEnd.j);
    const double gameRange = std::hypot(gameEndLocal.i, gameEndLocal.j);
    const double cx = csvEnd.i / csvRange, cy = csvEnd.j / csvRange;
    const double gx = gameEndLocal.i / gameRange, gy = gameEndLocal.j / gameRange;
    const double scale = gameRange / csvRange;
    const double heightError = gameEndLocal.k - csvEnd.k;
    const double flightTime = csvLocal.back().time - csvLocal.front().time;
    const auto placed = [&](const Vector3D& v, double up) {
        const double along = v.i * cx + v.j * cy;
        const double across = -v.i * cy + v.j * cx;
        return Vector3D(scale * (along * gx - across * gy), scale * (along * gy + across * gx), up);
    };

    double positionError = 0, velocityError = 0, accelerationError = 0;
    bool timesKept = true;
    for (std::size_t row = 0; row < csvLocal.size(); ++row)
    {
        const TrajectoryPoint& in = csvLocal[row];
        const TrajectoryPoint& out = fitted[row];
        const double progress = (in.time - csvLocal.front().time) / flightTime;
        const Vector3D position = placed(in.position, in.position.k + heightError * progress);
        const Vector3D velocity = placed(in.velocity, in.velocity.k + heightError / flightTime);
        const Vector3D acceleration = placed(in.acceleration, in.acceleration.k);
        positionError = std::max(positionError, length(subtract(gameFrame.toLocalPoint(out.position), position)));
        velocityError = std::max(velocityError, length(subtract(gameFrame.toLocalDirection(out.velocity), velocity)));
        accelerationError = std::max(accelerationError, length(subtract(gameFrame.toLocalDirection(out.acceleration), acceleration)));
        timesKept = timesKept && out.time == in.time;
    }
    check(timesKept, "every row keeps its CSV time");
    checkAtMost(positionError, 1e-6, "worst position error over all rows, m");
    checkAtMost(velocityError, 1e-9, "worst velocity error over all rows, m/s");
    checkAtMost(accelerationError, 1e-9, "worst acceleration error over all rows, m/s^2");
    std::printf("  (range x%.4f, height shifted %+.3f m by impact)\n", scale, heightError);
    return fitted;
}

std::vector<Vector3D> positionsOf(const std::vector<TrajectoryPoint>& rows)
{
    std::vector<Vector3D> positions;
    for (const auto& row : rows)
        positions.push_back(row.position);
    return positions;
}
}  // namespace

int main()
{
    const Frame singaporeA = wcsSite(1.35, 103.8, 15.0);
    const Frame singaporeB = wcsSite(1.30, 103.9, 10.0);
    const Frame oslo = wcsSite(59.9, 10.75, 20.0);
    const std::vector<TrajectoryPoint> kick = footballKick(30.0, 35.0, 25.0);

    {
        const Vector3D target = atHeading(250.0, 65.0, 3.0);
        const auto fitted = checkFit("WCS CSV (Singapore A) -> WCS points (Singapore B), football kick", kick, singaporeA, singaporeB, target);
        describeShape("new code", positionsOf(fitted), singaporeB, target);
        describeShape("original snippet", originalSnippet(toWorld(kick, singaporeA), singaporeB.origin, singaporeB.toWorldPoint(target)),
                      singaporeB, target);
    }
    {
        const Vector3D target = atHeading(0.0, 45.0, 0.0);
        const auto fitted = checkFit("WCS CSV (Singapore A) -> WCS points (Oslo), football kick due north", kick, singaporeA, oslo, target);
        describeShape("new code", positionsOf(fitted), oslo, target);
        describeShape("original snippet", originalSnippet(toWorld(kick, singaporeA), oslo.origin, oslo.toWorldPoint(target)), oslo, target);
    }
    {
        const std::vector<TrajectoryPoint> line = straightLine(atHeading(45.0, 3000.0, -50.0), 10.0);
        const Vector3D target = atHeading(0.0, 2000.0, 10.0);
        const auto fitted = checkFit("WCS CSV (Singapore A) -> WCS points (Singapore B), straight line", line, singaporeA, singaporeB, target);
        describeShape("new code", positionsOf(fitted), singaporeB, target);
        describeShape("original snippet", originalSnippet(toWorld(line, singaporeA), singaporeB.origin, singaporeB.toWorldPoint(target)),
                      singaporeB, target);
    }
    checkFit("local CSV (origin moved) -> WCS points (Singapore B)", footballKick(120.0, 35.0, 25.0),
             localOrigin(Vector3D(500, -200, 3)), singaporeB, atHeading(10.0, 40.0, -2.0));
    checkFit("local CSV -> local points", kick, localOrigin(Vector3D(0, 0, 0)), localOrigin(Vector3D(10, 20, 0)),
             Vector3D(30, -45, 1.5));
    checkFit("WCS CSV (Singapore A) -> local points", kick, singaporeA, localOrigin(Vector3D(-7, 4, 2)), atHeading(300.0, 52.0, 0.0));
    checkFit("WCS CSV climbing to 100 km, above the WCS band -> WCS points", straightLine(atHeading(80.0, 200000.0, 100000.0), 200.0),
             singaporeA, singaporeB, atHeading(10.0, 50000.0, 1000.0));

    const std::size_t top = static_cast<std::size_t>(
        std::max_element(kick.begin(), kick.end(), [](const auto& a, const auto& b) { return a.position.k < b.position.k; }) - kick.begin());
    const std::vector<TrajectoryPoint> halfArc = fromRow(kick, top);
    checkFit("half arc falling from its top -> WCS points, target above the CSV's own landing height", halfArc, singaporeA, singaporeB,
             atHeading(90.0, 30.0, -3.0));

    std::printf("\nLevel WCS line over the Earth, printed to 3 decimals, every 15 degrees, impact 2 m above and below\n");
    {
        int attempts = 0;
        int accepted = 0;
        double worstMiss = 0;
        std::string firstRefusal;
        for (int heading = 0; heading < 360; heading += 15)
        {
            const std::vector<TrajectoryPoint> line = levelLineOverTheEarth(1.35, 103.8, 15.0, heading, 200.0, 10.0);
            for (const double up : {2.0, -2.0})
            {
                ++attempts;
                const Vector3D end = singaporeB.toWorldPoint(atHeading(heading + 40.0, 150.0, up));
                try
                {
                    const std::vector<TrajectoryPoint> fitted = fitTrajectory(line, singaporeB.origin, end);
                    ++accepted;
                    worstMiss = std::max(worstMiss, length(subtract(fitted.back().position, end)));
                }
                catch (const std::invalid_argument& error)
                {
                    if (firstRefusal.empty())
                        firstRefusal = error.what();
                }
            }
        }
        check(accepted == attempts, std::to_string(accepted) + " of " + std::to_string(attempts) + " targets accepted" +
                                        (firstRefusal.empty() ? "" : "; first refusal: \"" + firstRefusal + "\""));
        checkAtMost(worstMiss, 1e-6, "worst impact miss, m");
    }

    std::printf("\nRejected inputs\n");
    const std::vector<TrajectoryPoint> csv = toWorld(kick, singaporeA);
    const Vector3D gameStart = singaporeB.origin;
    const Vector3D gameEnd = singaporeB.toWorldPoint(atHeading(250.0, 65.0, 3.0));
    expectRejected("one row", [&] { fitTrajectory({csv.front()}, gameStart, gameEnd); });
    expectRejected("time not increasing", [&] {
        auto broken = csv;
        broken[5].time = broken[4].time;
        fitTrajectory(broken, gameStart, gameEnd);
    });
    expectRejected("missing value", [&] {
        auto broken = csv;
        broken[8].velocity.j = std::numeric_limits<double>::quiet_NaN();
        fitTrajectory(broken, gameStart, gameEnd);
    });
    expectRejected("CSV mixing WCS and local rows", [&] {
        auto broken = csv;
        broken[12].position = Vector3D(100, 200, 3);
        fitTrajectory(broken, gameStart, gameEnd);
    });
    expectRejected("launch and impact points mixing frames", [&] { fitTrajectory(csv, gameStart, Vector3D(60, 20, 0)); });
    expectRejected("launch and impact the same place", [&] { fitTrajectory(csv, gameStart, gameStart); });
    expectRejected("CSV going straight up and down", [&] {
        fitTrajectory(toWorld(straightLine(Vector3D(0, 0, 0), 2.0), singaporeA), gameStart, gameEnd);
    });
    expectRejected("impact point so high the kick would arrive still climbing", [&] {
        fitTrajectory(csv, gameStart, singaporeB.toWorldPoint(atHeading(250.0, 65.0, 60.0)));
    });
    expectRejected("impact point so low the kick would be launched downwards", [&] {
        fitTrajectory(csv, gameStart, singaporeB.toWorldPoint(atHeading(250.0, 65.0, -80.0)));
    });
    expectRejected("impact point too high, with the impact row's velocity zeroed", [&] {
        auto zeroed = csv;
        zeroed.back().velocity = Vector3D(0, 0, 0);
        fitTrajectory(zeroed, gameStart, singaporeB.toWorldPoint(atHeading(250.0, 65.0, 60.0)));
    });
    expectRejected("half arc falling from its top, impact point too high", [&] {
        fitTrajectory(toWorld(halfArc, singaporeA), gameStart, singaporeB.toWorldPoint(atHeading(90.0, 30.0, 20.0)));
    });

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}

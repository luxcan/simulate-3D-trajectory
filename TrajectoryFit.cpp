#include "TrajectoryFit.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
// WGS-84, the ellipsoid that WCS (geocentric) coordinates are defined on.
constexpr double kSemiMajorAxis = 6378137.0;
constexpr double kFlattening = 1.0 / 298.257223563;
constexpr double kEccentricitySquared = kFlattening * (2.0 - kFlattening);

// A WCS launch point is about one Earth radius from the Earth's centre: this band runs from about
// 57 km below the polar radius to about 72 km above the equator. Local metres never come near it.
constexpr double kMinWcsRadius = 6.30e6;
constexpr double kMaxWcsRadius = 6.45e6;

// Horizontal travel shorter than this, in metres, has no usable direction.
constexpr double kMinHorizontalTravel = 1e-3;

// A CSV is an arc when some row rises above its launch-to-impact line by more than this share of
// its horizontal range. Rounding and the Earth's curve under a level line stay far below it.
constexpr double kMinArcHeight = 0.01;

// The only Vector3D constructor used. If yours differs, this is the one place to change.
Vector3D makeVector(double x, double y, double z)
{
    return Vector3D(x, y, z);
}

double dotProduct(const Vector3D& a, const Vector3D& b)
{
    return a.i * b.i + a.j * b.j + a.k * b.k;
}

double lengthOf(const Vector3D& v)
{
    return std::sqrt(dotProduct(v, v));
}

bool isFinite(const Vector3D& v)
{
    return std::isfinite(v.i) && std::isfinite(v.j) && std::isfinite(v.k);
}

// The launch point decides the frame. After that a point only disagrees if it is on the other side
// of the band's lower edge: a WCS trajectory may climb far above the band, local metres never reach it.
bool launchIsWcs(const Vector3D& launch)
{
    const double radius = lengthOf(launch);
    return radius >= kMinWcsRadius && radius <= kMaxWcsRadius;
}

bool fitsFrame(const Vector3D& position, bool wcs)
{
    return (lengthOf(position) >= kMinWcsRadius) == wcs;
}

std::string frameName(bool wcs)
{
    return wcs ? "WCS" : "local metres";
}

std::string distanceFromOrigin(const Vector3D& position)
{
    char text[96];
    std::snprintf(text, sizeof text, "%.0f m from (0, 0, 0); a WCS point is at least 6 300 000 m from it",
                  lengthOf(position));
    return text;
}

std::string metres(double value)
{
    char text[32];
    std::snprintf(text, sizeof text, "%+.1f m", value);
    return text;
}

// Latitude of the ellipsoid's normal, which is the local vertical. Geocentric latitude,
// atan2(z, hypot(x, y)), differs by up to 0.19 degrees and would tilt "up".
double geodeticLatitude(const Vector3D& wcs)
{
    const double horizontal = std::hypot(wcs.i, wcs.j);
    double latitude = std::atan2(wcs.k, horizontal * (1.0 - kEccentricitySquared));
    for (int iteration = 0; iteration < 20; ++iteration)
    {
        const double sinLatitude = std::sin(latitude);
        const double primeVerticalRadius =
            kSemiMajorAxis / std::sqrt(1.0 - kEccentricitySquared * sinLatitude * sinLatitude);
        const double next = std::atan2(wcs.k + kEccentricitySquared * primeVerticalRadius * sinLatitude, horizontal);
        const bool converged = std::abs(next - latitude) < 1e-14;
        latitude = next;
        if (converged)
            break;
    }
    return latitude;
}

// x east, y north, z up, centred on one point. For a WCS point this is the East-North-Up frame
// tangent to the ellipsoid there; for a local z-up point the axes are kept as they are.
class LocalFrame
{
public:
    static LocalFrame at(const Vector3D& origin, bool wcs)
    {
        if (!wcs)
            return LocalFrame(origin, makeVector(1, 0, 0), makeVector(0, 1, 0), makeVector(0, 0, 1));

        const double latitude = geodeticLatitude(origin);
        const double longitude = std::atan2(origin.j, origin.i);
        const double sinLatitude = std::sin(latitude);
        const double cosLatitude = std::cos(latitude);
        const double sinLongitude = std::sin(longitude);
        const double cosLongitude = std::cos(longitude);
        return LocalFrame(origin,
                          makeVector(-sinLongitude, cosLongitude, 0),
                          makeVector(-sinLatitude * cosLongitude, -sinLatitude * sinLongitude, cosLatitude),
                          makeVector(cosLatitude * cosLongitude, cosLatitude * sinLongitude, sinLatitude));
    }

    Vector3D pointToLocal(const Vector3D& world) const
    {
        return directionToLocal(makeVector(world.i - origin_.i, world.j - origin_.j, world.k - origin_.k));
    }

    // Velocity and acceleration turn with the axes but are never shifted by the origin.
    Vector3D directionToLocal(const Vector3D& world) const
    {
        return makeVector(dotProduct(world, east_), dotProduct(world, north_), dotProduct(world, up_));
    }

    Vector3D pointToWorld(const Vector3D& local) const
    {
        const Vector3D offset = directionToWorld(local);
        return makeVector(origin_.i + offset.i, origin_.j + offset.j, origin_.k + offset.k);
    }

    Vector3D directionToWorld(const Vector3D& local) const
    {
        return makeVector(east_.i * local.i + north_.i * local.j + up_.i * local.k,
                          east_.j * local.i + north_.j * local.j + up_.j * local.k,
                          east_.k * local.i + north_.k * local.j + up_.k * local.k);
    }

private:
    LocalFrame(const Vector3D& origin, const Vector3D& east, const Vector3D& north, const Vector3D& up)
        : origin_(origin), east_(east), north_(north), up_(up)
    {
    }

    Vector3D origin_;
    Vector3D east_;
    Vector3D north_;
    Vector3D up_;
};

void requireUsableRows(const std::vector<TrajectoryPoint>& csv)
{
    if (csv.size() < 2)
        throw std::invalid_argument("The CSV needs at least two rows, a launch and an impact.");

    for (std::size_t row = 0; row < csv.size(); ++row)
    {
        const TrajectoryPoint& point = csv[row];
        if (!std::isfinite(point.time) || !isFinite(point.position) || !isFinite(point.velocity) ||
            !isFinite(point.acceleration))
            throw std::invalid_argument("CSV data row " + std::to_string(row + 1) + " has a missing or non-numeric value.");

        if (row > 0 && !(point.time > csv[row - 1].time))
            throw std::invalid_argument("CSV data row " + std::to_string(row + 1) + ": time " +
                                        std::to_string(point.time) + " is not after the previous row's time " +
                                        std::to_string(csv[row - 1].time) + ".");
    }
}

// Every row must be in the launch row's frame: a file that mixes them is broken, not ambiguous.
bool csvIsWcs(const std::vector<TrajectoryPoint>& csv)
{
    const bool wcs = launchIsWcs(csv.front().position);
    for (std::size_t row = 1; row < csv.size(); ++row)
    {
        if (!fitsFrame(csv[row].position, wcs))
            throw std::invalid_argument("The CSV mixes frames: data row 1 is " + frameName(wcs) + " but data row " +
                                        std::to_string(row + 1) + " is " + frameName(!wcs) + " (" +
                                        distanceFromOrigin(csv[row].position) + ").");
    }
    return wcs;
}

bool gamePointsAreWcs(const Vector3D& gameStart, const Vector3D& gameEnd)
{
    if (!isFinite(gameStart) || !isFinite(gameEnd))
        throw std::invalid_argument("The launch or impact point has a missing or non-numeric value.");

    const bool wcs = launchIsWcs(gameStart);
    if (!fitsFrame(gameEnd, wcs))
        throw std::invalid_argument("The launch point is " + frameName(wcs) + " but the impact point is " +
                                    frameName(!wcs) + " (" + distanceFromOrigin(gameEnd) + ").");
    return wcs;
}

std::vector<TrajectoryPoint> toLocal(const std::vector<TrajectoryPoint>& csv, const LocalFrame& frame)
{
    std::vector<TrajectoryPoint> local = csv;  // keeps the times and anything else in the rows
    for (auto& row : local)
    {
        row.position = frame.pointToLocal(row.position);
        row.velocity = frame.directionToLocal(row.velocity);
        row.acceleration = frame.directionToLocal(row.acceleration);
    }
    return local;
}

// The height shift is linear in time, so it tilts the launch-to-impact line but keeps every row's
// height above that line. An arc therefore survives only while it still climbs out of the launch
// row (if it did) and still comes down onto the impact row (if it did); otherwise the ball would be
// launched downwards, or arrive still climbing, up through the target from below. Straight and
// one-way CSVs have no top to lose, so they take any target height. Heights are relative to the
// launch point, in local z-up metres.
void requireArcKeepsItsTop(const std::vector<TrajectoryPoint>& local, double horizontalRange, double targetHeight)
{
    const TrajectoryPoint& first = local.front();
    const TrajectoryPoint& second = local[1];
    const TrajectoryPoint& beforeLast = local[local.size() - 2];
    const TrajectoryPoint& last = local.back();
    const double flightTime = last.time - first.time;

    double highestAboveLine = 0;
    for (const auto& row : local)
        highestAboveLine =
            std::max(highestAboveLine, row.position.k - last.position.k * (row.time - first.time) / flightTime);
    if (highestAboveLine <= kMinArcHeight * horizontalRange)
        return;

    const double firstClimb = second.position.k - first.position.k;
    const double lastFall = beforeLast.position.k - last.position.k;
    const double lowestTarget = firstClimb > 0 ? last.position.k - firstClimb * flightTime / (second.time - first.time)
                                               : -std::numeric_limits<double>::infinity();
    const double highestTarget = lastFall > 0 ? last.position.k + lastFall * flightTime / (last.time - beforeLast.time)
                                              : std::numeric_limits<double>::infinity();
    if (targetHeight > lowestTarget && targetHeight < highestTarget)
        return;

    const std::string allowed = !std::isfinite(lowestTarget)    ? "below " + metres(highestTarget)
                                : !std::isfinite(highestTarget) ? "above " + metres(lowestTarget)
                                                                : "between " + metres(lowestTarget) + " and " + metres(highestTarget);
    throw std::invalid_argument("The impact point is " + metres(targetHeight) +
                                " in height from the launch point, but this arc only fits targets " + allowed +
                                ": beyond that the ball would be launched downwards or arrive still climbing.");
}
}  // namespace

std::vector<TrajectoryPoint> fitTrajectory(const std::vector<TrajectoryPoint>& csv,
                                           const Vector3D& gameStart,
                                           const Vector3D& gameEnd)
{
    requireUsableRows(csv);

    // Work where x and y are horizontal and z is up. Each side gets its own frame because "up" at
    // the place the CSV was recorded is not "up" at the game's launch point. In its own frame each
    // launch point is (0, 0, 0).
    const LocalFrame csvFrame = LocalFrame::at(csv.front().position, csvIsWcs(csv));
    const LocalFrame gameFrame = LocalFrame::at(gameStart, gamePointsAreWcs(gameStart, gameEnd));
    const std::vector<TrajectoryPoint> local = toLocal(csv, csvFrame);
    const Vector3D csvEnd = local.back().position;
    const Vector3D gameEndLocal = gameFrame.pointToLocal(gameEnd);

    const double csvRange = std::hypot(csvEnd.i, csvEnd.j);
    const double gameRange = std::hypot(gameEndLocal.i, gameEndLocal.j);
    if (csvRange < kMinHorizontalTravel)
        throw std::invalid_argument("The CSV trajectory lands where it was launched, so it has no direction to turn.");
    if (gameRange < kMinHorizontalTravel)
        throw std::invalid_argument("The launch and impact points are the same place.");
    requireArcKeepsItsTop(local, csvRange, gameEndLocal.k);

    // Turn the CSV to face the impact point and stretch it to the impact point's distance.
    const double rotationAngle = std::atan2(gameEndLocal.j, gameEndLocal.i) - std::atan2(csvEnd.j, csvEnd.i);
    const double scale = gameRange / csvRange;
    const double cosTurn = scale * std::cos(rotationAngle);
    const double sinTurn = scale * std::sin(rotationAngle);
    const auto turn = [cosTurn, sinTurn](const Vector3D& v, double z) {
        return makeVector(cosTurn * v.i - sinTurn * v.j, sinTurn * v.i + cosTurn * v.j, z);
    };

    // Shift the height linearly in time so the last row lands at the impact point's height. Linear
    // in time only adds a constant to Vz, so the vertical acceleration (gravity) is unchanged.
    const double launchTime = local.front().time;
    const double flightTime = local.back().time - launchTime;
    const double heightError = gameEndLocal.k - csvEnd.k;

    std::vector<TrajectoryPoint> fitted;
    fitted.reserve(local.size());
    for (const auto& point : local)
    {
        const double progress = (point.time - launchTime) / flightTime;  // 0 at launch, 1 at impact

        TrajectoryPoint out = point;
        out.position = gameFrame.pointToWorld(turn(point.position, point.position.k + heightError * progress));
        out.velocity = gameFrame.directionToWorld(turn(point.velocity, point.velocity.k + heightError / flightTime));
        out.acceleration = gameFrame.directionToWorld(turn(point.acceleration, point.acceleration.k));
        fitted.push_back(out);
    }
    return fitted;
}

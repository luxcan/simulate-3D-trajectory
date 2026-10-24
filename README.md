# Simulate 3D Trajectory

A simple C++ code required to place a recorded flight from a CSV file between any launch point and impact point you choose, to simulate a 3D trajectory based on CSV input.

Give it the CSV and the two points. It turns the recorded path to face the impact point, stretches it to the impact point's distance, and adjusts its height so the last row lands exactly on the impact point. You get back every row's position, velocity and acceleration, with the CSV's own times. When the impact point is the recorded flight's own distance and drop away from the launch point, the result is simply the recorded flight moved and turned: nothing is stretched or raised.

## Input

- **Columns:** `time, x, y, z, Vx, Vy, Vz, Ax, Ay, Az`. Times must increase (for example 0.1, 0.2, ...). The first row is the launch and the last row is the impact.
- **Coordinates:** either
  - **WCS**, the DIS / HLA RPR FOM World Coordinate System, which is the same as **ECEF**: geocentric metres on WGS-84, with z through the North Pole, or
  - **local metres with z up**.

  The frame is detected from the numbers: a WCS point is 6 300 000 to 6 450 000 m from (0, 0, 0). The CSV and the launch and impact points may each be in either frame, and the result comes back in the frame of the launch and impact points.

## Why not just rotate x and y?

In WCS, z is the Earth's axis, not "up". Near the equator (Singapore, for example) z points almost exactly north, so the x-y plane is a vertical plane, and turning the path in x-y tips it over instead of turning it. So the code:

1. Translates the CSV into an East-North-Up frame at its launch point, where x and y really are along the ground and z really is up.
2. Turns, stretches and adjusts the height there.
3. Translates the result back into WCS at the chosen launch point.

Velocity and acceleration are turned with the positions but never moved, so anything that reads them (orientation, interpolation, HLA dead reckoning) agrees with the path.

## Using it in your own project

Copy `TrajectoryFit.h` and `TrajectoryFit.cpp`. They need only:

- a `Vector3D` with public `i`, `j`, `k` and a `Vector3D(x, y, z)` constructor (if your constructor differs, change the one-line `makeVector` helper);
- a row struct with `time` (double) and `position`, `velocity`, `acceleration` (`Vector3D`). If you already have one, include it instead of the one in `TrajectoryFit.h`.

```cpp
try
{
    std::vector<TrajectoryPoint> path = fitTrajectory(csvRows, launchPoint, impactPoint);
    // play path back, or publish it
}
catch (const std::invalid_argument& error)
{
    // show error.what() to the user and keep the previous path
}
```

Bad input throws `std::invalid_argument` with a message meant for the user, for example:

- fewer than two rows, a missing or non-numeric value, or times that do not increase;
- a CSV that mixes WCS and local rows, or launch and impact points in different frames;
- launch and impact at the same place, or a CSV that lands where it was launched;
- an impact point too high or too low for an arc ("this arc only fits targets between -35.6 m and +29.4 m"), where the ball would otherwise arrive still climbing or be launched downwards. A straight-line model has no highest point to lose, so it accepts any height.

## Command-line app

`FitCsv` reads a CSV, fits it and writes the result as CSV, keeping the input's columns and their order:

```
FitCsv <input.csv> <output.csv> <launch x> <launch y> <launch z> <impact x> <impact y> <impact z>
```

Columns are found by their header names, in any order, ignoring case. Any other columns are copied through unchanged.

## Building, tests and samples

The scripts find the Visual C++ build tools with `vswhere` (see `tools\vcvars.cmd`). The sources are plain C++17, though so far they have only been built with Visual C++.

- `tests\build.cmd` builds and runs the tests: every row checked against geometry worked out independently, in both frames, including the original rotate-x-and-y approach for comparison and every rejected input.
- `samples\run.cmd` builds `FitCsv` and `MakeSamples`, writes three sample inputs and fits each from a launch point in Singapore to an impact point at the input's own distance and drop:
  - **A:** a football kick recorded in London, in WCS;
  - **B:** a straight-line model recorded in Sydney, in ECEF;
  - **C:** a football kick in local metres.

`standin\Vector3D.h` is a deliberately bare stand-in for your own `Vector3D`, so every build proves the fit needs nothing more from it.

## Limits

- **Stretching changes the speed.** A stretched flight keeps its hang time and its highest point, so it travels faster sideways. Stretching a drag-heavy flight a long way looks unphysical.
- **Short to medium range.** The work is done in a flat East-North-Up frame. Beyond about 20 km of range the middle of the flight starts to lose a few metres to the Earth's curve.
- **The first row must be the launch and the last row the impact.** Trim rows of the ball sitting still before launch or after landing.
- **Playback between rows is not included.** For smooth motion between the 0.1 s rows, keep the elapsed time in a `double`, find the two rows either side of it by time, and interpolate with cubic Hermite using the fitted position and velocity.

## Layout

| Path | What it is |
|---|---|
| `TrajectoryFit.h`, `TrajectoryFit.cpp` | The fit: the two files to copy into your project |
| `app/` | `FitCsv` and the CSV reader and writer |
| `tests/` | The tests and the geometry they share with the samples |
| `samples/` | The sample generator and the script that runs everything |
| `standin/` | The stand-in `Vector3D` |
| `tools/` | Visual C++ setup for the scripts |

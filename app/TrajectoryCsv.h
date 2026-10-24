#pragma once

#include <string>
#include <vector>

#include "TrajectoryFit.h"

// A trajectory CSV: a header row naming the columns, then one row per sample. The ten columns
// time, x, y, z, Vx, Vy, Vz, Ax, Ay, Az may come in any order (case and spaces are ignored);
// any other column is kept as text and written back unchanged.
struct TrajectoryCsv
{
    std::vector<std::string> header;              // as written in the file
    std::vector<std::vector<std::string>> cells;  // every data row, every column, as text
    std::vector<TrajectoryPoint> rows;            // the ten known columns, parsed
};

// The column order of the assignment's CSV.
extern const std::vector<std::string> kAssignmentHeader;

// Throws std::invalid_argument naming the line and column when the file cannot be read.
TrajectoryCsv readTrajectoryCsv(const std::string& path);

// Writes `rows` under `header`. Columns other than the ten known ones take their text from
// `otherCells` (one entry per row, laid out like `header`), so it must cover every row when there are any.
void writeTrajectoryCsv(const std::string& path,
                        const std::vector<std::string>& header,
                        const std::vector<TrajectoryPoint>& rows,
                        const std::vector<std::vector<std::string>>& otherCells = {});

#include "TrajectoryCsv.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <stdexcept>

namespace
{
enum Column
{
    kTime,
    kX,
    kY,
    kZ,
    kVx,
    kVy,
    kVz,
    kAx,
    kAy,
    kAz,
    kColumnCount
};

const std::array<const char*, kColumnCount> kColumnNames = {"time", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"};
constexpr int kOtherColumn = -1;

std::string trimmed(const std::string& text)
{
    const auto isSpace = [](unsigned char c) { return std::isspace(c) != 0; };
    const auto first = std::find_if_not(text.begin(), text.end(), isSpace);
    const auto last = std::find_if_not(text.rbegin(), text.rend(), isSpace).base();
    return first < last ? std::string(first, last) : std::string();
}

int columnOf(const std::string& name)
{
    std::string key = trimmed(name);
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    for (int column = 0; column < kColumnCount; ++column)
        if (key == kColumnNames[column])
            return column;
    return kOtherColumn;
}

std::vector<std::string> splitCommas(const std::string& line)
{
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (true)
    {
        const std::size_t comma = line.find(',', start);
        fields.push_back(trimmed(line.substr(start, comma - start)));
        if (comma == std::string::npos)
            return fields;
        start = comma + 1;
    }
}

double parseNumber(const std::string& text, std::size_t lineNumber, const std::string& column)
{
    std::size_t used = 0;
    double value = 0;
    try
    {
        value = std::stod(text, &used);
    }
    catch (const std::exception&)
    {
        used = 0;
    }
    if (text.empty() || used != text.size())
        throw std::invalid_argument("Line " + std::to_string(lineNumber) + ", column " + column + ": \"" + text +
                                    "\" is not a number.");
    return value;
}

double& fieldOf(TrajectoryPoint& point, int column)
{
    switch (column)
    {
    case kTime: return point.time;
    case kX: return point.position.i;
    case kY: return point.position.j;
    case kZ: return point.position.k;
    case kVx: return point.velocity.i;
    case kVy: return point.velocity.j;
    case kVz: return point.velocity.k;
    case kAx: return point.acceleration.i;
    case kAy: return point.acceleration.j;
    default: return point.acceleration.k;
    }
}

std::vector<int> columnsOf(const std::vector<std::string>& header)
{
    std::vector<int> columns;
    std::array<bool, kColumnCount> seen{};
    for (const auto& name : header)
    {
        const int column = columnOf(name);
        if (column != kOtherColumn)
        {
            if (seen[column])
                throw std::invalid_argument("The CSV header names column \"" + trimmed(name) + "\" twice.");
            seen[column] = true;
        }
        columns.push_back(column);
    }

    std::string missing;
    for (int column = 0; column < kColumnCount; ++column)
        if (!seen[column])
            missing += (missing.empty() ? "" : ", ") + std::string(kColumnNames[column]);
    if (!missing.empty())
        throw std::invalid_argument("The CSV header has no column for " + missing +
                                    ". The first line must name time, x, y, z, Vx, Vy, Vz, Ax, Ay, Az.");
    return columns;
}
}  // namespace

const std::vector<std::string> kAssignmentHeader = {"time", "x", "y", "z", "Ax", "Ay", "Az", "Vx", "Vy", "Vz"};

TrajectoryCsv readTrajectoryCsv(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::invalid_argument("Cannot open \"" + path + "\".");

    TrajectoryCsv csv;
    std::vector<int> columns;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(file, line))
    {
        ++lineNumber;
        if (trimmed(line).empty())
            continue;

        std::vector<std::string> fields = splitCommas(line);
        if (csv.header.empty())
        {
            columns = columnsOf(fields);
            csv.header = fields;
            continue;
        }
        if (fields.size() != csv.header.size())
            throw std::invalid_argument("Line " + std::to_string(lineNumber) + " has " + std::to_string(fields.size()) +
                                        " values but the header names " + std::to_string(csv.header.size()) + " columns.");

        TrajectoryPoint point{};
        for (std::size_t field = 0; field < fields.size(); ++field)
            if (columns[field] != kOtherColumn)
                fieldOf(point, columns[field]) = parseNumber(fields[field], lineNumber, csv.header[field]);
        csv.rows.push_back(point);
        csv.cells.push_back(std::move(fields));
    }
    if (csv.header.empty())
        throw std::invalid_argument("\"" + path + "\" is empty.");
    return csv;
}

void writeTrajectoryCsv(const std::string& path,
                        const std::vector<std::string>& header,
                        const std::vector<TrajectoryPoint>& rows,
                        const std::vector<std::vector<std::string>>& otherCells)
{
    const std::vector<int> columns = columnsOf(header);
    const bool hasOtherColumns = std::count(columns.begin(), columns.end(), kOtherColumn) > 0;
    if (hasOtherColumns && otherCells.size() != rows.size())
        throw std::invalid_argument("The header has extra columns but their values were not given for every row.");

    std::ofstream file(path);
    if (!file)
        throw std::invalid_argument("Cannot write \"" + path + "\".");

    for (std::size_t field = 0; field < header.size(); ++field)
        file << (field ? "," : "") << header[field];
    file << '\n';

    char text[64];
    for (std::size_t row = 0; row < rows.size(); ++row)
    {
        TrajectoryPoint point = rows[row];
        for (std::size_t field = 0; field < header.size(); ++field)
        {
            if (field)
                file << ',';
            if (columns[field] == kOtherColumn)
                file << otherCells[row][field];
            else
            {
                std::snprintf(text, sizeof text, columns[field] == kTime ? "%.10g" : "%.6f", fieldOf(point, columns[field]));
                file << text;
            }
        }
        file << '\n';
    }
    if (!file)
        throw std::invalid_argument("Writing \"" + path + "\" failed.");
}

#pragma once

// CSV and SVG output for a scenario run: the C++ stand-in for the thesis' xml2csv
// + pandas + matplotlib post-processing.

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "scenario.hpp"

namespace thesis {

struct Series {
    std::string name;
    std::vector<double> x;
    std::vector<double> y;
};

struct ChartOptions {
    std::string title;
    std::string xLabel;
    std::string yLabel;
    bool markers = false;  // draw points instead of a line (used for TTC)
    double yMin = std::numeric_limits<double>::quiet_NaN();  // NaN = fit to data
    double yMax = std::numeric_limits<double>::quiet_NaN();
    double xMin = std::numeric_limits<double>::quiet_NaN();
    double xMax = std::numeric_limits<double>::quiet_NaN();
    double referenceLine = std::numeric_limits<double>::quiet_NaN();  // e.g. TTC = 3 s
};

// A dependency-free line chart as an SVG file (NaN samples break the line).
void writeLineChartSvg(const std::string& path, const std::vector<Series>& series, const ChartOptions& options);

// Writes <dir>/<label>_*.csv and *.svg for one run, mirroring the thesis' outputs:
// gap error and its rate, the per-vehicle trace (position / velocity /
// acceleration / jerk), time to collision, time gap and the lane-change decision.
void writeRunReport(const std::string& dir, const std::string& label, const RunResult& result,
                    const ScenarioOptions& options);

// One line per model, the shape of Tables 1 and 2 in the paper.
struct SummaryRow {
    std::string model;
    double totalGapError;
    double totalTimeLost;
    double minTtc;
};
void writeSummary(const std::string& dir, const std::vector<SummaryRow>& rows);

}  // namespace thesis

#include "report.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace thesis {

namespace {

// Same palette order for vehicle 0 (lead) .. 4; chosen to stay distinguishable in both themes.
const char* const kPalette[] = {"#7f7f7f", "#1f77b4", "#d62728", "#2ca02c", "#9467bd"};

std::string formatNumber(double v) {
    std::ostringstream out;
    out << std::setprecision(6) << (v == 0 ? 0.0 : v);  // no "-0" tick labels
    return out.str();
}

std::string xmlEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            default: out += c;
        }
    }
    return out;
}

// "Nice" axis tick step covering `span` with about `count` ticks.
double niceStep(double span, int count) {
    if (span <= 0) return 1;
    const double raw = span / count;
    const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
    const double norm = raw / magnitude;
    double nice = 10;
    if (norm < 1.5) {
        nice = 1;
    } else if (norm < 3) {
        nice = 2;
    } else if (norm < 7) {
        nice = 5;
    }
    return nice * magnitude;
}

std::string csvCell(double v) { return std::isnan(v) ? "" : formatNumber(v); }

void writeColumnsCsv(const std::string& path, const std::vector<std::string>& names,
                     const std::vector<std::vector<double>>& columns) {
    std::ofstream out(path);
    for (size_t c = 0; c < names.size(); ++c) out << (c ? "," : "") << names[c];
    out << "\n";
    size_t rows = columns.empty() ? 0 : columns.front().size();
    for (const auto& col : columns) rows = std::min(rows, col.size());  // like the thesis' zip(*columns)
    for (size_t r = 0; r < rows; ++r) {
        for (size_t c = 0; c < columns.size(); ++c) out << (c ? "," : "") << csvCell(columns[c][r]);
        out << "\n";
    }
}

}  // namespace

void writeLineChartSvg(const std::string& path, const std::vector<Series>& series, const ChartOptions& o) {
    constexpr double kWidth = 760;
    constexpr double kHeight = 420;
    constexpr double kLeft = 70;
    constexpr double kRight = 20;
    constexpr double kTop = 40;
    constexpr double kBottom = 55;

    double xMin = o.xMin;
    double xMax = o.xMax;
    double yMin = o.yMin;
    double yMax = o.yMax;
    double dataXMin = 1e300;
    double dataXMax = -1e300;
    double dataYMin = 1e300;
    double dataYMax = -1e300;
    for (const Series& s : series) {
        for (size_t i = 0; i < s.x.size() && i < s.y.size(); ++i) {
            if (!std::isfinite(s.y[i])) continue;
            dataXMin = std::min(dataXMin, s.x[i]);
            dataXMax = std::max(dataXMax, s.x[i]);
            dataYMin = std::min(dataYMin, s.y[i]);
            dataYMax = std::max(dataYMax, s.y[i]);
        }
    }
    if (dataXMin > dataXMax) {  // no finite data at all
        dataXMin = 0;
        dataXMax = 1;
        dataYMin = 0;
        dataYMax = 1;
    }
    if (std::isnan(xMin)) xMin = dataXMin;
    if (std::isnan(xMax)) xMax = dataXMax;
    if (std::isnan(yMin)) yMin = dataYMin;
    if (std::isnan(yMax)) yMax = dataYMax;
    if (!std::isnan(o.referenceLine)) {
        yMin = std::min(yMin, o.referenceLine);
        yMax = std::max(yMax, o.referenceLine);
    }
    if (xMax <= xMin) xMax = xMin + 1;
    if (yMax <= yMin) yMax = yMin + 1;
    const double pad = (yMax - yMin) * 0.04;
    if (std::isnan(o.yMin)) yMin -= pad;
    if (std::isnan(o.yMax)) yMax += pad;

    const auto px = [&](double x) { return kLeft + (x - xMin) / (xMax - xMin) * (kWidth - kLeft - kRight); };
    const auto py = [&](double y) {
        return kHeight - kBottom - (y - yMin) / (yMax - yMin) * (kHeight - kTop - kBottom);
    };
    const char* const* const palette = kPalette;
    const size_t paletteSize = sizeof(kPalette) / sizeof(kPalette[0]);

    std::ofstream out(path);
    out << std::fixed << std::setprecision(2);
    out << R"(<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 )" << kWidth << " " << kHeight
        << R"(" font-family="sans-serif" font-size="12">)" << "\n";
    out << R"(<rect width="100%" height="100%" fill="#ffffff"/>)" << "\n";
    out << R"(<text x=")" << kWidth / 2 << R"(" y="22" text-anchor="middle" font-size="15">)" << xmlEscape(o.title)
        << "</text>\n";

    const double xStep = niceStep(xMax - xMin, 8);
    const double yStep = niceStep(yMax - yMin, 6);
    const double plotRight = kWidth - kRight;
    const double plotBottom = kHeight - kBottom;
    for (long i = static_cast<long>(std::ceil(xMin / xStep)); static_cast<double>(i) * xStep <= xMax + 1e-9; ++i) {
        const double x = static_cast<double>(i) * xStep;
        out << R"(<line x1=")" << px(x) << R"(" y1=")" << kTop << R"(" x2=")" << px(x) << R"(" y2=")" << plotBottom
            << R"(" stroke="#e6e6e6"/>)" << "\n"
            << R"(<text x=")" << px(x) << R"(" y=")" << plotBottom + 16 << R"(" text-anchor="middle">)"
            << formatNumber(x) << "</text>\n";
    }
    for (long i = static_cast<long>(std::ceil(yMin / yStep)); static_cast<double>(i) * yStep <= yMax + 1e-9; ++i) {
        const double y = static_cast<double>(i) * yStep;
        out << R"(<line x1=")" << kLeft << R"(" y1=")" << py(y) << R"(" x2=")" << plotRight << R"(" y2=")" << py(y)
            << R"(" stroke="#e6e6e6"/>)" << "\n"
            << R"(<text x=")" << kLeft - 6 << R"(" y=")" << py(y) + 4 << R"(" text-anchor="end">)" << formatNumber(y)
            << "</text>\n";
    }
    out << R"(<rect x=")" << kLeft << R"(" y=")" << kTop << R"(" width=")" << plotRight - kLeft << R"(" height=")"
        << plotBottom - kTop << R"(" fill="none" stroke="#444"/>)" << "\n";
    out << R"(<text x=")" << (kLeft + plotRight) / 2 << R"(" y=")" << kHeight - 12 << R"(" text-anchor="middle">)"
        << xmlEscape(o.xLabel) << "</text>\n";
    out << R"svg(<text transform="translate(16,)svg" << (kTop + plotBottom) / 2
        << R"svg() rotate(-90)" text-anchor="middle">)svg"
        << xmlEscape(o.yLabel) << "</text>\n";

    if (!std::isnan(o.referenceLine)) {
        out << R"(<line x1=")" << kLeft << R"(" y1=")" << py(o.referenceLine) << R"(" x2=")" << plotRight << R"(" y2=")"
            << py(o.referenceLine) << R"(" stroke="#d62728" stroke-dasharray="6 4"/>)" << "\n";
    }

    out << R"(<clipPath id="plot"><rect x=")" << kLeft << R"(" y=")" << kTop << R"(" width=")" << plotRight - kLeft
        << R"(" height=")" << plotBottom - kTop << R"("/></clipPath>)" << "\n" << R"svg(<g clip-path="url(#plot)">)svg" << "\n";
    for (size_t si = 0; si < series.size(); ++si) {
        const Series& s = series[si];
        const char* color = palette[si % paletteSize];
        if (o.markers) {
            for (size_t i = 0; i < s.x.size() && i < s.y.size(); ++i) {
                if (!std::isfinite(s.y[i])) continue;
                out << R"(<circle cx=")" << px(s.x[i]) << R"(" cy=")" << py(s.y[i]) << R"(" r="1.6" fill=")" << color
                    << R"("/>)" << "\n";
            }
            continue;
        }
        bool open = false;
        for (size_t i = 0; i < s.x.size() && i < s.y.size(); ++i) {
            if (!std::isfinite(s.y[i])) {
                if (open) out << "\"/>\n";
                open = false;
                continue;
            }
            if (!open) {
                out << R"(<polyline fill="none" stroke=")" << color << R"(" stroke-width="1.4" points=")";
                open = true;
            }
            out << px(s.x[i]) << "," << py(s.y[i]) << " ";
        }
        if (open) out << "\"/>\n";
    }
    out << "</g>\n";

    // legend, top-right inside the plot
    double ly = kTop + 14;
    for (size_t si = 0; si < series.size(); ++si) {
        const char* color = palette[si % paletteSize];
        out << R"(<line x1=")" << plotRight - 150 << R"(" y1=")" << ly - 4 << R"(" x2=")" << plotRight - 130
            << R"(" y2=")" << ly - 4 << R"(" stroke=")" << color << R"(" stroke-width="2"/>)" << "\n"
            << R"(<text x=")" << plotRight - 124 << R"(" y=")" << ly << R"(">)" << xmlEscape(series[si].name)
            << "</text>\n";
        ly += 16;
    }
    out << "</svg>\n";
}

void writeRunReport(const std::string& dir, const std::string& label, const RunResult& r, const ScenarioOptions& options) {
    std::filesystem::create_directories(dir);
    const std::string base = dir + "/" + label;

    const auto followerName = [&](int k) { return "Follower " + label + " Vehicle " + std::to_string(k); };

    // --- gap error / gap error rate, as the thesis' <Model>_Gap_error*.csv ---
    {
        std::vector<std::vector<double>> abs;
        for (const auto& series : r.gapError) {
            std::vector<double> a;
            for (double v : series) a.push_back(std::fabs(v));
            abs.push_back(a);
        }
        writeColumnsCsv(base + "_Gap_error.csv", {"Vehicle 1 Gap Error", "Vehicle 2 Gap Error", "Vehicle 3 Gap Error", "Vehicle 4 Gap Error"}, abs);
        writeColumnsCsv(base + "_Gap_error_rate.csv",
                        {"Vehicle 1 Gap Error Rate", "Vehicle 2 Gap Error Rate", "Vehicle 3 Gap Error Rate", "Vehicle 4 Gap Error Rate"},
                        {r.gapErrorRate[0], r.gapErrorRate[1], r.gapErrorRate[2], r.gapErrorRate[3]});

        std::vector<Series> plot;
        for (int k = 1; k <= 4; ++k) {
            Series s{followerName(k), {}, abs[k - 1]};
            for (size_t i = 0; i < s.y.size(); ++i) s.x.push_back(static_cast<double>(i));
            plot.push_back(std::move(s));
        }
        writeLineChartSvg(base + "_gap_error.svg", plot, {label + " |Gap Error| vs Time Step", "Time Step (s)", "Gap Error (s)"});
    }

    // --- per-vehicle traces: position, velocity, acceleration, jerk (the thesis' fcd-output plots) ---
    {
        std::ofstream out(base + "_trace.csv");
        out << "vehicle,time,x,speed,acceleration\n";
        for (int v = 0; v < 5; ++v) {
            const VehicleTrace& t = r.trace[v];
            for (size_t i = 0; i < t.time.size(); ++i) {
                out << v << "," << t.time[i] << "," << formatNumber(t.x[i]) << "," << formatNumber(t.speed[i]) << ","
                    << formatNumber(t.accel[i]) << "\n";
            }
        }
        std::vector<Series> position;
        std::vector<Series> velocity;
        std::vector<Series> acceleration;
        std::vector<Series> jerk;
        for (int v = 0; v < 5; ++v) {
            const VehicleTrace& t = r.trace[v];
            const std::string name = v == 0 ? "Lead Krauss Vehicle" : followerName(v);
            position.push_back({name, t.time, t.x});
            velocity.push_back({name, t.time, t.speed});
            acceleration.push_back({name, t.time, t.accel});
            // jerk: change in acceleration, starting at the third sample like the thesis
            Series j{name, {}, {}};
            for (size_t i = 2; i < t.accel.size(); ++i) {
                j.x.push_back(options.takeoverStep + static_cast<double>(j.y.size()));
                j.y.push_back(t.accel[i] - t.accel[i - 1]);
            }
            jerk.push_back(std::move(j));
        }
        writeLineChartSvg(base + "_position.svg", position, {label + " Vehicle Position vs Time Step", "Time Step (s)", "Position (m)", false, NAN, NAN, 550, 1000});
        writeLineChartSvg(base + "_velocity.svg", velocity, {label + " Vehicle Velocity vs Time Step", "Time Step (s)", "Velocity (m/s)", false, 22, 32});
        writeLineChartSvg(base + "_acceleration.svg", acceleration, {label + " Vehicle Acceleration vs Time Step", "Time Step (s)", "Acceleration (m/s^2)", false, NAN, NAN, NAN, NAN, 2.0});
        writeLineChartSvg(base + "_jerk.svg", jerk, {label + " Vehicle Jerk vs Time Step", "Time Step (s)", "Jerk (m/s^3)"});
    }

    // --- time to collision ---
    {
        std::vector<Series> ttc(4);
        for (int k = 0; k < 4; ++k) ttc[k].name = followerName(k + 1);
        for (size_t row = 0; row < r.ttc.size(); ++row) {
            for (int k = 0; k < 4; ++k) {
                ttc[k].x.push_back(options.takeoverStep + static_cast<double>(row));
                ttc[k].y.push_back(r.ttc[row][k]);
            }
        }
        writeLineChartSvg(base + "_ttc.svg", ttc, {label + " Vehicle TTC vs Time Step", "Time Step (s)", "TTC (s)", true, 0, 70, NAN, NAN, 3.0});
    }

    // --- time gap and lane-change decision (controlled models only) ---
    if (options.model == Model::Fis || options.model == Model::Gfs) {
        std::vector<Series> gap;
        for (int k = 1; k <= 4; ++k) {
            Series s{followerName(k), {}, r.gap[k - 1]};
            for (size_t i = 0; i < s.y.size(); ++i) s.x.push_back(options.takeoverStep + static_cast<double>(i));
            gap.push_back(std::move(s));
        }
        writeLineChartSvg(base + "_gap.svg", gap, {label + " Vehicle Gap vs Time Step", "Time Step (s)", "Gap (s)"});

        Series decision{"Decision to change lanes", {}, {}};
        for (size_t i = 0; i < r.laneChangeDecision.size(); ++i) {
            decision.x.push_back(static_cast<double>(i));
            decision.y.push_back(r.laneChangeDecision[i]);
        }
        writeLineChartSvg(base + "_lane_change_decision.svg", {decision}, {"Decision to Change Lanes vs Simulation Time", "Time Step", "Decision to Change Lanes"});
    }
}

void writeSummary(const std::string& dir, const std::vector<SummaryRow>& rows) {
    std::filesystem::create_directories(dir);
    std::ofstream out(dir + "/summary.csv");
    out << "model,total_gap_error_s,total_time_lost_s,min_ttc_s\n";
    for (const SummaryRow& r : rows) {
        out << r.model << "," << std::fixed << std::setprecision(2) << r.totalGapError << "," << r.totalTimeLost << "," << r.minTtc << "\n";
    }
}

}  // namespace thesis

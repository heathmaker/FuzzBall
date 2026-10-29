// Replication of the Master's thesis "Optimizing Platoon Time Gap Following on
// the Highway using Genetic Fuzzy Systems" (Palmer, Sathyan, Cohen): a platoon
// of four connected vehicles holds a 1 s time gap behind a human-driven
// (Krauss) lead on a 40-mile SUMO highway, controlled by hand-picked fuzzy
// systems (FIS), their genetically tuned counterparts (GFS), or SUMO's own
// Krauss / CACC models for comparison.
//
// The thesis was written in Python (scikit-fuzzy, EasyGA, TraCI, pandas,
// matplotlib); here the fuzzy systems are fuzzylib Mamdani engines, the SUMO
// link is libtraci, and results go out as CSV and SVG. See README.md in this
// folder for how it maps onto the thesis' four scenarios.
//
// Usage: thesis_run [--scenario 1-4] [--model all|krauss|cacc|fis|gfs]
//                [--scenario-dir DIR] [--out DIR] [--gui] [--seed N]

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "report.hpp"
#include "scenario.hpp"

namespace {

void usage() {
    std::cout << "Usage: thesis_run [--scenario 1|2|3|4] [--model all|krauss|cacc|fis|gfs]\n"
                 "                      [--scenario-dir DIR] [--out DIR] [--gui] [--seed N]\n\n"
                 "  scenario 1  single lane, cruising          scenario 3  two lanes, cruising\n"
                 "  scenario 2  single lane, lead slows down   scenario 4  two lanes, lead slows down\n";
}

}  // namespace

int main(int argc, char** argv) {
    thesis::ScenarioOptions base;
    base.scenarioDir = "tools/thesis_platoon";
    int scenario = 3;
    std::string modelArg = "all";
    std::string outDir;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::cerr << arg << " needs a value\n";
                std::exit(2);
            }
            return argv[++i];
        };
        if (arg == "--scenario") scenario = std::stoi(next());
        else if (arg == "--model") modelArg = next();
        else if (arg == "--scenario-dir") base.scenarioDir = next();
        else if (arg == "--out") outDir = next();
        else if (arg == "--seed") base.seed = static_cast<unsigned>(std::stoul(next()));
        else if (arg == "--gui") base.gui = true;
        else if (arg == "--help" || arg == "-h") { usage(); return 0; }
        else { std::cerr << "unknown option " << arg << "\n"; usage(); return 2; }
    }
    if (scenario < 1 || scenario > 4) { usage(); return 2; }
    base.singleLane = scenario <= 2;
    base.slowDown = scenario % 2 == 0;
    if (outDir.empty()) outDir = "thesis_results/scenario" + std::to_string(scenario);

    std::vector<thesis::Model> models;
    if (modelArg == "all") models = {thesis::Model::Krauss, thesis::Model::Cacc, thesis::Model::Fis, thesis::Model::Gfs};
    else if (modelArg == "krauss") models = {thesis::Model::Krauss};
    else if (modelArg == "cacc") models = {thesis::Model::Cacc};
    else if (modelArg == "fis") models = {thesis::Model::Fis};
    else if (modelArg == "gfs") models = {thesis::Model::Gfs};
    else { usage(); return 2; }

    std::cout << "Thesis scenario " << scenario << ": " << (base.singleLane ? "single-lane" : "two-lane")
              << " highway, " << (base.slowDown ? "lead slows down" : "cruising at 70 mph") << "\n\n";

    std::vector<thesis::SummaryRow> rows;
    for (thesis::Model model : models) {
        thesis::ScenarioOptions options = base;
        options.model = model;
        const thesis::FisSet fis = model == thesis::Model::Gfs ? thesis::FisSet::tuned() : thesis::FisSet::handPicked();

        std::cout << "Running " << thesis::modelName(model) << " ..." << std::flush;
        thesis::RunResult result;
        try {
            result = thesis::runScenario(options, fis);
        } catch (const std::exception& e) {
            std::cout << " failed: " << e.what() << "\n";
            return 1;
        }
        thesis::SummaryRow row{thesis::modelName(model), result.fitness(options.takeoverStep, options.endStep),
                               result.totalTimeLost(), result.minTtc(options.takeoverStep, options.endStep)};
        std::cout << " done\n";
        thesis::writeRunReport(outDir, thesis::modelName(model), result, options);
        rows.push_back(row);
    }
    thesis::writeSummary(outDir, rows);

    std::cout << "\n  Driving model   Total gap error (s)   Total time lost (s)   Min TTC (s)\n";
    for (const auto& r : rows) {
        std::cout << "  " << std::left << std::setw(14) << r.model << std::right << std::fixed << std::setprecision(2)
                  << std::setw(20) << r.totalGapError << std::setw(22) << r.totalTimeLost << std::setw(14) << r.minTtc
                  << "\n";
    }
    std::cout << "\nCSV and SVG plots written to " << outDir << "/\n";
    return 0;
}

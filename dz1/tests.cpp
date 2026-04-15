#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#define main dz1_program_main
#include "main.cpp"
#undef main

namespace fs = std::filesystem;

struct TestFailure : std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct TestCase {
    std::string name;
    std::function<void()> body;
};

std::vector<TestCase>& allTests() {
    static std::vector<TestCase> tests;
    return tests;
}

struct TestRegistrar {
    TestRegistrar(const std::string& name, std::function<void()> body) {
        allTests().push_back({name, std::move(body)});
    }
};

#define TEST(name)                              \
    void name();                                \
    TestRegistrar reg_##name(#name, name);      \
    void name()

void fail(const std::string& message) {
    throw TestFailure(message);
}

void requireTrue(bool condition, const std::string& message) {
    if (!condition) {
        fail(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::fabs(actual - expected) > tolerance) {
        std::ostringstream oss;
        oss << message << " (actual=" << std::setprecision(15) << actual
            << ", expected=" << expected << ", tol=" << tolerance << ")";
        fail(oss.str());
    }
}

void requireFinite(double value, const std::string& message) {
    if (!std::isfinite(value)) {
        std::ostringstream oss;
        oss << message << " (value=" << value << ")";
        fail(oss.str());
    }
}

std::string trim(const std::string& text) {
    size_t start = 0;
    while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) {
        ++start;
    }

    size_t end = text.size();
    while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
        --end;
    }

    return text.substr(start, end - start);
}

struct FlightSolution {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double t = 0.0;
    double h = std::numeric_limits<double>::quiet_NaN();
    bool ok = false;
};

FlightSolution solveFlight(double z0, double m, double d, double l, double v0) {
    FlightSolution solution;
    solution.a = d * G * m - 2.0 * d * d * l * v0;
    solution.b = -3.0 * G * m * m + 3.0 * d * l * m * v0;
    solution.c = 6.0 * m * m * z0;
    solution.t = solveCardanoTime(solution.a, solution.b, solution.c, solution.ok);

    if (solution.ok && m != 0.0) {
        solution.h = calcHorizontalDistance(solution.t, m, d, l, v0);
    }

    return solution;
}

double residual(double a, double b, double c, double t) {
    return a * t * t * t + b * t * t + c;
}

struct BoundaryCoefficients {
    double a;
    double b;
    double c;
    double acosArg;
};

BoundaryCoefficients makeBoundaryCoefficients(double targetAcosArg) {
    const double a = 1.0;
    const double b = std::sqrt(3.0);
    const double p = -(b * b) / 3.0;
    const double scale = std::sqrt(-3.0 / p);
    const double q = targetAcosArg * (2.0 * p) / (3.0 * scale);
    const double c = q - (2.0 * b * b * b) / 27.0;
    const double realAcosArg = (3.0 * q / (2.0 * p)) * scale;
    return {a, b, c, realAcosArg};
}

std::string readWholeFile(const fs::path& path) {
    std::ifstream input(path);
    std::ostringstream buffer;
    buffer << input.rdbuf();
    return buffer.str();
}

void writeWholeFile(const fs::path& path, const std::string& content) {
    std::ofstream output(path);
    output << content;
}

std::vector<std::string> splitNonEmptyLines(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream input(text);
    std::string line;
    while (std::getline(input, line)) {
        if (!trim(line).empty()) {
            lines.push_back(line);
        }
    }
    return lines;
}

std::map<std::string, std::string> parseReportValues(const std::string& report) {
    std::map<std::string, std::string> values;
    std::istringstream input(report);
    std::string line;
    while (std::getline(input, line)) {
        const size_t pos = line.find('=');
        if (pos == std::string::npos) {
            continue;
        }

        values[trim(line.substr(0, pos))] = trim(line.substr(pos + 1));
    }
    return values;
}

double parseDouble(const std::map<std::string, std::string>& values, const std::string& key) {
    auto it = values.find(key);
    requireTrue(it != values.end(), "Missing key in report: " + key);
    return std::stod(it->second);
}

std::string parseString(const std::map<std::string, std::string>& values, const std::string& key) {
    auto it = values.find(key);
    requireTrue(it != values.end(), "Missing key in report: " + key);
    return it->second;
}

struct ScopedCurrentPath {
    explicit ScopedCurrentPath(const fs::path& target) : old(fs::current_path()) {
        fs::current_path(target);
    }

    ~ScopedCurrentPath() {
        fs::current_path(old);
    }

    fs::path old;
};

struct TempDir {
    TempDir() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / ("dz1-tests-" + std::to_string(stamp));
        fs::create_directories(path);
    }

    ~TempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }

    fs::path path;
};

struct ProgramRun {
    int exitCode = 0;
    std::string console;
    std::string output;
    std::string report;
    std::string svg;
    bool svgExists = false;
};

ProgramRun runProgramWithFiles(const std::string& inputContent, const std::string& ammoContent) {
    TempDir temp;
    writeWholeFile(temp.path / "input.txt", inputContent);
    writeWholeFile(temp.path / "ammo_data.txt", ammoContent);

    ProgramRun run;
    {
        ScopedCurrentPath cwd(temp.path);
        std::ostringstream captured;
        std::streambuf* oldBuf = std::cout.rdbuf(captured.rdbuf());
        run.exitCode = dz1_program_main();
        std::cout.rdbuf(oldBuf);
        run.console = captured.str();
    }

    if (fs::exists(temp.path / "output.txt")) {
        run.output = readWholeFile(temp.path / "output.txt");
    }
    if (fs::exists(temp.path / "report.txt")) {
        run.report = readWholeFile(temp.path / "report.txt");
    }
    run.svgExists = fs::exists(temp.path / "trajectories.svg");
    if (run.svgExists) {
        run.svg = readWholeFile(temp.path / "trajectories.svg");
    }

    return run;
}

TEST(SolveCardanoReturnsPositiveRootForSimpleSpecialCase) {
    bool ok = false;
    const double t = solveCardanoTime(1.0, -4.0, 0.0, ok);

    requireTrue(ok, "Solver should find a positive root for t^2 * (t - 4) = 0");
    requireNear(t, 4.0, 1e-9, "Positive root should be 4");
    requireNear(residual(1.0, -4.0, 0.0, t), 0.0, 1e-6, "Returned root must satisfy the equation");
}

TEST(SolveCardanoRejectsCaseWithoutPositiveRoot) {
    bool ok = true;
    const double t = solveCardanoTime(1.0, 1.0, 1.0, ok);

    requireTrue(!ok || t <= 0.0, "Solver must reject a case without positive roots");
}

TEST(SolveCardanoAcceptsAcosArgJustBelowUpperBoundary) {
    const auto coeffs = makeBoundaryCoefficients(1.0 - 1e-12);
    bool ok = false;
    const double t = solveCardanoTime(coeffs.a, coeffs.b, coeffs.c, ok);

    requireNear(coeffs.acosArg, 1.0 - 1e-12, 1e-15, "Boundary helper must produce the requested acosArg");
    requireTrue(ok, "Solver should accept acosArg just below 1");
    requireNear(residual(coeffs.a, coeffs.b, coeffs.c, t), 0.0, 1e-6, "Accepted root should satisfy the equation");
}

TEST(SolveCardanoRejectsAcosArgJustAboveUpperBoundary) {
    const auto coeffs = makeBoundaryCoefficients(1.0 + 1e-12);
    bool ok = true;
    const double t = solveCardanoTime(coeffs.a, coeffs.b, coeffs.c, ok);

    (void)t;
    requireNear(coeffs.acosArg, 1.0 + 1e-12, 1e-15, "Boundary helper must produce the requested acosArg");
    requireTrue(!ok, "Solver should reject acosArg slightly above 1 without clamping");
}

TEST(SolveCardanoAcceptsAcosArgJustAboveLowerBoundary) {
    const auto coeffs = makeBoundaryCoefficients(-1.0 + 1e-12);
    bool ok = false;
    const double t = solveCardanoTime(coeffs.a, coeffs.b, coeffs.c, ok);

    requireNear(coeffs.acosArg, -1.0 + 1e-12, 1e-15, "Boundary helper must produce the requested acosArg");
    requireTrue(ok, "Solver should accept acosArg just above -1");
    requireNear(residual(coeffs.a, coeffs.b, coeffs.c, t), 0.0, 1e-6, "Accepted root should satisfy the equation");
}

TEST(SolveCardanoRejectsAcosArgJustBelowLowerBoundary) {
    const auto coeffs = makeBoundaryCoefficients(-1.0 - 1e-12);
    bool ok = true;
    const double t = solveCardanoTime(coeffs.a, coeffs.b, coeffs.c, ok);

    (void)t;
    requireNear(coeffs.acosArg, -1.0 - 1e-12, 1e-15, "Boundary helper must produce the requested acosArg");
    requireTrue(!ok, "Solver should reject acosArg slightly below -1 without clamping");
}

TEST(SolveCardanoRejectsNearZeroA) {
    bool ok = true;
    const double t = solveCardanoTime(1e-12, 1.0, 1.0, ok);

    requireTrue(!ok, "Near-zero a must be rejected");
    requireFinite(t, "Returned value must stay finite");
}

TEST(ZeroDragBallisticLimitIsCurrentlyRejected) {
    const auto flight = solveFlight(100.0, 1.0, 0.0, 0.0, 20.0);
    requireTrue(!flight.ok, "Current implementation rejects the zero-drag ballistic limit");
}

TEST(SmallDragApproximatesBallisticLimit) {
    const double z0 = 100.0;
    const double m = 1.0;
    const double d = 1e-6;
    const double l = 0.0;
    const double v0 = 20.0;
    const auto flight = solveFlight(z0, m, d, l, v0);
    const double expectedT = std::sqrt(2.0 * z0 / G);
    const double expectedH = v0 * expectedT;

    requireTrue(flight.ok, "Small drag case should be solvable");
    requireNear(flight.t, expectedT, 0.05, "Time should stay close to the ballistic limit");
    requireNear(flight.h, expectedH, 1.0, "Distance should stay close to the ballistic limit");
}

TEST(FlightTimeGrowsWithHeight) {
    const auto low = solveFlight(10.0, 10.0, 0.1, 0.2, 30.0);
    const auto mid = solveFlight(50.0, 10.0, 0.1, 0.2, 30.0);
    const auto high = solveFlight(100.0, 10.0, 0.1, 0.2, 30.0);

    requireTrue(low.ok && mid.ok && high.ok, "All height cases must be solvable");
    requireTrue(low.t < mid.t && mid.t < high.t, "Flight time must grow with initial height");
}

TEST(HorizontalDistanceGrowsWithSpeed) {
    const auto slow = solveFlight(100.0, 10.0, 0.05, 0.2, 10.0);
    const auto medium = solveFlight(100.0, 10.0, 0.05, 0.2, 20.0);
    const auto fast = solveFlight(100.0, 10.0, 0.05, 0.2, 40.0);

    requireTrue(slow.ok && medium.ok && fast.ok, "All speed cases must be solvable");
    requireTrue(slow.h < medium.h && medium.h < fast.h, "Horizontal distance must grow with speed");
}

TEST(HorizontalDistanceShrinksWithDrag) {
    const auto weak = solveFlight(100.0, 10.0, 0.01, 0.2, 40.0);
    const auto medium = solveFlight(100.0, 10.0, 0.05, 0.2, 40.0);
    const auto strong = solveFlight(100.0, 10.0, 0.2, 0.2, 40.0);

    requireTrue(weak.ok && medium.ok && strong.ok, "All drag cases must be solvable");
    requireTrue(weak.h > medium.h && medium.h > strong.h, "Horizontal distance must shrink as drag grows");
}

TEST(AltitudeIsNearZeroAtSolvedTime) {
    const auto flight = solveFlight(120.0, 1.2, 0.1, 0.0, 24.0);
    requireTrue(flight.ok, "Flight must be solvable");

    const double z = calcAltitude(flight.t, 120.0, flight.a, flight.b, 1.2);
    requireNear(z, 0.0, 1e-6, "Altitude at hit time must be near zero");
}

TEST(AltitudeAtZeroMatchesInitialHeight) {
    const double z0 = 75.0;
    const auto flight = solveFlight(z0, 1.2, 0.1, 0.0, 24.0);
    const double z = calcAltitude(0.0, z0, flight.a, flight.b, 1.2);

    requireNear(z, z0, 1e-12, "Altitude at t=0 must equal the initial height");
}

TEST(AltitudeDoesNotRiseAtStartForSimpleDrop) {
    const auto flight = solveFlight(100.0, 1.0, 0.05, 0.0, 20.0);
    requireTrue(flight.ok, "Reference flight must be solvable");

    double previous = calcAltitude(0.0, 100.0, flight.a, flight.b, 1.0);
    for (int step = 1; step <= 10; ++step) {
        const double t = 0.05 * static_cast<double>(step);
        const double z = calcAltitude(t, 100.0, flight.a, flight.b, 1.0);
        requireTrue(z <= previous + 1e-8, "Altitude should not climb above the previous sample in a simple drop");
        previous = z;
    }
}

TEST(ZeroSpeedKeepsHorizontalDistanceFiniteAndNearZero) {
    const auto flight = solveFlight(100.0, 2.0, 0.1, 0.0, 0.0);
    requireTrue(flight.ok, "Zero-speed case should still produce a fall time");
    requireFinite(flight.h, "Horizontal distance must stay finite");
    requireNear(flight.h, 0.0, 1e-9, "Without horizontal speed and lift, distance should stay near zero");
}

TEST(LargeNumbersStayFinite) {
    const auto flight = solveFlight(1e6, 1e4, 0.02, 0.1, 1e5);
    requireTrue(flight.ok, "Large-number case should stay solvable");
    requireFinite(flight.t, "Time must stay finite for large inputs");
    requireFinite(flight.h, "Distance must stay finite for large inputs");
}

TEST(ProgramSingleCaseProducesAllOutputs) {
    const std::string ammo = "TEST 0.35 0.07 0.0\n";
    const std::string input =
        "100\n50\n120\n160\n90\n24\n15\nTEST\n";

    const auto run = runProgramWithFiles(input, ammo);
    requireTrue(run.exitCode == 0, "Program must finish successfully for one valid case");
    requireTrue(splitNonEmptyLines(run.output).size() == 1, "output.txt must contain exactly one answer line");
    requireTrue(run.report.find("Case 1") != std::string::npos, "report.txt must contain Case 1");
    requireTrue(run.svgExists, "trajectories.svg must be created");
}

TEST(ProgramHandlesMultipleCases) {
    const std::string ammo = "TEST 0.35 0.07 0.0\nALT 0.60 0.10 0.0\n";
    const std::string input =
        "100\n50\n120\n160\n90\n24\n15\nTEST\n"
        "100\n50\n120\n210\n120\n24\n15\nALT\n";

    const auto run = runProgramWithFiles(input, ammo);
    requireTrue(run.exitCode == 0, "Program must finish successfully for multiple valid cases");
    requireTrue(splitNonEmptyLines(run.output).size() == 2, "output.txt must contain one line per case");
    requireTrue(run.report.find("Case 2") != std::string::npos, "report.txt must contain the second case");
    requireTrue(run.svg.find("Case 2") != std::string::npos, "SVG must contain a panel for the second case");
}

TEST(ProgramReportsUnknownAmmo) {
    const std::string ammo = "KNOWN 0.35 0.07 0.0\n";
    const std::string input =
        "100\n50\n120\n160\n90\n24\n15\nUNKNOWN\n";

    const auto run = runProgramWithFiles(input, ammo);
    requireTrue(run.exitCode == 1, "Unknown ammo must terminate with an error");
    requireTrue(run.output.find("unknown ammo type") != std::string::npos, "output.txt must contain the ammo error");
    requireTrue(run.report.find("unknown ammo type") != std::string::npos, "report.txt must contain the ammo error");
}

TEST(ProgramRejectsEmptyInput) {
    const std::string ammo = "TEST 0.35 0.07 0.0\n";
    const auto run = runProgramWithFiles("", ammo);

    requireTrue(run.exitCode == 1, "Empty input must terminate with an error");
    requireTrue(run.output.find("input file is empty or invalid") != std::string::npos, "output.txt must contain the empty-input error");
    requireTrue(run.report.find("input file is empty or invalid") != std::string::npos, "report.txt must contain the empty-input error");
}

TEST(ProgramRejectsZeroAttackDistance) {
    const std::string ammo = "TEST 0.35 0.07 0.0\n";
    const std::string input =
        "100\n50\n120\n100\n50\n24\n15\nTEST\n";

    const auto run = runProgramWithFiles(input, ammo);
    requireTrue(run.exitCode == 1, "Start equal to target must terminate with an error");
    requireTrue(run.output.find("invalid distance to target") != std::string::npos, "output.txt must contain the distance error");
}

TEST(GeometryWithoutManeuverKeepsIntermediateAtStart) {
    const auto reference = solveFlight(100.0, 1.0, 0.05, 0.0, 20.0);
    requireTrue(reference.ok, "Reference flight must be solvable");

    const double xd = 0.0;
    const double yd = 0.0;
    const double targetX = reference.h + 30.0;
    const double targetY = 0.0;
    const std::string ammo = "TEST 1.0 0.05 0.0\n";
    std::ostringstream input;
    input << xd << '\n' << yd << '\n' << 100.0 << '\n'
          << targetX << '\n' << targetY << '\n'
          << 20.0 << '\n' << 10.0 << '\n' << "TEST\n";

    const auto run = runProgramWithFiles(input.str(), ammo);
    requireTrue(run.exitCode == 0, "No-maneuver geometry case must succeed");

    const auto values = parseReportValues(run.report);
    requireTrue(parseString(values, "needManeuver") == "NO", "Maneuver must not be needed");
    requireNear(parseDouble(values, "intermediateX"), xd, 1e-6, "Intermediate X must stay at the start");
    requireNear(parseDouble(values, "intermediateY"), yd, 1e-6, "Intermediate Y must stay at the start");
}

TEST(GeometryWithManeuverKeepsIntermediateOnDroneTargetLine) {
    const auto reference = solveFlight(100.0, 1.0, 0.05, 0.0, 20.0);
    requireTrue(reference.ok, "Reference flight must be solvable");

    const double xd = 10.0;
    const double yd = 20.0;
    const double targetX = 25.0;
    const double targetY = 45.0;
    const std::string ammo = "TEST 1.0 0.05 0.0\n";
    std::ostringstream input;
    input << xd << '\n' << yd << '\n' << 100.0 << '\n'
          << targetX << '\n' << targetY << '\n'
          << 20.0 << '\n' << 10.0 << '\n' << "TEST\n";

    const auto run = runProgramWithFiles(input.str(), ammo);
    requireTrue(run.exitCode == 0, "Maneuver geometry case must succeed");

    const auto values = parseReportValues(run.report);
    requireTrue(parseString(values, "needManeuver") == "YES", "Maneuver must be needed");

    const double ix = parseDouble(values, "intermediateX");
    const double iy = parseDouble(values, "intermediateY");
    const double cross = (targetX - xd) * (iy - yd) - (targetY - yd) * (ix - xd);
    requireNear(cross, 0.0, 1e-2, "Intermediate point must stay on the drone-target line");
}

TEST(GeometryDropPointStaysOnAttackLine) {
    const double targetX = 200.0;
    const double targetY = 20.0;
    const std::string ammo = "TEST 1.0 0.05 0.0\n";
    const std::string input =
        "10\n20\n100\n200\n20\n20\n0\nTEST\n";

    const auto run = runProgramWithFiles(input, ammo);
    requireTrue(run.exitCode == 0, "Drop-point geometry case must succeed");

    const auto values = parseReportValues(run.report);
    const double ix = parseDouble(values, "intermediateX");
    const double iy = parseDouble(values, "intermediateY");
    const double fx = parseDouble(values, "fireX");
    const double fy = parseDouble(values, "fireY");
    const double cross = (targetX - ix) * (fy - iy) - (targetY - iy) * (fx - ix);
    requireNear(cross, 0.0, 1e-6, "Drop point must stay on the attack line");
}

TEST(GeometryExactRangeGivesZeroRatioAndDropAtAttackStart) {
    const auto reference = solveFlight(100.0, 1.0, 0.05, 0.0, 20.0);
    requireTrue(reference.ok, "Reference flight must be solvable");

    const std::string ammo = "TEST 1.0 0.05 0.0\n";
    std::ostringstream input;
    input << 0.0 << '\n' << 0.0 << '\n' << 100.0 << '\n'
          << reference.h << '\n' << 0.0 << '\n'
          << 20.0 << '\n' << 0.0 << '\n' << "TEST\n";

    const auto run = runProgramWithFiles(input.str(), ammo);
    requireTrue(run.exitCode == 0, "Exact-range geometry case must succeed");

    const auto values = parseReportValues(run.report);
    const double intermediateX = parseDouble(values, "intermediateX");
    const double intermediateY = parseDouble(values, "intermediateY");
    const double fireX = parseDouble(values, "fireX");
    const double fireY = parseDouble(values, "fireY");

    requireNear(parseDouble(values, "ratio"), 0.0, 1e-4, "ratio must be zero when h == attackDistance");
    requireNear(fireX, intermediateX, 1e-4, "Drop point X must match attack start");
    requireNear(fireY, intermediateY, 1e-4, "Drop point Y must match attack start");
}

TEST(GeometryLongerRangeKeepsRatioInsideUnitInterval) {
    const auto reference = solveFlight(100.0, 1.0, 0.05, 0.0, 20.0);
    requireTrue(reference.ok, "Reference flight must be solvable");

    const std::string ammo = "TEST 1.0 0.05 0.0\n";
    std::ostringstream input;
    input << 0.0 << '\n' << 0.0 << '\n' << 100.0 << '\n'
          << (reference.h + 15.0) << '\n' << 0.0 << '\n'
          << 20.0 << '\n' << 0.0 << '\n' << "TEST\n";

    const auto run = runProgramWithFiles(input.str(), ammo);
    requireTrue(run.exitCode == 0, "Longer-range geometry case must succeed");

    const auto values = parseReportValues(run.report);
    const double ratio = parseDouble(values, "ratio");
    requireTrue(ratio >= -1e-9 && ratio <= 1.0 + 1e-9, "ratio must stay inside [0, 1] for a normal case");
}

int main() {
    int passed = 0;
    int failed = 0;

    for (const auto& test : allTests()) {
        try {
            test.body();
            ++passed;
            std::cout << "[PASS] " << test.name << '\n';
        } catch (const std::exception& ex) {
            ++failed;
            std::cout << "[FAIL] " << test.name << ": " << ex.what() << '\n';
        } catch (...) {
            ++failed;
            std::cout << "[FAIL] " << test.name << ": unknown error\n";
        }
    }

    std::cout << "\nSummary: " << passed << " passed, " << failed << " failed\n";
    return failed == 0 ? 0 : 1;
}

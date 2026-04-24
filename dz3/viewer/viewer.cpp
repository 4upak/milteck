// 3D-візуалізатор симуляції дрона з dz3.
// Читає готові дані з файлів симуляції і малює сцену через raylib.

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "../json.hpp"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

using nlohmann::json;

struct Coord {
    float x = 0.0f;
    float y = 0.0f;
};

struct StepData {
    Coord pos;
    float direction = 0.0f;
    int state = 0;
    int targetIdx = -1;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

struct TargetsData {
    int targetCount = 0;
    int timeSteps = 0;
    float arrayTimeStep = 1.0f;
    float hitRadius = 2.0f;
    std::vector<std::vector<Coord>> targets;
};

struct FlightData {
    Coord startPos;
    float altitude = 120.0f;
    float initialDir = 0.0f;
    float attackSpeed = 30.0f;
    float accelPath = 50.0f;
    std::string ammoName;
    float arrayTimeStep = 1.0f;
    float simTimeStep = 0.1f;
    float hitRadius = 2.0f;
    float angularSpeed = 1.5f;
    float turnThreshold = 0.3f;

    double bombM = 0.0;
    double bombD = 0.0;
    double bombL = 0.0;

    int stepCount = 0;
    bool dropped = false;
    std::vector<StepData> steps;

    int dropStepIdx = -1;
    double dropOriginX = 0.0;
    double dropOriginY = 0.0;
    double dropDir = 0.0;
    double dropTOfFlight = 0.0;
    Coord targetPoint = {0.0f, 0.0f};
    double missDistance = -1.0;
    std::vector<Vector3> projectilePath;
    std::vector<float> projectileTimes;
    std::string projectileSource = "projectile.json";
    std::string fileSlug;
};

struct FlightPaths {
    std::string slug;
    std::string simulationPath;
    std::string projectilePath;
};

struct ActiveFlightState {
    int idx = 0;
    int idxNext = 0;
    float wx = 0.0f;
    float wy = 0.0f;
    float direction = 0.0f;
    int state = 0;
    int targetIdx = -1;
    bool projectileActive = false;
    Vector3 projectilePos = {0.0f, 0.0f, 0.0f};
};

static inline Vector3 toRl(float wx, float wy, float wh);

static std::string ammoNameFromSlug(const std::string &slug) {
    std::string name;
    for (size_t i = 0; i < slug.size(); i++) {
        char ch = slug[i];
        if (ch == '_') {
            name.push_back('-');
        } else if (ch >= 'a' && ch <= 'z') {
            name.push_back((char)(ch - 'a' + 'A'));
        } else {
            name.push_back(ch);
        }
    }
    return name;
}

static bool loadJson(const std::string &path, json &j) {
    std::ifstream f(path);
    if (!f.is_open()) return false;
    try {
        f >> j;
    } catch (...) {
        return false;
    }
    return true;
}

static bool readCoord(const json &j, Coord &c) {
    if (!j.is_object() || !j.contains("x") || !j.contains("y")) return false;
    c.x = j.at("x").get<float>();
    c.y = j.at("y").get<float>();
    return true;
}

static bool loadTargets(const std::string &path, TargetsData &d) {
    json j;
    if (!loadJson(path, j)) return false;
    if (!j.contains("targets") || !j.at("targets").is_array() || j.at("targets").empty()) return false;

    const auto &targets = j.at("targets");
    if (j.contains("targetCount") && j.contains("timeSteps")) {
        d.targetCount = j.at("targetCount").get<int>();
        d.timeSteps = j.at("timeSteps").get<int>();
    } else {
        d.targetCount = (int)targets.size();
        d.timeSteps = (int)targets.at(0).size();
    }
    if (d.targetCount <= 0 || d.timeSteps <= 0) return false;
    if ((int)targets.size() != d.targetCount) return false;

    d.targets.assign(d.targetCount, std::vector<Coord>(d.timeSteps));

    for (int i = 0; i < d.targetCount; i++) {
        const auto &targetNode = targets.at(i);
        const auto *series = &targetNode;
        if (targetNode.is_object()) {
            if (!targetNode.contains("positions") || !targetNode.at("positions").is_array()) return false;
            series = &targetNode.at("positions");
        }
        if (!series->is_array() || (int)series->size() != d.timeSteps) return false;
        for (int k = 0; k < d.timeSteps; k++) {
            if (!readCoord(series->at(k), d.targets[i][k])) return false;
        }
    }
    return true;
}

static bool loadSimulation(const std::string &path, FlightData &d) {
    json j;
    if (!loadJson(path, j)) return false;

    const auto &steps = j.at("steps");
    if (!steps.is_array()) return false;

    if (j.contains("config") && j.at("config").is_object()) {
        const auto &cfg = j.at("config");
        if (!readCoord(cfg.at("startPos"), d.startPos)) return false;
        d.altitude = cfg.at("altitude").get<float>();
        d.initialDir = cfg.at("initialDir").get<float>();
        d.attackSpeed = cfg.at("attackSpeed").get<float>();
        d.accelPath = cfg.at("accelPath").get<float>();
        d.ammoName = cfg.at("ammoName").get<std::string>();
        d.bombM = cfg.value("ammoMass", 0.0);
        d.bombD = cfg.value("ammoDrag", 0.0);
        d.bombL = cfg.value("ammoLift", 0.0);
        d.arrayTimeStep = cfg.at("arrayTimeStep").get<float>();
        d.simTimeStep = cfg.at("simTimeStep").get<float>();
        d.hitRadius = cfg.at("hitRadius").get<float>();
        d.angularSpeed = cfg.at("angularSpeed").get<float>();
        d.turnThreshold = cfg.at("turnThreshold").get<float>();
    }

    if (j.contains("summary") && j.at("summary").is_object()) {
        const auto &summary = j.at("summary");
        d.stepCount = summary.at("stepCount").get<int>();
        d.dropped = summary.at("dropped").get<bool>();
    } else {
        d.stepCount = j.value("totalSteps", (int)steps.size());
        d.dropped = false;
    }

    if ((int)steps.size() != d.stepCount) return false;

    d.steps.clear();
    d.steps.reserve(d.stepCount);
    for (const auto &item : steps) {
        StepData s;
        if (item.contains("position")) {
            if (!readCoord(item.at("position"), s.pos)) return false;
        } else {
            if (!readCoord(item.at("pos"), s.pos)) return false;
        }
        if (!readCoord(item.at("dropPoint"), s.dropPoint)) return false;
        if (!readCoord(item.at("aimPoint"), s.aimPoint)) return false;
        if (!readCoord(item.at("predictedTarget"), s.predictedTarget)) return false;
        s.direction = item.at("direction").get<float>();
        s.state = item.at("state").get<int>();
        if (item.contains("targetIndex")) s.targetIdx = item.at("targetIndex").get<int>();
        else s.targetIdx = item.at("targetIdx").get<int>();
        d.steps.push_back(s);
    }

    return d.stepCount > 0;
}

static bool loadProjectile(const std::string &path, FlightData &d) {
    json j;
    if (!loadJson(path, j)) return false;
    if (!j.contains("projectile") || !j.at("projectile").is_object()) return false;

    const auto &projectile = j.at("projectile");
    bool available = projectile.value("available", false);

    d.projectilePath.clear();
    d.projectileTimes.clear();
    d.projectileSource = std::filesystem::path(path).filename().string();
    d.dropStepIdx = projectile.value("dropStepIdx", -1);
    d.dropTOfFlight = projectile.value("timeOfFlight", 0.0);
    d.missDistance = -1.0;

    if (projectile.contains("dropOrigin")) {
        Coord dropOrigin;
        if (!readCoord(projectile.at("dropOrigin"), dropOrigin)) return false;
        d.dropOriginX = dropOrigin.x;
        d.dropOriginY = dropOrigin.y;
    }
    if (projectile.contains("targetPoint")) {
        Coord targetPoint;
        if (!readCoord(projectile.at("targetPoint"), targetPoint)) return false;
        d.targetPoint = targetPoint;
        d.dropDir = std::atan2(targetPoint.y - d.dropOriginY, targetPoint.x - d.dropOriginX);
    }

    if (!available) {
        d.dropped = false;
        d.dropStepIdx = -1;
        d.dropTOfFlight = 0.0;
        return true;
    }

    if (!projectile.contains("points") || !projectile.at("points").is_array()) return false;
    for (const auto &point : projectile.at("points")) {
        const auto &pos = point.at("pos");
        float t = point.at("t").get<float>();
        float x = pos.at("x").get<float>();
        float y = pos.at("y").get<float>();
        float z = pos.at("z").get<float>();
        d.projectileTimes.push_back(t);
        d.projectilePath.push_back(toRl(x, y, z));
    }

    d.dropped = available && !d.projectilePath.empty();
    return true;
}

static std::vector<FlightPaths> discoverFlightFiles(const std::string &dir) {
    namespace fs = std::filesystem;

    std::vector<FlightPaths> files;
    fs::path baseDir(dir);
    if (!fs::exists(baseDir) || !fs::is_directory(baseDir)) {
        return files;
    }

    for (const auto &entry : fs::directory_iterator(baseDir)) {
        if (!entry.is_regular_file()) continue;

        std::string name = entry.path().filename().string();
        if (name.rfind("simulation_", 0) != 0 || entry.path().extension() != ".json") continue;

        std::string slug = entry.path().stem().string().substr(std::strlen("simulation_"));
        fs::path projectilePath = baseDir / ("projectile_" + slug + ".json");
        files.push_back({slug,
                         entry.path().string(),
                         fs::exists(projectilePath) ? projectilePath.string() : std::string()});
    }

    std::sort(files.begin(), files.end(), [](const FlightPaths &lhs, const FlightPaths &rhs) {
        return lhs.slug < rhs.slug;
    });

    if (!files.empty()) return files;

    fs::path singleSimulation = baseDir / "simulation.json";
    fs::path singleProjectile = baseDir / "projectile.json";
    if (fs::exists(singleSimulation)) {
        files.push_back({"single",
                         singleSimulation.string(),
                         fs::exists(singleProjectile) ? singleProjectile.string() : std::string()});
    }

    return files;
}

static const char *stateName(int s) {
    switch (s) {
        case 0: return "STOPPED";
        case 1: return "ACCELERATING";
        case 2: return "DECELERATING";
        case 3: return "TURNING";
        case 4: return "MOVING";
        default: return "?";
    }
}

static Vector2 interpTarget(const TargetsData &d, int i, double t) {
    double tt = t / d.arrayTimeStep;
    int idx = ((int)std::floor(tt)) % d.timeSteps;
    if (idx < 0) idx += d.timeSteps;
    int next = (idx + 1) % d.timeSteps;
    double frac = tt - std::floor(tt);
    Vector2 r;
    r.x = (float)(d.targets[i][idx].x + (d.targets[i][next].x - d.targets[i][idx].x) * frac);
    r.y = (float)(d.targets[i][idx].y + (d.targets[i][next].y - d.targets[i][idx].y) * frac);
    return r;
}

static bool sampleProjectilePosition(const FlightData &d, double t, Vector3 &out) {
    if (d.projectilePath.empty()) return false;
    if (d.projectilePath.size() == 1 || d.projectileTimes.size() != d.projectilePath.size()) {
        out = d.projectilePath.back();
        return true;
    }

    if (t <= d.projectileTimes.front()) {
        out = d.projectilePath.front();
        return true;
    }
    if (t >= d.projectileTimes.back()) {
        out = d.projectilePath.back();
        return true;
    }

    for (size_t i = 1; i < d.projectileTimes.size(); ++i) {
        if (t > d.projectileTimes[i]) continue;

        float t0 = d.projectileTimes[i - 1];
        float t1 = d.projectileTimes[i];
        float alpha = 0.0f;
        if (t1 - t0 > 1e-6f) {
            alpha = (float)((t - t0) / (t1 - t0));
        }

        out = Vector3Lerp(d.projectilePath[i - 1], d.projectilePath[i], alpha);
        return true;
    }

    out = d.projectilePath.back();
    return true;
}

static inline Vector3 toRl(float wx, float wy, float wh) {
    return {wx, wh, wy};
}

static void drawDroneModel(const Vector3 &pos, float yawRad, Color bodyColor) {
    Color armColor = Fade(bodyColor, 0.85f);
    Color rotorColor = (Color){226, 232, 240, 255};
    Color accentColor = (Color){250, 204, 21, 255};

    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    rlRotatef(yawRad * RAD2DEG, 0.0f, 1.0f, 0.0f);

    DrawCube({0.0f, 0.0f, 0.0f}, 3.4f, 0.55f, 1.0f, bodyColor);
    DrawCube({0.6f, 0.22f, 0.0f}, 1.4f, 0.35f, 0.85f, (Color){125, 211, 252, 255});
    DrawCube({0.0f, 0.0f, 0.0f}, 0.28f, 0.14f, 5.6f, armColor);
    DrawCube({0.0f, 0.0f, 0.0f}, 5.2f, 0.14f, 0.28f, armColor);

    const Vector3 rotorCenters[] = {
        { 2.3f, 0.15f,  2.45f},
        { 2.3f, 0.15f, -2.45f},
        {-2.3f, 0.15f,  2.45f},
        {-2.3f, 0.15f, -2.45f},
    };
    for (const auto &rotor : rotorCenters) {
        DrawCylinder(rotor, 0.42f, 0.42f, 0.06f, 18, rotorColor);
        DrawLine3D({rotor.x - 0.75f, rotor.y + 0.03f, rotor.z},
                   {rotor.x + 0.75f, rotor.y + 0.03f, rotor.z}, Fade(rotorColor, 0.7f));
        DrawLine3D({rotor.x, rotor.y + 0.03f, rotor.z - 0.75f},
                   {rotor.x, rotor.y + 0.03f, rotor.z + 0.75f}, Fade(rotorColor, 0.7f));
    }

    DrawSphere({1.95f, 0.08f, 0.0f}, 0.18f, accentColor);
    DrawSphere({-1.9f, 0.08f, 0.0f}, 0.12f, (Color){248, 113, 113, 255});
    DrawCubeWires({0.0f, 0.0f, 0.0f}, 3.4f, 0.55f, 1.0f, Fade(BLACK, 0.45f));

    rlPopMatrix();
}

static void drawTankModel(const Vector3 &pos, float yawRad, Color bodyColor, bool selected) {
    Color trackColor = (Color){55, 65, 81, 255};
    Color turretColor = Fade(bodyColor, 0.9f);
    Color barrelColor = (Color){203, 213, 225, 255};
    Color highlightColor = selected ? YELLOW : Fade(WHITE, 0.35f);

    rlPushMatrix();
    rlTranslatef(pos.x, pos.y, pos.z);
    rlRotatef(yawRad * RAD2DEG, 0.0f, 1.0f, 0.0f);

    DrawCube({0.0f, 0.48f, 0.0f}, 3.3f, 0.85f, 2.0f, bodyColor);
    DrawCube({0.0f, 0.9f, 0.0f}, 1.55f, 0.55f, 1.35f, turretColor);
    DrawCube({1.95f, 0.94f, 0.0f}, 2.2f, 0.12f, 0.18f, barrelColor);
    DrawCube({0.0f, 0.22f, 1.15f}, 3.5f, 0.45f, 0.38f, trackColor);
    DrawCube({0.0f, 0.22f, -1.15f}, 3.5f, 0.45f, 0.38f, trackColor);

    const float wheelXs[] = {-1.2f, -0.35f, 0.5f, 1.35f};
    for (float x : wheelXs) {
        DrawCylinder({x, 0.18f, 1.15f}, 0.16f, 0.16f, 0.12f, 12, highlightColor);
        DrawCylinder({x, 0.18f, -1.15f}, 0.16f, 0.16f, 0.12f, 12, highlightColor);
    }

    DrawCubeWires({0.0f, 0.48f, 0.0f}, 3.3f, 0.85f, 2.0f, Fade(BLACK, 0.4f));
    rlPopMatrix();
}

[[maybe_unused]] static Color stateColor(int s) {
    switch (s) {
        case 0: return GRAY;
        case 1: return (Color){34, 197, 94, 255};
        case 2: return (Color){234, 179, 8, 255};
        case 3: return (Color){168, 85, 247, 255};
        case 4: return (Color){56, 189, 248, 255};
        default: return WHITE;
    }
}

static Color targetColor(int i) {
    static const Color cs[] = {
        (Color){239, 68, 68, 255},
        (Color){245, 158, 11, 255},
        (Color){34, 197, 94, 255},
        (Color){168, 85, 247, 255},
        (Color){234, 179, 8, 255},
        (Color){59, 130, 246, 255}
    };
    int n = (int)(sizeof(cs) / sizeof(cs[0]));
    if (i < 0) return WHITE;
    return cs[i % n];
}

static Color droneColor(int i) {
    static const Color cs[] = {
        (Color){56, 189, 248, 255},
        (Color){250, 204, 21, 255},
        (Color){34, 197, 94, 255},
        (Color){248, 113, 113, 255},
        (Color){168, 85, 247, 255},
        (Color){244, 114, 182, 255}
    };
    int n = (int)(sizeof(cs) / sizeof(cs[0]));
    return cs[i % n];
}

int main(int argc, char **argv) {
    std::string dir = "..";
    if (argc > 1) dir = argv[1];

    TargetsData targets;
    if (!loadTargets(dir + "/targets.json", targets)) {
        std::fprintf(stderr, "Cannot load %s/targets.json\n", dir.c_str());
        return 1;
    }

    std::vector<FlightPaths> files = discoverFlightFiles(dir);
    if (files.empty()) {
        std::fprintf(stderr, "Cannot find simulation data in %s\n", dir.c_str());
        return 1;
    }

    std::vector<FlightData> flights;
    flights.reserve(files.size());
    for (const auto &file : files) {
        FlightData flight;
        if (!loadSimulation(file.simulationPath, flight)) {
            std::fprintf(stderr, "Cannot load %s\n", file.simulationPath.c_str());
            return 1;
        }
        if (!file.projectilePath.empty() && !loadProjectile(file.projectilePath, flight)) {
            std::fprintf(stderr, "Cannot load %s\n", file.projectilePath.c_str());
            return 1;
        }
        if (file.projectilePath.empty()) {
            flight.projectilePath.clear();
            flight.projectileTimes.clear();
            flight.projectileSource = "not available";
            flight.dropStepIdx = -1;
            flight.dropTOfFlight = 0.0;
            flight.dropped = false;
        }
        flight.fileSlug = file.slug;
        if (flight.ammoName.empty()) {
            flight.ammoName = ammoNameFromSlug(file.slug);
        }
        flights.push_back(flight);
    }

    if (flights.empty() || flights[0].steps.empty()) {
        std::fprintf(stderr, "No flight data loaded from %s\n", dir.c_str());
        return 1;
    }

    targets.arrayTimeStep = flights[0].arrayTimeStep;
    targets.hitRadius = flights[0].hitRadius;

    for (auto &flight : flights) {
        flight.missDistance = -1.0;
        if (!flight.dropped || flight.projectilePath.empty() || flight.dropStepIdx < 0) continue;
        if (flight.dropStepIdx >= (int)flight.steps.size()) continue;
        int targetIdx = flight.steps[flight.dropStepIdx].targetIdx;
        if (targetIdx < 0 || targetIdx >= targets.targetCount) continue;

        double impactTime = flight.dropStepIdx * (double)flight.simTimeStep + flight.dropTOfFlight;
        Vector2 realTarget = interpTarget(targets, targetIdx, impactTime);
        const Vector3 &landing = flight.projectilePath.back();
        flight.missDistance = std::hypot((double)landing.x - (double)realTarget.x,
                                         (double)landing.z - (double)realTarget.y);
    }

    float minX = flights[0].steps[0].pos.x;
    float maxX = flights[0].steps[0].pos.x;
    float minY = flights[0].steps[0].pos.y;
    float maxY = flights[0].steps[0].pos.y;
    float maxAltitude = flights[0].altitude;

    for (const auto &flight : flights) {
        maxAltitude = std::max(maxAltitude, flight.altitude);
        for (const auto &step : flight.steps) {
            minX = std::min(minX, step.pos.x);
            maxX = std::max(maxX, step.pos.x);
            minY = std::min(minY, step.pos.y);
            maxY = std::max(maxY, step.pos.y);
        }
        for (const auto &point : flight.projectilePath) {
            minX = std::min(minX, point.x);
            maxX = std::max(maxX, point.x);
            minY = std::min(minY, point.z);
            maxY = std::max(maxY, point.z);
        }
    }
    for (int t = 0; t < targets.targetCount; t++) {
        for (int j = 0; j < targets.timeSteps; j++) {
            minX = std::min(minX, targets.targets[t][j].x);
            maxX = std::max(maxX, targets.targets[t][j].x);
            minY = std::min(minY, targets.targets[t][j].y);
            maxY = std::max(maxY, targets.targets[t][j].y);
        }
    }

    float cx = (minX + maxX) * 0.5f;
    float cy = (minY + maxY) * 0.5f;
    float worldSize = std::max(maxX - minX, maxY - minY);
    if (worldSize < 50.0f) worldSize = 50.0f;

    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE);
    InitWindow(1400, 900, "dz3 drone viewer");
    SetTargetFPS(60);

    Camera3D cam = {};
    cam.position = {cx + worldSize * 0.8f, worldSize * 0.9f, cy + worldSize * 0.8f};
    cam.target = {cx, maxAltitude * 0.4f, cy};
    cam.up = {0, 1, 0};
    cam.fovy = 55.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    auto setPerspective = [&]() {
        cam.position = {cx + worldSize * 0.8f, worldSize * 0.9f, cy + worldSize * 0.8f};
        cam.target = {cx, maxAltitude * 0.4f, cy};
        cam.up = {0, 1, 0};
        cam.fovy = 55.0f;
        cam.projection = CAMERA_PERSPECTIVE;
    };
    auto setTopDown = [&]() {
        cam.position = {cx, worldSize * 2.0f, cy};
        cam.target = {cx, 0, cy};
        cam.up = {0, 0, 1};
        cam.fovy = worldSize * 1.2f;
        cam.projection = CAMERA_ORTHOGRAPHIC;
    };
    auto orbitCamera = [&](float yawDelta, float pitchDelta) {
        Vector3 dirVec = Vector3Subtract(cam.position, cam.target);
        float r = Vector3Length(dirVec);
        if (r < 0.001f) return;

        float yaw = std::atan2(dirVec.z, dirVec.x) + yawDelta;
        float pitch = std::asin(dirVec.y / r) + pitchDelta;
        pitch = Clamp(pitch, -1.5f, 1.5f);

        cam.position.x = cam.target.x + r * std::cos(pitch) * std::cos(yaw);
        cam.position.z = cam.target.z + r * std::cos(pitch) * std::sin(yaw);
        cam.position.y = cam.target.y + r * std::sin(pitch);
    };

    double playbackTime = 0.0;
    bool playing = true;
    float speedMul = 1.0f;
    double totalTime = 0.0;
    double minSimTimeStep = flights[0].simTimeStep;
    for (const auto &flight : flights) {
        double flightTime = (flight.stepCount - 1) * (double)flight.simTimeStep;
        if (flight.dropped) flightTime += flight.dropTOfFlight;
        if (flightTime > totalTime) totalTime = flightTime;
        if (flight.simTimeStep < minSimTimeStep) minSimTimeStep = flight.simTimeStep;
    }
    if (totalTime <= 0.0) totalTime = 1.0;

    while (!WindowShouldClose()) {
        float dt = GetFrameTime();

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 md = GetMouseDelta();
            orbitCamera(-md.x * 0.005f, md.y * 0.005f);
        }
        float wheel = GetMouseWheelMove();
        if (wheel != 0) {
            Vector3 dirVec = Vector3Subtract(cam.position, cam.target);
            float r = Vector3Length(dirVec);
            r *= (wheel > 0 ? 0.85f : 1.18f);
            r = Clamp(r, 5.0f, worldSize * 5.0f);
            cam.position = Vector3Add(cam.target, Vector3Scale(Vector3Normalize(dirVec), r));
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
            Vector2 md = GetMouseDelta();
            Vector3 forward = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
            Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, cam.up));
            Vector3 up = Vector3CrossProduct(right, forward);
            float panScale = Vector3Length(Vector3Subtract(cam.position, cam.target)) * 0.0015f;
            Vector3 shift = Vector3Add(Vector3Scale(right, -md.x * panScale), Vector3Scale(up, md.y * panScale));
            cam.position = Vector3Add(cam.position, shift);
            cam.target = Vector3Add(cam.target, shift);
        }
        if (IsKeyPressed(KEY_R) || IsKeyPressed(KEY_P)) setPerspective();
        if (IsKeyPressed(KEY_T)) setTopDown();
        if (cam.projection == CAMERA_PERSPECTIVE) {
            if (IsKeyDown(KEY_Q)) orbitCamera(-1.5f * dt, 0.0f);
            if (IsKeyDown(KEY_E)) orbitCamera(1.5f * dt, 0.0f);
            if (IsKeyDown(KEY_Z)) orbitCamera(0.0f, -1.0f * dt);
            if (IsKeyDown(KEY_C)) orbitCamera(0.0f, 1.0f * dt);
        }

        if (IsKeyPressed(KEY_SPACE)) playing = !playing;
        if (IsKeyPressed(KEY_HOME)) { playbackTime = 0.0; playing = false; }
        if (IsKeyPressed(KEY_END))  { playbackTime = totalTime; playing = false; }
        if (IsKeyPressed(KEY_LEFT)) { playbackTime -= minSimTimeStep; playing = false; }
        if (IsKeyPressed(KEY_RIGHT)) { playbackTime += minSimTimeStep; playing = false; }
        if (IsKeyDown(KEY_LEFT_SHIFT) && IsKeyDown(KEY_LEFT))  playbackTime -= minSimTimeStep * 5 * dt * 60;
        if (IsKeyDown(KEY_LEFT_SHIFT) && IsKeyDown(KEY_RIGHT)) playbackTime += minSimTimeStep * 5 * dt * 60;
        if (IsKeyPressed(KEY_UP))   speedMul = std::min(8.0f, speedMul * 2.0f);
        if (IsKeyPressed(KEY_DOWN)) speedMul = std::max(0.125f, speedMul * 0.5f);
        if (IsKeyPressed(KEY_ZERO)) speedMul = 1.0f;

        if (playing) {
            playbackTime += dt * speedMul;
            if (playbackTime >= totalTime) {
                playbackTime = totalTime;
                playing = false;
            }
        }
        if (playbackTime < 0) playbackTime = 0;
        if (playbackTime > totalTime) playbackTime = totalTime;

        std::vector<ActiveFlightState> activeFlights(flights.size());
        for (size_t i = 0; i < flights.size(); i++) {
            const FlightData &flight = flights[i];
            ActiveFlightState state;

            double droneMaxT = (double)(flight.stepCount - 1) * flight.simTimeStep;
            double simT = std::min(playbackTime, droneMaxT);
            double sf = simT / flight.simTimeStep;
            int idx = (int)std::floor(sf);
            if (idx >= flight.stepCount - 1) idx = flight.stepCount - 1;
            if (idx < 0) idx = 0;
            int idxNext = std::min(idx + 1, flight.stepCount - 1);
            double frac = sf - idx;

            state.idx = idx;
            state.idxNext = idxNext;
            state.wx = flight.steps[idx].pos.x + (flight.steps[idxNext].pos.x - flight.steps[idx].pos.x) * (float)frac;
            state.wy = flight.steps[idx].pos.y + (flight.steps[idxNext].pos.y - flight.steps[idx].pos.y) * (float)frac;
            state.direction = flight.steps[idx].direction;
            state.state = flight.steps[idx].state;
            state.targetIdx = flight.steps[idx].targetIdx;

            if (flight.dropped) {
                double dropStart = flight.dropStepIdx * (double)flight.simTimeStep;
                double tFly = playbackTime - dropStart;
                if (tFly >= 0 && tFly <= flight.dropTOfFlight) {
                    state.projectileActive = sampleProjectilePosition(flight, tFly, state.projectilePos);
                } else if (tFly > flight.dropTOfFlight && !flight.projectilePath.empty()) {
                    state.projectileActive = true;
                    state.projectilePos = flight.projectilePath.back();
                }
            }
            activeFlights[i] = state;
        }

        BeginDrawing();
        ClearBackground((Color){15, 23, 42, 255});
        BeginMode3D(cam);

        float gridSize = worldSize * 3.0f;
        DrawPlane({cx, 0, cy}, {gridSize, gridSize}, (Color){30, 41, 59, 255});

        double pathStep = minSimTimeStep;
        int pathSamples = std::max(2, (int)std::ceil(playbackTime / pathStep) + 1);
        for (int t = 0; t < targets.targetCount; t++) {
            Color col = targetColor(t);
            Vector2 prev = interpTarget(targets, t, 0.0);
            for (int k = 1; k < pathSamples; k++) {
                double tt = std::min((double)k * pathStep, playbackTime);
                Vector2 cur = interpTarget(targets, t, tt);
                DrawLine3D(toRl(prev.x, prev.y, 0.05f), toRl(cur.x, cur.y, 0.05f), Fade(col, 0.55f));
                prev = cur;
            }
            Vector2 p = interpTarget(targets, t, playbackTime);
            Vector2 pAhead = interpTarget(targets, t, playbackTime + targets.arrayTimeStep * 0.1);
            float targetYaw = std::atan2(pAhead.y - p.y, pAhead.x - p.x);
            Vector3 pos = toRl(p.x, p.y, 0.45f);
            bool selected = false;
            for (size_t i = 0; i < activeFlights.size(); i++) {
                if (activeFlights[i].targetIdx == t) {
                    selected = true;
                    break;
                }
            }
            drawTankModel(pos, targetYaw, col, selected);
            DrawCircle3D({pos.x, 0.05f, pos.z}, targets.hitRadius, {1, 0, 0}, 90.0f, Fade(col, 0.5f));
            for (size_t i = 0; i < activeFlights.size(); i++) {
                if (activeFlights[i].targetIdx == t) {
                    DrawCircle3D({pos.x, 0.06f + 0.01f * (float)i, pos.z},
                                 targets.hitRadius * (1.45f + 0.12f * (float)i),
                                 {1, 0, 0}, 90.0f, droneColor((int)i));
                }
            }
        }

        for (size_t flightIdx = 0; flightIdx < flights.size(); flightIdx++) {
            const FlightData &flight = flights[flightIdx];
            const ActiveFlightState &state = activeFlights[flightIdx];
            Color col = droneColor((int)flightIdx);

            for (int i = 1; i <= state.idx; i++) {
                DrawLine3D(toRl(flight.steps[i - 1].pos.x, flight.steps[i - 1].pos.y, flight.altitude),
                           toRl(flight.steps[i].pos.x, flight.steps[i].pos.y, flight.altitude),
                           Fade(col, 0.75f));
            }

            Vector3 dronePos = toRl(state.wx, state.wy, flight.altitude);
            drawDroneModel(dronePos, state.direction, col);
            Vector3 nose = toRl(state.wx + std::cos(state.direction) * 4.8f,
                                state.wy + std::sin(state.direction) * 4.8f, flight.altitude);
            DrawLine3D(dronePos, nose, Fade(col, 0.8f));
            DrawSphere(nose, 0.35f, Fade(col, 0.9f));
            DrawLine3D(dronePos, {dronePos.x, 0.05f, dronePos.z}, Fade(col, 0.3f));
            DrawCircle3D({dronePos.x, 0.05f, dronePos.z}, 1.0f, {1, 0, 0}, 90.0f, Fade(col, 0.7f));

            if (flight.dropped) {
                for (size_t i = 1; i < flight.projectilePath.size(); i++) {
                    DrawLine3D(flight.projectilePath[i - 1], flight.projectilePath[i], Fade(col, 0.55f));
                }
                if (!flight.projectilePath.empty()) {
                    Vector3 landing = flight.projectilePath.back();
                    DrawSphere({landing.x, 0.05f, landing.z}, 0.8f, Fade(col, 0.9f));
                    DrawCircle3D({landing.x, 0.06f, landing.z}, flight.hitRadius, {1, 0, 0}, 90.0f, col);
                }
            }

            if (state.projectileActive) {
                DrawSphere(state.projectilePos, 0.9f, col);
                DrawLine3D(state.projectilePos, {state.projectilePos.x, 0.05f, state.projectilePos.z},
                           Fade(col, 0.45f));
            }
        }

        EndMode3D();

        for (size_t i = 0; i < flights.size(); i++) {
            const FlightData &flight = flights[i];
            const ActiveFlightState &state = activeFlights[i];
            Vector3 labelWorldPos = toRl(state.wx, state.wy, flight.altitude + 6.0f);
            Vector2 labelScreenPos = GetWorldToScreen(labelWorldPos, cam);
            int labelWidth = MeasureText(flight.ammoName.c_str(), 16);
            int labelX = (int)labelScreenPos.x - labelWidth / 2;
            int labelY = (int)labelScreenPos.y - 10;

            if (labelScreenPos.x >= 0.0f && labelScreenPos.x <= (float)GetScreenWidth() &&
                labelScreenPos.y >= 0.0f && labelScreenPos.y <= (float)GetScreenHeight()) {
                DrawText(flight.ammoName.c_str(), labelX + 1, labelY + 1, 16, Fade(BLACK, 0.75f));
                DrawText(flight.ammoName.c_str(), labelX, labelY, 16, droneColor((int)i));
            }
        }

        int scrW = GetScreenWidth();
        int scrH = GetScreenHeight();
        int infoHeight = 90 + (int)flights.size() * 24;
        DrawRectangle(10, 10, 520, infoHeight, (Color){15, 23, 42, 220});
        DrawRectangleLines(10, 10, 520, infoHeight, (Color){51, 65, 85, 255});
        DrawText("dz3 drone simulation", 22, 20, 22, (Color){226, 232, 240, 255});
        char buf[256];
        std::snprintf(buf, sizeof(buf), "Drones: %d   Targets: %d   t=%.2f / %.2f s   speed x%.2f",
                      (int)flights.size(), targets.targetCount, playbackTime, totalTime, speedMul);
        DrawText(buf, 22, 48, 16, LIGHTGRAY);
        std::snprintf(buf, sizeof(buf), "Altitude max=%.1f m, target hitR=%.1f m, source=%s",
                      maxAltitude, targets.hitRadius, dir.c_str());
        DrawText(buf, 22, 70, 16, LIGHTGRAY);
        DrawText(playing ? "> PLAYING" : "|| PAUSED", 400, 70, 16, playing ? GREEN : ORANGE);

        for (size_t i = 0; i < flights.size(); i++) {
            const FlightData &flight = flights[i];
            const ActiveFlightState &state = activeFlights[i];
            std::snprintf(buf, sizeof(buf), "%s  pos=(%.1f, %.1f)  %s  T%d  drop=%s  miss=%.2f m",
                          flight.ammoName.c_str(), state.wx, state.wy, stateName(state.state),
                          state.targetIdx, flight.dropped ? "YES" : "NO", flight.missDistance);
            DrawText(buf, 22, 98 + (int)i * 24, 16, droneColor((int)i));
        }

        DrawRectangle(scrW - 260, 10, 250, 270, (Color){15, 23, 42, 220});
        DrawRectangleLines(scrW - 260, 10, 250, 270, (Color){51, 65, 85, 255});
        DrawText("Controls", scrW - 250, 20, 18, WHITE);
        DrawText("Space: play/pause", scrW - 250, 46, 14, LIGHTGRAY);
        DrawText("Left/Right: frame -/+", scrW - 250, 66, 14, LIGHTGRAY);
        DrawText("Shift+L/R: fast scrub", scrW - 250, 86, 14, LIGHTGRAY);
        DrawText("Up/Down: speed x2 / x0.5", scrW - 250, 106, 14, LIGHTGRAY);
        DrawText("0: speed x1", scrW - 250, 126, 14, LIGHTGRAY);
        DrawText("Home/End: start/end", scrW - 250, 146, 14, LIGHTGRAY);
        DrawText("RMB drag: orbit", scrW - 250, 166, 14, LIGHTGRAY);
        DrawText("MMB drag: pan", scrW - 250, 186, 14, LIGHTGRAY);
        DrawText("Wheel: zoom", scrW - 250, 206, 14, LIGHTGRAY);
        DrawText("Q/E: rotate left/right", scrW - 250, 226, 14, LIGHTGRAY);
        DrawText("Z/C: tilt down/up", scrW - 250, 246, 14, LIGHTGRAY);
        DrawText("P: perspective, T: top view", scrW - 250, 266, 14, LIGHTGRAY);

        int barY = scrH - 70;
        int barX = 20;
        int barW = scrW - 40;
        int barH = 18;
        DrawRectangle(barX, barY, barW, barH, (Color){30, 41, 59, 255});
        DrawRectangleLines(barX, barY, barW, barH, (Color){71, 85, 105, 255});

        int secs = (int)std::ceil(totalTime);
        for (int s = 0; s <= secs; s++) {
            float xx = barX + (float)s / (float)(secs == 0 ? 1 : secs) * barW;
            DrawLine((int)xx, barY - 4, (int)xx, barY, (Color){100, 116, 139, 255});
            if (s % std::max(1, secs / 10) == 0) {
                DrawText(TextFormat("%ds", s), (int)xx - 6, barY - 20, 12, LIGHTGRAY);
            }
        }

        for (size_t i = 0; i < flights.size(); i++) {
            const FlightData &flight = flights[i];
            if (!flight.dropped || totalTime <= 0.0) continue;

            double dropStart = flight.dropStepIdx * (double)flight.simTimeStep;
            float dropXbar = barX + (float)(dropStart / totalTime) * barW;
            Color col = droneColor((int)i);
            DrawLine((int)dropXbar, barY - 2, (int)dropXbar, barY + barH + 2, col);
            DrawText(flight.ammoName.c_str(), (int)dropXbar - 18, barY + barH + 4 + (int)i * 12, 12, col);
        }

        float curX = barX + (float)(playbackTime / totalTime) * barW;
        DrawRectangle((int)curX - 2, barY - 4, 4, barH + 8, WHITE);

        Vector2 m = GetMousePosition();
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
            m.x >= barX && m.x <= barX + barW &&
            m.y >= barY - 10 && m.y <= barY + barH + 10) {
            playbackTime = (double)(m.x - barX) / barW * totalTime;
            playing = false;
        }

        DrawText(TextFormat("t = %.2f / %.2f s", playbackTime, totalTime), barX, scrH - 30, 14, LIGHTGRAY);
        int droppedCount = 0;
        for (const auto &flight : flights) {
            if (flight.dropped) droppedCount++;
        }
        DrawText(TextFormat("Dropped: %d / %d", droppedCount, (int)flights.size()), barX + 180, scrH - 30, 14,
                 droppedCount == (int)flights.size() ? (Color){34, 197, 94, 255} : ORANGE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

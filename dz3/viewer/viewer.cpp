// 3D-візуалізатор симуляції дрона з dz3.
// Читає config.json, ammo.json, targets.json, simulation.json і малює сцену через raylib.

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "../json.hpp"
#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"

using nlohmann::json;

static const double G = 9.81;
static const double EPS = 1e-9;

struct Coord {
    float x = 0.0f;
    float y = 0.0f;
};

struct AmmoEntry {
    std::string name;
    double mass = 0.0;
    double drag = 0.0;
    double lift = 0.0;
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

struct SimData {
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

    int targetCount = 0;
    int timeSteps = 0;
    int stepCount = 0;
    bool dropped = false;

    std::vector<std::vector<Coord>> targets;
    std::vector<StepData> steps;

    int dropStepIdx = -1;
    double dropOriginX = 0.0;
    double dropOriginY = 0.0;
    double dropDir = 0.0;
    double dropTOfFlight = 0.0;
    std::vector<Vector3> projectilePath;
};

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

static bool loadAmmo(const std::string &path, std::vector<AmmoEntry> &ammo) {
    json j;
    if (!loadJson(path, j) || !j.is_array()) return false;
    ammo.clear();
    for (const auto &item : j) {
        AmmoEntry a;
        a.name = item.at("name").get<std::string>();
        a.mass = item.at("mass").get<double>();
        a.drag = item.at("drag").get<double>();
        a.lift = item.at("lift").get<double>();
        ammo.push_back(a);
    }
    return !ammo.empty();
}

static bool loadTargets(const std::string &path, SimData &d) {
    json j;
    if (!loadJson(path, j)) return false;
    if (!j.contains("targets") || !j.at("targets").is_array() || j.at("targets").empty()) return false;

    d.targetCount = (int)j.at("targets").size();
    d.timeSteps = (int)j.at("targets").at(0).size();
    d.targets.assign(d.targetCount, std::vector<Coord>(d.timeSteps));

    for (int i = 0; i < d.targetCount; i++) {
        const auto &series = j.at("targets").at(i);
        if (!series.is_array() || (int)series.size() != d.timeSteps) return false;
        for (int k = 0; k < d.timeSteps; k++) {
            if (!readCoord(series.at(k), d.targets[i][k])) return false;
        }
    }
    return true;
}

static bool loadSimulation(const std::string &path, SimData &d) {
    json j;
    if (!loadJson(path, j)) return false;

    const auto &cfg = j.at("config");
    if (!readCoord(cfg.at("startPos"), d.startPos)) return false;
    d.altitude = cfg.at("altitude").get<float>();
    d.initialDir = cfg.at("initialDir").get<float>();
    d.attackSpeed = cfg.at("attackSpeed").get<float>();
    d.accelPath = cfg.at("accelPath").get<float>();
    d.ammoName = cfg.at("ammoName").get<std::string>();
    d.arrayTimeStep = cfg.at("arrayTimeStep").get<float>();
    d.simTimeStep = cfg.at("simTimeStep").get<float>();
    d.hitRadius = cfg.at("hitRadius").get<float>();
    d.angularSpeed = cfg.at("angularSpeed").get<float>();
    d.turnThreshold = cfg.at("turnThreshold").get<float>();

    const auto &summary = j.at("summary");
    d.stepCount = summary.at("stepCount").get<int>();
    d.dropped = summary.at("dropped").get<bool>();

    const auto &steps = j.at("steps");
    if (!steps.is_array() || (int)steps.size() != d.stepCount) return false;

    d.steps.clear();
    d.steps.reserve(d.stepCount);
    for (const auto &item : steps) {
        StepData s;
        if (!readCoord(item.at("pos"), s.pos)) return false;
        if (!readCoord(item.at("dropPoint"), s.dropPoint)) return false;
        if (!readCoord(item.at("aimPoint"), s.aimPoint)) return false;
        if (!readCoord(item.at("predictedTarget"), s.predictedTarget)) return false;
        s.direction = item.at("direction").get<float>();
        s.state = item.at("state").get<int>();
        s.targetIdx = item.at("targetIdx").get<int>();
        d.steps.push_back(s);
    }
    return d.stepCount > 0;
}

static int findAmmo(const std::vector<AmmoEntry> &ammo, const std::string &name) {
    for (size_t i = 0; i < ammo.size(); i++) {
        if (ammo[i].name == name) return (int)i;
    }
    return -1;
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

static double solveCardanoTime(double a, double b, double c, bool &ok) {
    ok = false;
    if (std::fabs(a) < EPS) return 0.0;
    double p = -(b * b) / (3.0 * a * a);
    double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;
    if (p >= 0.0) return 0.0;
    double acosArg = (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
    if (acosArg < -1.0 || acosArg > 1.0) return 0.0;
    double phi = std::acos(acosArg);
    double rootBase = 2.0 * std::sqrt(-p / 3.0);
    double t1 = rootBase * std::cos(phi / 3.0) - b / (3.0 * a);
    double t2 = rootBase * std::cos((phi + 2.0 * M_PI) / 3.0) - b / (3.0 * a);
    double t3 = rootBase * std::cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);
    if (t3 > EPS) { ok = true; return t3; }
    if (t2 > EPS) { ok = true; return t2; }
    if (t1 > EPS) { ok = true; return t1; }
    return 0.0;
}

static double calcHorizontalDistance(double t, double m, double d, double l, double V0) {
    double term1 = V0 * t;
    double term2 = -(t * t * d * V0) / (2.0 * m);
    double term3 = (t * t * t * (6.0 * d * G * l * m - 6.0 * d * d * (l * l - 1.0) * V0)) /
                   (36.0 * m * m);
    double term4 = (std::pow(t, 4.0) *
                    (-6.0 * d * d * G * l * (1.0 + l * l + l * l * l * l) * m +
                     3.0 * d * d * d * l * l * (1.0 + l * l) * V0 +
                     6.0 * d * d * d * l * l * l * l * (1.0 + l * l) * V0)) /
                   (36.0 * std::pow(1.0 + l * l, 2.0) * m * m * m);
    double term5 = (std::pow(t, 5.0) *
                    (3.0 * d * d * d * G * l * l * l * m -
                     3.0 * std::pow(d, 4.0) * l * l * (1.0 + l * l) * V0)) /
                   (36.0 * (1.0 + l * l) * std::pow(m, 4.0));
    return term1 + term2 + term3 + term4 + term5;
}

static bool computeBallistics(double zd, double V0, double m, double d, double l,
                              double &tOfFlight, double &hDist) {
    double a = d * G * m - 2.0 * d * d * l * V0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * V0;
    double c = 6.0 * m * m * zd;
    bool ok = false;
    double t = solveCardanoTime(a, b, c, ok);
    if (!ok || t <= EPS) return false;
    double h = calcHorizontalDistance(t, m, d, l, V0);
    if (h <= EPS) return false;
    tOfFlight = t;
    hDist = h;
    return true;
}

static Vector2 interpTarget(const SimData &d, int i, double t) {
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

static double estimateDroneTravelTime(double dist, double speed, double attackSpeed, double accel) {
    if (dist <= 0.0) return 0.0;
    if (accel <= EPS) return dist / attackSpeed;
    if (speed >= attackSpeed) return dist / attackSpeed;
    double distToCruise = (attackSpeed * attackSpeed - speed * speed) / (2.0 * accel);
    if (distToCruise >= dist) {
        double disc = speed * speed + 2.0 * accel * dist;
        if (disc < 0.0) disc = 0.0;
        return (-speed + std::sqrt(disc)) / accel;
    }
    double tAccel = (attackSpeed - speed) / accel;
    double tCruise = (dist - distToCruise) / attackSpeed;
    return tAccel + tCruise;
}

static bool computeDropPointFull(const SimData &d, int tgt, double currentTime,
                                 double posX, double posY, double droneSpeed,
                                 double &dropX, double &dropY,
                                 double &predX, double &predY,
                                 double &tOfFlight) {
    Vector2 cur = interpTarget(d, tgt, currentTime);
    double hDist = 0;
    if (!computeBallistics(d.altitude, d.attackSpeed, d.bombM, d.bombD, d.bombL, tOfFlight, hDist)) return false;

    double dxT = cur.x - posX;
    double dyT = cur.y - posY;
    double D = std::sqrt(dxT * dxT + dyT * dyT);
    if (D < EPS) return false;

    double accel = (double)d.attackSpeed * (double)d.attackSpeed / (2.0 * (double)d.accelPath);
    double approxDist = std::max(0.0, D - hDist);
    double approxArrival = estimateDroneTravelTime(approxDist, droneSpeed, d.attackSpeed, accel);
    double totalTime = approxArrival + tOfFlight;

    Vector2 t1 = interpTarget(d, tgt, currentTime);
    Vector2 t2 = interpTarget(d, tgt, currentTime + d.simTimeStep);
    double tvx = (t2.x - t1.x) / d.simTimeStep;
    double tvy = (t2.y - t1.y) / d.simTimeStep;

    predX = t1.x + tvx * totalTime;
    predY = t1.y + tvy * totalTime;
    double pdx = predX - posX;
    double pdy = predY - posY;
    double pD = std::sqrt(pdx * pdx + pdy * pdy);
    if (pD < EPS) return false;

    dropX = predX - pdx * hDist / pD;
    dropY = predY - pdy * hDist / pD;
    return true;
}

static void reconstructDrop(SimData &d) {
    if (d.stepCount < 2) return;

    auto speedAt = [&](int k) {
        if (k <= 0) return 0.0;
        double dx = d.steps[k].pos.x - d.steps[k - 1].pos.x;
        double dy = d.steps[k].pos.y - d.steps[k - 1].pos.y;
        return std::sqrt(dx * dx + dy * dy) / d.simTimeStep;
    };

    for (int step = 0; step < d.stepCount; step++) {
        int tgt = d.steps[step].targetIdx;
        if (tgt < 0) continue;
        double currentTime = step * (double)d.simTimeStep;
        double ox = d.steps[step].pos.x;
        double oy = d.steps[step].pos.y;
        double speed = speedAt(step);

        double dropX, dropY, predX, predY, tOfFlight;
        if (!computeDropPointFull(d, tgt, currentTime, ox, oy, speed,
                                  dropX, dropY, predX, predY, tOfFlight)) continue;

        double ddx = dropX - ox;
        double ddy = dropY - oy;
        double dropDist = std::sqrt(ddx * ddx + ddy * ddy);

        if (dropDist <= (double)d.hitRadius + 0.01) {
            d.dropped = true;
            d.dropStepIdx = step;
            d.dropOriginX = ox;
            d.dropOriginY = oy;
            double dirDx = predX - ox;
            double dirDy = predY - oy;
            if (std::sqrt(dirDx * dirDx + dirDy * dirDy) < EPS) d.dropDir = d.steps[step].direction;
            else d.dropDir = std::atan2(dirDy, dirDx);
            d.dropTOfFlight = tOfFlight;

            const int SAMPLES = 64;
            d.projectilePath.clear();
            d.projectilePath.reserve(SAMPLES + 1);
            double cosD = std::cos(d.dropDir);
            double sinD = std::sin(d.dropDir);
            for (int i = 0; i <= SAMPLES; i++) {
                double t = tOfFlight * i / SAMPLES;
                double h = calcHorizontalDistance(t, d.bombM, d.bombD, d.bombL, d.attackSpeed);
                double px = ox + cosD * h;
                double py = oy + sinD * h;
                double nrm = t / tOfFlight;
                double pz = (double)d.altitude * (1.0 - nrm * nrm);
                if (pz < 0.0) pz = 0.0;
                d.projectilePath.push_back({(float)px, (float)pz, (float)py});
            }
            return;
        }
    }
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

static Color stateColor(int s) {
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

int main(int argc, char **argv) {
    std::string dir = "..";
    if (argc > 1) dir = argv[1];

    SimData sim;
    std::vector<AmmoEntry> ammo;
    if (!loadAmmo(dir + "/ammo.json", ammo)) {
        std::fprintf(stderr, "Cannot load %s/ammo.json\n", dir.c_str());
        return 1;
    }
    if (!loadTargets(dir + "/targets.json", sim)) {
        std::fprintf(stderr, "Cannot load %s/targets.json\n", dir.c_str());
        return 1;
    }
    if (!loadSimulation(dir + "/simulation.json", sim)) {
        std::fprintf(stderr, "Cannot load %s/simulation.json\n", dir.c_str());
        return 1;
    }

    int ammoIdx = findAmmo(ammo, sim.ammoName);
    if (ammoIdx < 0) {
        std::fprintf(stderr, "Unknown ammo %s\n", sim.ammoName.c_str());
        return 1;
    }
    sim.bombM = ammo[ammoIdx].mass;
    sim.bombD = ammo[ammoIdx].drag;
    sim.bombL = ammo[ammoIdx].lift;

    reconstructDrop(sim);

    float minX = sim.steps[0].pos.x, maxX = sim.steps[0].pos.x;
    float minY = sim.steps[0].pos.y, maxY = sim.steps[0].pos.y;
    for (const auto &step : sim.steps) {
        minX = std::min(minX, step.pos.x);
        maxX = std::max(maxX, step.pos.x);
        minY = std::min(minY, step.pos.y);
        maxY = std::max(maxY, step.pos.y);
    }
    for (int t = 0; t < sim.targetCount; t++) {
        for (int j = 0; j < sim.timeSteps; j++) {
            minX = std::min(minX, sim.targets[t][j].x);
            maxX = std::max(maxX, sim.targets[t][j].x);
            minY = std::min(minY, sim.targets[t][j].y);
            maxY = std::max(maxY, sim.targets[t][j].y);
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
    cam.target = {cx, sim.altitude * 0.4f, cy};
    cam.up = {0, 1, 0};
    cam.fovy = 55.0f;
    cam.projection = CAMERA_PERSPECTIVE;

    auto setPerspective = [&]() {
        cam.position = {cx + worldSize * 0.8f, worldSize * 0.9f, cy + worldSize * 0.8f};
        cam.target = {cx, sim.altitude * 0.4f, cy};
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
    double totalTime = (sim.stepCount - 1) * (double)sim.simTimeStep;
    if (sim.dropped) totalTime += sim.dropTOfFlight;

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
        if (IsKeyPressed(KEY_LEFT)) { playbackTime -= sim.simTimeStep; playing = false; }
        if (IsKeyPressed(KEY_RIGHT)) { playbackTime += sim.simTimeStep; playing = false; }
        if (IsKeyDown(KEY_LEFT_SHIFT) && IsKeyDown(KEY_LEFT))  playbackTime -= sim.simTimeStep * 5 * dt * 60;
        if (IsKeyDown(KEY_LEFT_SHIFT) && IsKeyDown(KEY_RIGHT)) playbackTime += sim.simTimeStep * 5 * dt * 60;
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

        double droneMaxT = (double)(sim.stepCount - 1) * sim.simTimeStep;
        double simT = std::min(playbackTime, droneMaxT);
        double sf = simT / sim.simTimeStep;
        int idx = (int)std::floor(sf);
        if (idx >= sim.stepCount - 1) idx = sim.stepCount - 1;
        if (idx < 0) idx = 0;
        int idxNext = std::min(idx + 1, sim.stepCount - 1);
        double frac = sf - idx;

        float droneWx = sim.steps[idx].pos.x + (sim.steps[idxNext].pos.x - sim.steps[idx].pos.x) * (float)frac;
        float droneWy = sim.steps[idx].pos.y + (sim.steps[idxNext].pos.y - sim.steps[idx].pos.y) * (float)frac;
        float droneD = sim.steps[idx].direction;
        int droneSt = sim.steps[idx].state;
        int droneTg = sim.steps[idx].targetIdx;

        bool projActive = false;
        Vector3 projPos = {0, 0, 0};
        if (sim.dropped) {
            double dropStart = sim.dropStepIdx * (double)sim.simTimeStep;
            double tFly = playbackTime - dropStart;
            if (tFly >= 0 && tFly <= sim.dropTOfFlight) {
                projActive = true;
                double nrm = tFly / sim.dropTOfFlight;
                double hDist = calcHorizontalDistance(tFly, sim.bombM, sim.bombD, sim.bombL, sim.attackSpeed);
                double px = sim.dropOriginX + std::cos(sim.dropDir) * hDist;
                double py = sim.dropOriginY + std::sin(sim.dropDir) * hDist;
                double pz = (double)sim.altitude * (1.0 - nrm * nrm);
                if (pz < 0) pz = 0;
                projPos = toRl((float)px, (float)py, (float)pz);
            } else if (tFly > sim.dropTOfFlight && !sim.projectilePath.empty()) {
                projActive = true;
                projPos = sim.projectilePath.back();
            }
        }

        BeginDrawing();
        ClearBackground((Color){15, 23, 42, 255});
        BeginMode3D(cam);

        float gridSize = worldSize * 3.0f;
        DrawPlane({cx, 0, cy}, {gridSize, gridSize}, (Color){30, 41, 59, 255});
        rlPushMatrix();
        rlTranslatef(cx, 0.01f, cy);
        DrawGrid((int)(gridSize / 10.0f), 10.0f);
        rlPopMatrix();

        float axisLen = worldSize * 0.4f;
        DrawLine3D({cx, 0.1f, cy}, {cx + axisLen, 0.1f, cy}, (Color){239, 68, 68, 255});
        DrawLine3D({cx, 0.1f, cy}, {cx, 0.1f, cy + axisLen}, (Color){59, 130, 246, 255});
        DrawLine3D({cx, 0.1f, cy}, {cx, axisLen * 0.5f, cy}, (Color){34, 197, 94, 255});

        double pathStep = sim.simTimeStep;
        int pathSamples = std::max(2, (int)std::ceil(playbackTime / pathStep) + 1);
        for (int t = 0; t < sim.targetCount; t++) {
            Color col = targetColor(t);
            Vector2 prev = interpTarget(sim, t, 0.0);
            for (int k = 1; k < pathSamples; k++) {
                double tt = std::min((double)k * pathStep, playbackTime);
                Vector2 cur = interpTarget(sim, t, tt);
                DrawLine3D(toRl(prev.x, prev.y, 0.05f), toRl(cur.x, cur.y, 0.05f), Fade(col, 0.55f));
                prev = cur;
            }
            Vector2 p = interpTarget(sim, t, playbackTime);
            Vector2 pAhead = interpTarget(sim, t, playbackTime + sim.simTimeStep);
            float targetYaw = std::atan2(pAhead.y - p.y, pAhead.x - p.x);
            Vector3 pos = toRl(p.x, p.y, 0.45f);
            drawTankModel(pos, targetYaw, col, t == droneTg);
            DrawCircle3D({pos.x, 0.05f, pos.z}, sim.hitRadius, {1, 0, 0}, 90.0f, Fade(col, 0.5f));
            if (t == droneTg) {
                DrawCircle3D({pos.x, 0.06f, pos.z}, sim.hitRadius * 1.6f, {1, 0, 0}, 90.0f, YELLOW);
            }
        }

        for (int i = 1; i <= idx; i++) {
            DrawLine3D(toRl(sim.steps[i - 1].pos.x, sim.steps[i - 1].pos.y, sim.altitude),
                       toRl(sim.steps[i].pos.x, sim.steps[i].pos.y, sim.altitude),
                       (Color){56, 189, 248, 200});
        }

        Vector3 dronePos = toRl(droneWx, droneWy, sim.altitude);
        drawDroneModel(dronePos, droneD, (Color){56, 189, 248, 255});
        Vector3 nose = toRl(droneWx + std::cos(droneD) * 4.8f, droneWy + std::sin(droneD) * 4.8f, sim.altitude);
        DrawLine3D(dronePos, nose, YELLOW);
        DrawSphere(nose, 0.4f, YELLOW);
        DrawLine3D(dronePos, {dronePos.x, 0.05f, dronePos.z}, (Color){56, 189, 248, 80});
        DrawCircle3D({dronePos.x, 0.05f, dronePos.z}, 1.2f, {1, 0, 0}, 90.0f, (Color){56, 189, 248, 180});

        if (sim.dropped) {
            for (size_t i = 1; i < sim.projectilePath.size(); i++) {
                DrawLine3D(sim.projectilePath[i - 1], sim.projectilePath[i], (Color){248, 113, 113, 180});
            }
            if (!sim.projectilePath.empty()) {
                Vector3 landing = sim.projectilePath.back();
                DrawSphere({landing.x, 0.05f, landing.z}, 1.0f, (Color){248, 113, 113, 255});
                DrawCircle3D({landing.x, 0.06f, landing.z}, sim.hitRadius, {1, 0, 0}, 90.0f,
                             (Color){248, 113, 113, 255});
            }
        }
        if (projActive) {
            DrawSphere(projPos, 1.0f, (Color){250, 204, 21, 255});
            DrawLine3D(projPos, {projPos.x, 0.05f, projPos.z}, (Color){250, 204, 21, 120});
        }

        EndMode3D();

        int scrW = GetScreenWidth();
        int scrH = GetScreenHeight();
        DrawRectangle(10, 10, 420, 170, (Color){15, 23, 42, 220});
        DrawRectangleLines(10, 10, 420, 170, (Color){51, 65, 85, 255});
        DrawText("dz3 drone simulation", 22, 20, 22, (Color){226, 232, 240, 255});
        char buf[256];
        std::snprintf(buf, sizeof(buf), "Ammo: %s (m=%.2f, d=%.2f, l=%.1f)", sim.ammoName.c_str(), sim.bombM, sim.bombD, sim.bombL);
        DrawText(buf, 22, 48, 16, LIGHTGRAY);
        std::snprintf(buf, sizeof(buf), "Altitude=%.1f m, V0=%.1f m/s, hitR=%.1f m", sim.altitude, sim.attackSpeed, sim.hitRadius);
        DrawText(buf, 22, 70, 16, LIGHTGRAY);
        std::snprintf(buf, sizeof(buf), "Step: %d / %d    t=%.2f s    speed x%.2f", idx, sim.stepCount - 1, playbackTime, speedMul);
        DrawText(buf, 22, 92, 16, WHITE);
        std::snprintf(buf, sizeof(buf), "Drone pos=(%.1f, %.1f)  dir=%.2f rad", droneWx, droneWy, droneD);
        DrawText(buf, 22, 114, 16, LIGHTGRAY);
        DrawText(TextFormat("State: %s", stateName(droneSt)), 22, 136, 16, stateColor(droneSt));
        DrawText(TextFormat("Target: T%d", droneTg), 200, 136, 16, targetColor(droneTg));
        DrawText(playing ? "> PLAYING" : "|| PAUSED", 22, 156, 16, playing ? GREEN : ORANGE);

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

        if (sim.dropped) {
            double dropStart = sim.dropStepIdx * (double)sim.simTimeStep;
            float dropXbar = barX + (float)(dropStart / totalTime) * barW;
            DrawLine((int)dropXbar, barY - 2, (int)dropXbar, barY + barH + 2, (Color){248, 113, 113, 255});
            DrawText("drop", (int)dropXbar - 12, barY + barH + 4, 12, (Color){248, 113, 113, 255});
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
        DrawText(sim.dropped ? "Dropped: YES" : "Dropped: NO", barX + 180, scrH - 30, 14,
                 sim.dropped ? (Color){34, 197, 94, 255} : ORANGE);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}

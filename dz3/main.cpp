#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "json.hpp"

using nlohmann::json;

const double G = 9.81;
const double EPS = 1e-9;

enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

struct Coord {
    float x;
    float y;

    Coord operator+(const Coord &other) const {
        Coord result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
    }

    Coord operator-(const Coord &other) const {
        Coord result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
    }

    Coord operator*(float scalar) const {
        Coord result;
        result.x = x * scalar;
        result.y = y * scalar;
        return result;
    }

    Coord operator/(float scalar) const {
        Coord result;
        result.x = x / scalar;
        result.y = y / scalar;
        return result;
    }

    bool operator==(const Coord &other) const {
        return std::fabs(x - other.x) < EPS && std::fabs(y - other.y) < EPS;
    }

    Coord &operator+=(const Coord &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Coord &operator-=(const Coord &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
};

struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

struct DroneConfig {
    Coord startPos;
    float altitude;
    float initialDir;
    float attackSpeed;
    float accelPath;
    char ammoName[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;
};

struct SimStep {
    Coord pos;
    float direction;
    int state;
    int targetIdx;
    Coord dropPoint;
    Coord aimPoint;
    Coord predictedTarget;
};

float length(Coord coord) {
    return std::hypot(coord.x, coord.y);
}

Coord normalize(Coord coord) {
    float len = length(coord);
    if (len < (float)EPS) return {0.0f, 0.0f};
    return coord / len;
}

json coordToJson(const Coord &coord) {
    return json{{"x", coord.x}, {"y", coord.y}};
}

json stepToJson(const SimStep &step) {
    return json{
        {"pos", coordToJson(step.pos)},
        {"direction", step.direction},
        {"state", step.state},
        {"targetIdx", step.targetIdx},
        {"dropPoint", coordToJson(step.dropPoint)},
        {"aimPoint", coordToJson(step.aimPoint)},
        {"predictedTarget", coordToJson(step.predictedTarget)}
    };
}

bool loadJsonFile(const char *path, json &data) {
    std::ifstream fin(path);
    if (!fin.is_open()) {
        std::cout << "Error: cannot open " << path << "\n";
        return false;
    }

    try {
        fin >> data;
    } catch (const std::exception &e) {
        std::cout << "Error: invalid JSON in " << path << ": " << e.what() << "\n";
        return false;
    }
    return true;
}

bool readCoordObject(const json &node, Coord &coord) {
    if (!node.is_object() || !node.contains("x") || !node.contains("y")) return false;
    if (!node.at("x").is_number() || !node.at("y").is_number()) return false;
    coord.x = node.at("x").get<float>();
    coord.y = node.at("y").get<float>();
    return true;
}

bool loadConfig(const char *path, DroneConfig &config, int &maxSteps) {
    json data;
    if (!loadJsonFile(path, data)) return false;

    try {
        if (!readCoordObject(data.at("startPos"), config.startPos)) {
            std::cout << "Error: config.json has invalid startPos\n";
            return false;
        }

        config.altitude = data.at("altitude").get<float>();
        config.initialDir = data.at("initialDir").get<float>();
        config.attackSpeed = data.at("attackSpeed").get<float>();
        config.accelPath = data.at("accelPath").get<float>();

        std::string ammoName = data.at("ammoName").get<std::string>();
        if (ammoName.size() >= sizeof(config.ammoName)) {
            std::cout << "Error: ammoName is too long\n";
            return false;
        }
        std::strcpy(config.ammoName, ammoName.c_str());

        config.arrayTimeStep = data.at("arrayTimeStep").get<float>();
        config.simTimeStep = data.at("simTimeStep").get<float>();
        config.hitRadius = data.at("hitRadius").get<float>();
        config.angularSpeed = data.at("angularSpeed").get<float>();
        config.turnThreshold = data.at("turnThreshold").get<float>();
        maxSteps = data.at("maxSteps").get<int>();
    } catch (const std::exception &e) {
        std::cout << "Error: invalid config.json format: " << e.what() << "\n";
        return false;
    }

    return true;
}

bool loadAmmo(const char *path, AmmoParams *&ammoTable, int &ammoCount) {
    json data;
    if (!loadJsonFile(path, data)) return false;
    if (!data.is_array() || data.empty()) {
        std::cout << "Error: ammo.json must be a non-empty array\n";
        return false;
    }

    ammoCount = (int)data.size();
    ammoTable = new AmmoParams[ammoCount];

    for (int i = 0; i < ammoCount; i++) {
        try {
            std::string name = data.at(i).at("name").get<std::string>();
            if (name.size() >= sizeof(ammoTable[i].name)) {
                std::cout << "Error: ammo name is too long at index " << i << "\n";
                return false;
            }
            std::strcpy(ammoTable[i].name, name.c_str());
            ammoTable[i].mass = data.at(i).at("mass").get<float>();
            ammoTable[i].drag = data.at(i).at("drag").get<float>();
            ammoTable[i].lift = data.at(i).at("lift").get<float>();
        } catch (const std::exception &e) {
            std::cout << "Error: invalid ammo.json entry at index " << i << ": " << e.what() << "\n";
            return false;
        }
    }

    return true;
}

bool loadTargets(const char *path, Coord **&targets, int &targetCount, int &timeSteps) {
    json data;
    if (!loadJsonFile(path, data)) return false;
    if (!data.contains("targets") || !data.at("targets").is_array() || data.at("targets").empty()) {
        std::cout << "Error: targets.json must contain a non-empty targets array\n";
        return false;
    }

    const json &targetArray = data.at("targets");
    targetCount = (int)targetArray.size();
    if (!targetArray.at(0).is_array() || targetArray.at(0).empty()) {
        std::cout << "Error: targets.json must contain non-empty target series\n";
        return false;
    }

    timeSteps = (int)targetArray.at(0).size();
    targets = new Coord *[targetCount];
    for (int i = 0; i < targetCount; i++) {
        targets[i] = nullptr;
    }

    for (int i = 0; i < targetCount; i++) {
        if (!targetArray.at(i).is_array() || (int)targetArray.at(i).size() != timeSteps) {
            std::cout << "Error: all targets must have the same timeSteps count\n";
            return false;
        }

        targets[i] = new Coord[timeSteps];
        for (int j = 0; j < timeSteps; j++) {
            if (!readCoordObject(targetArray.at(i).at(j), targets[i][j])) {
                std::cout << "Error: invalid target coordinate at [" << i << "][" << j << "]\n";
                return false;
            }
        }
    }

    return true;
}

void freeTargets(Coord **&targets, int targetCount) {
    if (targets == nullptr) return;
    for (int i = 0; i < targetCount; i++) {
        delete[] targets[i];
        targets[i] = nullptr;
    }
    delete[] targets;
    targets = nullptr;
}

int findAmmo(const AmmoParams *ammoTable, int ammoCount, const char *name) {
    for (int i = 0; i < ammoCount; i++) {
        if (std::strcmp(name, ammoTable[i].name) == 0) return i;
    }
    return -1;
}

double solveCardanoTime(double a, double b, double c, bool &ok) {
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

double calcHorizontalDistance(double t, const AmmoParams &ammo, double V0) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
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

bool computeBallistics(double altitude, double V0, const AmmoParams &ammo,
                       double &tOfFlight, double &hDist) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    double a = d * G * m - 2.0 * d * d * l * V0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * V0;
    double c = 6.0 * m * m * altitude;

    bool ok = false;
    double t = solveCardanoTime(a, b, c, ok);
    if (!ok || t <= EPS) return false;

    double h = calcHorizontalDistance(t, ammo, V0);
    if (h <= EPS) return false;

    tOfFlight = t;
    hDist = h;
    return true;
}

Coord interpTarget(Coord **targetsInTime, int timeSteps, int targetIdx,
                   double t, double arrayTimeStep) {
    double tt = t / arrayTimeStep;
    int idx = ((int)std::floor(tt)) % timeSteps;
    if (idx < 0) idx += timeSteps;
    int next = (idx + 1) % timeSteps;
    double frac = tt - std::floor(tt);

    Coord result;
    result.x = static_cast<float>(targetsInTime[targetIdx][idx].x +
               (targetsInTime[targetIdx][next].x - targetsInTime[targetIdx][idx].x) * frac);
    result.y = static_cast<float>(targetsInTime[targetIdx][idx].y +
               (targetsInTime[targetIdx][next].y - targetsInTime[targetIdx][idx].y) * frac);
    return result;
}

double normAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double estimateDroneTravelTime(double dist, double speed, double attackSpeed, double accel) {
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

bool computeDropPoint(Coord **targetPositions, int timeSteps, int targetIdx,
                      double currentTime, double arrayTimeStep, double simTimeStep,
                      double altitude, double V0, const AmmoParams &ammo,
                      const Coord &dronePos, double droneSpeed, double accel,
                      Coord &dropPoint, Coord &predictedTarget,
                      double &arrivalTime, double &tOfFlight) {
    Coord currentTarget = interpTarget(targetPositions, timeSteps, targetIdx, currentTime, arrayTimeStep);

    double hDist;
    if (!computeBallistics(altitude, V0, ammo, tOfFlight, hDist)) return false;

    Coord toCurrentTarget = currentTarget - dronePos;
    double currentDistance = length(toCurrentTarget);
    if (currentDistance < EPS) return false;

    double approxDist = currentDistance - hDist;
    if (approxDist < 0.0) approxDist = 0.0;
    double approxArrival = estimateDroneTravelTime(approxDist, droneSpeed, V0, accel);
    double totalTime = approxArrival + tOfFlight;

    Coord targetNow = interpTarget(targetPositions, timeSteps, targetIdx, currentTime, arrayTimeStep);
    Coord targetNext = interpTarget(targetPositions, timeSteps, targetIdx, currentTime + simTimeStep, arrayTimeStep);
    Coord targetVelocity = (targetNext - targetNow) / (float)simTimeStep;

    predictedTarget = targetNow + targetVelocity * (float)totalTime;

    Coord toPredictedTarget = predictedTarget - dronePos;
    double predictedDistance = length(toPredictedTarget);
    if (predictedDistance < EPS) return false;

    dropPoint = predictedTarget - normalize(toPredictedTarget) * (float)hDist;
    Coord toDropPoint = dropPoint - dronePos;
    arrivalTime = estimateDroneTravelTime(length(toDropPoint), droneSpeed, V0, accel);
    return true;
}

double computeTimeToStop(DroneState state, double speed, double attackSpeed,
                         double accel, double turnRemainingTime) {
    switch (state) {
        case STOPPED:      return 0.0;
        case ACCELERATING: return speed / accel;
        case MOVING:       return attackSpeed / accel;
        case DECELERATING: return speed / accel;
        case TURNING:      return turnRemainingTime;
    }
    return 0.0;
}

bool writeSimulationJson(const char *path, const DroneConfig &config,
                         SimStep *steps, int stepCount, int targetCount, int timeSteps,
                         int selectedTarget, bool dropped,
                         const Coord &finalDropPoint, double currentTime) {
    json data;
    data["config"] = {
        {"startPos", coordToJson(config.startPos)},
        {"altitude", config.altitude},
        {"initialDir", config.initialDir},
        {"attackSpeed", config.attackSpeed},
        {"accelPath", config.accelPath},
        {"ammoName", config.ammoName},
        {"arrayTimeStep", config.arrayTimeStep},
        {"simTimeStep", config.simTimeStep},
        {"hitRadius", config.hitRadius},
        {"angularSpeed", config.angularSpeed},
        {"turnThreshold", config.turnThreshold}
    };
    data["summary"] = {
        {"stepCount", stepCount},
        {"targetCount", targetCount},
        {"timeSteps", timeSteps},
        {"selectedTarget", selectedTarget},
        {"dropped", dropped},
        {"finalDropPoint", coordToJson(finalDropPoint)},
        {"currentTime", currentTime}
    };
    data["steps"] = json::array();
    for (int i = 0; i < stepCount; i++) {
        data["steps"].push_back(stepToJson(steps[i]));
    }

    std::ofstream fout(path);
    if (!fout.is_open()) {
        std::cout << "Error: cannot create " << path << "\n";
        return false;
    }
    fout << std::setw(2) << data << "\n";
    return true;
}

int main() {
    DroneConfig config = {};
    AmmoParams *ammoTable = nullptr;
    Coord **targetsInTime = nullptr;
    SimStep *steps = nullptr;
    int ammoCount = 0;
    int targetCount = 0;
    int timeSteps = 0;
    int maxSteps = 0;

    auto cleanup = [&]() {
        delete[] ammoTable;
        ammoTable = nullptr;
        freeTargets(targetsInTime, targetCount);
        delete[] steps;
        steps = nullptr;
    };

    if (!loadConfig("config.json", config, maxSteps)) {
        cleanup();
        return 1;
    }
    if (!loadAmmo("ammo.json", ammoTable, ammoCount)) {
        cleanup();
        return 1;
    }
    if (!loadTargets("targets.json", targetsInTime, targetCount, timeSteps)) {
        cleanup();
        return 1;
    }

    int ammoIdx = findAmmo(ammoTable, ammoCount, config.ammoName);
    if (ammoIdx < 0) {
        std::cout << "Error: unknown ammo type\n";
        cleanup();
        return 1;
    }

    if (config.simTimeStep <= 0.0f || config.arrayTimeStep <= 0.0f ||
        config.accelPath <= 0.0f || config.attackSpeed <= 0.0f) {
        std::cout << "Error: invalid numeric parameters (step/accel/speed must be > 0)\n";
        cleanup();
        return 1;
    }
    if (config.altitude <= 0.0f) {
        std::cout << "Error: invalid altitude (must be > 0)\n";
        cleanup();
        return 1;
    }
    if (config.angularSpeed <= 0.0f) {
        std::cout << "Error: invalid angularSpeed (must be > 0)\n";
        cleanup();
        return 1;
    }
    if (config.hitRadius <= 0.0f) {
        std::cout << "Error: invalid hitRadius (must be > 0)\n";
        cleanup();
        return 1;
    }
    if (config.turnThreshold < 0.0f) {
        std::cout << "Error: invalid turnThreshold (must be >= 0)\n";
        cleanup();
        return 1;
    }
    if (maxSteps <= 0 || ammoCount <= 0 || targetCount <= 0 || timeSteps <= 0) {
        std::cout << "Error: invalid dynamic sizes in JSON files\n";
        cleanup();
        return 1;
    }

    steps = new SimStep[maxSteps + 1];
    const AmmoParams &selectedAmmo = ammoTable[ammoIdx];

    double accel = (double)config.attackSpeed * (double)config.attackSpeed /
                   (2.0 * (double)config.accelPath);

    Coord dronePos = config.startPos;
    double dir = config.initialDir;
    double speed = 0.0;
    DroneState state = ACCELERATING;
    double currentTime = 0.0;
    int selectedTarget = -1;
    double turnRemainingTime = 0.0;
    int stepCount = 0;

    bool dropped = false;
    Coord finalDropPoint = dronePos;

    while (stepCount < maxSteps && !dropped) {
        double bestTotal = 1e18;
        int bestTarget = -1;
        Coord bestDropPoint = {0.0f, 0.0f};
        Coord bestPredictedTarget = {0.0f, 0.0f};
        Coord bestAimPoint = {0.0f, 0.0f};

        for (int i = 0; i < targetCount; i++) {
            Coord dropPoint;
            Coord predictedTarget;
            double arrivalTime;
            double tOfFlight;

            bool ok = computeDropPoint(targetsInTime, timeSteps, i,
                                       currentTime, config.arrayTimeStep, config.simTimeStep,
                                       (double)config.altitude, (double)config.attackSpeed, selectedAmmo,
                                       dronePos, speed, accel,
                                       dropPoint, predictedTarget, arrivalTime, tOfFlight);
            if (!ok) continue;

            double total = arrivalTime + tOfFlight;

            if (i != selectedTarget && selectedTarget >= 0) {
                Coord toDropPoint = dropPoint - dronePos;
                double desiredDir = std::atan2(toDropPoint.y, toDropPoint.x);
                double deltaAngle = normAngle(desiredDir - dir);
                double stopTime = computeTimeToStop(state, speed, (double)config.attackSpeed,
                                                    accel, turnRemainingTime);

                if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                    double turnTime = std::fabs(deltaAngle) / (double)config.angularSpeed;
                    double newTravel = estimateDroneTravelTime(length(toDropPoint), 0.0,
                                                               (double)config.attackSpeed, accel);
                    total = stopTime + turnTime + newTravel + tOfFlight;
                } else {
                    double turnTime = std::fabs(deltaAngle) / (double)config.angularSpeed;
                    total = arrivalTime + tOfFlight + turnTime;
                }
            }

            if (total < bestTotal) {
                bestTotal = total;
                bestTarget = i;
                bestDropPoint = dropPoint;
                bestPredictedTarget = predictedTarget;
                bestAimPoint = predictedTarget;
            }
        }

        if (bestTarget < 0) {
            steps[stepCount] = {
                dronePos,
                static_cast<float>(dir),
                (int)state,
                selectedTarget,
                dronePos,
                dronePos,
                dronePos
            };
            stepCount++;
            break;
        }

        selectedTarget = bestTarget;
        finalDropPoint = bestDropPoint;

        steps[stepCount] = {
            dronePos,
            static_cast<float>(dir),
            (int)state,
            selectedTarget,
            bestDropPoint,
            bestAimPoint,
            bestPredictedTarget
        };
        stepCount++;

        Coord toBestDropPoint = bestDropPoint - dronePos;
        double distToDrop = length(toBestDropPoint);
        if (distToDrop <= (double)config.hitRadius) {
            dropped = true;
            break;
        }

        double desiredDir = std::atan2(toBestDropPoint.y, toBestDropPoint.x);
        double deltaAngle = normAngle(desiredDir - dir);
        double maxTurn = (double)config.angularSpeed * (double)config.simTimeStep;

        if (state != TURNING && state != DECELERATING) {
            if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                state = (speed > EPS) ? DECELERATING : TURNING;
            }
        }

        if (state == TURNING) {
            if (std::fabs(deltaAngle) <= maxTurn) {
                dir = desiredDir;
                turnRemainingTime = 0.0;
                state = ACCELERATING;
            } else {
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
                turnRemainingTime = std::fabs(deltaAngle) / (double)config.angularSpeed -
                                    (double)config.simTimeStep;
                if (turnRemainingTime < 0.0) turnRemainingTime = 0.0;
            }
        } else if (state == DECELERATING) {
            double newSpeed = speed - accel * (double)config.simTimeStep;
            if (newSpeed > 0.0) {
                double avgV = (speed + newSpeed) / 2.0;
                dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * (double)config.simTimeStep);
                dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * (double)config.simTimeStep);
                speed = newSpeed;
            } else {
                double tDecel = speed / accel;
                double avgV = speed / 2.0;
                dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * tDecel);
                dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * tDecel);
                speed = 0.0;

                double tRem = (double)config.simTimeStep - tDecel;
                if (tRem < 0.0) tRem = 0.0;
                double turnPart = (double)config.angularSpeed * tRem;

                if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                    if (std::fabs(deltaAngle) <= turnPart) {
                        dir = desiredDir;
                        turnRemainingTime = 0.0;
                        state = ACCELERATING;
                    } else {
                        dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);
                        state = TURNING;
                        turnRemainingTime = std::fabs(deltaAngle) / (double)config.angularSpeed - tRem;
                        if (turnRemainingTime < 0.0) turnRemainingTime = 0.0;
                    }
                } else {
                    if (std::fabs(deltaAngle) <= turnPart) dir = desiredDir;
                    else dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);

                    state = ACCELERATING;
                    double newV = accel * tRem;
                    if (newV > (double)config.attackSpeed) newV = (double)config.attackSpeed;
                    double avg = newV / 2.0;
                    dronePos.x = static_cast<float>(dronePos.x + avg * std::cos(dir) * tRem);
                    dronePos.y = static_cast<float>(dronePos.y + avg * std::sin(dir) * tRem);
                    speed = newV;
                    if (speed >= (double)config.attackSpeed - EPS) state = MOVING;
                }
            }
        } else {
            if (std::fabs(deltaAngle) <= maxTurn) {
                dir = desiredDir;
            } else {
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
            }

            if (state == STOPPED) {
                state = ACCELERATING;
            }

            if (state == ACCELERATING) {
                double newSpeed = speed + accel * (double)config.simTimeStep;
                if (newSpeed >= (double)config.attackSpeed) {
                    double tAccel = ((double)config.attackSpeed - speed) / accel;
                    if (tAccel < 0.0) tAccel = 0.0;
                    double avgV1 = (speed + (double)config.attackSpeed) / 2.0;
                    double tMove = (double)config.simTimeStep - tAccel;
                    dronePos.x = static_cast<float>(dronePos.x + avgV1 * std::cos(dir) * tAccel
                                 + (double)config.attackSpeed * std::cos(dir) * tMove);
                    dronePos.y = static_cast<float>(dronePos.y + avgV1 * std::sin(dir) * tAccel
                                 + (double)config.attackSpeed * std::sin(dir) * tMove);
                    speed = (double)config.attackSpeed;
                    state = MOVING;
                } else {
                    double avgV = (speed + newSpeed) / 2.0;
                    dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * (double)config.simTimeStep);
                    dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * (double)config.simTimeStep);
                    speed = newSpeed;
                }
            } else if (state == MOVING) {
                dronePos.x = static_cast<float>(dronePos.x + (double)config.attackSpeed * std::cos(dir) *
                             (double)config.simTimeStep);
                dronePos.y = static_cast<float>(dronePos.y + (double)config.attackSpeed * std::sin(dir) *
                             (double)config.simTimeStep);
            }
        }

        currentTime += (double)config.simTimeStep;
    }

    if (!writeSimulationJson("simulation.json", config, steps, stepCount, targetCount, timeSteps,
                             selectedTarget, dropped, finalDropPoint, currentTime)) {
        cleanup();
        return 1;
    }

    std::cout << "Simulation finished. Steps: " << stepCount << "\n";
    std::cout << "Selected target at end: " << selectedTarget << "\n";
    std::cout << "Dropped: " << (dropped ? "YES" : "NO (limit or no target)") << "\n";

    cleanup();
    return 0;
}

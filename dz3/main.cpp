// Стандартні заголовки
#define _USE_MATH_DEFINES
#include <cmath>
#include <cctype>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// Бібліотека JSON
#include "json.hpp"

using nlohmann::json;

// Перемикачі логування
#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

// Звичайний лог
#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
#define LOG(msg)
#endif

// Налагоджувальний лог
#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
#define DEBUG(msg)
#endif

// Гравітація
const double G = 9.81;
// Похибка для порівняння
const double EPS = 1e-9;

// Стани дрона
enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

// 2D-координата з базовими операціями
struct Coord {
    float x;
    float y;

    // Сума координат
    Coord operator+(const Coord& other) const {
        Coord result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
    }

    // Різниця координат
    Coord operator-(const Coord& other) const {
        Coord result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
    }

    // Множення на скаляр
    Coord operator*(float scalar) const {
        Coord result;
        result.x = x * scalar;
        result.y = y * scalar;
        return result;
    }

    // Ділення на скаляр
    Coord operator/(float scalar) const {
        Coord result;
        result.x = x / scalar;
        result.y = y / scalar;
        return result;
    }

    // Порівняння з похибкою
    bool operator==(const Coord& other) const {
        return std::fabs(x - other.x) < EPS && std::fabs(y - other.y) < EPS;
    }

    // Додавання з присвоєнням
    Coord& operator+=(const Coord& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    // Віднімання з присвоєнням
    Coord& operator-=(const Coord& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
};

// Параметри боєприпасу
struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

// Конфігурація дрона
struct DroneConfig {
    Coord startPos;      // Початкова позиція
    float altitude;      // Висота польоту
    float initialDir;    // Початковий напрямок
    float attackSpeed;   // Крейсерська швидкість
    float accelPath;     // Дистанція розгону
    char ammoName[32];   // Назва боєприпасу
    float arrayTimeStep; // Крок часу цілей
    float simTimeStep;   // Крок симуляції
    float hitRadius;     // Радіус скидання
    float angularSpeed;  // Кутова швидкість
    float turnThreshold; // Поріг розвороту
};

// Стан дрона на одному кроці
struct SimStep {
    Coord pos;             // Поточна позиція
    Coord dropPoint;       // Точка скидання
    Coord predictedTarget; // Прогнозована позиція цілі
    float direction;       // Напрямок руху
    int state;             // Стан дрона
    int targetIdx;         // Індекс цілі
};

// Точка траєкторії боєприпасу
struct ProjectilePoint {
    double t;  // Час від скидання
    Coord pos; // Горизонтальна позиція
    double z;  // Висота
};

// Результат одного прогону
struct SimulationRunResult {
    int stepCount = 0;                           // Кількість кроків
    int selectedTarget = -1;                     // Вибрана ціль
    bool dropped = false;                        // Ознака скидання
    Coord finalDropPoint = {0.0f, 0.0f};         // Фінальна точка скидання
    double currentTime = 0.0;                    // Поточний час
    int dropStepIdx = -1;                        // Крок скидання
    Coord dropOrigin = {0.0f, 0.0f};             // Позиція скидання
    Coord targetPoint = {0.0f, 0.0f};            // Точка цілі
    double dropTOfFlight = 0.0;                  // Час польоту
    std::vector<SimStep> steps;                  // Кроки симуляції
    std::vector<ProjectilePoint> projectilePath; // Траєкторія боєприпасу
};

// Довжина вектора
float length(const Coord& coord) {
    return std::hypot(coord.x, coord.y);
}

// Нормалізація вектора
Coord normalize(const Coord& coord) {
    float len = length(coord);
    if (len < (float)EPS)
        return {0.0f, 0.0f};
    return coord / len;
}

// Запис координати у JSON-потік
void writeCoordJson(std::ostream& out, const Coord& coord) {
    out << "{\"x\": " << coord.x << ", \"y\": " << coord.y << "}";
}

// Запис кроку симуляції у JSON-потік
void writeStepJson(std::ostream& out, const SimStep& step, const char* indent) {
    out << indent << "{\n";
    out << indent << "  \"position\": ";
    writeCoordJson(out, step.pos);
    out << ",\n";
    out << indent << "  \"direction\": " << step.direction << ",\n";
    out << indent << "  \"state\": " << step.state << ",\n";
    out << indent << "  \"targetIndex\": " << step.targetIdx << ",\n";
    out << indent << "  \"dropPoint\": ";
    writeCoordJson(out, step.dropPoint);
    out << ",\n";
    out << indent << "  \"aimPoint\": ";
    writeCoordJson(out, step.predictedTarget);
    out << ",\n";
    out << indent << "  \"predictedTarget\": ";
    writeCoordJson(out, step.predictedTarget);
    out << "\n"
        << indent << "}";
}

// Запис точки траєкторії у JSON-потік
void writeProjectilePointJson(std::ostream& out, const ProjectilePoint& point, const char* indent) {
    out << indent << "{\n";
    out << indent << "  \"t\": " << point.t << ",\n";
    out << indent << "  \"pos\": {\"x\": " << point.pos.x
        << ", \"y\": " << point.pos.y
        << ", \"z\": " << point.z << "}\n";
    out << indent << "}";
}

// Читання JSON з файла
bool loadJsonFile(const char* path, json& data) {
    std::ifstream fin(path);
    if (!fin.is_open()) {
        std::cout << "Error: cannot open " << path << "\n";
        return false;
    }

    try {
        fin >> data;
    } catch (const std::exception& e) {
        std::cout << "Error: invalid JSON in " << path << ": " << e.what() << "\n";
        return false;
    }
    return true;
}

// Читання координати з JSON
bool readCoordObject(const json& node, Coord& coord) {
    if (!node.is_object() || !node.contains("x") || !node.contains("y"))
        return false;
    if (!node.at("x").is_number() || !node.at("y").is_number())
        return false;
    coord.x = node.at("x").get<float>();
    coord.y = node.at("y").get<float>();
    return true;
}

// Завантаження конфігурації дрона
bool loadConfig(const char* path, DroneConfig& config, int& maxSteps) {
    json data;
    if (!loadJsonFile(path, data))
        return false;

    try {
        if (!data.contains("drone") || !data.at("drone").is_object()) {
            std::cout << "Error: config.json must contain drone object\n";
            return false;
        }
        if (!data.contains("simulation") || !data.at("simulation").is_object()) {
            std::cout << "Error: config.json must contain simulation object\n";
            return false;
        }

        const json& drone = data.at("drone");
        const json& simulation = data.at("simulation");

        // Обов'язкова стартова позиція
        if (!drone.contains("position") || !readCoordObject(drone.at("position"), config.startPos)) {
            std::cout << "Error: config.json has invalid drone.position\n";
            return false;
        }

        // Основні параметри польоту
        config.altitude = drone.at("altitude").get<float>();
        config.initialDir = drone.at("initialDirection").get<float>();
        config.attackSpeed = drone.at("attackSpeed").get<float>();
        config.accelPath = drone.at("accelerationPath").get<float>();

        // Перевірка довжини назви боєприпасу
        std::string ammoName = data.at("ammo").get<std::string>();
        if (ammoName.size() >= sizeof(config.ammoName)) {
            std::cout << "Error: ammo name is too long\n";
            return false;
        }
        std::strcpy(config.ammoName, ammoName.c_str());

        // Параметри симуляції та повороту
        config.arrayTimeStep = data.at("targetArrayTimeStep").get<float>();
        config.simTimeStep = simulation.at("timeStep").get<float>();
        config.hitRadius = simulation.at("hitRadius").get<float>();
        config.angularSpeed = drone.at("angularSpeed").get<float>();
        config.turnThreshold = drone.at("turnThreshold").get<float>();
        // Максимальна кількість кроків
        maxSteps = data.contains("maxSteps") ? data.at("maxSteps").get<int>() : 0;
    } catch (const std::exception& e) {
        std::cout << "Error: invalid config.json format: " << e.what() << "\n";
        return false;
    }

    return true;
}

// Завантаження таблиці боєприпасів
bool loadAmmo(const char* path, std::vector<AmmoParams>& ammoTable, int& ammoCount) {
    json data;
    if (!loadJsonFile(path, data))
        return false;
    // Очікується непорожній масив
    if (!data.is_array() || data.empty()) {
        std::cout << "Error: ammo.json must be a non-empty array\n";
        return false;
    }

    ammoCount = (int)data.size();
    // Виділяємо місце під таблицю
    ammoTable.resize(ammoCount);

    for (int i = 0; i < ammoCount; i++) {
        try {
            // Перевірка довжини назви
            std::string name = data.at(i).at("name").get<std::string>();
            if (name.size() >= sizeof(ammoTable[i].name)) {
                std::cout << "Error: ammo name is too long at index " << i << "\n";
                return false;
            }
            std::strcpy(ammoTable[i].name, name.c_str());
            // Фізичні параметри
            ammoTable[i].mass = data.at(i).at("mass").get<float>();
            ammoTable[i].drag = data.at(i).at("drag").get<float>();
            ammoTable[i].lift = data.at(i).at("lift").get<float>();
        } catch (const std::exception& e) {
            std::cout << "Error: invalid ammo.json entry at index " << i << ": " << e.what() << "\n";
            return false;
        }
    }

    return true;
}

// Читання цілей у плоский масив
bool loadTargets(const char* path, std::vector<Coord>& targets, int& targetCount, int& timeSteps) {
    json data;
    if (!loadJsonFile(path, data))
        return false;
    // Очікується масив цілей
    if (!data.contains("targets") || !data.at("targets").is_array() || data.at("targets").empty()) {
        std::cout << "Error: targets.json must contain a non-empty targets array\n";
        return false;
    }

    const json& targetArray = data.at("targets");
    if (!data.contains("targetCount") || !data.contains("timeSteps")) {
        std::cout << "Error: targets.json must contain targetCount and timeSteps\n";
        return false;
    }

    targetCount = data.at("targetCount").get<int>();
    timeSteps = data.at("timeSteps").get<int>();
    if (targetCount <= 0 || timeSteps <= 0) {
        std::cout << "Error: targetCount and timeSteps must be positive\n";
        return false;
    }
    if ((int)targetArray.size() != targetCount) {
        std::cout << "Error: targetCount does not match targets array size\n";
        return false;
    }

    targets.resize((size_t)targetCount * (size_t)timeSteps);

    for (int i = 0; i < targetCount; i++) {
        // Кожна ціль має мати `timeSteps` позицій
        if (!targetArray.at(i).is_object() ||
            !targetArray.at(i).contains("positions") ||
            !targetArray.at(i).at("positions").is_array() ||
            (int)targetArray.at(i).at("positions").size() != timeSteps) {
            std::cout << "Error: each target must contain positions with timeSteps entries\n";
            return false;
        }

        const json& positions = targetArray.at(i).at("positions");
        for (int j = 0; j < timeSteps; j++) {
            if (!readCoordObject(positions.at(j), targets[(size_t)i * (size_t)timeSteps + (size_t)j])) {
                std::cout << "Error: invalid target coordinate at [" << i << "][" << j << "]\n";
                return false;
            }
        }
    }

    return true;
}

// Пошук боєприпасу за назвою
int findAmmo(const std::vector<AmmoParams>& ammoTable, const char* name) {
    int ammoCount = (int)ammoTable.size();
    for (int i = 0; i < ammoCount; i++) {
        if (std::strcmp(name, ammoTable[i].name) == 0)
            return i;
    }
    return -1;
}

// Розв'язання кубічного рівняння методом Кардано.
// Повертає найменший додатний корінь.
double solveCardanoTime(double a, double b, double c, bool& ok) {
    ok = false;
    // Вироджений випадок
    if (std::fabs(a) < EPS)
        return 0.0;

    // Зведення до форми `t^3 + p*t + q = 0`
    double A = b / a;
    double p = -(A * A) / 3.0;
    double q = (2.0 * A * A * A) / 27.0 + c / a;
    double shift = A / 3.0; // Зсув до початкової змінної
    // Дискримінант визначає кількість дійсних коренів
    double discriminant = (q * q) / 4.0 + (p * p * p) / 27.0;

    std::vector<double> roots;
    roots.reserve(3);
    // Уникаємо дублювання однакових коренів
    auto addRoot = [&](double root) {
        for (size_t i = 0; i < roots.size(); i++) {
            if (std::fabs(roots[i] - root) < 1e-7)
                return;
        }
        roots.push_back(root);
    };

    if (discriminant > EPS) {
        // Один дійсний корінь
        double sqrtDisc = std::sqrt(discriminant);
        double u = std::cbrt(-q / 2.0 + sqrtDisc);
        double v = std::cbrt(-q / 2.0 - sqrtDisc);
        addRoot(u + v - shift);
    } else if (std::fabs(discriminant) <= EPS) {
        // Граничний випадок з подвійним коренем
        double u = std::cbrt(-q / 2.0);
        addRoot(2.0 * u - shift);
        addRoot(-u - shift);
    } else {
        // Три дійсні корені
        double acosArg = (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
        // Обмежуємо аргумент `acos`
        if (acosArg < -1.0)
            acosArg = -1.0;
        if (acosArg > 1.0)
            acosArg = 1.0;

        double phi = std::acos(acosArg);
        double rootBase = 2.0 * std::sqrt(-p / 3.0);
        addRoot(rootBase * std::cos(phi / 3.0) - shift);
        addRoot(rootBase * std::cos((phi + 2.0 * M_PI) / 3.0) - shift);
        addRoot(rootBase * std::cos((phi + 4.0 * M_PI) / 3.0) - shift);
    }

    // Беремо найменший додатний корінь
    double best = 0.0;
    bool found = false;
    for (size_t i = 0; i < roots.size(); i++) {
        if (roots[i] <= EPS)
            continue; // Ігноруємо недодатні корені
        if (!found || roots[i] < best) {
            best = roots[i];
            found = true;
        }
    }

    if (!found)
        return 0.0;
    ok = true;
    return best;
}

// Горизонтальна дальність польоту
double calcHorizontalDistance(double t, const AmmoParams& ammo, double V0) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    // Базовий член
    double term1 = V0 * t;
    // Поправки на опір і підйомну силу
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
    // Підсумкова дальність
    return term1 + term2 + term3 + term4 + term5;
}

// Висота боєприпасу в момент `t`
double calcAltitude(double t, double z0, const AmmoParams& ammo, double V0) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    double a = d * G * m - 2.0 * d * d * l * V0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * V0;
    double z = z0 + (b * t * t) / (6.0 * m * m) + (a * t * t * t) / (6.0 * m * m);
    // Висота не може бути від'ємною
    if (z < 0.0)
        z = 0.0;
    return z;
}

// Обчислення часу польоту і дальності
bool computeBallistics(double altitude, double V0, const AmmoParams& ammo,
                       double& tOfFlight, double& hDist) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    // Коефіцієнти рівняння для часу падіння
    double a = d * G * m - 2.0 * d * d * l * V0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * V0;
    double c = 6.0 * m * m * altitude;

    bool ok = false;
    // Час падіння
    double t = solveCardanoTime(a, b, c, ok);
    if (!ok || t <= EPS)
        return false;

    // Горизонтальна дальність
    double h = calcHorizontalDistance(t, ammo, V0);
    if (h <= EPS)
        return false;

    tOfFlight = t;
    hDist = h;
    return true;
}

// Лінійна інтерполяція позиції цілі
Coord interpTarget(const std::vector<Coord>& targetsInTime, int timeSteps, int targetIdx,
                   double t, double arrayTimeStep) {
    double tt = t / arrayTimeStep;
    int idx = ((int)std::floor(tt)) % timeSteps;
    if (idx < 0)
        idx += timeSteps;              // Корекція для від'ємного часу
    int next = (idx + 1) % timeSteps;  // Наступний кадр
    double frac = tt - std::floor(tt); // Частка між кадрами

    // Інтерполяція між двома кадрами
    const Coord& current = targetsInTime[(size_t)targetIdx * (size_t)timeSteps + (size_t)idx];
    const Coord& nextCoord = targetsInTime[(size_t)targetIdx * (size_t)timeSteps + (size_t)next];
    Coord result;
    result.x = static_cast<float>(current.x + (nextCoord.x - current.x) * frac);
    result.y = static_cast<float>(current.y + (nextCoord.y - current.y) * frac);
    return result;
}

// Нормалізація кута до `[-PI; PI]`
double normAngle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

// Оцінка часу проходження дистанції
double estimateDroneTravelTime(double dist, double speed, double attackSpeed, double accel) {
    if (dist <= 0.0)
        return 0.0; // Немає руху
    if (accel <= EPS)
        return dist / attackSpeed; // Рух без розгону
    if (speed >= attackSpeed)
        return dist / attackSpeed; // Уже на крейсерській

    // Дистанція до крейсерської швидкості
    double distToCruise = (attackSpeed * attackSpeed - speed * speed) / (2.0 * accel);
    if (distToCruise >= dist) {
        // Розгін без виходу на крейсерську
        double disc = speed * speed + 2.0 * accel * dist;
        if (disc < 0.0)
            disc = 0.0;
        return (-speed + std::sqrt(disc)) / accel;
    }

    // Розгін, потім рух на крейсерській
    double tAccel = (attackSpeed - speed) / accel;
    double tCruise = (dist - distToCruise) / attackSpeed;
    return tAccel + tCruise;
}

// Обчислення точки скидання для цілі
bool computeDropPoint(const std::vector<Coord>& targetPositions, int timeSteps, int targetIdx,
                      double currentTime, double arrayTimeStep, double simTimeStep,
                      double altitude, double V0, const AmmoParams& ammo,
                      const Coord& dronePos, double droneSpeed, double accel,
                      Coord& dropPoint, Coord& predictedTarget,
                      double& arrivalTime, double& tOfFlight) {
    // 1. Поточна позиція цілі
    Coord currentTarget = interpTarget(targetPositions, timeSteps, targetIdx, currentTime, arrayTimeStep);

    // 2. Балістика боєприпасу
    double hDist;
    if (!computeBallistics(altitude, V0, ammo, tOfFlight, hDist))
        return false;

    // 3. Оцінка точки скидання
    Coord toCurrentTarget = currentTarget - dronePos;
    double currentDistance = length(toCurrentTarget);
    if (currentDistance < EPS)
        return false;

    // Попередня відстань до точки скидання
    double approxDist = currentDistance - hDist;
    if (approxDist < 0.0)
        approxDist = 0.0;
    // Час підльоту і падіння
    double approxArrival = estimateDroneTravelTime(approxDist, droneSpeed, V0, accel);
    double totalTime = approxArrival + tOfFlight;

    // 4. Прогноз руху цілі
    Coord targetNow = interpTarget(targetPositions, timeSteps, targetIdx, currentTime, arrayTimeStep);
    Coord targetNext = interpTarget(targetPositions, timeSteps, targetIdx, currentTime + simTimeStep, arrayTimeStep);
    Coord targetVelocity = (targetNext - targetNow) / (float)simTimeStep;

    // Прогнозована позиція цілі
    predictedTarget = targetNow + targetVelocity * (float)totalTime;

    Coord toPredictedTarget = predictedTarget - dronePos;
    double predictedDistance = length(toPredictedTarget);
    if (predictedDistance < EPS)
        return false;

    // 5. Остаточна точка скидання
    dropPoint = predictedTarget - normalize(toPredictedTarget) * (float)hDist;
    Coord toDropPoint = dropPoint - dronePos;
    // Час до точки скидання
    arrivalTime = estimateDroneTravelTime(length(toDropPoint), droneSpeed, V0, accel);
    return true;
}

// Побудова траєкторії боєприпасу
bool buildProjectilePath(const DroneConfig& config, const AmmoParams& ammo,
                         const SimStep* steps, int stepCount, bool dropped,
                         int& dropStepIdx, Coord& dropOrigin, Coord& targetPoint,
                         double& tOfFlight, std::vector<ProjectilePoint>& projectilePath) {
    // Початкові значення
    projectilePath.clear();
    dropStepIdx = -1;
    dropOrigin = {0.0f, 0.0f};
    targetPoint = {0.0f, 0.0f};
    tOfFlight = 0.0;

    // Без скидання траєкторія порожня
    if (!dropped || stepCount <= 0)
        return true;

    // Параметри польоту
    double hDist = 0.0;
    if (!computeBallistics((double)config.altitude, (double)config.attackSpeed, ammo, tOfFlight, hDist)) {
        return false;
    }

    // Останній крок відповідає скиданню
    dropStepIdx = stepCount - 1;
    dropOrigin = steps[dropStepIdx].pos;
    targetPoint = steps[dropStepIdx].predictedTarget;

    // Напрямок польоту боєприпасу
    Coord dir = {
        static_cast<float>(std::cos(steps[dropStepIdx].direction)),
        static_cast<float>(std::sin(steps[dropStepIdx].direction))};
    // Запасний спосіб визначення напрямку
    if (length(dir) < EPS) {
        dir = normalize(steps[dropStepIdx].dropPoint - dropOrigin);
        if (length(dir) < EPS) {
            dir = normalize(targetPoint - dropOrigin);
            if (length(dir) < EPS)
                return false;
        }
    }

    // Дискретизація траєкторії
    const int samples = 64;
    projectilePath.reserve(samples + 1);
    for (int i = 0; i <= samples; i++) {
        double t = tOfFlight * i / samples;
        double h = calcHorizontalDistance(t, ammo, (double)config.attackSpeed);
        ProjectilePoint point;
        point.t = t;
        point.pos = dropOrigin + dir * (float)h;                                              // Позиція в XY
        point.z = calcAltitude(t, (double)config.altitude, ammo, (double)config.attackSpeed); // Висота
        projectilePath.push_back(point);
    }

    return true;
}

// Час до повної зупинки
double computeTimeToStop(DroneState state, double speed, double attackSpeed,
                         double accel, double turnRemainingTime) {
    switch (state) {
    case STOPPED:
        return 0.0; // Уже зупинений
    case ACCELERATING:
        return speed / accel; // Зупинка під час розгону
    case MOVING:
        return attackSpeed / accel; // Зупинка з крейсерської
    case DECELERATING:
        return speed / accel; // Завершення гальмування
    case TURNING:
        return turnRemainingTime; // Завершення повороту
    }
    return 0.0;
}

// Запис результату симуляції у JSON
bool writeSimulationJson(const char* path, const DroneConfig& config,
                         const AmmoParams& selectedAmmo,
                         SimStep* steps, int stepCount, int targetCount, int timeSteps,
                         int selectedTarget, bool dropped,
                         const Coord& finalDropPoint, double currentTime) {
    // Поки що не використовуються
    (void)config;
    (void)selectedAmmo;
    (void)targetCount;
    (void)timeSteps;
    (void)selectedTarget;
    (void)dropped;
    (void)finalDropPoint;
    (void)currentTime;
    std::ofstream fout(path);
    if (!fout.is_open()) {
        std::cout << "Error: cannot create " << path << "\n";
        return false;
    }
    fout << std::setprecision(std::numeric_limits<double>::max_digits10);
    fout << "{\n";
    fout << "  \"totalSteps\": " << stepCount << ",\n";
    fout << "  \"steps\": [";
    if (stepCount > 0)
        fout << "\n";
    for (int i = 0; i < stepCount; i++) {
        writeStepJson(fout, steps[i], "    ");
        if (i + 1 < stepCount)
            fout << ",";
        fout << "\n";
    }
    if (stepCount > 0) {
        fout << "  ]\n";
    } else {
        fout << "]\n";
    }
    fout << "}\n";
    return true;
}

// Запис траєкторії боєприпасу у JSON
bool writeProjectileJson(const char* path, bool dropped,
                         int dropStepIdx, const Coord& dropOrigin, const Coord& targetPoint,
                         double dropTOfFlight, const std::vector<ProjectilePoint>& projectilePath) {
    std::ofstream fout(path);
    if (!fout.is_open()) {
        std::cout << "Error: cannot create " << path << "\n";
        return false;
    }
    fout << std::setprecision(std::numeric_limits<double>::max_digits10);
    fout << "{\n";
    fout << "  \"projectile\": {\n";
    fout << "    \"available\": " << ((dropped && !projectilePath.empty()) ? "true" : "false") << ",\n";
    fout << "    \"dropStepIdx\": " << dropStepIdx << ",\n";
    fout << "    \"dropOrigin\": ";
    writeCoordJson(fout, dropOrigin);
    fout << ",\n";
    fout << "    \"targetPoint\": ";
    writeCoordJson(fout, targetPoint);
    fout << ",\n";
    fout << "    \"timeOfFlight\": " << dropTOfFlight << ",\n";
    fout << "    \"points\": [";
    if (!projectilePath.empty())
        fout << "\n";
    for (size_t i = 0; i < projectilePath.size(); i++) {
        writeProjectilePointJson(fout, projectilePath[i], "      ");
        if (i + 1 < projectilePath.size())
            fout << ",";
        fout << "\n";
    }
    if (!projectilePath.empty()) {
        fout << "    ]\n";
    } else {
        fout << "]\n";
    }
    fout << "  }\n";
    fout << "}\n";
    return true;
}

// Побудова слага для імені файла
std::string makeAmmoFileSlug(const char* ammoName) {
    std::string slug;
    slug.reserve(std::strlen(ammoName));
    for (size_t i = 0; ammoName[i] != '\0'; i++) {
        unsigned char ch = static_cast<unsigned char>(ammoName[i]);
        if (std::isalnum(ch)) {
            // Букви і цифри залишаємо
            slug.push_back(static_cast<char>(std::tolower(ch)));
        } else if (!slug.empty() && slug.back() != '_') {
            // Роздільники замінюємо на `_`
            slug.push_back('_');
        }
    }

    // Видаляємо `_` в кінці
    while (!slug.empty() && slug.back() == '_') {
        slug.pop_back();
    }
    // Запасна назва
    if (slug.empty()) {
        slug = "ammo";
    }
    return slug;
}

// Симуляція для одного боєприпасу
bool runSingleAmmoSimulation(const DroneConfig& baseConfig, const AmmoParams& selectedAmmo,
                             const std::vector<Coord>& targetsInTime, int targetCount, int timeSteps, int maxSteps,
                             SimulationRunResult& result) {
    // Копія конфігурації для поточного боєприпасу
    DroneConfig config = baseConfig;
    std::strcpy(config.ammoName, selectedAmmo.name);

    // Початковий стан результату
    result = {};
    result.steps.reserve(maxSteps + 1);

    // Прискорення за довжиною розгону
    double accel = (double)config.attackSpeed * (double)config.attackSpeed /
                   (2.0 * (double)config.accelPath);

    // Початковий стан дрона
    Coord dronePos = config.startPos;
    double dir = config.initialDir;
    double speed = 0.0;
    DroneState state = ACCELERATING;
    double currentTime = 0.0;
    int selectedTarget = -1;
    double turnRemainingTime = 0.0;

    // Основний цикл симуляції
    while ((int)result.steps.size() < maxSteps && !result.dropped) {
        // Пошук найкращої цілі на поточному кроці
        double bestDropDistance = 1e18; // Початковий максимум
        double bestTotal = 1e18;
        int bestTarget = -1;
        Coord bestDropPoint = {0.0f, 0.0f};
        Coord bestPredictedTarget = {0.0f, 0.0f};

        for (int i = 0; i < targetCount; i++) {
            // Оцінка точки скидання для цілі
            Coord dropPoint;
            Coord predictedTarget;
            double arrivalTime;
            double tOfFlight;

            bool ok = computeDropPoint(targetsInTime, timeSteps, i,
                                       currentTime, config.arrayTimeStep, config.simTimeStep,
                                       (double)config.altitude, (double)config.attackSpeed, selectedAmmo,
                                       dronePos, speed, accel,
                                       dropPoint, predictedTarget, arrivalTime, tOfFlight);
            if (!ok)
                continue; // Пропускаємо недосяжну ціль

            // Базова оцінка часу
            double total = arrivalTime + tOfFlight;
            Coord toDropPoint = dropPoint - dronePos;
            double dropDistance = length(toDropPoint);

            // Штраф за зміну цілі
            if (i != selectedTarget && selectedTarget >= 0) {
                double desiredDir = std::atan2(toDropPoint.y, toDropPoint.x);
                double deltaAngle = normAngle(desiredDir - dir);
                // Час на завершення поточного маневру
                double stopTime = computeTimeToStop(state, speed, (double)config.attackSpeed,
                                                    accel, turnRemainingTime);

                if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                    // Повний розворот з повторним розгоном
                    double turnTime = std::fabs(deltaAngle) / (double)config.angularSpeed;
                    double newTravel = estimateDroneTravelTime(length(toDropPoint), 0.0,
                                                               (double)config.attackSpeed, accel);
                    total = stopTime + turnTime + newTravel + tOfFlight;
                } else {
                    // Невеликий поворот на ходу
                    double turnTime = std::fabs(deltaAngle) / (double)config.angularSpeed;
                    total = arrivalTime + tOfFlight + turnTime;
                }
            }

            // Пріоритет: ближча точка скидання, далі менший час
            if (dropDistance < bestDropDistance - EPS ||
                (std::fabs(dropDistance - bestDropDistance) <= EPS && total < bestTotal)) {
                bestDropDistance = dropDistance;
                bestTotal = total;
                bestTarget = i;
                bestDropPoint = dropPoint;
                bestPredictedTarget = predictedTarget;
            }
        }

        // Якщо ціль не знайдена, завершуємо цикл
        if (bestTarget < 0) {
            result.steps.push_back({dronePos,
                                    dronePos,
                                    dronePos,
                                    static_cast<float>(dir),
                                    (int)state,
                                    selectedTarget});
            break;
        }

        // Зберігаємо вибрану ціль і поточний крок
        selectedTarget = bestTarget;
        result.finalDropPoint = bestDropPoint;

        result.steps.push_back({dronePos,
                                bestDropPoint,
                                bestPredictedTarget,
                                static_cast<float>(dir),
                                (int)state,
                                selectedTarget});

        DEBUG("Step " << (int)result.steps.size()
                      << " pos=(" << dronePos.x << "," << dronePos.y << ")"
                      << " dir=" << dir
                      << " target=" << selectedTarget
                      << " state=" << (int)state);

        // Вектор до точки скидання
        Coord toBestDropPoint = bestDropPoint - dronePos;
        double distToDrop = length(toBestDropPoint);
        double desiredDir = std::atan2(toBestDropPoint.y, toBestDropPoint.x);
        double deltaAngle = normAngle(desiredDir - dir);
        // Максимальний поворот за крок
        double maxTurn = (double)config.angularSpeed * (double)config.simTimeStep;

        // Умова скидання
        if (distToDrop <= (double)config.hitRadius &&
            std::fabs(deltaAngle) <= (double)config.turnThreshold) {
            result.dropped = true;
            break;
        }

        // Перехід до розвороту
        if (state != TURNING && state != DECELERATING) {
            if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                // На швидкості спочатку гальмуємо
                state = (speed > EPS) ? DECELERATING : TURNING;
            }
        }

        // Оновлення стану руху

        if (state == TURNING) {
            // Поворот на місці
            if (std::fabs(deltaAngle) <= maxTurn) {
                // Поворот завершено
                dir = desiredDir;
                turnRemainingTime = 0.0;
                state = ACCELERATING;
            } else {
                // Продовжимо поворот на наступному кроці
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
                turnRemainingTime = std::fabs(deltaAngle) / (double)config.angularSpeed -
                                    (double)config.simTimeStep;
                if (turnRemainingTime < 0.0)
                    turnRemainingTime = 0.0;
            }
        } else if (state == DECELERATING) {
            // Гальмування
            double newSpeed = speed - accel * (double)config.simTimeStep;
            if (newSpeed > 0.0) {
                // Рух із середньою швидкістю за крок
                double avgV = (speed + newSpeed) / 2.0;
                dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * (double)config.simTimeStep);
                dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * (double)config.simTimeStep);
                speed = newSpeed;
            } else {
                // Зупинка всередині кроку
                double tDecel = speed / accel;
                double avgV = speed / 2.0;
                dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * tDecel);
                dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * tDecel);
                speed = 0.0;

                // Решта кроку для повороту або розгону
                double tRem = (double)config.simTimeStep - tDecel;
                if (tRem < 0.0)
                    tRem = 0.0;
                double turnPart = (double)config.angularSpeed * tRem;

                if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                    // Великий кут: переходимо до повороту
                    if (std::fabs(deltaAngle) <= turnPart) {
                        // Поворот завершено в цьому кроці
                        dir = desiredDir;
                        turnRemainingTime = 0.0;
                        state = ACCELERATING;
                    } else {
                        // Поворот продовжиться на наступному кроці
                        dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);
                        state = TURNING;
                        turnRemainingTime = std::fabs(deltaAngle) / (double)config.angularSpeed - tRem;
                        if (turnRemainingTime < 0.0)
                            turnRemainingTime = 0.0;
                    }
                } else {
                    // Малий кут: підрулюємо і розганяємось
                    if (std::fabs(deltaAngle) <= turnPart)
                        dir = desiredDir;
                    else
                        dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);

                    // Розгін на залишку кроку
                    state = ACCELERATING;
                    double newV = accel * tRem;
                    if (newV > (double)config.attackSpeed)
                        newV = (double)config.attackSpeed;
                    double avg = newV / 2.0;
                    dronePos.x = static_cast<float>(dronePos.x + avg * std::cos(dir) * tRem);
                    dronePos.y = static_cast<float>(dronePos.y + avg * std::sin(dir) * tRem);
                    speed = newV;
                    // Перехід до крейсерського руху
                    if (speed >= (double)config.attackSpeed - EPS)
                        state = MOVING;
                }
            }
        } else {
            // Рух уперед з підруленням
            if (std::fabs(deltaAngle) <= maxTurn) {
                dir = desiredDir;
            } else {
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
            }

            // Після зупинки знову розганяємось
            if (state == STOPPED) {
                state = ACCELERATING;
            }

            if (state == ACCELERATING) {
                // Розгін
                double newSpeed = speed + accel * (double)config.simTimeStep;
                if (newSpeed >= (double)config.attackSpeed) {
                    // Перехід на крейсерську всередині кроку
                    double tAccel = ((double)config.attackSpeed - speed) / accel;
                    if (tAccel < 0.0)
                        tAccel = 0.0;
                    double avgV1 = (speed + (double)config.attackSpeed) / 2.0;
                    double tMove = (double)config.simTimeStep - tAccel;
                    // Розгін, потім крейсерський рух
                    dronePos.x = static_cast<float>(dronePos.x + avgV1 * std::cos(dir) * tAccel + (double)config.attackSpeed * std::cos(dir) * tMove);
                    dronePos.y = static_cast<float>(dronePos.y + avgV1 * std::sin(dir) * tAccel + (double)config.attackSpeed * std::sin(dir) * tMove);
                    speed = (double)config.attackSpeed;
                    state = MOVING;
                } else {
                    // Крейсерська швидкість ще не досягнута
                    double avgV = (speed + newSpeed) / 2.0;
                    dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * (double)config.simTimeStep);
                    dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * (double)config.simTimeStep);
                    speed = newSpeed;
                }
            } else if (state == MOVING) {
                // Рух на крейсерській швидкості
                dronePos.x = static_cast<float>(dronePos.x + (double)config.attackSpeed * std::cos(dir) *
                                                                 (double)config.simTimeStep);
                dronePos.y = static_cast<float>(dronePos.y + (double)config.attackSpeed * std::sin(dir) *
                                                                 (double)config.simTimeStep);
            }
        }

        // Перехід до наступного кроку
        currentTime += (double)config.simTimeStep;
    }

    // Підсумок прогону
    result.stepCount = (int)result.steps.size();
    result.selectedTarget = selectedTarget;
    result.currentTime = currentTime;

    // Побудова траєкторії після скидання
    if (!buildProjectilePath(config, selectedAmmo, result.steps.data(), result.stepCount, result.dropped,
                             result.dropStepIdx, result.dropOrigin, result.targetPoint,
                             result.dropTOfFlight, result.projectilePath)) {
        return false;
    }

    return true;
}

// Точка входу
int main() {
    // Початкові дані
    DroneConfig config = {};
    std::vector<AmmoParams> ammoTable;
    std::vector<Coord> targetsInTime;
    int ammoCount = 0;
    int targetCount = 0;
    int timeSteps = 0;
    int maxSteps = 0;

    // 1. Завантаження конфігурації
    if (!loadConfig("config.json", config, maxSteps)) {
        return 1;
    }
    LOG("Config loaded: attackSpeed=" << config.attackSpeed
                                      << ", altitude=" << config.altitude
                                      << ", ammo=" << config.ammoName);

    // 2. Завантаження боєприпасів
    if (!loadAmmo("ammo.json", ammoTable, ammoCount)) {
        return 1;
    }
    LOG("Ammo loaded: " << ammoCount << " entries");

    // 3. Завантаження цілей
    if (!loadTargets("targets.json", targetsInTime, targetCount, timeSteps)) {
        return 1;
    }
    LOG("Targets loaded: " << targetCount << " targets, " << timeSteps << " time steps");

    // Якщо `maxSteps` не задано, оцінюємо його
    if (maxSteps <= 0) {
        double stepRatio = std::ceil((double)config.arrayTimeStep / (double)config.simTimeStep);
        if (stepRatio < 1.0)
            stepRatio = 1.0;
        maxSteps = timeSteps * (int)stepRatio * targetCount;
    }

    // Перевірка числових параметрів
    if (config.simTimeStep <= 0.0f || config.arrayTimeStep <= 0.0f ||
        config.accelPath <= 0.0f || config.attackSpeed <= 0.0f) {
        std::cout << "Error: invalid numeric parameters (step/accel/speed must be > 0)\n";
        return 1;
    }
    if (config.altitude <= 0.0f) {
        // Висота має бути додатною
        std::cout << "Error: invalid altitude (must be > 0)\n";
        return 1;
    }
    if (config.angularSpeed <= 0.0f) {
        // Кутова швидкість має бути додатною
        std::cout << "Error: invalid angularSpeed (must be > 0)\n";
        return 1;
    }
    if (config.hitRadius <= 0.0f) {
        // Радіус ураження має бути додатним
        std::cout << "Error: invalid hitRadius (must be > 0)\n";
        return 1;
    }
    if (config.turnThreshold < 0.0f) {
        std::cout << "Error: invalid turnThreshold (must be >= 0)\n";
        return 1;
    }
    if (maxSteps <= 0 || ammoCount <= 0 || targetCount <= 0 || timeSteps <= 0) {
        std::cout << "Error: invalid dynamic sizes in JSON files\n";
        return 1;
    }

    LOG("Running simulation for all ammo types: " << ammoCount);

    for (int ammoIdx = 0; ammoIdx < ammoCount; ammoIdx++) {
        const AmmoParams& selectedAmmo = ammoTable[ammoIdx];
        DroneConfig runConfig = config;
        std::strcpy(runConfig.ammoName, selectedAmmo.name);

        LOG("Ammo found: " << selectedAmmo.name);

        // Окремий прогін для кожного боєприпасу
        SimulationRunResult result;
        if (!runSingleAmmoSimulation(runConfig, selectedAmmo, targetsInTime, targetCount, timeSteps, maxSteps, result)) {
            std::cout << "Error: cannot simulate ammo " << selectedAmmo.name << "\n";
            return 1;
        }

        std::string ammoSlug = makeAmmoFileSlug(selectedAmmo.name);
        std::string simulationPath = "simulation_" + ammoSlug + ".json";
        std::string projectilePath = "projectile_" + ammoSlug + ".json";

        if (!writeSimulationJson(simulationPath.c_str(), runConfig, selectedAmmo,
                                 result.steps.data(), result.stepCount, targetCount, timeSteps,
                                 result.selectedTarget, result.dropped, result.finalDropPoint, result.currentTime)) {
            return 1;
        }

        if (!writeProjectileJson(projectilePath.c_str(), result.dropped,
                                 result.dropStepIdx, result.dropOrigin, result.targetPoint,
                                 result.dropTOfFlight, result.projectilePath)) {
            return 1;
        }

        LOG("Simulation complete for " << selectedAmmo.name << ". Steps: " << result.stepCount);
        LOG("Selected target at end: " << result.selectedTarget);
        LOG("Dropped: " << (result.dropped ? "YES" : "NO (limit or no target)"));
    }

    return 0;
}

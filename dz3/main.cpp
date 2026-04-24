// Нижче підтягуємо всяку стандартну фігню, без якої далі нікуди:
// матан, стрічки, файли, виведення - весь джентльменський набір
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

// А це той самий крутий header для JSON-а, щоб не писати парсер руками, як даун
#include "json.hpp"

using nlohmann::json;

// Перемикачі логів: як шось пішло не так - вмикаєш DEBUG і дивишся, де косяк
#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

// Звичайний лог - коли хочеться просто подивитись шо там по процесу
#if ENABLE_LOG
  #define LOG(msg) std::cout << "[LOG] " << msg << std::endl
#else
  #define LOG(msg)
#endif

// Дебажні сопли - для ситуацій, коли вже зовсім пригоріло і шось не те
#if ENABLE_DEBUG
  #define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl
#else
  #define DEBUG(msg)
#endif

// Гравітуха - тягне все донизу, як зарплату в кінці місяця
const double G = 9.81;
// Епсилон - наша паличка-виручалочка для порівняння флоатів, бо точного рівно не буває
const double EPS = 1e-9;

// Стани нашого птаха: шо він зараз робить в небі
// STOPPED - завис, як на парі в понеділок
// ACCELERATING - розганяється, газує по повній
// DECELERATING - гальмує, бо треба розвернутись
// TURNING - крутиться на місці, шукає куди чухрати
// MOVING - чешить на крейсерській, як альфач
enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

// Проста коордінатка - два флоатіка і вагон перегружених операторів,
// щоб далі писати вектори нормально, а не через жопу
struct Coord {
    float x;
    float y;

    // Плюсик - складаємо дві точки, звичайна арифметика
    Coord operator+(const Coord &other) const {
        Coord result;
        result.x = x + other.x;
        result.y = y + other.y;
        return result;
    }

    // Мінусик - рахуємо вектор від однієї точки до іншої (куда бігти)
    Coord operator-(const Coord &other) const {
        Coord result;
        result.x = x - other.x;
        result.y = y - other.y;
        return result;
    }

    // Множимо на число - розтягуємо вектор, як гумку
    Coord operator*(float scalar) const {
        Coord result;
        result.x = x * scalar;
        result.y = y * scalar;
        return result;
    }

    // Ділення - ну аналогічно, тіки навпаки, стискаємо
    Coord operator/(float scalar) const {
        Coord result;
        result.x = x / scalar;
        result.y = y / scalar;
        return result;
    }

    // Перевірка на однаковість - через епсилон, бо флоати то така халепа
    bool operator==(const Coord &other) const {
        return std::fabs(x - other.x) < EPS && std::fabs(y - other.y) < EPS;
    }

    // Інкремент вектором - зручно в циклах, щоб не писати a = a + b щоразу
    Coord &operator+=(const Coord &other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    // Та саме, тіки віднімаємо
    Coord &operator-=(const Coord &other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }
};

// Параметри нашого бахкала: шо летить, скіки важить і як гальмує в повітрі
// mass - маса кіла, drag - опір повітря (чим більше - тим гірше летить),
// lift - піднімальна сила, планеруха (якщо боєприпас вміє планерувати)
struct AmmoParams {
    char name[32];
    float mass;
    float drag;
    float lift;
};

// Повний конфіг дрона - всі налаштування в одному місці, щоб потім не бігати по коду
struct DroneConfig {
    Coord startPos;        // звідки стартуємо, типу база
    float altitude;        // висота польоту, де чухаємо над землею
    float initialDir;      // в який бік дивимся на старті (радіани)
    float attackSpeed;     // крейсерська швидкість, на якій топимо газ
    float accelPath;       // шлях розгону - скіки метрів треба щоб вийти на крейсерську
    char ammoName[32];     // назва боєприпасу, яким будемо бахкати
    float arrayTimeStep;   // крок часу в табличці цілей (з файла)
    float simTimeStep;     // крок симуляції, як часто оновлюємо стан
    float hitRadius;       // радіус попадання в точку скидання - ближче вже пофіг
    float angularSpeed;    // швидкість повороту, скіки радіан/сек може крутитись
    float turnThreshold;   // якщо кут менше цього - забиваємо на розворот, котимо так
};

// Один крок симуляції - шо дрон робив у даний момент часу
struct SimStep {
    Coord pos;             // де він був
    Coord dropPoint;       // куди планував скинути
    Coord predictedTarget; // де буде ціль коли боєприпас долетить
    float direction;       // куди дивився
    int state;             // шо робив (з нашого енума)
    int targetIdx;         // кого збирався мочити
};

// Точка на траєкторії снаряда після скидання - час і де він у просторі
struct ProjectilePoint {
    double t;              // секунди від моменту скидання
    Coord pos;             // горизонтальне положення (X, Y)
    double z;              // висота, поки летить донизу
};

// Весь результат прогону симулятора - зібрали в кучу, щоб потім віддати
struct SimulationRunResult {
    int stepCount = 0;                          // скіки кроків натупали
    int selectedTarget = -1;                    // на кого в підсумку залипли
    bool dropped = false;                       // чи кинули (чи затисли яйця і не скинули)
    Coord finalDropPoint = {0.0f, 0.0f};        // точка скидання в фіналі
    double currentTime = 0.0;                   // скіки часу просимулили
    int dropStepIdx = -1;                       // на якому кроці бахнули
    Coord dropOrigin = {0.0f, 0.0f};            // звідки випустили подарунок
    Coord targetPoint = {0.0f, 0.0f};           // куди цілились
    double dropTOfFlight = 0.0;                 // час польоту снаряда до землі
    std::vector<SimStep> steps;                 // всі кроки в масивчику
    std::vector<ProjectilePoint> projectilePath; // траєкторія снаряда для красоти
};

// Довжина вектора - класична теорема Піфагора, тіки через hypot, щоб не ловити переповнення
float length(const Coord &coord) {
    return std::hypot(coord.x, coord.y);
}

// Нормалізація - робимо вектор одиничним (довжина = 1)
// Якшо вектор нульовий - повертаємо нулі, бо ділити на нуль - гріх великий
Coord normalize(const Coord &coord) {
    float len = length(coord);
    if (len < (float)EPS) return {0.0f, 0.0f};
    return coord / len;
}

// Конвертація кординати напряму в потік - без проміжного DOM, шоб не тримати дві копії даних
void writeCoordJson(std::ostream &out, const Coord &coord) {
    out << "{\"x\": " << coord.x << ", \"y\": " << coord.y << "}";
}

// Крок симуляції теж одразу серіалізуємо у файл - формат той самий, аллокацій менше
void writeStepJson(std::ostream &out, const SimStep &step, const char *indent) {
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
    out << "\n" << indent << "}";
}

// Точку траєкторії теж сиплемо напряму в потік - без проміжного json-дерева
void writeProjectilePointJson(std::ostream &out, const ProjectilePoint &point, const char *indent) {
    out << indent << "{\n";
    out << indent << "  \"t\": " << point.t << ",\n";
    out << indent << "  \"pos\": {\"x\": " << point.pos.x
        << ", \"y\": " << point.pos.y
        << ", \"z\": " << point.z << "}\n";
    out << indent << "}";
}

// Зчитуємо JSON-ку з файла в пам'ять. Якшо шось пішло не так - маєм облом, кажем про це
bool loadJsonFile(const char *path, json &data) {
    std::ifstream fin(path);
    if (!fin.is_open()) {
        std::cout << "Error: cannot open " << path << "\n";
        return false;
    }

    try {
        fin >> data;
    } catch (const std::exception &e) {
        // ну тут або синтаксис поламаний, або хтось крапку з комою забув
        std::cout << "Error: invalid JSON in " << path << ": " << e.what() << "\n";
        return false;
    }
    return true;
}

// Читаємо координату з JSON-ноди. Перевіряємо шо там x і y є, і шо вони числа, а не якась дичь
bool readCoordObject(const json &node, Coord &coord) {
    if (!node.is_object() || !node.contains("x") || !node.contains("y")) return false;
    if (!node.at("x").is_number() || !node.at("y").is_number()) return false;
    coord.x = node.at("x").get<float>();
    coord.y = node.at("y").get<float>();
    return true;
}

// Завантажуємо конфіг дрона з config.json - всі параметри політу та симуляції
// Якщо хоч одне поле поламане - фейл, бо без цього нема шо робити
bool loadConfig(const char *path, DroneConfig &config, int &maxSteps) {
    json data;
    if (!loadJsonFile(path, data)) return false;

    try {
        if (!data.contains("drone") || !data.at("drone").is_object()) {
            std::cout << "Error: config.json must contain drone object\n";
            return false;
        }
        if (!data.contains("simulation") || !data.at("simulation").is_object()) {
            std::cout << "Error: config.json must contain simulation object\n";
            return false;
        }

        const json &drone = data.at("drone");
        const json &simulation = data.at("simulation");

        // стартова позиція - обов'язкова штука, без неї нема звідки стартувати
        if (!drone.contains("position") || !readCoordObject(drone.at("position"), config.startPos)) {
            std::cout << "Error: config.json has invalid drone.position\n";
            return false;
        }

        // тягнемо решту параметрів: висота, напрямок, швидкість, розгін
        config.altitude = drone.at("altitude").get<float>();
        config.initialDir = drone.at("initialDirection").get<float>();
        config.attackSpeed = drone.at("attackSpeed").get<float>();
        config.accelPath = drone.at("accelerationPath").get<float>();

        // назва боєприпасу. Якшо хтось вписав дуже довге ім'я - посилаємо лесом,
        // бо наш буфер коротенький, нефіг переповнювати
        std::string ammoName = data.at("ammo").get<std::string>();
        if (ammoName.size() >= sizeof(config.ammoName)) {
            std::cout << "Error: ammo name is too long\n";
            return false;
        }
        std::strcpy(config.ammoName, ammoName.c_str());

        // часові інтервали, радіус попадання, швидкість повороту і поріг розвороту
        config.arrayTimeStep = data.at("targetArrayTimeStep").get<float>();
        config.simTimeStep = simulation.at("timeStep").get<float>();
        config.hitRadius = simulation.at("hitRadius").get<float>();
        config.angularSpeed = drone.at("angularSpeed").get<float>();
        config.turnThreshold = drone.at("turnThreshold").get<float>();
        // скіки максимум кроків крутити симуль, щоб не повис до кінця часів
        maxSteps = data.contains("maxSteps") ? data.at("maxSteps").get<int>() : 0;
    } catch (const std::exception &e) {
        std::cout << "Error: invalid config.json format: " << e.what() << "\n";
        return false;
    }

    return true;
}

// Підгружаємо табличку боєприпасів - тримаємо її в vector, шоб не морочитись з delete[]
bool loadAmmo(const char *path, std::vector<AmmoParams> &ammoTable, int &ammoCount) {
    json data;
    if (!loadJsonFile(path, data)) return false;
    // якщо там пусто або не масив - то шо ми взагалі симулювати будемо, дим?
    if (!data.is_array() || data.empty()) {
        std::cout << "Error: ammo.json must be a non-empty array\n";
        return false;
    }

    ammoCount = (int)data.size();
    // вся табличка лежить одним шматком пам'яті без ручного керування
    ammoTable.resize(ammoCount);

    for (int i = 0; i < ammoCount; i++) {
        try {
            // ім'я бахкала - рубимо, якщо завелике
            std::string name = data.at(i).at("name").get<std::string>();
            if (name.size() >= sizeof(ammoTable[i].name)) {
                std::cout << "Error: ammo name is too long at index " << i << "\n";
                return false;
            }
            std::strcpy(ammoTable[i].name, name.c_str());
            // фізичні параметри - маса, опір, підйом
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

// Читаємо цілі в один плоский масив: [номер цілі * timeSteps + момент часу]
bool loadTargets(const char *path, std::vector<Coord> &targets, int &targetCount, int &timeSteps) {
    json data;
    if (!loadJsonFile(path, data)) return false;
    // без масива targets далі робити нічого - шо мочити то?
    if (!data.contains("targets") || !data.at("targets").is_array() || data.at("targets").empty()) {
        std::cout << "Error: targets.json must contain a non-empty targets array\n";
        return false;
    }

    const json &targetArray = data.at("targets");
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
        // перевірка шо всі цілі однакової довжини - інакше буде невідповідь по часу
        if (!targetArray.at(i).is_object() ||
            !targetArray.at(i).contains("positions") ||
            !targetArray.at(i).at("positions").is_array() ||
            (int)targetArray.at(i).at("positions").size() != timeSteps) {
            std::cout << "Error: each target must contain positions with timeSteps entries\n";
            return false;
        }

        const json &positions = targetArray.at(i).at("positions");
        for (int j = 0; j < timeSteps; j++) {
            if (!readCoordObject(positions.at(j), targets[(size_t)i * (size_t)timeSteps + (size_t)j])) {
                std::cout << "Error: invalid target coordinate at [" << i << "][" << j << "]\n";
                return false;
            }
        }
    }

    return true;
}

// Шукаємо боєприпас за назвою в нашій табличці. Якщо нема - повертаєм -1, як знак біди
int findAmmo(const std::vector<AmmoParams> &ammoTable, const char *name) {
    int ammoCount = (int)ammoTable.size();
    for (int i = 0; i < ammoCount; i++) {
        if (std::strcmp(name, ammoTable[i].name) == 0) return i;
    }
    return -1;
}

// Розв'язуємо кубічне рівняння методом Кардано - шукаємо, за скіки часу снаряд до землі долетить.
// Жорстка алгебра: a*t^3 + b*t^2 + c = 0, але без квадратного члена.
// Повертаємо найменший додатний корінь - бо нас цікавить перший момент падіння, а не якісь химери
double solveCardanoTime(double a, double b, double c, bool &ok) {
    ok = false;
    // якщо коефіцієнт при кубі нулячий - то це вже не кубічне, йдем лісом
    if (std::fabs(a) < EPS) return 0.0;

    // приводим до виду t^3 + p*t + q = 0 (формула Кардано класична)
    double A = b / a;
    double p = -(A * A) / 3.0;
    double q = (2.0 * A * A * A) / 27.0 + c / a;
    double shift = A / 3.0;  // зсув для повернення до оригінального t
    // дискрімінант: за знаком розуміємо скіки коренів реальних
    double discriminant = (q * q) / 4.0 + (p * p * p) / 27.0;

    std::vector<double> roots;
    roots.reserve(3);
    // мала лямбда, щоб не пхати дубль-корінь двічі (а то з чисельності бува)
    auto addRoot = [&](double root) {
        for (size_t i = 0; i < roots.size(); i++) {
            if (std::fabs(roots[i] - root) < 1e-7) return;
        }
        roots.push_back(root);
    };

    if (discriminant > EPS) {
        // один дійсний корінь (решта - комплексні, їх ігнорим)
        double sqrtDisc = std::sqrt(discriminant);
        double u = std::cbrt(-q / 2.0 + sqrtDisc);
        double v = std::cbrt(-q / 2.0 - sqrtDisc);
        addRoot(u + v - shift);
    } else if (std::fabs(discriminant) <= EPS) {
        // граничний випадок - два дійсні корені (один подвійний)
        double u = std::cbrt(-q / 2.0);
        addRoot(2.0 * u - shift);
        addRoot(-u - shift);
    } else {
        // тригонометричний варіант - три дійсні корені, юзаємо косінуси
        double acosArg = (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
        // клемпим арґумент acos-а щоб не вилетіти за межі [-1; 1] через чисельну похибку
        if (acosArg < -1.0) acosArg = -1.0;
        if (acosArg > 1.0) acosArg = 1.0;

        double phi = std::acos(acosArg);
        double rootBase = 2.0 * std::sqrt(-p / 3.0);
        addRoot(rootBase * std::cos(phi / 3.0) - shift);
        addRoot(rootBase * std::cos((phi + 2.0 * M_PI) / 3.0) - shift);
        addRoot(rootBase * std::cos((phi + 4.0 * M_PI) / 3.0) - shift);
    }

    // серед знайдених коренів беремо найменший додатний - це і є момент падіння
    double best = 0.0;
    bool found = false;
    for (size_t i = 0; i < roots.size(); i++) {
        if (roots[i] <= EPS) continue;  // від'ємні і нулі - до побачення
        if (!found || roots[i] < best) {
            best = roots[i];
            found = true;
        }
    }

    if (!found) return 0.0;
    ok = true;
    return best;
}

// Рахуємо як далеко снаряд пролетить по горизонталі за час t
// Тут вже чиста фізика з урахуванням опору та планерухи - формула з розкладом у ряд Тейлора
double calcHorizontalDistance(double t, const AmmoParams &ammo, double V0) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    // term1 - базовий рух: швидкість на час (як у школі)
    double term1 = V0 * t;
    // term2 і далі - поправки на опір і піднімальну силу, всяка фізична фігня другого і вище порядків
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
    // складаємо всі шматки докупи - і маєм дальність
    return term1 + term2 + term3 + term4 + term5;
}

// А це висота снаряда в момент часу t - скіки залишилось до землі
// z0 - стартова висота (звідки скинули), формула схожа на вільне падіння з поправками
double calcAltitude(double t, double z0, const AmmoParams &ammo, double V0) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    double a = d * G * m - 2.0 * d * d * l * V0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * V0;
    double z = z0 + (b * t * t) / (6.0 * m * m) + (a * t * t * t) / (6.0 * m * m);
    // в землю не йдемо - якщо мінус, то значить вже гепнувся, кліпаєм до нуля
    if (z < 0.0) z = 0.0;
    return z;
}

// Головна балістична функція - рахує за скіки снаряд впаде і як далеко долетить
// altitude - з якої висоти скидуєм, V0 - швидкість дрона на момент скидання
bool computeBallistics(double altitude, double V0, const AmmoParams &ammo,
                       double &tOfFlight, double &hDist) {
    double m = ammo.mass;
    double d = ammo.drag;
    double l = ammo.lift;
    // коефіцієнти кубічного рівняння для пошуку часу падіння (з формули висоти = 0)
    double a = d * G * m - 2.0 * d * d * l * V0;
    double b = -3.0 * G * m * m + 3.0 * d * l * m * V0;
    double c = 6.0 * m * m * altitude;

    bool ok = false;
    // шукаємо за скіки впаде - через Кардано
    double t = solveCardanoTime(a, b, c, ok);
    if (!ok || t <= EPS) return false;

    // а тепер рахуємо, куди за цей час долетіло по горизонталі
    double h = calcHorizontalDistance(t, ammo, V0);
    if (h <= EPS) return false;

    tOfFlight = t;
    hDist = h;
    return true;
}

// Інтерполяція позиції цілі в часі - між двома сусідніми кадрами лінійно прикидуємо, де ціль зара
// Циклічно обертаємо індекси, щоб не вилетіти за межі масиву (коли час більший за повний цикл)
Coord interpTarget(const std::vector<Coord> &targetsInTime, int timeSteps, int targetIdx,
                   double t, double arrayTimeStep) {
    double tt = t / arrayTimeStep;
    int idx = ((int)std::floor(tt)) % timeSteps;
    if (idx < 0) idx += timeSteps;  // якщо час від'ємний, підправляємо індекс (щоб не було сюрпризів)
    int next = (idx + 1) % timeSteps;  // сусідній кадр - крутим по колу
    double frac = tt - std::floor(tt);  // дробова частинка - на скіки між кадрами ми зараз

    // лінійна інтерполяція: між idx і next беремо лінійку
    const Coord &current = targetsInTime[(size_t)targetIdx * (size_t)timeSteps + (size_t)idx];
    const Coord &nextCoord = targetsInTime[(size_t)targetIdx * (size_t)timeSteps + (size_t)next];
    Coord result;
    result.x = static_cast<float>(current.x + (nextCoord.x - current.x) * frac);
    result.y = static_cast<float>(current.y + (nextCoord.y - current.y) * frac);
    return result;
}

// Приводимо кут у діапазон [-PI; PI], щоб не було різних кутів по суті однакових
// (типу 370° і 10° - то одне й теж, нормалізація все приводить в норму)
double normAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Скіки часу треба дрону, шоб пройти відстань dist з поточною швидкістю speed,
// враховуючи шо він буде прискорюватись до attackSpeed з прискоренням accel
// Буває два кейси: або не встиг розігнатись, або вже на крейсерській сидить
double estimateDroneTravelTime(double dist, double speed, double attackSpeed, double accel) {
    if (dist <= 0.0) return 0.0;  // нема куди бігти
    if (accel <= EPS) return dist / attackSpeed;  // нема прискорення - все на крейсерській
    if (speed >= attackSpeed) return dist / attackSpeed;  // вже на макс - не розганяємось далі

    // рахуємо скіки треба метрів щоб вийти на крейсерську
    double distToCruise = (attackSpeed * attackSpeed - speed * speed) / (2.0 * accel);
    if (distToCruise >= dist) {
        // коротка відстань - навіть розігнатись до max не встигнемо
        double disc = speed * speed + 2.0 * accel * dist;
        if (disc < 0.0) disc = 0.0;
        return (-speed + std::sqrt(disc)) / accel;
    }

    // нормальна відстань: спочатку розганяємось, потім чешимо на крейсерській
    double tAccel = (attackSpeed - speed) / accel;
    double tCruise = (dist - distToCruise) / attackSpeed;
    return tAccel + tCruise;
}

// Найбільш жирна функція планування: рахує, куди кидати бахкало, щоб попасти в ціль
// Враховує шо ціль рухається, що дрон поки летить, що снаряд має час падіння
// Повертає точку скидання, передбачену позицію цілі, час прильоту і час польоту снаряда
bool computeDropPoint(const std::vector<Coord> &targetPositions, int timeSteps, int targetIdx,
                      double currentTime, double arrayTimeStep, double simTimeStep,
                      double altitude, double V0, const AmmoParams &ammo,
                      const Coord &dronePos, double droneSpeed, double accel,
                      Coord &dropPoint, Coord &predictedTarget,
                      double &arrivalTime, double &tOfFlight) {
    // 1. Де ціль зараз?
    Coord currentTarget = interpTarget(targetPositions, timeSteps, targetIdx, currentTime, arrayTimeStep);

    // 2. Скіки буде летіти снаряд і куди приземлиться
    double hDist;
    if (!computeBallistics(altitude, V0, ammo, tOfFlight, hDist)) return false;

    // 3. Наскіки нам треба підлетіти, щоб з нашої висоти кинуте попало
    Coord toCurrentTarget = currentTarget - dronePos;
    double currentDistance = length(toCurrentTarget);
    if (currentDistance < EPS) return false;

    // приблизна відстань до точки скидання = до цілі мінус дальність польоту снаряда
    double approxDist = currentDistance - hDist;
    if (approxDist < 0.0) approxDist = 0.0;
    // скіки часу на це все піде (доліт + падіння снаряда)
    double approxArrival = estimateDroneTravelTime(approxDist, droneSpeed, V0, accel);
    double totalTime = approxArrival + tOfFlight;

    // 4. Рахуємо куди ціль за цей час утече - тобто беремо її швидкість (з двох близьких кадрів)
    //    і екстраполюємо. Типу упередження на рухому мішень
    Coord targetNow = interpTarget(targetPositions, timeSteps, targetIdx, currentTime, arrayTimeStep);
    Coord targetNext = interpTarget(targetPositions, timeSteps, targetIdx, currentTime + simTimeStep, arrayTimeStep);
    Coord targetVelocity = (targetNext - targetNow) / (float)simTimeStep;

    // прогноз: де ціль буде в момент, коли снаряд долетить
    predictedTarget = targetNow + targetVelocity * (float)totalTime;

    Coord toPredictedTarget = predictedTarget - dronePos;
    double predictedDistance = length(toPredictedTarget);
    if (predictedDistance < EPS) return false;

    // 5. Точка скидання = прогноз мінус дальність снаряда в напрямку на ціль
    //    (тобто треба скинути рівно так, щоб снаряд зніс предиктед-точку)
    dropPoint = predictedTarget - normalize(toPredictedTarget) * (float)hDist;
    Coord toDropPoint = dropPoint - dronePos;
    // час на доліт до цієї точки скидання
    arrivalTime = estimateDroneTravelTime(length(toDropPoint), droneSpeed, V0, accel);
    return true;
}

// Будуємо траєкторію снаряда після скидання - для візуалізації у вьюері
// Просто нарізаємо 64 точки вздовж польоту і рахуємо x/y/z для кожної
bool buildProjectilePath(const DroneConfig &config, const AmmoParams &ammo,
                         const SimStep *steps, int stepCount, bool dropped,
                         int &dropStepIdx, Coord &dropOrigin, Coord &targetPoint,
                         double &tOfFlight, std::vector<ProjectilePoint> &projectilePath) {
    // скидаємо все в дефолт на випадок фейлу
    projectilePath.clear();
    dropStepIdx = -1;
    dropOrigin = {0.0f, 0.0f};
    targetPoint = {0.0f, 0.0f};
    tOfFlight = 0.0;

    // якщо не кидали - нема шо малювати, просто йдемо додому
    if (!dropped || stepCount <= 0) return true;

    // балістика - як довго і як далеко
    double hDist = 0.0;
    if (!computeBallistics((double)config.altitude, (double)config.attackSpeed, ammo, tOfFlight, hDist)) {
        return false;
    }

    // беремо останній крок - там скидання відбулось
    dropStepIdx = stepCount - 1;
    dropOrigin = steps[dropStepIdx].pos;
    targetPoint = steps[dropStepIdx].predictedTarget;

    // напрямок польоту снаряда = напрямок, куди дрон дивився при скиданні
    Coord dir = {
        static_cast<float>(std::cos(steps[dropStepIdx].direction)),
        static_cast<float>(std::sin(steps[dropStepIdx].direction))
    };
    // якщо напрямок невалідний - пробуємо вгадати з точок скидання/цілі, хай хоч щось буде
    if (length(dir) < EPS) {
        dir = normalize(steps[dropStepIdx].dropPoint - dropOrigin);
        if (length(dir) < EPS) {
            dir = normalize(targetPoint - dropOrigin);
            if (length(dir) < EPS) return false;
        }
    }

    // насемплюємо 64+1 точку вздовж траєкторії - для плавного малювання
    const int samples = 64;
    projectilePath.reserve(samples + 1);
    for (int i = 0; i <= samples; i++) {
        double t = tOfFlight * i / samples;
        double h = calcHorizontalDistance(t, ammo, (double)config.attackSpeed);
        ProjectilePoint point;
        point.t = t;
        point.pos = dropOrigin + dir * (float)h;  // позиція в XY по горизонталі
        point.z = calcAltitude(t, (double)config.altitude, ammo, (double)config.attackSpeed);  // висота
        projectilePath.push_back(point);
    }

    return true;
}

// Скіки часу треба щоб дрон зупинився з поточного стану - для планування розвороту
// Якшо стоїмо - то нуль, якшо їдем - то скіки прогальмувати, якшо крутимось - скіки дорежемось
double computeTimeToStop(DroneState state, double speed, double attackSpeed,
                         double accel, double turnRemainingTime) {
    switch (state) {
        case STOPPED:      return 0.0;                    // і так стоїмо, куди зупинятись
        case ACCELERATING: return speed / accel;          // скіки прогальмувати з поточної швидкості
        case MOVING:       return attackSpeed / accel;    // гальмуємо з крейсерської до нуля
        case DECELERATING: return speed / accel;          // вже гальмуємо, лишилось стіки
        case TURNING:      return turnRemainingTime;      // крутимось - чекаєм поки докрутиться
    }
    return 0.0;
}

// Пишемо результат симуляції в JSON - весь хід подій, крок за кроком, для вьюера
// Тут спрощена версія: пишемо тіки шаги, решту параметрів проігноровано (void-каст щоб компілятор не скиглив)
bool writeSimulationJson(const char *path, const DroneConfig &config,
                         const AmmoParams &selectedAmmo,
                         SimStep *steps, int stepCount, int targetCount, int timeSteps,
                         int selectedTarget, bool dropped,
                         const Coord &finalDropPoint, double currentTime) {
    // ці параметри поки шо не юзаємо - тре щоб компайлер не бурчав на unused
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
    if (stepCount > 0) fout << "\n";
    for (int i = 0; i < stepCount; i++) {
        writeStepJson(fout, steps[i], "    ");
        if (i + 1 < stepCount) fout << ",";
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

// Окремо пишемо траєкторію снаряда - для 3D-картинки польоту бахкала до цілі
bool writeProjectileJson(const char *path, bool dropped,
                         int dropStepIdx, const Coord &dropOrigin, const Coord &targetPoint,
                         double dropTOfFlight, const std::vector<ProjectilePoint> &projectilePath) {
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
    if (!projectilePath.empty()) fout << "\n";
    for (size_t i = 0; i < projectilePath.size(); i++) {
        writeProjectilePointJson(fout, projectilePath[i], "      ");
        if (i + 1 < projectilePath.size()) fout << ",";
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

// Робимо з назви бахкала пристойний слаг для імені файлу:
// викидаємо все непотрібне, лишаємо тіки букви/цифри, роздільники замінюємо на _
// Наприклад "RKG-3 EM" -> "rkg_3_em"
std::string makeAmmoFileSlug(const char *ammoName) {
    std::string slug;
    slug.reserve(std::strlen(ammoName));
    for (size_t i = 0; ammoName[i] != '\0'; i++) {
        unsigned char ch = static_cast<unsigned char>(ammoName[i]);
        if (std::isalnum(ch)) {
            // буква/цифра - тягнем у слаг, приводимо до нижнього регістру
            slug.push_back(static_cast<char>(std::tolower(ch)));
        } else if (!slug.empty() && slug.back() != '_') {
            // всяка нечистоть - замінюємо на підкреслення, але не дублюємо
            slug.push_back('_');
        }
    }

    // прибираємо хвостові підкреслення, якщо назва закінчувалась на сміття
    while (!slug.empty() && slug.back() == '_') {
        slug.pop_back();
    }
    // запасний варіант - якщо взагалі нічого не вийшло, беремо дефолтну назву
    if (slug.empty()) {
        slug = "ammo";
    }
    return slug;
}

// МЯСО САМЕ - прогін симуляції для одного типу бахкала.
// Тут вся логіка: дрон оглядає цілі, обирає найближчу, летить/повертає/скидає
bool runSingleAmmoSimulation(const DroneConfig &baseConfig, const AmmoParams &selectedAmmo,
                             const std::vector<Coord> &targetsInTime, int targetCount, int timeSteps, int maxSteps,
                             SimulationRunResult &result) {
    // робимо копію конфіга і підмінюємо туди ім'я боєприпаса - щоб у звіті було правильно
    DroneConfig config = baseConfig;
    std::strcpy(config.ammoName, selectedAmmo.name);

    // обнуляємо результат і резервуємо місце - щоб пуш не ре-алокував
    result = {};
    result.steps.reserve(maxSteps + 1);

    // формула прискорення: V^2 = 2*a*S => a = V^2 / (2*S), шкільна класика
    double accel = (double)config.attackSpeed * (double)config.attackSpeed /
                   (2.0 * (double)config.accelPath);

    // початкові умови: стартова поза, напрямок, швидкість 0, стан - розганяємось
    Coord dronePos = config.startPos;
    double dir = config.initialDir;
    double speed = 0.0;
    DroneState state = ACCELERATING;
    double currentTime = 0.0;
    int selectedTarget = -1;
    double turnRemainingTime = 0.0;

    // ГОЛОВНИЙ ЦИКЛ: крутимо поки не вичерпали кроки або поки не бахнули
    while ((int)result.steps.size() < maxSteps && !result.dropped) {
        // перебираємо всі цілі і шукаємо найкращу на цей момент
        double bestDropDistance = 1e18;  // дуже велике число, щоб перший кандидат однозначно виграв
        double bestTotal = 1e18;
        int bestTarget = -1;
        Coord bestDropPoint = {0.0f, 0.0f};
        Coord bestPredictedTarget = {0.0f, 0.0f};

        for (int i = 0; i < targetCount; i++) {
            // для кожної цілі рахуємо куди кидати і скіки часу вся бодяга займе
            Coord dropPoint;
            Coord predictedTarget;
            double arrivalTime;
            double tOfFlight;

            bool ok = computeDropPoint(targetsInTime, timeSteps, i,
                                       currentTime, config.arrayTimeStep, config.simTimeStep,
                                       (double)config.altitude, (double)config.attackSpeed, selectedAmmo,
                                       dronePos, speed, accel,
                                       dropPoint, predictedTarget, arrivalTime, tOfFlight);
            if (!ok) continue;  // балістика не зійшлася - пропускаємо ціль нахер

            // базова оцінка: час долетіти + час польоту снаряда
            double total = arrivalTime + tOfFlight;
            Coord toDropPoint = dropPoint - dronePos;
            double dropDistance = length(toDropPoint);

            // якщо міняємо ціль на нову - рахуємо штраф на розворот
            if (i != selectedTarget && selectedTarget >= 0) {
                double desiredDir = std::atan2(toDropPoint.y, toDropPoint.x);
                double deltaAngle = normAngle(desiredDir - dir);
                // скіки часу займе зупинка поточного маневру
                double stopTime = computeTimeToStop(state, speed, (double)config.attackSpeed,
                                                    accel, turnRemainingTime);

                if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                    // кут серйозний - треба повністю зупинитись, крутнутись і знов розігнатися
                    double turnTime = std::fabs(deltaAngle) / (double)config.angularSpeed;
                    double newTravel = estimateDroneTravelTime(length(toDropPoint), 0.0,
                                                               (double)config.attackSpeed, accel);
                    total = stopTime + turnTime + newTravel + tOfFlight;
                } else {
                    // кут маленький - крутимось на ходу, додаємо тіки час повороту
                    double turnTime = std::fabs(deltaAngle) / (double)config.angularSpeed;
                    total = arrivalTime + tOfFlight + turnTime;
                }
            }

            // вибираємо кращу: спочатку по близькості точки скидання, потім по часу
            // (типу пріоритет - хто ближче, а якщо однаково близько - хто швидше)
            if (dropDistance < bestDropDistance - EPS ||
                (std::fabs(dropDistance - bestDropDistance) <= EPS && total < bestTotal)) {
                bestDropDistance = dropDistance;
                bestTotal = total;
                bestTarget = i;
                bestDropPoint = dropPoint;
                bestPredictedTarget = predictedTarget;
            }
        }

        // якщо жодна ціль не підійшла - халепа, фіксуємо останній стан і виходимо з циклу
        if (bestTarget < 0) {
            result.steps.push_back({
                dronePos,
                dronePos,
                dronePos,
                static_cast<float>(dir),
                (int)state,
                selectedTarget
            });
            break;
        }

        // зафіксували вибір цілі і зберегли крок в масив
        selectedTarget = bestTarget;
        result.finalDropPoint = bestDropPoint;

        result.steps.push_back({
            dronePos,
            bestDropPoint,
            bestPredictedTarget,
            static_cast<float>(dir),
            (int)state,
            selectedTarget
        });

        DEBUG("Step " << (int)result.steps.size()
              << " pos=(" << dronePos.x << "," << dronePos.y << ")"
              << " dir=" << dir
              << " target=" << selectedTarget
              << " state=" << (int)state);

        // рахуємо вектор до точки скидання: куди і як далеко треба крутити
        Coord toBestDropPoint = bestDropPoint - dronePos;
        double distToDrop = length(toBestDropPoint);
        double desiredDir = std::atan2(toBestDropPoint.y, toBestDropPoint.x);
        double deltaAngle = normAngle(desiredDir - dir);
        // максимум, на скіки можна крутнутись за один тік симуляції
        double maxTurn = (double)config.angularSpeed * (double)config.simTimeStep;

        // БАХ! Якщо ми вже в радіусі скидання і кут норм - скидаємо подарунок і йдемо додому
        if (distToDrop <= (double)config.hitRadius &&
            std::fabs(deltaAngle) <= (double)config.turnThreshold) {
            result.dropped = true;
            break;
        }

        // перехід у режим розвороту: якщо кут великий і ми в норм стані - почнемо гальмувати/крутитись
        if (state != TURNING && state != DECELERATING) {
            if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                // якщо на швидкості - спочатку гальмуємо, бо на ходу кучеряво не крутиться
                state = (speed > EPS) ? DECELERATING : TURNING;
            }
        }

        // ---- БЛОК РЕАЛЬНОГО РУХУ: дивимось в якому стані і діємо ----

        if (state == TURNING) {
            // крутимось на місці, швидкість = 0
            if (std::fabs(deltaAngle) <= maxTurn) {
                // встигли докрутитись за цей тік - переходимо на розгін
                dir = desiredDir;
                turnRemainingTime = 0.0;
                state = ACCELERATING;
            } else {
                // не встигли - докрутимо скіки змогли, решту - наступного разу
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
                turnRemainingTime = std::fabs(deltaAngle) / (double)config.angularSpeed -
                                    (double)config.simTimeStep;
                if (turnRemainingTime < 0.0) turnRemainingTime = 0.0;
            }
        } else if (state == DECELERATING) {
            // гальмування: мінусуємо прискорення від швидкості
            double newSpeed = speed - accel * (double)config.simTimeStep;
            if (newSpeed > 0.0) {
                // ще рухаємось, просто повільніше - рухаємось з середньою швидкістю за тік
                double avgV = (speed + newSpeed) / 2.0;
                dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * (double)config.simTimeStep);
                dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * (double)config.simTimeStep);
                speed = newSpeed;
            } else {
                // зупинились десь посеред тіка - доїхали кінцевий шматок з дорозгоном потім
                double tDecel = speed / accel;
                double avgV = speed / 2.0;
                dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * tDecel);
                dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * tDecel);
                speed = 0.0;

                // залишок тіка - використаємо для повороту/розгону
                double tRem = (double)config.simTimeStep - tDecel;
                if (tRem < 0.0) tRem = 0.0;
                double turnPart = (double)config.angularSpeed * tRem;

                if (std::fabs(deltaAngle) > (double)config.turnThreshold) {
                    // кут великий - будемо крутитись
                    if (std::fabs(deltaAngle) <= turnPart) {
                        // встигли докрутитись в цьому ж тіку
                        dir = desiredDir;
                        turnRemainingTime = 0.0;
                        state = ACCELERATING;
                    } else {
                        // не встигли - перейдемо в TURNING на наступний крок
                        dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);
                        state = TURNING;
                        turnRemainingTime = std::fabs(deltaAngle) / (double)config.angularSpeed - tRem;
                        if (turnRemainingTime < 0.0) turnRemainingTime = 0.0;
                    }
                } else {
                    // кут дрібний - можемо докрутитись на ходу і одразу розганятися
                    if (std::fabs(deltaAngle) <= turnPart) dir = desiredDir;
                    else dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);

                    // розганяємося на залишку тіка
                    state = ACCELERATING;
                    double newV = accel * tRem;
                    if (newV > (double)config.attackSpeed) newV = (double)config.attackSpeed;
                    double avg = newV / 2.0;
                    dronePos.x = static_cast<float>(dronePos.x + avg * std::cos(dir) * tRem);
                    dronePos.y = static_cast<float>(dronePos.y + avg * std::sin(dir) * tRem);
                    speed = newV;
                    // якщо уже набрали крейсерську - переходимо у MOVING
                    if (speed >= (double)config.attackSpeed - EPS) state = MOVING;
                }
            }
        } else {
            // стан ACCELERATING / MOVING / STOPPED - тут уже їдем вперед
            // трохи підрулюємо напрямок (якщо є невеликий кут)
            if (std::fabs(deltaAngle) <= maxTurn) {
                dir = desiredDir;
            } else {
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
            }

            // якщо з якоїсь причини стояли - починаємо розганятись
            if (state == STOPPED) {
                state = ACCELERATING;
            }

            if (state == ACCELERATING) {
                // розгін: швидкість росте, позиція йде по інтегралу середньої швидкості
                double newSpeed = speed + accel * (double)config.simTimeStep;
                if (newSpeed >= (double)config.attackSpeed) {
                    // перевалили через крейсерську всередині тіка - розбиваємо на дві фази
                    double tAccel = ((double)config.attackSpeed - speed) / accel;
                    if (tAccel < 0.0) tAccel = 0.0;
                    double avgV1 = (speed + (double)config.attackSpeed) / 2.0;
                    double tMove = (double)config.simTimeStep - tAccel;
                    // перша фаза - розгін, друга - вже летимо на крейсерській
                    dronePos.x = static_cast<float>(dronePos.x + avgV1 * std::cos(dir) * tAccel
                                 + (double)config.attackSpeed * std::cos(dir) * tMove);
                    dronePos.y = static_cast<float>(dronePos.y + avgV1 * std::sin(dir) * tAccel
                                 + (double)config.attackSpeed * std::sin(dir) * tMove);
                    speed = (double)config.attackSpeed;
                    state = MOVING;
                } else {
                    // ще розганяємось, крейсерської не досягли
                    double avgV = (speed + newSpeed) / 2.0;
                    dronePos.x = static_cast<float>(dronePos.x + avgV * std::cos(dir) * (double)config.simTimeStep);
                    dronePos.y = static_cast<float>(dronePos.y + avgV * std::sin(dir) * (double)config.simTimeStep);
                    speed = newSpeed;
                }
            } else if (state == MOVING) {
                // просто чешимо на максі, координата = швидкість * час
                dronePos.x = static_cast<float>(dronePos.x + (double)config.attackSpeed * std::cos(dir) *
                             (double)config.simTimeStep);
                dronePos.y = static_cast<float>(dronePos.y + (double)config.attackSpeed * std::sin(dir) *
                             (double)config.simTimeStep);
            }
        }

        // час іде вперед, тік завершено
        currentTime += (double)config.simTimeStep;
    }

    // підбиваємо підсумок усього прогону
    result.stepCount = (int)result.steps.size();
    result.selectedTarget = selectedTarget;
    result.currentTime = currentTime;

    // і малюємо траєкторію снаряда (якщо таки бахнули)
    if (!buildProjectilePath(config, selectedAmmo, result.steps.data(), result.stepCount, result.dropped,
                             result.dropStepIdx, result.dropOrigin, result.targetPoint,
                             result.dropTOfFlight, result.projectilePath)) {
        return false;
    }

    return true;
}

// ================ ГОЛОВНА ФУНКЦІЯ - точка входу всього свята ================
int main() {
    // оголошуємо всю потрібну байду з нулями, щоб у cleanup не було несподіванок
    DroneConfig config = {};
    std::vector<AmmoParams> ammoTable;
    std::vector<Coord> targetsInTime;
    int ammoCount = 0;
    int targetCount = 0;
    int timeSteps = 0;
    int maxSteps = 0;

    // крок 1: читаємо конфіг - якщо зламався, то гейм овер
    if (!loadConfig("config.json", config, maxSteps)) {
        return 1;
    }
    LOG("Config loaded: attackSpeed=" << config.attackSpeed
        << ", altitude=" << config.altitude
        << ", ammo=" << config.ammoName);

    // крок 2: підтягуємо табличку бахкал
    if (!loadAmmo("ammo.json", ammoTable, ammoCount)) {
        return 1;
    }
    LOG("Ammo loaded: " << ammoCount << " entries");

    // крок 3: завантажуємо цілі з їх траєкторіями
    if (!loadTargets("targets.json", targetsInTime, targetCount, timeSteps)) {
        return 1;
    }
    LOG("Targets loaded: " << targetCount << " targets, " << timeSteps << " time steps");

    // якщо в конфігу не задали максимум кроків - прикидаємо з голови, щоб не зависнути навічно
    if (maxSteps <= 0) {
        double stepRatio = std::ceil((double)config.arrayTimeStep / (double)config.simTimeStep);
        if (stepRatio < 1.0) stepRatio = 1.0;
        maxSteps = timeSteps * (int)stepRatio * targetCount;
    }

    // ---- ВАЛІДАЦІЯ ВСЬОГО ПОСПІЛЬ: якщо шось некошерне - одразу ідемо лісом ----
    if (config.simTimeStep <= 0.0f || config.arrayTimeStep <= 0.0f ||
        config.accelPath <= 0.0f || config.attackSpeed <= 0.0f) {
        std::cout << "Error: invalid numeric parameters (step/accel/speed must be > 0)\n";
        return 1;
    }
    if (config.altitude <= 0.0f) {
        // без висоти скидати нема як - бахнеш сам собі по ногам
        std::cout << "Error: invalid altitude (must be > 0)\n";
        return 1;
    }
    if (config.angularSpeed <= 0.0f) {
        // якщо дрон не вміє крутитись - то він просто літаюча палка, так не годиться
        std::cout << "Error: invalid angularSpeed (must be > 0)\n";
        return 1;
    }
    if (config.hitRadius <= 0.0f) {
        // нульовий радіус попадання = ніколи не попадеш, ніфіга годі
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
        const AmmoParams &selectedAmmo = ammoTable[ammoIdx];
        DroneConfig runConfig = config;
        std::strcpy(runConfig.ammoName, selectedAmmo.name);

        LOG("Ammo found: " << selectedAmmo.name);

        // запускаємо окрему симуляцію для кожного типу боєприпасу
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

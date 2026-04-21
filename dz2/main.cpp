#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

// Загальні константи симуляції.
const double G = 9.81;
const double EPS = 1e-9;
const int MAX_STEPS = 10000;
const int NUM_TARGETS = 5;
const int TARGET_FRAMES = 60;
const int NUM_AMMO = 5;

// Стани дрона. Значення 0..4 використовуються у вихідному файлі.
enum DroneState {
    STOPPED = 0,
    ACCELERATING = 1,
    DECELERATING = 2,
    TURNING = 3,
    MOVING = 4
};

// Таблиця боєприпасів.
// За умовою пошук веде через for, а не через if-else.
char bombNames[NUM_AMMO][15] = {"VOG-17", "M67", "RKG-3", "GLIDING-VOG", "GLIDING-RKG"};
float bombM[NUM_AMMO] = {0.35f, 0.6f, 1.2f, 0.45f, 1.4f};
float bombD[NUM_AMMO] = {0.07f, 0.10f, 0.10f, 0.10f, 0.10f};
float bombL[NUM_AMMO] = {0.0f, 0.0f, 0.0f, 1.0f, 1.0f};

// Пошук боєприпасу за назвою. Повертає індекс або -1.
int findAmmo(const char *name) {
    for (int i = 0; i < NUM_AMMO; i++) {
        if (std::strcmp(name, bombNames[i]) == 0) {
            return i;
        }
    }
    return -1;
}

// Кубічне рівняння через Кардано з ДЗ1. Без змін по суті.
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

// Горизонтальна дальність балістичного польоту (степеневий ряд до t^5).
double calcHorizontalDistance(double t, double m, double d, double l, double V0) {
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

// Єдина обгортка: дає час польоту та горизонтальну дальність.
// Повертає false, якщо балістика некоректна (некоректний режим/дані).
bool computeBallistics(double zd, double V0, double m, double d, double l,
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

// Лінійна інтерполяція координат цілі з масиву [5][60].
// Масив зациклений, тому беремо модуль по TARGET_FRAMES.
void interpTarget(const float targetX[NUM_TARGETS][TARGET_FRAMES],
                  const float targetY[NUM_TARGETS][TARGET_FRAMES],
                  int i, double t, double arrayTimeStep,
                  double &x, double &y) {
    double tt = t / arrayTimeStep;
    int idx = ((int)std::floor(tt)) % TARGET_FRAMES;
    if (idx < 0) idx += TARGET_FRAMES;
    int next = (idx + 1) % TARGET_FRAMES;
    double frac = tt - std::floor(tt);
    x = targetX[i][idx] + (targetX[i][next] - targetX[i][idx]) * frac;
    y = targetY[i][idx] + (targetY[i][next] - targetY[i][idx]) * frac;
}

// Нормалізує кут до діапазону [-pi, pi].
double normAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// Оцінка часу, за який дрон подолає відстань `dist` до точки скиду.
// Враховує фазу розгону від поточної швидкості до крейсерської та
// наступну фазу руху на attackSpeed. Якщо дистанції не вистачає навіть
// на розгін - розв'язуємо квадратне dist = v0*t + a*t^2/2.

// Для оцінки часу до точки скиду використано спрощення:
// дрон розганяється до attackSpeed і далі рухається на ній.
// Це узгоджено з тим, що в балістичній моделі ДЗ1 використовується V0 = attackSpeed.
double estimateDroneTravelTime(double dist, double speed, double attackSpeed, double accel) {
    if (dist <= 0.0) return 0.0;
    if (accel <= EPS) return dist / attackSpeed;

    // Уже на крейсерській (або вище) - просто ділимо.
    if (speed >= attackSpeed) return dist / attackSpeed;

    // Скільки ще треба проїхати, щоб набрати attackSpeed.
    double distToCruise = (attackSpeed * attackSpeed - speed * speed) / (2.0 * accel);

    if (distToCruise >= dist) {
        // Не встигає вийти на крейсерську - розганяється весь шлях.
        double disc = speed * speed + 2.0 * accel * dist;
        if (disc < 0.0) disc = 0.0;
        return (-speed + std::sqrt(disc)) / accel;
    }

    double tAccel = (attackSpeed - speed) / accel;
    double tCruise = (dist - distToCruise) / attackSpeed;
    return tAccel + tCruise;
}

// Lead targeting + розрахунок точки скиду для конкретної цілі.
// Повертає false, якщо ціль/балістика некоректні.
bool computeDropPoint(const float targetXArr[NUM_TARGETS][TARGET_FRAMES],
                      const float targetYArr[NUM_TARGETS][TARGET_FRAMES],
                      int i, double currentTime, double arrayTimeStep, double simTimeStep,
                      double zd, double V0, double m, double d, double l,
                      double posX, double posY,
                      double droneSpeed, double accel,
                      double &dropX, double &dropY,
                      double &arrivalTime, double &tOfFlight) {
    double curTx, curTy;
    interpTarget(targetXArr, targetYArr, i, currentTime, arrayTimeStep, curTx, curTy);

    double hDist;
    if (!computeBallistics(zd, V0, m, d, l, tOfFlight, hDist)) return false;

    double dxT = curTx - posX;
    double dyT = curTy - posY;
    double D = std::sqrt(dxT * dxT + dyT * dyT);
    if (D < EPS) return false;

    // Початкова оцінка часу прильоту до точки скиду (з урахуванням розгону).
    double approxDist = D - hDist;
    if (approxDist < 0.0) approxDist = 0.0;
    double approxArrival = estimateDroneTravelTime(approxDist, droneSpeed, V0, accel);
    double totalTime = approxArrival + tOfFlight;

    // Швидкість цілі через кінцеві різниці.
    double tx1, ty1, tx2, ty2;
    interpTarget(targetXArr, targetYArr, i, currentTime, arrayTimeStep, tx1, ty1);
    interpTarget(targetXArr, targetYArr, i, currentTime + simTimeStep, arrayTimeStep, tx2, ty2);
    double tvx = (tx2 - tx1) / simTimeStep;
    double tvy = (ty2 - ty1) / simTimeStep;

    // Прогнозована позиція цілі в момент прильоту.
    double predX = tx1 + tvx * totalTime;
    double predY = ty1 + tvy * totalTime;

    double pdx = predX - posX;
    double pdy = predY - posY;
    double pD = std::sqrt(pdx * pdx + pdy * pdy);
    if (pD < EPS) return false;

    // Точка скиду лежить на лінії дрон -> прогнозована ціль,
    // на відстані (pD - hDist) від дрона.
    dropX = predX - pdx * hDist / pD;
    dropY = predY - pdy * hDist / pD;

    double dropDx = dropX - posX;
    double dropDy = dropY - posY;
    double dropD = std::sqrt(dropDx * dropDx + dropDy * dropDy);
    arrivalTime = estimateDroneTravelTime(dropD, droneSpeed, V0, accel);
    return true;
}

// Скільки ще часу знадобиться дрону, щоб зупинитись.
// Використовується для додавання штрафу при зміні цілі.
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

void expandBounds(double x, double y,
                  double &minX, double &maxX,
                  double &minY, double &maxY) {
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
}

void writeSimulationSvg(const double outX[MAX_STEPS + 1],
                        const double outY[MAX_STEPS + 1],
                        int stepCount,
                        const float targetXInTime[NUM_TARGETS][TARGET_FRAMES],
                        const float targetYInTime[NUM_TARGETS][TARGET_FRAMES],
                        double arrayTimeStep,
                        double simTimeStep,
                        int selectedTarget,
                        bool dropped,
                        double finalDropX,
                        double finalDropY,
                        double currentTime) {
    std::ofstream svg("simulation.svg");
    if (!svg.is_open()) {
        std::cout << "Warning: cannot create simulation.svg\n";
        return;
    }

    const int width = 1100;
    const int height = 820;
    const double margin = 70.0;
    const char *targetColors[NUM_TARGETS] = {
        "#ef4444", "#f59e0b", "#22c55e", "#a855f7", "#eab308"
    };

    double minX = outX[0], maxX = outX[0];
    double minY = outY[0], maxY = outY[0];

    for (int i = 0; i < stepCount; i++) {
        expandBounds(outX[i], outY[i], minX, maxX, minY, maxY);
    }

    int targetSamples = std::max(2, stepCount);
    for (int target = 0; target < NUM_TARGETS; target++) {
        for (int step = 0; step < targetSamples; step++) {
            double t = step * simTimeStep;
            double tx, ty;
            interpTarget(targetXInTime, targetYInTime, target, t, arrayTimeStep, tx, ty);
            expandBounds(tx, ty, minX, maxX, minY, maxY);
        }
    }

    if (dropped) {
        expandBounds(finalDropX, finalDropY, minX, maxX, minY, maxY);
    }

    double padX = std::max(20.0, (maxX - minX) * 0.1);
    double padY = std::max(20.0, (maxY - minY) * 0.1);
    minX -= padX;
    maxX += padX;
    minY -= padY;
    maxY += padY;

    double worldW = std::max(1.0, maxX - minX);
    double worldH = std::max(1.0, maxY - minY);

    auto mapX = [&](double x) {
        return margin + (x - minX) / worldW * (width - 2.0 * margin);
    };
    auto mapY = [&](double y) {
        return height - margin - (y - minY) / worldH * (height - 2.0 * margin);
    };

    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\" viewBox=\"0 0 "
        << width << ' ' << height << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#0f172a\"/>\n";
    svg << "<style>\n";
    svg << "text { fill: #e2e8f0; font-family: Arial, sans-serif; }\n";
    svg << ".title { font-size: 22px; font-weight: bold; }\n";
    svg << ".meta { font-size: 14px; }\n";
    svg << ".small { font-size: 12px; }\n";
    svg << ".grid { stroke: #334155; stroke-width: 1; }\n";
    svg << ".frame { fill: #111827; stroke: #475569; stroke-width: 2; }\n";
    svg << ".drone { fill: none; stroke: #38bdf8; stroke-width: 4; }\n";
    svg << ".drop { fill: #f8fafc; stroke: #f97316; stroke-width: 3; }\n";
    svg << ".start { fill: #22c55e; }\n";
    svg << ".end { fill: #38bdf8; }\n";
    svg << ".legend-line { stroke-width: 3; fill: none; }\n";
    svg << "</style>\n";

    svg << "<text x=\"40\" y=\"38\" class=\"title\">Drone Simulation</text>\n";
    svg << "<text x=\"40\" y=\"62\" class=\"meta\">steps=" << stepCount
        << ", totalTime=" << std::fixed << std::setprecision(2) << currentTime
        << " s, selectedTarget=" << selectedTarget
        << ", dropped=" << (dropped ? "YES" : "NO") << "</text>\n";

    svg << "<rect x=\"" << margin << "\" y=\"" << margin
        << "\" width=\"" << (width - 2.0 * margin)
        << "\" height=\"" << (height - 2.0 * margin)
        << "\" class=\"frame\"/>\n";

    for (int i = 0; i <= 10; i++) {
        double gx = margin + (width - 2.0 * margin) * i / 10.0;
        double gy = margin + (height - 2.0 * margin) * i / 10.0;
        svg << "<line x1=\"" << gx << "\" y1=\"" << margin
            << "\" x2=\"" << gx << "\" y2=\"" << (height - margin)
            << "\" class=\"grid\"/>\n";
        svg << "<line x1=\"" << margin << "\" y1=\"" << gy
            << "\" x2=\"" << (width - margin) << "\" y2=\"" << gy
            << "\" class=\"grid\"/>\n";
    }

    for (int target = 0; target < NUM_TARGETS; target++) {
        svg << "<path d=\"";
        for (int step = 0; step < targetSamples; step++) {
            double t = step * simTimeStep;
            double tx, ty;
            interpTarget(targetXInTime, targetYInTime, target, t, arrayTimeStep, tx, ty);
            if (step == 0) {
                svg << "M " << mapX(tx) << ' ' << mapY(ty) << ' ';
            } else {
                svg << "L " << mapX(tx) << ' ' << mapY(ty) << ' ';
            }
        }
        svg << "\" fill=\"none\" stroke=\"" << targetColors[target]
            << "\" stroke-width=\"2\" stroke-opacity=\"0.75\"/>\n";

        double finalTx, finalTy;
        interpTarget(targetXInTime, targetYInTime, target, currentTime, arrayTimeStep, finalTx, finalTy);
        svg << "<circle cx=\"" << mapX(finalTx) << "\" cy=\"" << mapY(finalTy)
            << "\" r=\"5\" fill=\"" << targetColors[target] << "\"/>\n";
        svg << "<text x=\"" << (mapX(finalTx) + 8) << "\" y=\"" << (mapY(finalTy) - 8)
            << "\" class=\"small\">T" << target << "</text>\n";
    }

    svg << "<path d=\"";
    for (int i = 0; i < stepCount; i++) {
        if (i == 0) {
            svg << "M " << mapX(outX[i]) << ' ' << mapY(outY[i]) << ' ';
        } else {
            svg << "L " << mapX(outX[i]) << ' ' << mapY(outY[i]) << ' ';
        }
    }
    svg << "\" class=\"drone\"/>\n";

    svg << "<circle cx=\"" << mapX(outX[0]) << "\" cy=\"" << mapY(outY[0])
        << "\" r=\"7\" class=\"start\"/>\n";
    svg << "<text x=\"" << (mapX(outX[0]) + 10) << "\" y=\"" << (mapY(outY[0]) - 10)
        << "\" class=\"small\">start</text>\n";

    svg << "<circle cx=\"" << mapX(outX[stepCount - 1]) << "\" cy=\"" << mapY(outY[stepCount - 1])
        << "\" r=\"7\" class=\"end\"/>\n";
    svg << "<text x=\"" << (mapX(outX[stepCount - 1]) + 10)
        << "\" y=\"" << (mapY(outY[stepCount - 1]) + 18)
        << "\" class=\"small\">drone end</text>\n";

    if (dropped) {
        svg << "<circle cx=\"" << mapX(finalDropX) << "\" cy=\"" << mapY(finalDropY)
            << "\" r=\"10\" class=\"drop\"/>\n";
        svg << "<text x=\"" << (mapX(finalDropX) + 12)
            << "\" y=\"" << (mapY(finalDropY) - 12)
            << "\" class=\"small\">drop point</text>\n";
    }

    svg << "<rect x=\"40\" y=\"" << (height - 120)
        << "\" width=\"300\" height=\"88\" rx=\"10\" fill=\"#111827\" stroke=\"#334155\"/>\n";
    svg << "<line x1=\"58\" y1=\"" << (height - 95) << "\" x2=\"102\" y2=\"" << (height - 95)
        << "\" class=\"legend-line\" stroke=\"#38bdf8\"/>\n";
    svg << "<text x=\"112\" y=\"" << (height - 90) << "\" class=\"small\">drone path</text>\n";
    svg << "<circle cx=\"80\" cy=\"" << (height - 68) << "\" r=\"6\" class=\"start\"/>\n";
    svg << "<text x=\"112\" y=\"" << (height - 63) << "\" class=\"small\">start</text>\n";
    svg << "<circle cx=\"80\" cy=\"" << (height - 41) << "\" r=\"8\" class=\"drop\"/>\n";
    svg << "<text x=\"112\" y=\"" << (height - 36) << "\" class=\"small\">drop point</text>\n";

    svg << "</svg>\n";
}

int main() {
    // Усі параметри беремо з input.txt.
    std::ifstream fin("input.txt");
    if (!fin.is_open()) {
        std::cout << "Error: cannot open input.txt\n";
        return 1;
    }

    float xd, yd, zd;
    float initialDir;
    float attackSpeed;
    float accelerationPath;
    char ammo_name[32];
    float arrayTimeStep;
    float simTimeStep;
    float hitRadius;
    float angularSpeed;
    float turnThreshold;

    if (!(fin >> xd >> yd >> zd
              >> initialDir
              >> attackSpeed
              >> accelerationPath
              >> ammo_name
              >> arrayTimeStep
              >> simTimeStep
              >> hitRadius
              >> angularSpeed
              >> turnThreshold)) {
        std::cout << "Error: invalid input.txt format\n";
        return 1;
    }

    int ammoIdx = findAmmo(ammo_name);
    if (ammoIdx < 0) {
        std::cout << "Error: unknown ammo type\n";
        return 1;
    }

    double m = bombM[ammoIdx];
    double d = bombD[ammoIdx];
    double l = bombL[ammoIdx];

    // Перевірка всіх числових параметрів на адекватність.
    // Без цього симуляція може мовчки піти у нескінченний цикл або дати NaN.
    if (simTimeStep <= 0.0f || arrayTimeStep <= 0.0f ||
        accelerationPath <= 0.0f || attackSpeed <= 0.0f) {
        std::cout << "Error: invalid numeric parameters (step/accel/speed must be > 0)\n";
        return 1;
    }

    if (zd <= 0.0f) {
        std::cout << "Error: invalid altitude zd (must be > 0)\n";
        return 1;
    }

    if (angularSpeed <= 0.0f) {
        std::cout << "Error: invalid angularSpeed (must be > 0)\n";
        return 1;
    }

    if (hitRadius <= 0.0f) {
        std::cout << "Error: invalid hitRadius (must be > 0)\n";
        return 1;
    }

    if (turnThreshold < 0.0f) {
        std::cout << "Error: invalid turnThreshold (must be >= 0)\n";
        return 1;
    }

    // Координати цілей у форматі 5 рядків X, потім 5 рядків Y.
    std::ifstream ftargets("targets.txt");
    if (!ftargets.is_open()) {
        std::cout << "Error: cannot open targets.txt\n";
        return 1;
    }

    static float targetXInTime[NUM_TARGETS][TARGET_FRAMES];
    static float targetYInTime[NUM_TARGETS][TARGET_FRAMES];

    for (int i = 0; i < NUM_TARGETS; i++) {
        for (int j = 0; j < TARGET_FRAMES; j++) {
            if (!(ftargets >> targetXInTime[i][j])) {
                std::cout << "Error: targets.txt incomplete (X part)\n";
                return 1;
            }
        }
    }
    for (int i = 0; i < NUM_TARGETS; i++) {
        for (int j = 0; j < TARGET_FRAMES; j++) {
            if (!(ftargets >> targetYInTime[i][j])) {
                std::cout << "Error: targets.txt incomplete (Y part)\n";
                return 1;
            }
        }
    }

    // Прискорення з формули a = V^2 / (2 * S).
    double accel = (double)attackSpeed * (double)attackSpeed /
                   (2.0 * (double)accelerationPath);

    // Поточний стан симуляції.
    double posX = xd;
    double posY = yd;
    double dir = initialDir;
    double speed = 0.0;
    DroneState state = ACCELERATING;
    double currentTime = 0.0;
    int selectedTarget = -1;
    double turnRemainingTime = 0.0;

    // Буфери виводу. Розміщуємо статично, щоб не зіпсувати стек.
    static double outX[MAX_STEPS + 1];
    static double outY[MAX_STEPS + 1];
    static double outDir[MAX_STEPS + 1];
    static int outState[MAX_STEPS + 1];
    static int outTarget[MAX_STEPS + 1];
    int stepCount = 0;

    bool dropped = false;
    double finalDropX = posX;
    double finalDropY = posY;

    while (stepCount < MAX_STEPS && !dropped) {
        // Для кожної цілі рахуємо точку скиду і час прильоту.
        double bestTotal = 1e18;
        int bestTarget = -1;
        double bestDropX = 0.0, bestDropY = 0.0;

        for (int i = 0; i < NUM_TARGETS; i++) {
            double dropX, dropY, arrivalTime, tOfFlight;
            bool ok = computeDropPoint(targetXInTime, targetYInTime, i,
                                       currentTime, arrayTimeStep, simTimeStep,
                                       (double)zd, (double)attackSpeed, m, d, l,
                                       posX, posY,
                                       speed, accel,
                                       dropX, dropY, arrivalTime, tOfFlight);
            if (!ok) continue;

            double total = arrivalTime + tOfFlight;

            // Штраф за зміну цілі: зупинка + поворот + повторний розгін.
            // Якщо це та сама ціль - штрафу немає.
            if (i != selectedTarget && selectedTarget >= 0) {
                double desiredDir = std::atan2(dropY - posY, dropX - posX);
                double deltaAngle = normAngle(desiredDir - dir);
                double stopTime = computeTimeToStop(state, speed, (double)attackSpeed,
                                                    accel, turnRemainingTime);

                if (std::fabs(deltaAngle) > (double)turnThreshold) {
                    // Великий кут: дрон зупиняється, повертається і рушає з 0.
                    // Тому шлях до точки скиду треба перерахувати з нульової швидкості.
                    double turnTime = std::fabs(deltaAngle) / (double)angularSpeed;
                    double ddx = dropX - posX;
                    double ddy = dropY - posY;
                    double dropDist = std::sqrt(ddx * ddx + ddy * ddy);
                    double newTravel = estimateDroneTravelTime(dropDist, 0.0,
                                                               (double)attackSpeed, accel);
                    total = stopTime + turnTime + newTravel + tOfFlight;
                } else {
                    // Малий кут: корекція курсу на ходу, без зупинки.
                    // Але поворот усе одно займе якийсь час - додаємо його.
                    double turnTime = std::fabs(deltaAngle) / (double)angularSpeed;
                    total = arrivalTime + tOfFlight + turnTime;
                }
            }

            if (total < bestTotal) {
                bestTotal = total;
                bestTarget = i;
                bestDropX = dropX;
                bestDropY = dropY;
            }
        }

        if (bestTarget < 0) {
            // Немає жодної валідної цілі. Записуємо крок і виходимо.
            outX[stepCount] = posX;
            outY[stepCount] = posY;
            outDir[stepCount] = dir;
            outState[stepCount] = (int)state;
            outTarget[stepCount] = selectedTarget;
            stepCount++;
            break;
        }

        selectedTarget = bestTarget;
        finalDropX = bestDropX;
        finalDropY = bestDropY;

        // Фіксуємо стан на цьому кроці у вихідні масиви.
        outX[stepCount] = posX;
        outY[stepCount] = posY;
        outDir[stepCount] = dir;
        outState[stepCount] = (int)state;
        outTarget[stepCount] = selectedTarget;
        stepCount++;

        // Якщо вже в межах радіусу ураження - скидаємо і виходимо.
        double dropDx = bestDropX - posX;
        double dropDy = bestDropY - posY;
        double distToDrop = std::sqrt(dropDx * dropDx + dropDy * dropDy);
        if (distToDrop <= (double)hitRadius) {
            dropped = true;
            break;
        }

        // Бажаний курс на точку скиду.
        double desiredDir = std::atan2(dropDy, dropDx);
        double deltaAngle = normAngle(desiredDir - dir);
        double maxTurn = (double)angularSpeed * (double)simTimeStep;

        // --- Машина станів ---
        // Якщо кут великий і ми ще не гальмуємо/не повертаємо - запускаємо
        // потрібну фазу. Вже розпочаті TURNING/DECELERATING не перериваємо.
        if (state != TURNING && state != DECELERATING) {
            if (std::fabs(deltaAngle) > (double)turnThreshold) {
                state = (speed > EPS) ? DECELERATING : TURNING;
            }
        }

        if (state == TURNING) {
            // Стоїмо (v=0) і крутимось з кутовою швидкістю.
            if (std::fabs(deltaAngle) <= maxTurn) {
                dir = desiredDir;
                turnRemainingTime = 0.0;
                state = ACCELERATING;
            } else {
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
                turnRemainingTime = std::fabs(deltaAngle) / (double)angularSpeed - (double)simTimeStep;
                if (turnRemainingTime < 0.0) turnRemainingTime = 0.0;
            }
        } else if (state == DECELERATING) {
            // Гальмуємо до повної зупинки. Рішення, що далі (TURNING або ACCELERATING),
            // приймаємо тільки після того, як швидкість реально впала в 0.
            double newSpeed = speed - accel * (double)simTimeStep;
            if (newSpeed > 0.0) {
                double avgV = (speed + newSpeed) / 2.0;
                posX += avgV * std::cos(dir) * (double)simTimeStep;
                posY += avgV * std::sin(dir) * (double)simTimeStep;
                speed = newSpeed;
            } else {
                // Зупинка всередині кроку. Залишок часу використовуємо
                // для нової фази - поворот або розгін.
                double tDecel = speed / accel;
                double avgV = speed / 2.0;
                posX += avgV * std::cos(dir) * tDecel;
                posY += avgV * std::sin(dir) * tDecel;
                speed = 0.0;

                double tRem = (double)simTimeStep - tDecel;
                if (tRem < 0.0) tRem = 0.0;
                double turnPart = (double)angularSpeed * tRem;

                if (std::fabs(deltaAngle) > (double)turnThreshold) {
                    // Переходимо у поворот на місці.
                    if (std::fabs(deltaAngle) <= turnPart) {
                        dir = desiredDir;
                        turnRemainingTime = 0.0;
                        state = ACCELERATING;
                    } else {
                        dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);
                        state = TURNING;
                        turnRemainingTime = std::fabs(deltaAngle) / (double)angularSpeed - tRem;
                        if (turnRemainingTime < 0.0) turnRemainingTime = 0.0;
                    }
                } else {
                    // Кут став малим ще до зупинки - одразу починаємо розгін.
                    if (std::fabs(deltaAngle) <= turnPart) dir = desiredDir;
                    else dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * turnPart);

                    state = ACCELERATING;
                    double newV = accel * tRem;
                    if (newV > (double)attackSpeed) newV = (double)attackSpeed;
                    double avg = newV / 2.0;
                    posX += avg * std::cos(dir) * tRem;
                    posY += avg * std::sin(dir) * tRem;
                    speed = newV;
                    if (speed >= (double)attackSpeed - EPS) state = MOVING;
                }
            }
        } else {
            // STOPPED / ACCELERATING / MOVING - кут вже в межах порогу,
            // коригуємо курс на ходу.
            if (std::fabs(deltaAngle) <= maxTurn) {
                dir = desiredDir;
            } else {
                dir = normAngle(dir + (deltaAngle > 0.0 ? 1.0 : -1.0) * maxTurn);
            }

            if (state == STOPPED) {
                state = ACCELERATING;
            }

            if (state == ACCELERATING) {
                double newSpeed = speed + accel * (double)simTimeStep;
                if (newSpeed >= (double)attackSpeed) {
                    // Частина кроку - розгін, решта - крейсерська швидкість.
                    double tAccel = ((double)attackSpeed - speed) / accel;
                    if (tAccel < 0.0) tAccel = 0.0;
                    double avgV1 = (speed + (double)attackSpeed) / 2.0;
                    double tMove = (double)simTimeStep - tAccel;
                    posX += avgV1 * std::cos(dir) * tAccel
                          + (double)attackSpeed * std::cos(dir) * tMove;
                    posY += avgV1 * std::sin(dir) * tAccel
                          + (double)attackSpeed * std::sin(dir) * tMove;
                    speed = (double)attackSpeed;
                    state = MOVING;
                } else {
                    double avgV = (speed + newSpeed) / 2.0;
                    posX += avgV * std::cos(dir) * (double)simTimeStep;
                    posY += avgV * std::sin(dir) * (double)simTimeStep;
                    speed = newSpeed;
                }
            } else if (state == MOVING) {
                posX += (double)attackSpeed * std::cos(dir) * (double)simTimeStep;
                posY += (double)attackSpeed * std::sin(dir) * (double)simTimeStep;
            }
        }

        currentTime += (double)simTimeStep;
    }

    // Фінальний запис у simulation.txt.
    std::ofstream fout("simulation.txt");
    if (!fout.is_open()) {
        std::cout << "Error: cannot create simulation.txt\n";
        return 1;
    }

    fout << stepCount << "\n";

    fout << std::setprecision(6) << std::fixed;
    for (int i = 0; i < stepCount; i++) {
        fout << outX[i] << ' ' << outY[i];
        if (i + 1 < stepCount) fout << ' ';
    }
    fout << '\n';

    for (int i = 0; i < stepCount; i++) {
        fout << outDir[i];
        if (i + 1 < stepCount) fout << ' ';
    }
    fout << '\n';

    for (int i = 0; i < stepCount; i++) {
        fout << outState[i];
        if (i + 1 < stepCount) fout << ' ';
    }
    fout << '\n';

    for (int i = 0; i < stepCount; i++) {
        fout << outTarget[i];
        if (i + 1 < stepCount) fout << ' ';
    }
    fout << '\n';

    writeSimulationSvg(outX, outY, stepCount,
                       targetXInTime, targetYInTime,
                       arrayTimeStep, simTimeStep,
                       selectedTarget, dropped,
                       finalDropX, finalDropY, currentTime);

    std::cout << "Simulation finished. Steps: " << stepCount << "\n";
    std::cout << "Selected target at end: " << selectedTarget << "\n";
    std::cout << "Dropped: " << (dropped ? "YES" : "NO (limit or no target)") << "\n";

    return 0;
}

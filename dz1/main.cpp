#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

const double G = 9.81;
const double EPS = 1e-9;

// Тут просто зберігаємо параметри боєприпасу.
// m - маса, d - опір, l - підйомна сила, якщо снаряд може планерувати.
struct AmmoInfo {
    double m;
    double d;
    double l;
};

struct TrajectoryPoint {
    double t;
    double x;
    double y;
    double z;
};

// Сюди складаємо все, що нарахували по одному кейсу.
// Потім це йде і в `output/report`, і в SVG, шоб двічі не перераховувати.
struct CaseResult {
    int caseNumber;               // Номер тестового кейсу з input.txt
    double xd;                    // Початкова координата дрона по осі X
    double yd;                    // Початкова координата дрона по осі Y
    double zd;                    // Висота дрона в момент початку розрахунку
    double targetX;               // Координата цілі по осі X
    double targetY;               // Координата цілі по осі Y
    double attackSpeed;           // Швидкість атаки дрона перед скиданням
    double accelerationPath;      // Довжина розгону дрона перед атакою
    std::string ammoName;         // Назва обраного боєприпасу
    AmmoInfo ammo;                // Параметри цього боєприпасу: маса, drag, lift
    double timeOfFlight;          // Обчислений час польоту боєприпасу до землі
    double horizontalDistance;    // Горизонтальна дистанція, яку пролетить боєприпас
    double distanceToTarget;      // Пряма відстань від дрона до цілі
    bool needManeuver;            // Ознака того, чи потрібно відлетіти далі перед атакою
    double intermediateX;         // X-координата проміжної точки для маневру
    double intermediateY;         // Y-координата проміжної точки для маневру
    double ratio;                 // Пропорція для обчислення точки скиду відносно цілі
    double fireX;                 // X-координата точки скиду боєприпасу
    double fireY;                 // Y-координата точки скиду боєприпасу
    std::vector<TrajectoryPoint> projectilePath;
};

// Шукаємо потрібний тип боєприпасу в табличці.
// Формат рядка в `ammo_data.txt` простий: name m d l.
// Якщо знайшли - заповнюємо `ammo`, якщо ні то просто false.
bool getAmmoInfo(const char ammoName[], AmmoInfo& ammo) {
    std::ifstream ammoFile("ammo_data.txt");
    if (!ammoFile.is_open()) {
        return false;
    }

    char fileAmmoName[64];
    double m, d, l;

    while (ammoFile >> fileAmmoName >> m >> d >> l) {
        if (std::strcmp(ammoName, fileAmmoName) == 0) {
            ammo.m = m;
            ammo.d = d;
            ammo.l = l;
            return true;
        }
    }

    return false;
}

// Розв'язуємо кубічне через Кардано.
double solveCardanoTime(double a, double b, double c, bool& ok) {
    ok = false;

    // Якщо `a` майже 0, то модель вже якось криво себе веде, так шо виходимо.
    if (std::fabs(a) < EPS) {
        return 0.0;
    }

    // Зводимо рівняння до вигляду, з яким Кардано вже нормально працює.
    double p = -(b * b) / (3.0 * a * a);
    double q = (2.0 * b * b * b) / (27.0 * a * a * a) + c / a;

    // Тут потрібен випадок `p < 0`, інакше далі формула не зайде.
    if (p >= 0.0) {
        return 0.0;
    }

    // Заодно страхуємо `acos`, бо якщо вилізти за [-1; 1], буде вже фігня.
    double acosArg = (3.0 * q / (2.0 * p)) * std::sqrt(-3.0 / p);
    if (acosArg < -1.0 || acosArg > 1.0) {
        return 0.0;
    }

    double phi = std::acos(acosArg);
    double rootBase = 2.0 * std::sqrt(-p / 3.0);

    double t1 = rootBase * std::cos(phi / 3.0) - b / (3.0 * a);
    double t2 = rootBase * std::cos((phi + 2.0 * M_PI) / 3.0) - b / (3.0 * a);
    double t3 = rootBase * std::cos((phi + 4.0 * M_PI) / 3.0) - b / (3.0 * a);

    // За умовою спочатку пробуємо саме цей корінь.
    if (t3 > EPS) {
        ok = true;
        return t3;
    }

    // Якщо основний не підійшов, беремо запасні, а то раптом розв'язок все ж є.
    if (t2 > EPS) {
        ok = true;
        return t2;
    }

    if (t1 > EPS) {
        ok = true;
        return t1;
    }

    return 0.0;
}

// Рахуємо горизонтальну дальність.
double calcHorizontalDistance(double t, double m, double d, double l, double V0) {
    double term1 = V0 * t;

    double term2 = -(t * t * d * V0) / (2.0 * m);

    double term3 =
        (t * t * t * (6.0 * d * G * l * m - 6.0 * d * d * (l * l - 1.0) * V0)) /
        (36.0 * m * m);

    double term4 =
        (std::pow(t, 4.0) *
         (-6.0 * d * d * G * l * (1.0 + l * l + l * l * l * l) * m +
          3.0 * d * d * d * l * l * (1.0 + l * l) * V0 +
          6.0 * d * d * d * l * l * l * l * (1.0 + l * l) * V0)) /
        (36.0 * std::pow(1.0 + l * l, 2.0) * m * m * m);

    double term5 =
        (std::pow(t, 5.0) *
         (3.0 * d * d * d * G * l * l * l * m -
          3.0 * std::pow(d, 4.0) * l * l * (1.0 + l * l) * V0)) /
        (36.0 * (1.0 + l * l) * std::pow(m, 4.0));

    return term1 + term2 + term3 + term4 + term5;
}

// Висота в момент часу `t`.
// Потрібно в основному для бокового графіка; нижче землі не пускаємо, обрізаємо в 0.
double calcAltitude(double t, double z0, double a, double b, double m) {
    double z = z0 + (b * t * t) / (6.0 * m * m) + (a * t * t * t) / (6.0 * m * m);
    if (z < 0.0) {
        z = 0.0;
    }
    return z;
}

std::vector<TrajectoryPoint> buildProjectilePath(double fireX, double fireY,
                                                 double targetX, double targetY,
                                                 double z0, double timeOfFlight,
                                                 double horizontalDistance,
                                                 const AmmoInfo& ammo,
                                                 double attackSpeed) {
    std::vector<TrajectoryPoint> path;
    const int samples = 64;

    double dirX = targetX - fireX;
    double dirY = targetY - fireY;
    double dirLen = std::sqrt(dirX * dirX + dirY * dirY);
    if (dirLen <= EPS || horizontalDistance <= EPS || timeOfFlight <= EPS) {
        return path;
    }

    double a = ammo.d * G * ammo.m - 2.0 * ammo.d * ammo.d * ammo.l * attackSpeed;
    double b = -3.0 * G * ammo.m * ammo.m + 3.0 * ammo.d * ammo.l * ammo.m * attackSpeed;

    path.reserve(samples + 1);
    for (int step = 0; step <= samples; ++step) {
        double t = timeOfFlight * static_cast<double>(step) / static_cast<double>(samples);
        double distance = calcHorizontalDistance(t, ammo.m, ammo.d, ammo.l, attackSpeed);
        if (distance < 0.0) {
            distance = 0.0;
        }
        if (distance > horizontalDistance) {
            distance = horizontalDistance;
        }

        double ratio = distance / horizontalDistance;
        double z = calcAltitude(t, z0, a, b, ammo.m);

        TrajectoryPoint point;
        point.t = t;
        point.x = fireX + dirX * ratio;
        point.y = fireY + dirY * ratio;
        point.z = z;
        path.push_back(point);
    }

    return path;
}

std::string escapeJson(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size());

    for (size_t i = 0; i < value.size(); ++i) {
        char ch = value[i];
        if (ch == '\\' || ch == '"') {
            escaped.push_back('\\');
        }
        escaped.push_back(ch);
    }

    return escaped;
}

void writeProjectileTrajectoryFile(const std::vector<CaseResult>& results) {
    std::ofstream trajFile("projectile_trajectory.json");
    if (!trajFile.is_open()) {
        std::cout << "Warning: cannot create projectile_trajectory.json\n";
        return;
    }

    trajFile << "{\n";
    trajFile << "  \"cases\": [\n";
    for (size_t i = 0; i < results.size(); ++i) {
        const CaseResult& r = results[i];
        trajFile << "    {\n";
        trajFile << "      \"caseNumber\": " << r.caseNumber << ",\n";
        trajFile << "      \"ammoName\": \"" << escapeJson(r.ammoName) << "\",\n";
        trajFile << "      \"dropPoint\": {\"x\": " << r.fireX << ", \"y\": " << r.fireY << ", \"z\": " << r.zd << "},\n";
        trajFile << "      \"targetPoint\": {\"x\": " << r.targetX << ", \"y\": " << r.targetY << ", \"z\": 0},\n";
        trajFile << "      \"timeOfFlight\": " << r.timeOfFlight << ",\n";
        trajFile << "      \"horizontalDistance\": " << r.horizontalDistance << ",\n";
        trajFile << "      \"points\": [\n";

        for (size_t j = 0; j < r.projectilePath.size(); ++j) {
            const TrajectoryPoint& point = r.projectilePath[j];
            trajFile << "        {\"t\": " << point.t
                     << ", \"x\": " << point.x
                     << ", \"y\": " << point.y
                     << ", \"z\": " << point.z << "}";
            if (j + 1 < r.projectilePath.size()) {
                trajFile << ',';
            }
            trajFile << '\n';
        }

        trajFile << "      ]\n";
        trajFile << "    }";
        if (i + 1 < results.size()) {
            trajFile << ',';
        }
        trajFile << '\n';
    }
    trajFile << "  ]\n";
    trajFile << "}\n";

    std::cout << "Projectile trajectory written to projectile_trajectory.json\n";
}

// Пишемо шапку SVG: розміри, фон, стилі. Тут без магії.
void writeSvgHeader(std::ofstream& svg, int width, int height) {
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height
        << "\" viewBox=\"0 0 " << width << ' ' << height << "\">\n";
    svg << "<rect width=\"100%\" height=\"100%\" fill=\"#0f172a\"/>\n";
    svg << "<style>\n";
    svg << "text { fill: #e2e8f0; font-family: Arial, sans-serif; }\n";
    svg << ".label { font-size: 13px; }\n";
    svg << ".title { font-size: 18px; font-weight: bold; }\n";
    svg << ".small { font-size: 11px; }\n";
    svg << ".legend { font-size: 12px; }\n";
    svg << ".grid { stroke: #334155; stroke-width: 1; }\n";
    svg << ".axis { stroke: #94a3b8; stroke-width: 2; }\n";
    svg << ".path { fill: none; stroke: #38bdf8; stroke-width: 3; }\n";
    svg << ".guide { fill: none; stroke: #64748b; stroke-width: 1.5; stroke-dasharray: 6 4; }\n";
    svg << "</style>\n";
}

// Тут збираємо `trajectories.svg`.
// Для кожного кейсу малюємо вид зверху і збоку, шоб було зрозуміліше, шо взагалі сталося.
void writeTrajectorySvg(const std::vector<CaseResult>& results) {
    std::ofstream svg("trajectories.svg");
    if (!svg.is_open()) {
        std::cout << "Warning: cannot create trajectories.svg\n";
        return;
    }

    int panelWidth = 980;
    int panelHeight = 320;
    int headerHeight = 110;
    int totalHeight = headerHeight + static_cast<int>(results.size()) * panelHeight;
    writeSvgHeader(svg, panelWidth, totalHeight);

    svg << "<text x=\"30\" y=\"30\" class=\"title\">Trajectory Visualization</text>\n";
    svg << "<rect x=\"30\" y=\"42\" width=\"920\" height=\"52\" rx=\"10\" fill=\"#111827\" stroke=\"#334155\"/>\n";
    svg << "<circle cx=\"55\" cy=\"61\" r=\"6\" fill=\"#22c55e\"/>\n";
    svg << "<text x=\"68\" y=\"65\" class=\"legend\">drone - початкова позиція дрона</text>\n";
    svg << "<circle cx=\"315\" cy=\"61\" r=\"6\" fill=\"#f59e0b\"/>\n";
    svg << "<text x=\"328\" y=\"65\" class=\"legend\">intermediate - точка маневру перед атакою</text>\n";
    svg << "<circle cx=\"625\" cy=\"61\" r=\"6\" fill=\"#38bdf8\"/>\n";
    svg << "<text x=\"638\" y=\"65\" class=\"legend\">drop - точка скиду боєприпасу</text>\n";
    svg << "<circle cx=\"55\" cy=\"82\" r=\"6\" fill=\"#ef4444\"/>\n";
    svg << "<text x=\"68\" y=\"86\" class=\"legend\">target - координати цілі</text>\n";
    svg << "<line x1=\"315\" y1=\"82\" x2=\"380\" y2=\"82\" class=\"guide\"/>\n";
    svg << "<text x=\"390\" y=\"86\" class=\"legend\">маршрут: drone -> intermediate -> drop -> target</text>\n";

    for (size_t i = 0; i < results.size(); ++i) {
        const CaseResult& r = results[i];

        // Кожен кейс малюємо в своїй смузі, інакше все налізе і буде каша.
        double baseY = static_cast<double>(headerHeight) + static_cast<double>(i) * panelHeight;

        double topLeftX = 30.0;
        double topLeftY = baseY + 20.0;
        double topWidth = 430.0;
        double topHeight = 220.0;

        double sideLeftX = 520.0;
        double sideLeftY = baseY + 20.0;
        double sideWidth = 430.0;
        double sideHeight = 220.0;

        // Підбираємо межі для top-view по всіх важливих точках.
        // Трохи з запасом, шоб маркери не прилипали до країв.
        double minX = r.fireX;
        double maxX = r.targetX;
        if (r.xd < minX) minX = r.xd;
        if (r.xd > maxX) maxX = r.xd;
        if (r.intermediateX < minX) minX = r.intermediateX;
        if (r.intermediateX > maxX) maxX = r.intermediateX;

        double minY = r.fireY;
        double maxY = r.targetY;
        if (r.yd < minY) minY = r.yd;
        if (r.yd > maxY) maxY = r.yd;
        if (r.intermediateY < minY) minY = r.intermediateY;
        if (r.intermediateY > maxY) maxY = r.intermediateY;

        double padX = std::max(20.0, (maxX - minX) * 0.15);
        double padY = std::max(20.0, (maxY - minY) * 0.15);
        minX -= padX;
        maxX += padX;
        minY -= padY;
        maxY += padY;

        // Розміри світу для поточного кейсу.
        // `max(1.0, ...)` тут чисто захист від ділення на нуль, про всяк випадок.
        double worldWidth = std::max(1.0, maxX - minX);
        double worldHeight = std::max(1.0, maxY - minY);

        // Переводимо реальні X/Y в координати картинки.
        auto mapTopX = [&](double x) {
            return topLeftX + (x - minX) / worldWidth * topWidth;
        };
        auto mapTopY = [&](double y) {
            return topLeftY + topHeight - (y - minY) / worldHeight * topHeight;
        };

        // Те саме для бокового профілю.
        auto mapSideX = [&](double distance) {
            return sideLeftX + distance / std::max(r.horizontalDistance, 1.0) * sideWidth;
        };
        auto mapSideY = [&](double z) {
            return sideLeftY + sideHeight - z / std::max(r.zd, 1.0) * sideHeight;
        };

        svg << "<text x=\"" << topLeftX << "\" y=\"" << baseY
            << "\" class=\"title\">Case " << r.caseNumber
            << " (" << r.ammoName << ")</text>\n";

        svg << "<rect x=\"" << topLeftX << "\" y=\"" << topLeftY
            << "\" width=\"" << topWidth << "\" height=\"" << topHeight
            << "\" fill=\"#111827\" stroke=\"#475569\"/>\n";
        svg << "<rect x=\"" << sideLeftX << "\" y=\"" << sideLeftY
            << "\" width=\"" << sideWidth << "\" height=\"" << sideHeight
            << "\" fill=\"#111827\" stroke=\"#475569\"/>\n";

        svg << "<text x=\"" << topLeftX << "\" y=\"" << topLeftY - 6
            << "\" class=\"label\">Top view (X/Y)</text>\n";
        svg << "<text x=\"" << sideLeftX << "\" y=\"" << sideLeftY - 6
            << "\" class=\"label\">Side view (distance/height)</text>\n";

        // Пунктиром просто показуємо загальний напрямок атаки.
        svg << "<line x1=\"" << mapTopX(r.xd) << "\" y1=\"" << mapTopY(r.yd)
            << "\" x2=\"" << mapTopX(r.targetX) << "\" y2=\"" << mapTopY(r.targetY)
            << "\" class=\"guide\"/>\n";

        // Якщо маневр потрібен, позначаємо проміжну точку, куди дрон спочатку йде.
        if (r.needManeuver) {
            svg << "<circle cx=\"" << mapTopX(r.intermediateX)
                << "\" cy=\"" << mapTopY(r.intermediateY)
                << "\" r=\"5\" fill=\"#f59e0b\"/>\n";
            svg << "<text x=\"" << mapTopX(r.intermediateX) + 8
                << "\" y=\"" << mapTopY(r.intermediateY) - 8
                << "\" class=\"small\">intermediate</text>\n";
        }

        svg << "<circle cx=\"" << mapTopX(r.xd) << "\" cy=\"" << mapTopY(r.yd)
            << "\" r=\"6\" fill=\"#22c55e\"/>\n";
        svg << "<text x=\"" << mapTopX(r.xd) + 8 << "\" y=\"" << mapTopY(r.yd) - 8
            << "\" class=\"small\">drone</text>\n";

        svg << "<circle cx=\"" << mapTopX(r.fireX) << "\" cy=\"" << mapTopY(r.fireY)
            << "\" r=\"6\" fill=\"#38bdf8\"/>\n";
        svg << "<text x=\"" << mapTopX(r.fireX) + 8 << "\" y=\"" << mapTopY(r.fireY) - 8
            << "\" class=\"small\">drop</text>\n";

        svg << "<circle cx=\"" << mapTopX(r.targetX) << "\" cy=\"" << mapTopY(r.targetY)
            << "\" r=\"6\" fill=\"#ef4444\"/>\n";
        svg << "<text x=\"" << mapTopX(r.targetX) + 8 << "\" y=\"" << mapTopY(r.targetY) - 8
            << "\" class=\"small\">target</text>\n";

        svg << "<line x1=\"" << mapSideX(0.0) << "\" y1=\"" << mapSideY(0.0)
            << "\" x2=\"" << mapSideX(r.horizontalDistance) << "\" y2=\"" << mapSideY(0.0)
            << "\" class=\"axis\"/>\n";

        // Бокову траєкторію малюємо кусочно, по набору точок.
        // `samples = 60` вистачає, шоб виглядало норм, а не рвано.
        svg << "<path d=\"";
        const int samples = 60;
        for (int step = 0; step <= samples; ++step) {
            double t = r.timeOfFlight * static_cast<double>(step) / static_cast<double>(samples);
            double distance = calcHorizontalDistance(t, r.ammo.m, r.ammo.d, r.ammo.l, r.attackSpeed);
            if (distance < 0.0) {
                distance = 0.0;
            }
            if (distance > r.horizontalDistance) {
                distance = r.horizontalDistance;
            }

            // Тут ще раз рахуємо висоту для конкретного кроку.
            // Так, формула довга, але зате не треба городити зайві змінні вище.
            double z = calcAltitude(t, r.zd, r.ammo.d * G * r.ammo.m - 2.0 * r.ammo.d * r.ammo.d * r.ammo.l * r.attackSpeed,
                                    -3.0 * G * r.ammo.m * r.ammo.m + 3.0 * r.ammo.d * r.ammo.l * r.ammo.m * r.attackSpeed,
                                    r.ammo.m);

            if (step == 0) {
                svg << "M " << mapSideX(distance) << ' ' << mapSideY(z) << ' ';
            } else {
                svg << "L " << mapSideX(distance) << ' ' << mapSideY(z) << ' ';
            }
        }
        svg << "\" class=\"path\"/>\n";

        svg << "<text x=\"" << sideLeftX << "\" y=\"" << sideLeftY + sideHeight + 22
            << "\" class=\"small\">t=" << std::fixed << std::setprecision(2) << r.timeOfFlight
            << " s, h=" << r.horizontalDistance
            << " m, D=" << r.distanceToTarget << " m</text>\n";
    }

    svg << "</svg>\n";
    std::cout << "Trajectory visualization written to trajectories.svg\n";
}

int main() {
    // Читаємо вхід з `input.txt`.
    // Коротку відповідь кидаємо в `output.txt`, подробиці - в `report.txt`.
    std::ifstream fin("input.txt");
    std::ofstream fout("output.txt");
    std::ofstream freport("report.txt");

    if (!fin.is_open()) {
        std::cout << "Error: cannot open input.txt\n";
        return 1;
    }

    if (!fout.is_open()) {
        std::cout << "Error: cannot open output.txt\n";
        return 1;
    }

    if (!freport.is_open()) {
        std::cout << "Error: cannot open report.txt\n";
        return 1;
    }

    std::cout << "Program started\n";
    std::cout << "Reading data from input.txt\n";

    // Змінні під один поточний кейс.
    double xd, yd, zd;
    double targetX, targetY;
    double attackSpeed;
    double accelerationPath;
    char ammo_name[64];
    int caseNumber = 0;

    // Сюди копимо всі кейси, потім по них одним махом малюємо SVG.
    std::vector<CaseResult> results;

    // У файлі може бути кілька наборів підряд.
    // Кожні 8 значень - це один кейс, нічого хитрого.
    while (fin >> xd >> yd >> zd
               >> targetX >> targetY
               >> attackSpeed
               >> accelerationPath
               >> ammo_name) {
        caseNumber++;
        std::cout << "\n=== Case " << caseNumber << " ===\n";
        std::cout << "Drone: x=" << xd << ", y=" << yd << ", z=" << zd << '\n';
        std::cout << "Target: x=" << targetX << ", y=" << targetY << '\n';
        std::cout << "Attack speed: " << attackSpeed << '\n';
        std::cout << "Acceleration path: " << accelerationPath << '\n';
        std::cout << "Ammo: " << ammo_name << '\n';

        // Підтягуємо параметри вибраного боєприпасу.
        AmmoInfo ammo;
        if (!getAmmoInfo(ammo_name, ammo)) {
            std::cout << "Error: unknown ammo type\n";
            fout << "Case " << caseNumber << ": Error: unknown ammo type\n";
            freport << "Case " << caseNumber << ": Error: unknown ammo type\n";
            return 1;
        }

        std::cout << "Ammo parameters loaded: m=" << ammo.m
                  << ", d=" << ammo.d
                  << ", l=" << ammo.l << '\n';

        // Коефіцієнти для рівняння часу польоту.
        double a = ammo.d * G * ammo.m - 2.0 * ammo.d * ammo.d * ammo.l * attackSpeed;
        double b = -3.0 * G * ammo.m * ammo.m + 3.0 * ammo.d * ammo.l * ammo.m * attackSpeed;
        double c = 6.0 * ammo.m * ammo.m * zd;

        std::cout << "Cardano coefficients: a=" << a
                  << ", b=" << b
                  << ", c=" << c << '\n';

        bool timeOk = false;
        double t = solveCardanoTime(a, b, c, timeOk);
        if (!timeOk || t <= EPS) {
            std::cout << "Error: invalid flight time\n";
            fout << "Case " << caseNumber << ": Error: invalid flight time\n";
            freport << "Case " << caseNumber << ": Error: invalid flight time\n";
            return 1;
        }

        // Скільки по горизонталі встигне пролетіти за знайдений час.
        std::cout << "Time of flight: " << t << '\n';

        double h = calcHorizontalDistance(t, ammo.m, ammo.d, ammo.l, attackSpeed);
        if (h <= EPS) {
            std::cout << "Error: invalid horizontal distance\n";
            fout << "Case " << caseNumber << ": Error: invalid horizontal distance\n";
            freport << "Case " << caseNumber << ": Error: invalid horizontal distance\n";
            return 1;
        }

        std::cout << "Horizontal distance: " << h << '\n';

        // Звичайна дистанція від дрона до цілі по площині.
        double dx = targetX - xd;
        double dy = targetY - yd;
        double D = std::sqrt(dx * dx + dy * dy);
        if (D <= EPS) {
            std::cout << "Error: invalid distance to target\n";
            fout << "Case " << caseNumber << ": Error: invalid distance to target\n";
            freport << "Case " << caseNumber << ": Error: invalid distance to target\n";
            return 1;
        }

        std::cout << "Distance to target D: " << D << '\n';

        double intermediateX = xd;
        double intermediateY = yd;

        // Якщо місця для розгону + скиду не вистачає, дрону треба спочатку відійти назад.
        bool needManeuver = (h + accelerationPath > D);

        std::cout << "Checking maneuver: h + accelerationPath = "
                  << (h + accelerationPath) << '\n';
        std::cout << "Need maneuver: " << (needManeuver ? "YES" : "NO") << '\n';

        if (needManeuver) {
            intermediateX = targetX - (targetX - xd) * (h + accelerationPath) / D;
            intermediateY = targetY - (targetY - yd) * (h + accelerationPath) / D;
            std::cout << "Intermediate point: x=" << intermediateX
                      << ", y=" << intermediateY << '\n';
        }

        // Після маневру вважаємо, що атака стартує вже з нової точки.
        // Від неї і шукаємо, де саме має бути скид.
        double attackStartX = intermediateX;
        double attackStartY = intermediateY;
        double attackDx = targetX - attackStartX;
        double attackDy = targetY - attackStartY;
        double attackDistance = std::sqrt(attackDx * attackDx + attackDy * attackDy);
        if (attackDistance <= EPS) {
            std::cout << "Error: invalid attack start distance\n";
            fout << "Case " << caseNumber << ": Error: invalid attack start distance\n";
            freport << "Case " << caseNumber << ": Error: invalid attack start distance\n";
            return 1;
        }

        // `ratio` показує частку шляху від старту атаки до точки скиду.
        double ratio = 0.0;
        double fireX = attackStartX;
        double fireY = attackStartY;

        ratio = (attackDistance - h) / attackDistance;
        fireX = attackStartX + attackDx * ratio;
        fireY = attackStartY + attackDy * ratio;

        std::cout << "Ratio: " << ratio << '\n';
        std::cout << "Drop point: x=" << fireX << ", y=" << fireY << '\n';
        std::cout << "Case " << caseNumber << " completed\n";

        // Зберігаємо все, що нарахували, шоб потім намалювати і не рахувати по новій.
        CaseResult result;
        result.caseNumber = caseNumber;
        result.xd = xd;
        result.yd = yd;
        result.zd = zd;
        result.targetX = targetX;
        result.targetY = targetY;
        result.attackSpeed = attackSpeed;
        result.accelerationPath = accelerationPath;
        result.ammoName = ammo_name;
        result.ammo = ammo;
        result.timeOfFlight = t;
        result.horizontalDistance = h;
        result.distanceToTarget = D;
        result.needManeuver = needManeuver;
        result.intermediateX = intermediateX;
        result.intermediateY = intermediateY;
        result.ratio = ratio;
        result.fireX = fireX;
        result.fireY = fireY;
        result.projectilePath = buildProjectilePath(fireX, fireY, targetX, targetY,
                                                    zd, t, h, ammo, attackSpeed);
        results.push_back(result);

        // В `output.txt` пишемо тільки відповідь по суті.
        fout << fireX << ' ' << fireY << '\n';

        // А в `report.txt` вже складаємо весь детальний розклад.
        freport << "Case " << caseNumber << '\n';
        freport << "ammoName = " << ammo_name << '\n';
        freport << "timeOfFlight = " << t << '\n';
        freport << "horizontalDistance = " << h << '\n';
        freport << "distanceToTarget = " << D << '\n';
        freport << "needManeuver = " << (needManeuver ? "YES" : "NO") << '\n';
        freport << "intermediateX = " << intermediateX << '\n';
        freport << "intermediateY = " << intermediateY << '\n';
        freport << "ratio = " << ratio << '\n';
        freport << "fireX = " << fireX << '\n';
        freport << "fireY = " << fireY << '\n';
        freport << '\n';
    }

    if (caseNumber == 0) {
        std::cout << "Error: input file is empty or invalid\n";
        fout << "Error: input file is empty or invalid\n";
        freport << "Error: input file is empty or invalid\n";
        return 1;
    }

    // В кінці добиваємо візуалізацію, раз уже всі кейси є.
    std::cout << "\nAll cases processed successfully\n";
    std::cout << "Results written to output.txt\n";
    std::cout << "Detailed report written to report.txt\n";
    writeProjectileTrajectoryFile(results);
    writeTrajectorySvg(results);

    return 0;
}

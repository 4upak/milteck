# Homework 07: ООП-рефакторинг ДЗ3 (інтерфейси, фабрика, стратегія)

Це підпроєкт за матеріалами Заняття 13–14: рефактор монолітного `dz3/main.cpp`
у набір незалежних компонентів з чіткими інтерфейсами. Використовуються три
патерни з лекції: **Стратегія** (різні солвери підмінюються в runtime),
**Фабрика** (створення реалізацій за `enum class`), і базова **Інкапсуляція /
Поліморфізм** (інтерфейси з `=0` віртуальними методами).

## Що робить програма

`homework_07_cli` отримує теку з конфігом місії, читає `config.json` +
`ammo.json` + `targets.json` і для кожної цілі друкує точку скиду боєприпасу,
обчислену балістичним солвером. Той самий цикл `while (mission.hasNext())`
працював би з будь-яким іншим солвером або провайдером цілей — потрібно лише
додати ще один `case` у фабрику.

## Структура

```text
homework_07/
  CMakeLists.txt
  README.md
  include/
    types.hpp                 # Coord, Target, AmmoParams, MissionConfig, DropPoint, Homework07Error
    i_target_provider.hpp     # інтерфейс ITargetProvider
    i_ballistic_solver.hpp    # інтерфейс IBallisticSolver
    i_config_loader.hpp       # інтерфейс IConfigLoader
    json_target_provider.hpp  # реалізація ITargetProvider
    analytical_solver.hpp     # реалізація IBallisticSolver
    file_config_loader.hpp    # реалізація IConfigLoader
    factory.hpp               # createSolver / createProvider / createLoader
    mission_processor.hpp     # Strategy: init/hasNext/step/reset/changeSolver
  src/
    analytical_solver.cpp
    json_target_provider.cpp
    file_config_loader.cpp
    factory.cpp
    mission_processor.cpp
    main.cpp
  tests/
    homework_07_tests.cpp
  data/
    config.json
    ammo.json
    targets.json
    invalid_ammo_config.json  # для негативного тест-кейсу
  third_party/
    json.hpp                  # nlohmann/json (single header, скопійовано з dz3)
```

## Архітектура

### Три інтерфейси (`= 0` методи + `virtual ~`)

| Інтерфейс           | Методи                                                                | Що інкапсулює                  |
| ------------------- | --------------------------------------------------------------------- | ------------------------------ |
| `ITargetProvider`   | `getTargetCount()`, `getTarget(int idx)`                              | Звідки беремо список цілей     |
| `IBallisticSolver`  | `solve(dronePos, targetPos, altitude, attackSpeed, ammo)`             | Як рахуємо точку скиду         |
| `IConfigLoader`     | `load(source)`, `getConfig()`, `getAmmoParams()`                      | Звідки беремо конфіг і боєприпас |

### Реалізації

| Інтерфейс           | Реалізація              | Що робить                                          |
| ------------------- | ----------------------- | -------------------------------------------------- |
| `ITargetProvider`   | `JsonTargetProvider`    | Читає список цілей з JSON-файла                    |
| `IBallisticSolver`  | `AnalyticalSolver`      | Кардано + полінома горизонтальної дальності (ДЗ1)  |
| `IConfigLoader`     | `FileConfigLoader`      | Читає `config.json` і `ammo.json` з теки           |

### Фабрика

```cpp
enum class SolverType   { Analytical };
enum class ProviderType { Json };
enum class LoaderType   { File };

IBallisticSolver* createSolver(SolverType type);
ITargetProvider*  createProvider(ProviderType type, const std::string& param);
IConfigLoader*    createLoader(LoaderType type);
```

Фабрика повертає сирий вказівник на інтерфейс. Викликаючий код володіє
об'єктом і робить `delete` сам — це навмисне дотримання шаблону з лекції 14
(перед тим як перейти на `std::unique_ptr` у наступних заняттях).

### MissionProcessor (Стратегія)

```cpp
class MissionProcessor {
public:
  MissionProcessor(IConfigLoader* loader,
                   ITargetProvider* targets,
                   IBallisticSolver* solver);

  void init(const std::string& configSource);
  bool hasNext() const;
  DropPoint step();
  void reset();
  void changeSolver(IBallisticSolver* solver);
};
```

`MissionProcessor` тримає три невласницькі (non-owning) вказівники на
інтерфейси. Він не знає, чи це JSON-провайдер чи тестовий мок, чи це
аналітичний солвер чи табличний. `changeSolver` дозволяє підмінити алгоритм
посеред обробки списку — це і є патерн Стратегія в дії, і тест
`MissionProcessor.ChangeSolverSwapsStrategyOnTheFly` це безпосередньо
перевіряє через `CountingSolver` mock-реалізацію `IBallisticSolver`.

## Формати файлів даних

`config.json`:

```json
{
  "drone": {
    "position": { "x": 0.0, "y": 0.0 },
    "altitude": 120.0,
    "attackSpeed": 30.0
  },
  "ammo": "VOG-17"
}
```

`ammo.json` — масив рядків боєприпасів з полями `name`, `mass`, `drag`, `lift`.

`targets.json`:

```json
{
  "targets": [
    {
      "name": "Tank-1",
      "pos":      { "x": 200.0, "y": 150.0 },
      "velocity": { "x":   2.0, "y":   0.0 }
    }
  ]
}
```

`velocity` зараз зберігається у структурі `Target` для майбутньої логіки
прогнозування руху цілі (як у `dz3`), але `AnalyticalSolver` на цьому занятті
її не використовує — солвер бачить лише статичну точку цілі.

## Збірка

З кореня репозиторію:

```bash
cmake --preset debug
cmake --build --preset debug
```

Результати:

- `build/debug/homework_07/homework_07_cli`
- `build/debug/homework_07/homework_07_tests`

## Запуск

```bash
./build/debug/homework_07/homework_07_cli homework_07/data
```

Очікуваний вивід:

```text
mission start drone=(0.000, 0.000) altitude=120.000 ammo=VOG-17 targets=5
target 0 'Tank-1' pos=(200.000, 150.000) drop=(99.934, 74.951) tof=6.616 range=125.082
target 1 'APC-2' pos=(350.000, -80.000) drop=(228.063, -52.129) tof=6.616 range=125.082
target 2 'Pickup-3' pos=(90.000, 220.000) drop=(42.640, 104.231) tof=6.616 range=125.082
target 3 'Bunker-4' pos=(500.000, 500.000) drop=(411.554, 411.554) tof=6.616 range=125.082
target 4 'Howitzer-5' pos=(280.000, 410.000) drop=(209.458, 306.707) tof=6.616 range=125.082
```

Альтернативний шлях до файла з цілями:

```bash
./build/debug/homework_07/homework_07_cli homework_07/data path/to/other_targets.json
```

## Тести

```bash
ctest --test-dir build/debug --output-on-failure -R "AnalyticalSolver|JsonTargetProvider|FileConfigLoader|Factory|MissionProcessor"
```

Покриті сценарії:

- `AnalyticalSolver` повертає відоме `(fire_x, fire_y) ≈ (173.759, 173.759)`
  для референсного входу VOG-17.
- `AnalyticalSolver` кидає `Homework07Error` на нульовій висоті та на збіжній
  позиції дрона й цілі.
- `JsonTargetProvider` читає 5 цілей зі sample-файла і коректно обробляє
  відсутній файл та індекс поза межами.
- `FileConfigLoader` завантажує конфіг + резолвить параметри `VOG-17`,
  кидає до `load()` й на невідомому боєприпасі.
- Фабрика створює всі три типи без помилок.
- `MissionProcessor` ітерує всі цілі через `hasNext()`/`step()`, потім
  кидає на наступний `step()`; `reset()` повертає курсор на 0;
  `changeSolver()` підмінює стратегію на льоту (перевірено mock-солвером).
- `MissionProcessor` валідує не-`nullptr` залежності у конструкторі.

## Quality

З кореня репозиторію:

```bash
make format   # clang-format / cmake-format на змінених файлах homework_07
make lint     # clang-tidy на src/ і tests/ homework_07
make test     # configure + build + ctest
make quality  # format + test + lint
```

## Обмеження / нотатки

- `delete` у `main()` і фабриках з raw pointer — навмисний навчальний крок з
  лекції 14. У продакшені тут жив би `std::unique_ptr<IBallisticSolver>` etc.
- `Target.velocity` зчитується з файла, але `AnalyticalSolver` його не
  використовує. Це місце для майбутнього `PredictiveSolver` без зміни
  `MissionProcessor`.
- Додавання нової реалізації (наприклад `TableSolver`) зведеться до:
  1. новий клас у `include/` + `src/`,
  2. одного `case` у `factory.cpp`,
  3. одного `enum` value у `factory.hpp`.
  Жоден існуючий клас не змінюється — це і є мета поліморфізму.

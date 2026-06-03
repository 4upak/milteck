# Homework 08: STL-рефакторинг homework_07 + чистий проектний layout

Це підпроєкт за матеріалами Заняття 16 (3.4) — "Вступ до STL". Узяли
готовий ООП-рефакторинг із [homework_07](../homework_07/README.md) і
зробили дві речі:

1. **Привели структуру файлів до ладу** — розклали `.h`-заголовки і `.cpp`
   реалізації по тематичних підпапках (`interfaces/`, `solvers/`,
   `providers/`, `config/`).
2. **Замінили C-стиль конструкції на STL-контейнери** там, де це має сенс:
   `std::unordered_map` для пошуку за назвою боєприпасу, `std::array` для
   фіксованих наборів значень (корені кубічного рівняння Кардано), плюс
   range-based for замість індексних циклів.

Поведінка CLI та контракт Strategy/Factory залишилися ідентичними з
`homework_07` — той самий вивід на тих самих даних.

## Структура

```text
homework_08/
├── CMakeLists.txt
├── README.md
├── data/
│   ├── config.json
│   ├── ammo.json
│   ├── targets.json
│   └── invalid_ammo_config.json     # для негативного тест-кейсу
├── include/
│   ├── Types.h                       # Coord, Target, AmmoParams, MissionConfig, DropPoint, Homework08Error
│   ├── MissionProcessor.h            # Strategy: init/hasNext/step/reset/changeSolver
│   ├── interfaces/
│   │   ├── ITargetProvider.h
│   │   ├── IBallisticSolver.h
│   │   └── IConfigLoader.h
│   ├── solvers/
│   │   └── AnalyticalSolver.h
│   ├── providers/
│   │   └── JsonTargetProvider.h
│   └── config/
│       ├── FileConfigLoader.h
│       └── ComponentFactory.h
├── src/
│   ├── main.cpp                      # CLI entry point
│   ├── MissionProcessor.cpp
│   ├── solvers/
│   │   └── AnalyticalSolver.cpp
│   ├── providers/
│   │   └── JsonTargetProvider.cpp
│   └── config/
│       ├── FileConfigLoader.cpp
│       └── ComponentFactory.cpp
├── tests/
│   └── homework_08_tests.cpp
└── third_party/
    └── json.hpp                      # nlohmann/json (single header)
```

Усі `.h`-файли мають `#pragma once`. Залежності зведені до мінімуму:
заголовки тягнуть тільки те, що їм безпосередньо потрібно (`Types.h` плюс
відповідний інтерфейс), важкі залежності типу `<fstream>`, `<filesystem>`,
`json.hpp` живуть лише в `.cpp`.

## STL-рефакторинг — що саме змінилося

### 1. `std::unordered_map<std::string, AmmoParams>` у `FileConfigLoader`

**Було (homework_07/src/file_config_loader.cpp):** `find_ammo()` лінійно
проходила масивом боєприпасів, порівнюючи `name` з шуканим — `O(n)` на
кожне завантаження.

**Стало (homework_08/src/config/FileConfigLoader.cpp,
`load_ammo_table()`):** усі рядки з `ammo.json` потрапляють у
`std::unordered_map<std::string, AmmoParams>`, після чого вибір боєприпасу
за `config.ammo_name` робиться за `O(1)`. Додатково геттер `ammoTable()`
дає змогу клієнтському коду подивитися всі завантажені варіанти без
повторного читання файла. Це канонічний приклад з лекції — заміна
лінійного перебору / `if/else`-ланцюжка на хеш-таблицю.

### 2. `std::array<double, 3>` у `AnalyticalSolver`

**Було:** три корені методу Кардано тримались у трьох іменованих змінних
`root_1`, `root_2`, `root_3` із трьома послідовними `if`-ами для вибору
найменшого додатного.

**Стало:** `std::array<double, 3> roots{...}` плюс `for (const double
candidate : roots) { ... }` — фіксований розмір відомий на компіляції,
нуль overhead, дані на стеку, природний range-based for.

### 3. `std::vector<Target>` + `std::unordered_map<std::string, std::size_t>` у `JsonTargetProvider`

`std::vector<Target>` для впорядкованого доступу був і раніше. Додалось:

- `targets()` — повертає `const std::vector<Target>&` напряму, що дає
  змогу будь-якому викликачу (CLI, тестам, `MissionProcessor`) ходити по
  списку через range-based for без `getTargetCount() + getTarget(idx)`.
- `findByName(std::string)` — `O(1)` пошук цілі за назвою через
  `std::unordered_map<std::string, std::size_t>` (мапа тримає індекс
  у векторі, щоб не дублювати дані).

### 4. Range-based for замість індексних циклів

Усі цикли виду `for (std::size_t i = 0; i < items.size(); ++i)` у
парсингу JSON (`JsonTargetProvider`, `load_ammo_table`) переписано на
`for (const auto& entry : items)`. У `main.cpp` зовнішній цикл по цілях
теж використовує range-based for над `provider->targets()`.

### 5. Послідовне використання `std::string` / `std::filesystem::path`

Уся робота з шляхами — через `std::filesystem::path` (`base / "config.json"`
замість конкатенації рядків), уся робота з текстом — через `std::string`,
без `char[]` чи `strcmp`. CLI вивід — через `'\n'` (не `std::endl`) для
відповідності SL.io.50.

## Що **не** замінювали і чому

- Фабрика `createSolver(SolverType)` залишилась на `switch` по `enum class`,
  а не `std::map<SolverType, std::function<...>>`. Для замкненого набору
  enum-значень `switch` дає компайл-тайм перевірку повноти і не потребує
  динамічної пам'яті на саму таблицю. Map тут був би доречний для
  **runtime-реєстрації** (наприклад плагін-механізму), що зараз не
  потрібно.
- `std::list` навмисно не використовується — лекція прямо каже, що на
  практиці `std::vector` майже завжди швидший за рахунок cache locality,
  а вставки посередині нам тут не потрібні.

## Архітектура

Три інтерфейси (`ITargetProvider`, `IBallisticSolver`, `IConfigLoader`) +
три реалізації (`JsonTargetProvider`, `AnalyticalSolver`,
`FileConfigLoader`) + фабрика по `enum class` + Strategy-процесор
(`MissionProcessor`) — той самий розклад, що і в `homework_07`. Тести
покривають як збережені поведінки (`MissionProcessor` ітерує всі цілі,
кидає на надлишкових `step()`, `changeSolver()` підмінює стратегію на
льоту), так і нові STL-точки:

- `JsonTargetProvider.ExposesTargetsAsVector`
- `JsonTargetProvider.RangeBasedForVisitsAllTargets`
- `JsonTargetProvider.FindByNameUsesHashTable`
- `FileConfigLoader.AmmoTableContainsAllEntries`

## Збірка

З кореня репозиторію:

```bash
cmake --preset debug
cmake --build --preset debug
```

Результати:

- `build/debug/homework_08/homework_08_cli`
- `build/debug/homework_08/homework_08_tests`

## Запуск

```bash
./build/debug/homework_08/homework_08_cli homework_08/data
```

Очікуваний вивід (ідентичний `homework_07` — зміна внутрішніх контейнерів
поведінки не змінила):

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
./build/debug/homework_08/homework_08_cli homework_08/data path/to/other_targets.json
```

## Тести

```bash
./build/debug/homework_08/homework_08_tests
```

або через ctest (там тести з `homework_07` і `homework_08` ділять
suite-імена, тому простіше прогнати один бінарник напряму):

```bash
ctest --test-dir build/debug --output-on-failure
```

## Quality

З кореня репозиторію:

```bash
make format   # clang-format / cmake-format на змінених файлах homework_08
make lint     # clang-tidy на src/ і tests/ homework_08
make test     # configure + build + ctest
make quality  # format + test + lint
```

## Обмеження / нотатки

- Власність створених фабрикою об'єктів і досі тримається на raw pointer
  + `delete` (як у `homework_07`) — це навмисний навчальний крок з лекції 14.
- `Target.velocity` зчитується, але `AnalyticalSolver` його не використовує —
  залишок під майбутній `PredictiveSolver`.
- Для нової реалізації (наприклад `TableSolver` чи
  `UnorderedSetTargetProvider`) достатньо: новий клас у `include/<group>/`
  та `src/<group>/`, один `case` у `ComponentFactory.cpp`, один enum value
  у `ComponentFactory.h`. Жоден існуючий клас не змінюється.

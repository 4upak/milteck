# Homework 09: smart pointers, State pattern, table ballistic solver

`homework_09` продовжує `homework_08` і закриває три незалежні вимоги ДЗ9:

1. перехід з owning raw pointers на `std::unique_ptr`;
2. заміна drone state-machine `switch/case` на класи станів;
3. додавання табличного балістичного solver-а з 5D linear interpolation.

## Структура

```text
homework_09/
├── CMakeLists.txt
├── README.md
├── data/
│   ├── config.json
│   ├── ammo.json                 # оновлені m/d/l з ДЗ9
│   ├── targets.json
│   ├── invalid_ammo_config.json
│   └── ballistic_table.txt       # 5D table для TableSolver
├── include/
│   ├── Types.h
│   ├── MissionProcessor.h
│   ├── config/ComponentFactory.h
│   ├── drone/DroneState.h
│   ├── interfaces/*.h
│   ├── providers/JsonTargetProvider.h
│   └── solvers/
│       ├── AnalyticalSolver.h
│       └── TableSolver.h
├── src/
│   ├── main.cpp
│   ├── MissionProcessor.cpp
│   ├── config/*.cpp
│   ├── drone/DroneState.cpp
│   ├── providers/JsonTargetProvider.cpp
│   └── solvers/
│       ├── AnalyticalSolver.cpp
│       └── TableSolver.cpp
└── tests/homework_09_tests.cpp
```

## 1. Smart pointers

Фабрика більше не повертає owning raw pointer:

```cpp
std::unique_ptr<IBallisticSolver> createSolver(SolverType type);
std::unique_ptr<ITargetProvider> createProvider(ProviderType type, const std::string& param);
std::unique_ptr<IConfigLoader> createLoader(LoaderType type);
```

`MissionProcessor` володіє компонентами:

```cpp
std::unique_ptr<IConfigLoader> loader_;
std::unique_ptr<ITargetProvider> targets_;
std::unique_ptr<IBallisticSolver> solver_;
```

Заміна стратегії також передає власність:

```cpp
void changeSolver(std::unique_ptr<IBallisticSolver> solver);
```

Ручні `delete` з `homework_08` прибрані.

## 2. State pattern для дрона

Додано модуль `include/drone/DroneState.h` + `src/drone/DroneState.cpp`.

Базовий інтерфейс:

```cpp
class IDroneState {
public:
  virtual ~IDroneState() = default;
  virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;
  virtual const char* name() const = 0;
};
```

Окремі класи станів:

- `StateStopped`
- `StateAccelerating`
- `StateDecelerating`
- `StateTurning`
- `StateMoving`

Спільні дані живуть у `DroneContext`. Переходи між станами виконуються через
`return std::make_unique<NextState>()`. У `homework_09` немає `DroneState enum`
і немає state-machine `switch/case`.

## 3. TableSolver

Додано `TableSolver`, який реалізує `IBallisticSolver`.

Файл таблиці: `data/ballistic_table.txt`.

Формат:

```text
nZ nV nM nD nL
axisZ0...
axisV0...
axisM...
axisD...
axisL...
t hDist
...
```

Порядок даних: `Z0 -> V0 -> m -> d -> l` від зовнішньої осі до внутрішньої.

`BallisticTable::lookup()`:

- знаходить нижній індекс і `frac` для кожної з 5 осей через `lower_bound`;
- clamp-ить значення поза межами таблиці до найближчого краю;
- бере 32 вершини 5D гіперкуба;
- згортає вкладеними `lerp`: `32 -> 16 -> 8 -> 4 -> 2 -> 1`;
- повертає `time_of_flight` і `horizontal_distance`.

`TableSolver::solve()` використовує ці два значення і рахує `DropPoint` так само,
як `AnalyticalSolver`: уздовж напрямку від дрона до цілі.

## Оновлені параметри ammo

```text
VOG-17       m=0.35  d=0.004  l=0
M67          m=0.60  d=0.005  l=0
RKG-3        m=1.20  d=0.007  l=0
GLIDING-VOG  m=0.45  d=0.005  l=0.005
GLIDING-RKG  m=1.40  d=0.007  l=0.005
```

## Збірка

З кореня репозиторію:

```bash
cmake --preset debug
cmake --build --preset debug --target homework_09_cli homework_09_tests
```

або всі тести репозиторію:

```bash
make test
```

## Запуск

Аналітичний solver:

```bash
./build/debug/homework_09/homework_09_cli homework_09/data
```

Табличний solver з таблицею за замовчуванням:

```bash
./build/debug/homework_09/homework_09_cli homework_09/data homework_09/data/targets.json table
```

Табличний solver з явним файлом:

```bash
./build/debug/homework_09/homework_09_cli \
  homework_09/data \
  homework_09/data/targets.json \
  table \
  homework_09/data/ballistic_table.txt
```

## Тести

```bash
./build/debug/homework_09/homework_09_tests
```

Покрито:

- `unique_ptr` фабрики і ownership у `MissionProcessor`;
- `changeSolver(std::unique_ptr<...>)`;
- оновлені ammo parameters;
- 5D interpolation і clamp у `BallisticTable`;
- `TableSolver` geometry;
- State transitions для `Stopped`, `Turning`, `Accelerating`, `Moving`, `Decelerating`.

# Homework 10: multithreaded mission simulation

`homework_10` продовжує `homework_09` і розносить симуляцію на три окремі потоки:

1. `ThreadSafeTargetProvider` — рухає цілі вздовж траєкторій;
2. `DronePhysics` — інтегрує фізику дрона і виконує команди;
3. `MissionProcessor` — планує місію, рахує балістику і пише `simulation.json`.

## Основні вимоги ДЗ10

- спільні дані захищені через `std::mutex` або `std::atomic`;
- стоп-прапорці — `std::atomic<bool>`;
- потоків через `detach()` немає, завершення йде через `join()`;
- команди фізиці передаються через власний шаблон `ThreadSafeQueue<T>`;
- цілі стартують тільки після `start()`, коли `isThreadReady()` вже true;
- `MissionProcessor` не інтегрує стан дрона, а читає `DroneTelemetry` з `DronePhysics`;
- `Target` назовні містить тільки `pos`, `velocity`, `name`; траєкторії приватні для provider-а;
- лог місії містить `timeSecSinceStart`, щоб checker міг враховувати нерівномірні real-time кроки.

## Структура

```text
homework_10/
├── CMakeLists.txt
├── README.md
├── data/
│   ├── config.json
│   ├── ammo.json
│   ├── targets.json
│   └── ballistic_table.txt
├── include/
│   ├── Types.h
│   ├── MissionProcessor.h
│   ├── config/ComponentFactory.h
│   ├── drone/DronePhysics.h
│   ├── drone/DroneState.h
│   ├── interfaces/*.h
│   ├── providers/ThreadSafeTargetProvider.h
│   ├── solvers/*.h
│   └── utils/ThreadSafeQueue.h
├── src/
│   ├── main.cpp
│   ├── MissionProcessor.cpp
│   ├── config/*.cpp
│   ├── drone/DronePhysics.cpp
│   ├── drone/DroneState.cpp
│   ├── providers/ThreadSafeTargetProvider.cpp
│   └── solvers/*.cpp
└── tests/homework_10_tests.cpp
```

## Конфігурація

`data/config.json` має секцію `simulation`:

```json
"simulation": {
  "simTimeStep": 0.05,
  "arrayTimeStep": 0.05,
  "targetTimeStep": 0.05,
  "physicsTimeStep": 0.01,
  "timeScale": 10.0,
  "maxMissionTime": 1.0
}
```

Якщо параметра немає, `FileConfigLoader` використовує дефолт.

## Targets

`ThreadSafeTargetProvider` підтримує формат з траєкторією:

```json
{
  "name": "Tank-1",
  "trajectory": [
    { "x": 200.0, "y": 150.0 },
    { "x": 200.1, "y": 150.0 }
  ]
}
```

Для сумісності також підтримується старий формат `pos + velocity`: з нього provider будує коротку зациклену траєкторію.

## Запуск потоків

`main()` створює три об'єкти і три `std::thread`, чекає готовності, потім дає старт:

```cpp
std::thread providerThread(&ThreadSafeTargetProvider::run, &provider);
std::thread physicsThread(&DronePhysics::run, &physics);
std::thread missionThread(&MissionProcessor::run, &mission);

while (!provider.isThreadReady() || !physics.isThreadReady() || !mission.isThreadReady()) {
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

provider.start();
physics.start();
mission.start();

missionThread.join();
physics.stop();
provider.stop();
physicsThread.join();
providerThread.join();
```

## Збірка

```bash
cmake --preset debug
cmake --build --preset debug --target homework_10_cli homework_10_tests
```

## Тести

```bash
./build/debug/homework_10/homework_10_tests
```

Покрито:

- FIFO-поведінку `ThreadSafeQueue`;
- читання нових simulation-параметрів;
- snapshot-и та рух `ThreadSafeTargetProvider` після `start()`;
- виконання команд у `DronePhysics`;
- запуск `MissionProcessor` у власному потоці та запис `simulation.json`.

## CLI

```bash
./build/debug/homework_10/homework_10_cli \
  homework_10/data \
  homework_10/data/targets.json \
  analytical \
  homework_10/data/ballistic_table.txt \
  simulation.json
```

Вихідний файл має формат:

```json
{
  "steps": [
    {
      "position": { "x": 0.0, "y": 0.0 },
      "direction": 0.0,
      "state": "Stopped",
      "targetIndex": 0,
      "dropPoint": { "x": 91.5, "y": 65.3 },
      "aimPoint": { "x": 91.5, "y": 65.3 },
      "predictedTarget": { "x": 210.0, "y": 150.0 },
      "timeSecSinceStart": 0.0
    }
  ]
}
```

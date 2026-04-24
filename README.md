# milteck

## DZ3: запуск симуляції та візуалізації

`dz3` складається з двох частин:

1. `drone_sim` запускає симуляцію та генерує JSON-файли з результатами.
2. `viewer` читає ці файли й показує політ дрона та траєкторії боєприпасів.

## Які файли використовуються

Вхідні файли в `dz3`:

- `config.json` - параметри дрона та симуляції
- `ammo.json` - список типів боєприпасів
- `targets.json` - траєкторії цілей у часі

Вихідні файли після запуску симуляції:

- `simulation_<ammo>.json` - результат симуляції для кожного типу боєприпасу
- `projectile_<ammo>.json` - траєкторія скинутого боєприпасу для кожного типу

Наприклад: `simulation_vog_17.json`, `simulation_gliding_vog.json`, `projectile_rkg_3.json`.

## Швидкий запуск

Найпростіший спосіб запустити `dz3`:

```bash
cd dz3
./run_all.sh
```

Скрипт автоматично:

- збирає `drone_sim`
- видаляє старі згенеровані `simulation_*.json` і `projectile_*.json`
- запускає симуляцію для всіх типів боєприпасів з `ammo.json`
- налаштовує та збирає `viewer`
- запускає `viewer`

Якщо потрібно лише перескласти й запустити симуляцію без відкриття вікна візуалізації:

```bash
cd dz3
DRONE_SKIP_VIEWER=1 ./run_all.sh
```

## Ручний запуск

### macOS

Для `viewer` потрібен `raylib`. Зазвичай встановлення виглядає так:

```bash
brew install raylib
```

Далі запуск по кроках:

```bash
cd dz3
make
./drone_sim
cmake -S viewer -B viewer/build
cmake --build viewer/build
./viewer/build/drone_viewer .
```

### Linux

Потрібні:

- C++ compiler
- `cmake`
- `raylib`

Наприклад, на Ubuntu/Debian:

```bash
sudo apt update
sudo apt install build-essential cmake libraylib-dev
```

Після встановлення залежностей:

```bash
cd dz3
make
./drone_sim
cmake -S viewer -B viewer/build
cmake --build viewer/build
./viewer/build/drone_viewer .
```

### Windows

Підійде зв'язка Visual Studio + CMake.

Потрібно встановити:

- Visual Studio з workload `Desktop development with C++`
- `CMake`
- `raylib`

Далі можна:

1. Зібрати симулятор у `dz3`.
2. Відкрити `dz3/viewer` як CMake-проєкт.
3. Зібрати `drone_viewer`.
4. Запустити `drone_viewer`, передавши йому шлях до папки `dz3` як аргумент.

## Як працює viewer

`viewer` читає:

- `targets.json`
- усі файли виду `simulation_*.json`
- відповідні файли `projectile_*.json`

Тобто після запуску `drone_sim` візуалізатор автоматично підхоплює результати для всіх боєприпасів, які є в `ammo.json`.

## Керування viewer

- `Space` - play/pause
- `Left` / `Right` - крок назад / вперед
- `Shift + Left` / `Shift + Right` - швидке перемотування
- `Up` / `Down` - змінити швидкість відтворення
- `0` - повернути швидкість `x1`
- `Home` / `End` - початок / кінець
- `RMB drag` - обертання камери
- `MMB drag` - панорамування
- `Wheel` - zoom
- `Q` / `E` - повернути камеру вліво / вправо
- `Z` / `C` - нахилити камеру вниз / вгору
- `P` - перспективний вид
- `T` - вид зверху

## Важливо

- `viewer` не запускає симуляцію й не генерує JSON сам
- якщо ви змінили `config.json`, `ammo.json` або `targets.json`, спочатку знову запустіть `drone_sim`
- поточна версія `dz3` створює окремі файли симуляції та траєкторії для кожного типу боєприпасу, а не один спільний `simulation.json`

# milteck

## DZ3: запуск симуляции и визуализации

`dz3` состоит из двух частей:

1. `drone_sim` запускает симуляцию и генерирует JSON-файлы с результатами.
2. `viewer` читает эти файлы и показывает полет дрона и траектории боеприпасов.

## Какие файлы используются

Входные файлы в `dz3`:

- `config.json` - параметры дрона и симуляции
- `ammo.json` - список типов боеприпасов
- `targets.json` - траектории целей по времени

Выходные файлы после запуска симуляции:

- `simulation_<ammo>.json` - результат симуляции для каждого типа боеприпаса
- `projectile_<ammo>.json` - траектория сброшенного боеприпаса для каждого типа

Например: `simulation_vog_17.json`, `simulation_gliding_vog.json`, `projectile_rkg_3.json`.

## Быстрый запуск

Самый простой способ запустить `dz3`:

```bash
cd dz3
./run_all.sh
```

Скрипт автоматически:

- собирает `drone_sim`
- удаляет старые сгенерированные `simulation_*.json` и `projectile_*.json`
- запускает симуляцию для всех типов боеприпасов из `ammo.json`
- настраивает и собирает `viewer`
- запускает `viewer`

Если нужно только пересобрать и прогнать симуляцию без открытия окна визуализации:

```bash
cd dz3
DRONE_SKIP_VIEWER=1 ./run_all.sh
```

## Ручной запуск

### macOS

Для `viewer` нужен `raylib`. Обычно установка выглядит так:

```bash
brew install raylib
```

Дальше запуск по шагам:

```bash
cd dz3
make
./drone_sim
cmake -S viewer -B viewer/build
cmake --build viewer/build
./viewer/build/drone_viewer .
```

### Linux

Нужны:

- C++ compiler
- `cmake`
- `raylib`

Например, на Ubuntu/Debian:

```bash
sudo apt update
sudo apt install build-essential cmake libraylib-dev
```

После установки зависимостей:

```bash
cd dz3
make
./drone_sim
cmake -S viewer -B viewer/build
cmake --build viewer/build
./viewer/build/drone_viewer .
```

### Windows

Подойдет связка Visual Studio + CMake.

Нужно установить:

- Visual Studio с workload `Desktop development with C++`
- `CMake`
- `raylib`

Дальше можно:

1. Собрать симулятор в `dz3`.
2. Открыть `dz3/viewer` как CMake-проект.
3. Собрать `drone_viewer`.
4. Запустить `drone_viewer`, передав ему путь к папке `dz3` как аргумент.

## Как работает viewer

`viewer` читает:

- `targets.json`
- все файлы вида `simulation_*.json`
- соответствующие файлы `projectile_*.json`

То есть после запуска `drone_sim` визуализатор автоматически подхватывает результаты для всех боеприпасов, которые есть в `ammo.json`.

## Управление viewer

- `Space` - play/pause
- `Left` / `Right` - шаг назад / вперед
- `Shift + Left` / `Shift + Right` - быстрая перемотка
- `Up` / `Down` - изменить скорость воспроизведения
- `0` - вернуть скорость `x1`
- `Home` / `End` - начало / конец
- `RMB drag` - вращение камеры
- `MMB drag` - панорамирование
- `Wheel` - zoom
- `Q` / `E` - повернуть камеру влево / вправо
- `Z` / `C` - наклонить камеру вниз / вверх
- `P` - перспективный вид
- `T` - вид сверху

## Важно

- `viewer` не запускает симуляцию и не генерирует JSON сам
- если вы изменили `config.json`, `ammo.json` или `targets.json`, сначала снова запустите `drone_sim`
- текущая версия `dz3` создает отдельные файлы симуляции и траектории для каждого типа боеприпаса, а не один общий `simulation.json`

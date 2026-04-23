# milteck
DZ3: запуск симуляції та візуалізації

Структура вхідних/вихідних файлів
- config.json      - параметри дрона та симуляції
- ammo.json        - список боєприпасів
- targets.json     - координати цілей у часі
- simulation.json  - результат роботи симуляції

Порядок запуску
1. Спочатку запустити симуляцію (`main.cpp` / `drone_sim`) - вона створює `simulation.json`
2. Потім запустити viewer - він читає `config.json`, `ammo.json`, `targets.json`, `simulation.json`


====================
macOS
====================

1. Збирання симуляції:
cd dz3
make

2. Запуск симуляції:
./drone_sim

3. Збирання viewer:
cd viewer
mkdir -p build
cd build
cmake ..
cmake --build .

4. Запуск viewer:
./drone_viewer ../..

Примітка:
- Для viewer має бути встановлений `raylib`
- На macOS це зазвичай робиться через Homebrew:
  brew install raylib


====================
Linux
====================

1. Встановити залежності:
- компілятор C++
- CMake
- raylib

Наприклад, на Ubuntu/Debian:
sudo apt update
sudo apt install build-essential cmake libraylib-dev

2. Збирання симуляції:
cd dz3
make

3. Запуск симуляції:
./drone_sim

4. Збирання viewer:
cd viewer
mkdir -p build
cd build
cmake ..
cmake --build .

5. Запуск viewer:
./drone_viewer ../..


====================
Windows
====================

Visual Studio + CMake

1. Встановити:
- Visual Studio з Desktop development with C++
- CMake
- raylib

2. Симуляцію можна зібрати так:
cd dz3
cl /EHsc /std:c++17 main.cpp

або відкрити проєкт в IDE і зібрати вручну

3. Viewer:
- відкрити `dz3/viewer` у Visual Studio як CMake-проєкт
- зібрати проєкт `drone_viewer`
- запускати з аргументом:
  ..\..


====================
Керування viewer
====================

- Space - play/pause
- Left / Right - крок назад / вперед
- Shift + Left / Right - швидке перемотування
- Up / Down - змінити швидкість відтворення
- 0 - повернути швидкість x1
- Home / End - початок / кінець
- RMB drag - обертання камери
- MMB drag - панорамування
- Wheel - zoom
- Q / E - повернути камеру вліво / вправо
- Z / C - нахилити камеру вниз / вгору
- P - перспективний вид
- T - вид зверху


====================
Важливо
====================

- Viewer не створює `simulation.json`, він лише читає його
- Якщо ви змінили `config.json`, `ammo.json` або `targets.json`, спочатку знову запустіть `drone_sim`

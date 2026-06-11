# Homework 06: Ballistic Drop Solver

Цей підпроєкт оформлює задачу з ДЗ 1 як нормальний C++ workflow для ДЗ 6:
окрема доменна бібліотека, консольна програма, CMake target-и, CTest/Google
Test тести, форматування і `clang-tidy`.

## Що робить програма

`ballistics_cli` рахує точку скиду боєприпасу з дрона за такими даними:

- позиція дрона `drone_x drone_y drone_z`;
- позиція цілі `target_x target_y`;
- швидкість атаки `attack_speed`;
- дистанція розгону `acceleration_path`;
- назва боєприпасу `ammo_name`.

Фізична модель взята з попереднього ДЗ: обчислюється час падіння, горизонтальна
дальність боєприпасу, потреба у проміжній точці маневру і фінальна точка скиду.
Некоректний вхід або невідомий тип боєприпасу повертають контрольовану помилку,
а не аварійне падіння.

## Структура

```text
homework_06/
  CMakeLists.txt
  README.md
  include/
    ballistics.hpp
  src/
    ballistics.cpp
    main.cpp
  tests/
    ballistics_tests.cpp
  data/
    ammo_data.txt
    sample_vog17.txt
    unknown_ammo.txt
```

Основна логіка живе в target-і `ballistics`. Консольна програма
`ballistics_cli` тільки читає дані, викликає бібліотеку і друкує стабільний
результат. Тести напряму перевіряють функції з бібліотеки.

## Збірка

Запускати з кореня репозиторію:

```bash
cmake --preset debug
cmake --build --preset debug
```

Після цього executable буде тут:

```text
build/debug/homework_06/ballistics_cli
```

## Запуск

Базовий запуск з вбудованою таблицею боєприпасів:

```bash
./build/debug/homework_06/ballistics_cli homework_06/data/sample_vog17.txt
```

Очікуваний результат:

```text
case 1 fire_x 76.740 fire_y 34.494 time_of_flight 6.616 horizontal_distance 100.066 needs_maneuver yes
```

Запуск із зовнішньою таблицею боєприпасів:

```bash
./build/debug/homework_06/ballistics_cli \
  homework_06/data/sample_vog17.txt \
  homework_06/data/ammo_data.txt
```

Перевірка контрольованої помилки:

```bash
./build/debug/homework_06/ballistics_cli homework_06/data/unknown_ammo.txt
```

Очікувано програма завершується з ненульовим кодом і друкує:

```text
error: unknown ammo type: UNKNOWN-AMMO
```

## Формат вхідного файлу

Один сценарій містить 8 значень у такому порядку:

```text
drone_x
drone_y
drone_z
target_x
target_y
attack_speed
acceleration_path
ammo_name
```

У файлі може бути кілька сценаріїв підряд. Для кожного сценарію програма друкує
окремий рядок результату.

Приклад:

```text
100
50
120
160
90
24
15
VOG-17
```

## Формат таблиці боєприпасів

Кожен рядок має формат:

```text
ammo_name mass drag lift
```

Приклад:

```text
VOG-17 0.35 0.07 0.0
GLIDING-VOG 0.45 0.10 1.0
```

Якщо другий аргумент CLI не передано, використовується вбудована таблиця з
канонічними значеннями для `VOG-17`, `M67`, `RKG-3`, `GLIDING-VOG` і
`GLIDING-RKG`.

## Тести

Тести зареєстровані в CTest:

```bash
ctest --test-dir build/debug --output-on-failure
```

Покриті сценарії:

- еталонний приклад для `VOG-17`;
- невідомий тип боєприпасу;
- планеруючий боєприпас зі скінченним додатним часом польоту;
- нульова відстань до цілі;
- сценарій, де потрібна проміжна точка маневру.

## Quality workflow

Перед PR можна запустити одну команду з кореня репозиторію:

```bash
./scripts/check-quality
```

Вона виконує:

- `clang-format` для змінених C++ файлів;
- `cmake-format` для `homework_06/CMakeLists.txt`;
- `cmake --preset debug`;
- `cmake --build --preset debug`;
- `ctest --test-dir build/debug --output-on-failure`;
- `clang-tidy` для файлів `homework_06/src/*.cpp` і тестів.

Окремі команди також доступні через `Makefile`:

```bash
make format
make test
make lint
make quality
```

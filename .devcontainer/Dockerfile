FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        gdb \
        git \
        make \
        ninja-build \
        pkg-config \
        libasound2-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        libwayland-dev \
        libx11-dev \
        libxcursor-dev \
        libxi-dev \
        libxinerama-dev \
        libxkbcommon-dev \
        libxrandr-dev \
        xorg-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/raysan5/raylib.git /tmp/raylib \
    && cmake -S /tmp/raylib -B /tmp/raylib/build -DBUILD_SHARED_LIBS=ON \
    && cmake --build /tmp/raylib/build -j"$(nproc)" \
    && cmake --install /tmp/raylib/build \
    && rm -rf /tmp/raylib

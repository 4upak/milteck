FROM mcr.microsoft.com/devcontainers/cpp:1-ubuntu-24.04

RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        gdb \
        make \
        ninja-build \
        pkg-config \
        libraylib-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
        xorg-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

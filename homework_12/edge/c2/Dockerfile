FROM ubuntu:22.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       build-essential \
       cmake \
       ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /src
COPY CMakeLists.txt ./
COPY src/ ./src/
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --target c2_service -j"$(nproc)"

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /etc/c2 /var/log/c2

COPY --from=builder /src/build/c2_service /usr/local/bin/c2_service

HEALTHCHECK --interval=5s --timeout=2s --retries=3 \
 CMD test -f /tmp/c2_healthy || exit 1

ENTRYPOINT ["/usr/local/bin/c2_service"]

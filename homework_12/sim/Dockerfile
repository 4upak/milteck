FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends ca-certificates curl python3 python3-pip git \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir empy==3.3.4 pexpect future
WORKDIR /opt
RUN git clone --depth 1 --branch Rover-4.5 https://github.com/ArduPilot/ardupilot.git
WORKDIR /opt/ardupilot
RUN git submodule update --init --recursive --depth 1 || git submodule update --init --recursive
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

#!/usr/bin/env bash
set -euo pipefail
cd /opt/ardupilot/Tools/autotest
exec python3 sim_vehicle.py -v Rover --no-mavproxy --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551

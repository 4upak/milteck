# Homework 12 — C2/SITL Docker Compose bring-up

This solution implements the C2 service for the homework scenario:

```text
auto_stub -> c2_service -> FC
```

## Run

Start SITL:

```bash
cd homework_12
docker compose -f sim/compose.sitl.yml up -d --build
docker compose -f sim/compose.sitl.yml ps
docker exec fc_sim sh -lc 'tail -n 80 /tmp/Rover.log'
```

Start edge stack:

```bash
cd homework_12/edge
docker compose up -d --build
docker compose ps
docker compose logs -f c2_service
```

Use QGC with UDP `14550`. For the C2 scenario use **Guided + Arm** and do not press **Start Mission**.

C2 log file on host:

```bash
cat homework_12/edge/logs/c2.log
```

## Expected behavior

- `DISARMED`: block waypoints.
- `ARMED_HOLD`: block waypoints and call `fc.hold()` once on state entry.
- `ARMED_GUIDED`: forward waypoints via `fc.go_to_ned(north, east)`.
- `ARMED_MANUAL`: block waypoints and do not interfere with manual control.

Required log examples:

```text
[C2] state: DISARMED -> ARMED_GUIDED
[C2] fwd: north=50 east=0
[C2] blocked: waypoint in ARMED_MANUAL
```

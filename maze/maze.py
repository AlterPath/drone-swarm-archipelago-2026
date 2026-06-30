import os
os.environ.setdefault("DRONE_NS", "/drone_1")

import math
import time
from core import ros_bridge
from core.launch import current_drone_ns
from core.swarm import Drone
import sverk_interfaces

ALTITUDE = 0.7
DIRS = [(1, 0), (0, 1), (-1, 0), (0, -1)]


def yaw_of(t):
    y = getattr(t, "yaw", 0.0)
    return y if (y is not None and math.isfinite(y)) else 0.0


def scan(sensors, yaw):
    data = sensors.lidar.read_points_raw(frame="sensor")["points"]
    dist = [float("inf")] * 4
    n = 0
    for i in range(0, len(data), 5):
        x, y, z = data[i], data[i + 1], data[i + 2]
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if abs(z) > 0.35:
            continue
        rho = math.hypot(x, y)
        if rho < 0.08:
            continue
        n += 1
        ang = math.degrees(math.atan2(y, x) + yaw)
        for k in range(4):
            if abs((ang - 90 * k + 180) % 360 - 180) <= 30:
                dist[k] = min(dist[k], rho)
                break
    return dist, n


def reached_goal(drone):
    marker = None
    if marker is not None:
        try:
            t = drone.get_telemetry(frame_id=marker)
            return all(math.isfinite(v) for v in (t.x, t.y, t.z))
        except Exception:
            return False
    return False


def hover(drone, x, y):
    drone.navigate(x=x, y=y, z=ALTITUDE, yaw=0.0, frame_id="map", speed=0.35, wait=False)


def move(drone, sensors, k, x, y):
    drone.navigate(x=x, y=y, z=ALTITUDE, yaw=0.0, frame_id="map", speed=0.35, wait=False)
    deadline = time.time() + 10.0
    while time.time() < deadline:
        ros_bridge.publish("/swarm/cmd", {"phase": "go"})
        t = drone.get_telemetry()
        if math.hypot(t.x - x, t.y - y) < 0.12:
            return True
        d, n = scan(sensors, yaw_of(t))
        if n == 0 or d[k] < 0.3:
            hover(drone, t.x, t.y)
            time.sleep(0.3)
            return False
        time.sleep(0.1)
    t = drone.get_telemetry()
    hover(drone, t.x, t.y)
    return False


def main():
    drone = Drone(current_drone_ns())
    sensors = sverk_interfaces.init()

    drone.takeoff(z=ALTITUDE, wait=True)
    drone.set_led_effect("fill", r=0, g=200, b=255)

    base = drone.get_telemetry()
    x0, y0 = base.x, base.y

    hover(drone, x0, y0)
    settle = time.time() + 2.0
    while time.time() < settle:
        ros_bridge.publish("/swarm/cmd", {"phase": "stage"})
        time.sleep(0.1)
    for _ in range(15):
        ros_bridge.publish("/swarm/cmd", {"phase": "go"})
        time.sleep(0.05)

    visited = {(0, 0)}
    stack = [(0, 0)]
    steps = 0

    while stack and steps < 600:
        steps += 1
        if reached_goal(drone):
            break

        cur = stack[-1]
        cx, cy = x0 + cur[0] * 0.9, y0 + cur[1] * 0.9

        t = drone.get_telemetry()
        d, n = scan(sensors, yaw_of(t))
        if n == 0:
            hover(drone, cx, cy)
            time.sleep(0.3)
            continue

        cands = []
        for k in range(4):
            nb = (cur[0] + DIRS[k][0], cur[1] + DIRS[k][1])
            if nb not in visited and d[k] > 1.2:
                cands.append((k, nb))
        cands.sort(key=lambda kn: kn[1][1] - kn[1][0])

        moved = False
        for k, nb in cands:
            if move(drone, sensors, k, x0 + nb[0] * 0.9, y0 + nb[1] * 0.9):
                visited.add(nb)
                stack.append(nb)
                moved = True
                break
            visited.add(nb)
            move(drone, sensors, (k + 2) % 4, cx, cy)

        if not moved:
            stack.pop()
            if not stack:
                break
            prev = stack[-1]
            kb = DIRS.index((prev[0] - cur[0], prev[1] - cur[1]))
            move(drone, sensors, kb, x0 + prev[0] * 0.9, y0 + prev[1] * 0.9)

    t = drone.get_telemetry()
    hold = time.time() + 3.0
    while time.time() < hold:
        hover(drone, t.x, t.y)
        ros_bridge.publish("/swarm/cmd", {"phase": "go"})
        time.sleep(0.1)

    for _ in range(40):
        ros_bridge.publish("/swarm/cmd", {"phase": "land"})
        time.sleep(0.1)

    drone.land()
    drone.close()


if __name__ == "__main__":
    main()
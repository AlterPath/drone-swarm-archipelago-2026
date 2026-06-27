import math
import time
from core import ros_bridge
from core.launch import current_drone_ns
from core.swarm import Drone

ALTITUDE = 0.2
CRUISE = 1.2
TICK = 0.1
REACH_EPS = 0.12
STEP = 0.2
SWING = 0.55

CONES = [(1.0, 0.0), (2.0, 0.0)]
KEY = [
    (0.0, 0.0),
    (0.5, 0.30),
    (CONES[0][0], SWING),
    (1.5, 0.0),
    (CONES[1][0], -SWING),
    (2.6, -0.30),
    (3.4, 0.0),
    (4.4, 0.0),
    (5.2, 0.0),
]


def densify(points, step):
    out = [points[0]]
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        length = math.hypot(bx - ax, by - ay)
        n = max(1, int(round(length / step)))
        for k in range(1, n + 1):
            t = k / n
            out.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return out


def main():
    drone = Drone(current_drone_ns())
    drone.takeoff(z=ALTITUDE, wait=True)
    drone.set_led_effect("fill", r=0, g=200, b=255)

    drone.navigate(x=0.0, y=0.0, z=ALTITUDE, frame_id="map", speed=CRUISE, wait=False)
    settle = time.time() + 4.0
    while time.time() < settle:
        ros_bridge.publish("/swarm/cmd", {"phase": "stage"})
        time.sleep(0.1)

    for _ in range(15):
        ros_bridge.publish("/swarm/cmd", {"phase": "go"})
        time.sleep(0.05)

    path = densify(KEY, STEP)
    for x, y in path:
        drone.navigate(x=x, y=y, z=ALTITUDE, frame_id="map", speed=CRUISE, wait=False)
        while True:
            ros_bridge.publish("/swarm/cmd", {"phase": "go"})
            t = drone.get_telemetry()
            if math.hypot(t.x - x, t.y - y) < REACH_EPS:
                break
            time.sleep(TICK)

    fx, fy = path[-1]
    hold = time.time() + 5.0
    while time.time() < hold:
        drone.navigate(x=fx, y=fy, z=ALTITUDE, frame_id="map", speed=CRUISE, wait=False)
        ros_bridge.publish("/swarm/cmd", {"phase": "go"})
        time.sleep(TICK)

    for _ in range(40):
        ros_bridge.publish("/swarm/cmd", {"phase": "land"})
        time.sleep(0.1)

    drone.land()
    drone.close()


if __name__ == "__main__":
    main()
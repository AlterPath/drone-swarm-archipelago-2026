import math
import time
from core import ros_bridge
from core.launch import current_drone_ns
from core.swarm import Drone

ALTITUDE = 0.2
CRUISE = 1.5
TICK = 0.1
SPACING = 0.6


def predecessor(ns):
    n = int(ns.rsplit("_", 1)[1])
    return "/drone_%d" % (n - 1)


def trail_point(buf, head, spacing):
    if not buf:
        return head
    acc = 0.0
    prev = head
    for p in reversed(buf):
        acc += math.hypot(prev[0] - p[0], prev[1] - p[1])
        if acc >= spacing - 1e-6:
            return p
        prev = p
    return buf[0]


def main():
    ns = current_drone_ns()
    rank = int(ns.rsplit("_", 1)[1]) - 1
    pred_ns = predecessor(ns)
    drone = Drone(ns)

    pred = {"x": None, "y": None, "buf": []}
    phase = {"go": False, "land": False}

    def on_pred(msg):
        x, y = msg["x"], msg["y"]
        pred["x"], pred["y"] = x, y
        buf = pred["buf"]
        if not buf or math.hypot(x - buf[-1][0], y - buf[-1][1]) > 0.05:
            buf.append((x, y))
            if len(buf) > 80:
                del buf[0]

    def on_cmd(msg):
        p = msg.get("phase")
        if p == "go":
            phase["go"] = True
        elif p == "land":
            phase["land"] = True

    unsub_pred = ros_bridge.subscribe("%s/telemetry" % pred_ns, on_pred)
    unsub_cmd = ros_bridge.subscribe("/swarm/cmd", on_cmd)

    drone.takeoff(z=ALTITUDE, wait=True)
    drone.set_led_effect("fill", r=255, g=120, b=0)

    slot_x = -SPACING * rank
    while not phase["go"] and not phase["land"]:
        drone.navigate(x=slot_x, y=0.0, z=ALTITUDE, frame_id="map", speed=CRUISE, wait=False)
        time.sleep(TICK)

    if pred["x"] is not None:
        px, py = pred["x"], pred["y"]
        pred["buf"] = [(px - 0.3 * i, py) for i in range(6, 0, -1)] + [(px, py)]

    try:
        while not phase["land"]:
            if pred["x"] is not None:
                tx, ty = trail_point(pred["buf"], (pred["x"], pred["y"]), SPACING)
                drone.navigate(x=tx, y=ty, z=ALTITUDE, frame_id="map", speed=CRUISE, wait=False)
            time.sleep(TICK)
    finally:
        unsub_pred()
        unsub_cmd()
    drone.land()
    drone.close()


if __name__ == "__main__":
    main()
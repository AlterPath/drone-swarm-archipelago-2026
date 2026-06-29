"""

Этот код запускается нормально, если запускать его 
через run файла, а не всего проекта.

"""
import math

from core.swarm import Swarm
import sverk_interfaces


def line(a, b):
    k = max(1, round(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.2))
    return [(a[0] + (b[0] - a[0]) * i / k, a[1] + (b[1] - a[1]) * i / k) for i in range(1, k + 1)]


def densify(points):
    path = [points[0]]
    for p in points[1:]: path += line(path[-1], p)
    return path


def build_path():
    bucket = (4.0, 0.0)
    outside = (-1.5, 0.0)
    wall = -2.0
    far = 5.5
    stop = bucket[0] + 0.7
    lanes = 2
    depths = [far - (far - stop) * j / (lanes - 1) for j in range(lanes)]
    anchors = [outside, (0.0, 0.0), (0.5, wall), (far, wall)]
    cur = wall
    for dep in depths:
        anchors.append((dep, cur))
        anchors.append((dep, -cur))
        cur = -cur
    anchors += [(stop, wall), (4.4, wall), (4.4, 0.8), (0.0, 0.0), outside]
    return densify(anchors), outside, bucket


def main():
    swarm = Swarm(["/drone_1", "/drone_2", "/drone_3", "/drone_4"])
    drones = list(swarm)
    sensor = sverk_interfaces.init()
    n = len(drones)
    gap = 4

    path, outside, bucket = build_path()
    nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - bucket[0], path[i][1] - bucket[1]))
    scan_at = nearest + int((n - 1) / 2 * gap)

    swarm.takeoff(z=0.3, wait=True)

    scan = None
    for t in range(len(path) + gap * (n - 1)):
        for i, d in enumerate(drones):
            idx = t - i * gap
            if idx < 0: continue
            if idx >= len(path): x, y = outside[0], (i - (n - 1) / 2.0) * 0.8
            else:
                x, y = path[idx]
            d.navigate(x=x, y=y, z=0.3, speed=0.5, frame_id="map", wait=False)
        if scan is None and t >= scan_at: scan = sensor.lidar.read_points_raw(frame="world", all_drones=True)
        swarm.wait(0.25)
    swarm.land()
    swarm.close()


if __name__ == "__main__":
    main()
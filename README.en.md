# Drone Swarm: Archipelago 2026 Tasks

[Русская версия](README.md)

Code for three tasks of the Archipelago 2026 intensive (June 2026): Snake, Maze and Warehouse. The tasks were solved as a team; all the code in this repository is mine.

## Tasks

### Snake (`snake/`)

Four drones fly in a column, weaving between cones.

- **The leader** (`drone_1.py`) flies a slalom route through waypoints densified with a 0.2 m step. It broadcasts the flight phases (`stage`, `go`, `land`) to the shared `/swarm/cmd` topic.
- **The followers** (`drone_2.py` … `drone_4.py`) subscribe to their predecessor's telemetry. They keep breadcrumbs of its path and stay 0.6 m behind along the trajectory rather than in a straight line, so the column repeats the bends of the route.

### Maze (`maze/maze.py`)

A drone explores a maze with a lidar.

- The point cloud is split into four ±30° sectors around the movement directions to see where the way is free.
- 0.9 × 0.9 m cells are explored depth-first (DFS), with a visited set and a stack for backtracking.
- During a move the drone stops if there is less than 0.3 m ahead.

### Warehouse (`warehouse/main.py`)

A swarm of four drones flies through a warehouse.

- The shared route is built from waypoints: the entrance, a pass along the wall, lanes between the racks and the way back.
- The drones fly in a column, each four points behind the previous one along the route.
- A sensor reading is taken at the point next to the container.
- At the end the drones leave the area and land in a line.

## Platform

The code runs on the organizers' simulator and drones. The `Drone` and `Swarm` classes, `ros_bridge` (the `core` module) and the sensors (`sverk_interfaces`) come from the intensive's platform and are not included.

Code comments are in Russian.

## Tech stack

Python, ROS (through the platform bridge), lidar, multi-drone control.

## License

[MIT](LICENSE)

---

<sub>The code was written in June 2026 and published on GitHub in November 2026. Commit dates follow the dates of the original files; the README was added at publication.</sub>

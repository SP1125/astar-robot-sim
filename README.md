# A* Robot Navigation Simulator

A 2D robot navigation simulator in Python (pygame + NumPy). A differential-heading robot plans and follows a path to a goal while **moving obstacles** drift around the map, replanning as the environment changes.

<!-- Add a demo GIF here: ![demo](docs/demo.gif) -->

## What it does

Each frame the robot runs a small navigation stack:

1. **Occupancy grid**: obstacles are inflated by the robot radius and marked on a 20 px grid. Moving obstacles are *swept forward* ~15 frames, so the planner avoids where they are going, not just where they are. Built with NumPy slicing (~0.01 ms).
2. **A\***: 8-connected, octile heuristic, no diagonal corner-cutting.
3. **Path pruning**: line-of-sight "string pulling" removes unnecessary grid zig-zags.
4. **Smoothing**: Catmull-Rom spline through the pruned waypoints, falling back to the pruned polyline if the spline would clip an obstacle.
5. **Path following**: a heading controller with limited turn rate; it only drives forward when roughly facing the waypoint.
6. **Replanning**: every 30 frames, and immediately (rate-limited) when the robot gets close to or touches an obstacle. A full plan takes about 5 ms.
7. **Reactive fallback**: if an obstacle gets within a few pixels of the robot, a repulsion-vector behaviour takes over until it is clear.

## Run it

```bash
pip install -r requirements.txt
python astar_robot_sim.py
```

| Flag | Meaning |
|---|---|
| `--runs N` | Number of random scenarios before exiting (default 4) |
| `--seed S` | Seed for reproducible scenarios |
| `--headless` | No window, runs as fast as possible and prints a summary (useful for benchmarking) |

### Controls

| Key | Action |
|---|---|
| `R` | Force a replan |
| `A` | Toggle autopilot / manual |
| `N` | Generate a new random map |
| `D` | Show the swept (predicted) obstacle boxes |
| Arrow keys | Manual drive (up/down move, left/right turn) |
| `Esc` | Quit |

## Results

Headless runs on 3 seeds × 10 random scenarios (3 moving obstacles, robot slightly faster than the slowest obstacles): **30/30 reached the target**, with a mean of roughly 9-16 replans per run. Some runs had brief contact with an obstacle, which the simulator resolves by pushing the robot out; they are counted as "contact frames" in the per-run output. Reproduce with:

```bash
python astar_robot_sim.py --headless --runs 10 --seed 1
```

## Code layout

Everything is in `astar_robot_sim.py`:

- `Robot`, `Target`, `Obstacle`: entities (path following, reactive avoidance, bouncing obstacles)
- `build_occupancy`, `astar`, `prune_path`, `catmull_rom_chain`, `plan_path`: planning pipeline (pure functions)
- `Simulation`: scenario generation, update loop, collision resolution, rendering, per-run stats

## Known limitations / next steps

- Pure 2D kinematic simulation: no dynamics, sensor noise, or localisation. The robot knows the true obstacle positions.
- Obstacle motion is predicted with a constant-velocity bounding box, which is conservative; a proper velocity-obstacle or time-indexed planner would be tighter.
- Replans from scratch each time instead of using an incremental planner such as D\* Lite.
- Collision handling is position correction, not a physical response.
- Next step: port the planner to ROS 2 (nav2-style costmap + planner plugin) and run it on a real or Gazebo robot.

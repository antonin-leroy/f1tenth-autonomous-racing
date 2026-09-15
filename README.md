# F1TENTH Autonomous Racing

Reactive and planned navigation on a 1/10-scale autonomous race car: gap following,
pure pursuit, scan matching and emergency braking. Robotics course at École
Polytechnique, 2026 — ROS 1, Python and C++.

> **Scope and attribution.** The lab skeletons, handouts and simulator come from
> [f1tenth/f1tenth_labs](https://github.com/f1tenth/f1tenth_labs) (F1TENTH, UPenn).
> This repository contains **only the code I wrote**. Files I did not modify are not
> included — which is also why this is not buildable as a ROS package. It is meant to
> be read, not installed.

![Gap follow on the track](media/gap_follow.gif)

## Results

| Controller | Top speed | Needs a map? |
|---|---|---|
| Gap follow (reactive) | [x] m/s | no |
| Pure pursuit (on a logged line) | [x] m/s | yes — pre-recorded waypoints |

![Planned vs actual path](media/actual-versus-predicted-path.png)

## What I implemented

### Gap follow with disparity extension — `gap_follow/`

The skeleton provides the node structure and the LiDAR callback. What I added:

- Preprocessing: NaN and inf mapped to 10 m, ranges capped at 3 m.
- **Disparity extension.** Wherever two consecutive range readings differ by more than
  0.30 m, the nearer obstacle is widened by half the car width (0.50 m). Without this
  the car clips the inside of corners: it steers toward the gap it *sees* rather than
  the gap it can physically fit through.
- Steering smoothed across frames, which removes the oscillation that appears at speed.

### Pure pursuit — `pure_pursuit/`

Follows a racing line recorded beforehand by driving the car manually.

- Waypoints are transformed into the car frame rather than tracked in the map frame,
  which keeps the geometry to a single arc computation.
- Lookahead distance 1.2 m, wheelbase 0.33 m, steering clamped at 0.4 rad.
- Publishes the target point and the full track as RViz markers — most of the tuning
  time went into watching the lookahead point rather than reading numbers.

[One sentence on how you chose 1.2 m, and what happened at 0.8 and at 2.0.]

### Emergency braking — `safety/`

Time-to-collision from the LiDAR scan and the odometry, braking below a fixed
threshold. Runs underneath every other controller.

### Scan matching — `scan_matching/`

[Keep this section only if the diff confirms it. Say which files are yours —
correspondence search, transform estimation — and what the failure mode was.]

### Waypoint logging — `pure_pursuit/waypoint_logger.py`

Records the car's pose while it is driven manually, producing the racing line used by
pure pursuit. The line in `waypoints/` is downsampled from the raw log.

## Limitations

- **No online replanning.** The RRT lab was not completed, so the car is either purely
  reactive (gap follow) or follows a fixed pre-recorded line (pure pursuit). It cannot
  route around an obstacle that was not there when the line was logged.
- Pure pursuit uses a single lookahead distance for the whole track, so it understeers
  in the tight corners and is conservative on the straights. A speed-dependent
  lookahead is the obvious next step.
- [The failure mode you actually hit on the real car.]

## Layout

    gap_follow/      reactive planner
    pure_pursuit/    path follower + waypoint logger
    safety/          emergency braking
    scan_matching/   ICP-style scan matching
    waypoints/       recorded racing line, downsampled
    docs/            project slides
    media/           video and plots

## Credits

Lab skeletons, simulator and course material: [f1tenth/f1tenth_labs](https://github.com/f1tenth/f1tenth_labs), MIT licence.
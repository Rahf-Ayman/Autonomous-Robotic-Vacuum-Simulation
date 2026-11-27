# Technical Design

## System overview

This repository models an autonomous vacuum-and-mop robot in Webots R2025a.
The robot follows a layout-tuned coverage route using live LiDAR, GPS, compass,
contact, cliff, and downward distance-sensor inputs. It learns carpet during
vacuuming, keeps mopping strips west of the current rug, and returns to its known
home charger. The Python controller uses only the Webots devices declared in
`worlds/project.wbt`.

## Simulation environment

The user-editable scene is a 6 m by 6 m studio apartment with wood flooring,
a sofa, bed nook, kitchen appliances, worktops, a wall desk, office chair, and a
plant. Furniture and the rug remain editable in Webots. The foundation,
floor, and perimeter walls remain locked. Moving large furniture requires
retuning the route keep-out bands; moving the rug does not require preloading
its position because the carpet map is learned during vacuuming.

The robot starts at `(2.00, 2.60, 0.075)` near the charger at `(2.00, 2.85)`.
The pre-dock approach point is `(2.00, 1.80)`. The runtime carpet map starts empty
and is populated only by surfaces encountered during vacuuming.

## Robot model and sensors

The robot is a 3.5 kg differential-drive platform with two velocity-controlled
wheels and a passive caster. Wheel commands are clamped to 6.28 rad/s.

| Device | Role |
| --- | --- |
| `clearview_lidar` | 256 horizontal rays over 180 degrees, 0.1–5.5 m range; five-sector reactive obstacle avoidance |
| `carpet_sensor` | Downward distance sensor; detects the rug edge and contributes to runtime carpet learning |
| `bumper` | Front contact recovery |
| Four `cliff_sensor_*` devices | Downward infrared edge detection |
| `wall_sensor` | Right-side proximity input |
| `gps` | Simulation ground-truth localization |
| `compass` | Heading estimation |
| Battery API | Return-to-dock and charging decisions |
| Two LEDs | Mission-state indication |

The LiDAR is divided into left, front-left, front, front-right, and right sectors.
The controller reads `getRangeImage()` for reactive avoidance. The carpet sensor
uses its existing lookup table and detects the low rug edge. Because the wheels
climb the 5 mm rug and restore the sensor's height above the surface, GPS height
confirms the traversable rug interior. Detected cells are stored only after the
robot encounters them during vacuuming, and the charger area is excluded.

## Coverage navigation

The controller generates a fixed top-down boustrophedon route inside the 6 m by
6 m apartment. The default bounds are X = -2.65..2.65 m and Y = -2.05..2.15 m,
with 0.35 m strip spacing and a 0.22 m waypoint radius. `_strip_x_bounds` keeps
strips clear of the desk, office chair, bed corners, and charger approach.

GPS and compass guide the active waypoint. A 0.55 m look-ahead blends the next
heading near a target, and forward speed scales down for large heading errors
so the robot turns in place instead of driving in tight circles. A 10-second
frustration timeout skips a waypoint that stops making progress. The route is intentionally tuned to the current
furniture layout; moving large furniture requires updating these keep-out bands.

## Mission state machine

1. `UNDOCKING`: reverse from the charger and turn toward the apartment.
2. `VACUUMING`: follow the coverage route, avoid obstacles, and learn carpet cells.
3. `MOPPING`: follow west-side carpet-clear strips and recover if carpet is sensed.
4. `ESCAPE`: reverse and turn after contact, cliff detection, or stalled motion.
5. `RETURNING`: navigate directly or through a western approach to the pre-dock point.
6. `ALIGNING`: align to the charger heading with a bounded PID controller.
7. `DOCKING`: approach using GPS lateral and compass heading corrections.
8. `CHARGING`: hold until 95 percent charge or the charging timeout.
9. `FINISHED`: stop after both cleaning phases complete.

Bumper and cliff events preempt cleaning and returning. A positional watchdog
triggers recovery after less than 0.05 m movement over 10 seconds. A cycle-time
watchdog reports intervals above 150 ms.

## Waypoint following and obstacle avoidance

The active waypoint advances when the robot reaches the 0.22 m target radius or
after 10 seconds without at least 0.04 m progress. Completed waypoint indices are
retained per mission phase across charging cycles. A phase completes after every
waypoint is marked done. Battery below 20 percent and the 1,200-second phase limit
also trigger return without discarding mission progress.

Reactive LiDAR avoidance has priority over waypoint following. The front,
front-left, front-right, and flank sectors trigger at 0.25 m or 0.20 m. Asymmetric
readings bias wheel speeds away from the obstacle, while front contact causes an
in-place turn toward the clearer flank.

## Carpet learning and mopping

During `VACUUMING`, a positive carpet classification adds the sensed cell to an
in-memory 0.25 m carpet grid. Classification combines the downward sensor with a
GPS height increase greater than 4 mm relative to the first floor sample taken
away from the charger. Isolated sensor spikes are not stored without height or
learned-cell confirmation. The charger area is excluded so its raised base is
not learned as carpet. The map starts empty and persists across charging cycles
for the current run only.

During `MOPPING`, strips intersecting the learned carpet Y range plus a 0.35 m
transition buffer stop west of the learned western edge with a 0.45 m buffer,
because the bed leaves no traversable lane on the rug's east side. Speed also
scales down within 0.60 m of a mopping waypoint to prevent corner overshoot onto
the rug. After a
charging interruption, the robot resumes through the clear northern approach
and western corridor to the first incomplete waypoint instead of crossing the
rug or repeating completed strips. No rug coordinates are preloaded. Live
sensor, height, and learned-cell checks trigger recovery if carpet is encountered. Recovery ends only after the sensor returns to its
calibrated floor range and neither height nor the learned map confirms carpet.
Low infrared cliff readings are ignored on a confirmed raised surface or a
learned carpet cell, preventing the dark rug and its edge from being mistaken
for a drop.

## Docking and charging

The return controller targets `(2.00, 1.80)` directly, or uses the clear western
corridor through `(-1.10, -1.00)` and `(-1.10, 0.75)` when returning from the
southern part of the room. This keeps the return path around the desk and office
chair instead of recovering repeatedly against them. At the approach point, a PID
controller (`Kp = 0.045`, `Ki = 0`, `Kd = 0.001`) aligns the robot to the dock.

Docking combines GPS X correction with heading correction. Contact near the
dock, Y beyond 2.70 m, or a stall beyond Y = 2.45 m confirms docking. A stall
outside that region or a 45-second timeout returns to alignment.

The robot battery is `[10000, 10000, 1000]`; the charger reservoir is
`[10000000, 10000000, 1000]`. Charging ends at 95 percent or after 90 seconds.
An incomplete phase resumes; a completed phase advances to the next phase; two
completed phases lead to `FINISHED`.

## Source layout

```text
controllers/roomba_controller/roomba_controller.py  Coverage and mission control
docs/diagrams/draw.io/                              Editable diagram sources
docs/diagrams/images/                              Exported diagrams
docs/technical-design.md                            This document
worlds/project.wbt                                  User-editable robot and apartment
```

## Scope and attribution

This is an independently implemented simulation prototype. GPS provides
ground-truth localization, so the navigation stack is not a deployable SLAM
system. Product and feature names identify the physical inspiration only; no
manufacturer manual, firmware, or proprietary imagery is distributed.

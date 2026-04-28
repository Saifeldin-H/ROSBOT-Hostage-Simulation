# Principles of Robotics Project

## Overview

## Rescue Scenario Demos

Run any rescue demo by setting `RESCUE_SCENARIO` before starting Compose:

```bash
RESCUE_SCENARIO=scenario_1 docker compose up --build
RESCUE_SCENARIO=scenario_2 docker compose up --build
RESCUE_SCENARIO=scenario_3 docker compose up --build
RESCUE_SCENARIO=scenario_4 docker compose up --build
```

The selected scenario controls the Gazebo world, hostage actors, robot start
pose, and mission behavior.

- `scenario_1`: priority conference-room hostage clearance.
- `scenario_2`: multi-room sweep with hostages in the open office and kitchen-side area.
- `scenario_3`: constrained-access validation with scenario-only clutter near the target room.
- `scenario_4`: locate the hostage, pause exploration, and guide the hostage back to the safe zone.

The main rescue status topic is `/rescue` and publishes JSON in
`std_msgs/msg/String`.

Useful inspection commands:

```bash
docker compose exec hostage_mission ros2 topic echo /rescue
docker compose logs -f frontier_explorer
docker compose down
```

Hostage actor topics:

- `/hostage_1/pose`, `/hostage_1/cmd_vel`
- `/hostage_2/pose`, `/hostage_2/cmd_vel` when the scenario has two hostages


## Frontier Explorer Algorithm

The exploration logic lives in `FrontierExplorerNode` and implements a frontier-based exploration strategy for a mobile robot using an occupancy grid map and Nav2's `NavigateToPose` action.

### What the node listens to

- A `nav_msgs/OccupancyGrid` map on the configured map topic.
- The robot pose from TF, using one of the configured base frames such as `base_link` or `base_footprint`.
- The Nav2 `NavigateToPose` action server.

### Core idea

A frontier is a known free map cell that touches at least one unknown cell. Those cells represent the boundary between explored and unexplored space, so driving the robot toward them expands the mapped area.

### Step-by-step behavior

1. Wait for the map and Nav2 to become available.
2. Read the robot's current position from TF in the map frame.
3. Scan the occupancy grid and mark frontier cells.
4. Group neighboring frontier cells into clusters.
5. Compute the centroid of each cluster in world coordinates.
6. Filter out poor candidates:
   - clusters smaller than `min_frontier_size`
   - clusters closer than `goal_tolerance`
   - clusters near previously blacklisted failed goals
7. Sort the remaining clusters by:
   - shortest distance to the robot
   - then largest cluster size as a tiebreaker
8. Send the best cluster centroid to Nav2 as the next goal.
9. If the goal succeeds, repeat the process on the next timer tick.
10. If the goal fails repeatedly, blacklist that area and choose a different frontier later.

### How frontier cells are detected

For each non-border grid cell, the node checks:

- the cell must be free or at least not occupied above `occupancy_threshold`
- the cell must have at least one unknown neighbor with occupancy `-1`

This makes the frontier set the reachable-looking edge of currently explored space.

### How clustering works

The node uses a breadth-first search over frontier cells:

- with 4-connectivity or 8-connectivity depending on `frontier_connectivity`
- each connected component becomes one `FrontierCluster`

For each cluster, it stores:

- the list of cells
- the centroid in world coordinates
- the Euclidean distance from the robot to that centroid

### Goal selection policy

This implementation uses a greedy local policy:

- prefer the nearest valid frontier cluster
- if two candidates are equally near, prefer the larger cluster

That keeps the algorithm simple and responsive, though it does not explicitly optimize for global map coverage or path cost through obstacles.

### Failure handling and blacklist

If Nav2 rejects or fails a goal:

- the node increments a retry counter for that frontier region
- nearby goals are grouped using a quantized retry key based on `frontier_blacklist_radius`
- once the retry count reaches `retry_limit`, that frontier is blacklisted

Blacklisted frontiers are skipped in future selection so the robot does not get stuck retrying the same unreachable area forever.

### Important implementation detail

The goal orientation is currently set to the identity quaternion (`w = 1.0`), so the robot is only told where to go, not what heading to adopt at the target. The exploration behavior therefore depends mainly on the position of the selected frontier centroid.

### Main parameters

- `map_topic`: occupancy grid source
- `robot_base_frames`: TF frames used to locate the robot
- `nav_to_pose_action`: Nav2 action name
- `frontier_connectivity`: 4 or 8 neighbor clustering
- `min_frontier_size`: minimum cluster size to consider
- `occupancy_threshold`: maximum occupancy value still treated as traversable for frontier detection
- `goal_tolerance`: ignore frontiers already very close to the robot
- `retry_limit`: number of failed attempts before blacklisting
- `replan_period_sec`: how often the node searches for a new frontier
- `frontier_blacklist_radius`: radius used to merge failed goal locations into one blocked region

### Summary

In short, the node repeatedly finds the boundary between known free space and unknown space, groups that boundary into candidate regions, chooses the nearest reasonable cluster, and asks Nav2 to drive there. If a region keeps failing, it gets blacklisted so exploration can continue elsewhere.

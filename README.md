# PHostage simulation

ROS 2 Humble rescue simulation for a Husarion ROSbot in a Gazebo office world.
The robot builds a map with SLAM Toolbox, selects exploration goals from map
frontiers, navigates with Nav2, detects Gazebo hostage actors with the onboard
camera, and publishes mission status as JSON.

The project is launched through Docker Compose. A single environment variable,
`RESCUE_SCENARIO`, selects the Gazebo world, hostage layout, robot start pose,
and mission behavior.

## Repository Layout

| Path | Purpose |
| --- | --- |
| `docker-compose.yml` | Starts Gazebo, SLAM Toolbox, Nav2, RViz, frontier exploration, and the hostage mission. |
| `src/launch/run_office_simulation.py` | Maps `RESCUE_SCENARIO` to the selected scenario SDF and robot spawn pose. |
| `src/launch/office_simulation.launch.py` | Starts Gazebo, bridges ROS/Gazebo topics, and spawns the ROSbot. |
| `src/ros2_ws/src/frontier_explorer` | Python ROS 2 package that chooses autonomous exploration goals from `/map`. |
| `src/ros2_ws/src/hostage_sim` | Gazebo actor plugin, mission node, and optional hostage patrol node. |
| `src/husarion_gz_worlds` | Vendored Husarion Gazebo worlds plus project scenario SDF files and models. |
| `src/config/frontier_params.yaml` | Runtime parameters for frontier selection and blacklisting. |
| `src/config/office_nav2.rviz` | RViz configuration for the office rescue simulation. |
| `docs/rescue_scenario_demo.md` | Longer technical brief for presentation and demo planning. |

## Prerequisites

- Docker and Docker Compose.
- X11 display forwarding for Gazebo and RViz.
- NVIDIA Container Runtime if using the current Compose file as-is, because the
  GUI services request `runtime: nvidia`.

Set `GZ_HEADLESS_MODE=True` to reduce Gazebo rendering load:

```bash
GZ_HEADLESS_MODE=True RESCUE_SCENARIO=scenario_1 docker compose up --build
```

## Run a Rescue Scenario

```bash
RESCUE_SCENARIO=scenario_1 docker compose up --build
RESCUE_SCENARIO=scenario_2 docker compose up --build
RESCUE_SCENARIO=scenario_3 docker compose up --build
RESCUE_SCENARIO=scenario_4 docker compose up --build
```

Useful inspection commands:

```bash
docker compose exec hostage_mission ros2 topic echo /rescue
docker compose logs -f frontier_explorer
docker compose logs -f hostage_mission
docker compose logs -f rosbot_simulation
docker compose down
```

## Compose Services

| Service | Image / build | Responsibility |
| --- | --- | --- |
| `rosbot_simulation` | `src/docker/simulation/Dockerfile` | Builds `hostage_sim`, prefetches Gazebo Fuel models, launches Gazebo, loads the selected scenario, bridges topics, and spawns the robot. |
| `slam` | `husarion/slam-toolbox:humble` | Runs online SLAM and publishes `/map`. |
| `navigation` | `husarion/navigation2:humble` | Runs Nav2 and exposes the `NavigateToPose` action. |
| `rviz2` | `husarion/rosbot-gazebo:humble` | Opens RViz with the project office navigation view. |
| `frontier_explorer` | `src/docker/frontier/Dockerfile` | Builds/runs the frontier exploration node from the local workspace. |
| `hostage_mission` | `project-rosbot-gazebo-hostage:humble` | Runs mission logic and scenario-specific hostage helper nodes. |

## Scenarios

| Scenario | World file | Hostages | Behavior |
| --- | --- | ---: | --- |
| `scenario_1` | `src/husarion_gz_worlds/scenarios/scenario_1.sdf` | 1 | Priority conference-room hostage clearance. `hostage_1` patrols a short path near `(8.00, -6.25)`. |
| `scenario_2` | `src/husarion_gz_worlds/scenarios/scenario_2.sdf` | 2 | Multi-room sweep with hostages near `(2.60, -9.17)` and `(12.20, -1.80)`. |
| `scenario_3` | `src/husarion_gz_worlds/scenarios/scenario_3.sdf` | 1 | Constrained-access validation with scenario-only blockers near the target room. |
| `scenario_4` | `src/husarion_gz_worlds/scenarios/scenario_4.sdf` | 1 | Locate the hostage, pause exploration, return to the safe zone, and command the hostage to follow. |

All scenarios currently spawn the robot at `(0.38, -0.14, yaw -1.51)`.

## Main ROS Interfaces

| Topic / action | Type | Producer / consumer | Purpose |
| --- | --- | --- | --- |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM Toolbox -> frontier explorer | Occupancy grid used for frontier detection. |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | frontier explorer / mission -> Nav2 | Navigation goals for exploration and return behavior. |
| `/rescue` | `std_msgs/msg/String` | hostage mission | JSON mission state and detection results. |
| `/rescue_mission/exploration_enabled` | `std_msgs/msg/Bool` | hostage mission -> frontier explorer | Pauses/resumes autonomous frontier goal selection. |
| `/rescue_mission/hostage_distance` | `std_msgs/msg/Float32` | hostage mission | Distance to the active hostage target. |
| `/rescue_mission/hostage_rescued` | `std_msgs/msg/Bool` | hostage mission | High-level rescued flag. |
| `/hostage_1/pose`, `/hostage_2/pose` | `geometry_msgs/msg/Pose` | Gazebo bridge | Actor pose topics. |
| `/hostage_1/cmd_vel`, `/hostage_2/cmd_vel` | `geometry_msgs/msg/Twist` | mission/patrol -> Gazebo bridge | Commands for moving hostage actors. |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | robot camera | Color image used for hostage detection. |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | robot camera | Depth image used to validate camera-based detection. |

A typical `/rescue` payload is JSON:

```json
{
  "scenario": "scenario_1",
  "phase": "complete",
  "hostages": [
    {
      "id": "hostage_1",
      "zone": "conference_room",
      "acquired": true,
      "detected": true,
      "camera_visible": true,
      "pose": {"x": 8.0, "y": -6.25, "z": 0.0}
    }
  ],
  "cleared_zones": ["conference_room"],
  "safe_zone": null,
  "message": "All hostage locations confirmed.",
  "detection_method": "camera_color_image"
}
```

## Frontier Explorer

`frontier_explorer` repeatedly looks for the boundary between known free cells
and unknown cells in the occupancy grid:

1. Wait for `/map`, TF, and the Nav2 `NavigateToPose` action server.
2. Find free known cells with at least one unknown neighbor.
3. Cluster frontier cells with the configured connectivity.
4. Filter clusters that are too small, too close, or already blacklisted.
5. Score valid clusters by size, local clearance, and distance.
6. Send the best centroid to Nav2.
7. Blacklist failed frontier regions so exploration can continue elsewhere.

Current defaults in `src/config/frontier_params.yaml`:

| Parameter | Value |
| --- | ---: |
| `frontier_connectivity` | `8` |
| `min_frontier_size` | `20` |
| `occupancy_threshold` | `50` |
| `goal_tolerance` | `0.9` |
| `retry_limit` | `1` |
| `replan_period_sec` | `2.0` |
| `frontier_blacklist_radius` | `1.2` |
| `frontier_distance_weight` | `1.0` |
| `frontier_size_weight` | `1.5` |
| `frontier_clearance_radius_cells` | `6` |
| `frontier_clearance_weight` | `2.0` |

The mission node can pause the explorer through
`/rescue_mission/exploration_enabled`. When paused, the explorer cancels an
active frontier goal and skips new frontier selection until resumed.

## Hostage Mission

`hostage_sim` provides:

- `HostageActorController`, a Gazebo system plugin that publishes actor poses
  and applies velocity commands to actors.
- `hostage_mission_node.py`, the ROS 2 mission state machine.
- `hostage_patrol_node.py`, used by Scenario 1 to move the first hostage along a
  short repeating path.

The mission node combines TF, actor poses, RGB camera images, and depth images.
It looks for hostage model colors in the expected camera region, confirms
hostages after repeated visual evidence, publishes `/rescue`, and coordinates
exploration pause/return behavior. Scenario 4 adds a return phase that drives
the robot back to the safe zone while commanding the hostage actor to follow.

## Attribution and Third-Party Content

`git submodule status` is currently empty for this repository. The third-party
content used by the project is vendored under `src/husarion_gz_worlds` or
downloaded into the Docker image at build time.

| Content | Location / use | Attribution | License / source |
| --- | --- | --- | --- |
| Husarion Gazebo Worlds | `src/husarion_gz_worlds` base package, worlds, launch file, and office environment assets | Husarion; package authors Paweł Kowalski and Rafał Górecki; maintainer Husarion | Apache License 2.0. See `src/husarion_gz_worlds/LICENSE`. Package metadata lists `https://husarion.com/` and repository metadata `https://github.com/husarion/panther_ros`. |
| ROSbot model | `src/husarion_gz_worlds/models/Rosbot` | Łukasz Mitka, Husarion | Included with the vendored Husarion Gazebo content. |
| Hostage actor mesh | `src/husarion_gz_worlds/models/HostageActor/meshes/walk.dae` | Mingfei / Gazebo Fuel | Creative Commons Attribution 4.0 International. Source: `https://fuel.gazebosim.org/1.0/Mingfei/models/actor`. |
| Office surfaces | `src/husarion_gz_worlds/models/Surfaces` | Automatically generated from the Gazebo Great Editor; `info@openrobotics.org` | Included with the vendored Gazebo world assets. |
| Gazebo Fuel office props | Downloaded during image build by `src/docker/simulation/prefetch_fuel_models.sh`; also referenced from scenario/world SDF files | Open Robotics / Gazebo Fuel | Source models from `https://fuel.gazebosim.org/1.0/openrobotics/models`: `adjtable`, `bathroomsink`, `coffeetable`, `deskchair`, `drawer`, `fridge`, `mopcart2`, `mopcart3`, `officechairblack`, `squareshelf`, `table`, `toilet`, `trashbin`, `whitecabinet`, and `woodenchair`. |
| ROS 2, Gazebo, Nav2, SLAM Toolbox, ros_gz_bridge | Runtime dependencies and Docker images | Open Robotics, ROS, Gazebo, Nav2, and SLAM Toolbox contributors; Husarion Docker images for ROSbot stacks | Pulled through base images and ROS packages listed in the Dockerfiles and `docker-compose.yml`. |

Project-specific scenario files, mission logic, frontier exploration code, and
Docker glue in this repository are separate from the vendored upstream assets.

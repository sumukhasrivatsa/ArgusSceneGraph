# ARGUS
### Real-Time Open-Vocabulary Affordance Grounding for Reactive Manipulation

<br>

> *You tell a robot to pick up a ball. It sees five other objects it knows nothing about. What should it do with them?*

ARGUS answers that question. Given a single natural language prompt, the system detects every object in the scene, constructs a 3D scene graph from live perception, and uses an LLM to reason about affordances across the whole scene — not just the objects the user mentioned. Those affordance weights drive a priority-aware constraint relaxation loop that replans intelligently when the first path fails.

---

## Overview

Most LLM-based manipulation pipelines describe the scene in text and hope the model reasons correctly about space. ARGUS grounds that reasoning in metric reality before any LLM call is made.

**The pipeline:**

```
User prompt
    ↓
YOLO (yolov8l.pt) — open-vocabulary detection, all objects in scene
    ↓
Depth projection + TF — every detection into 3D world frame
    ↓
Scene Graph — positions, spatial relations, fragility, reachability, path analysis
    ↓
LLM (Claude) — affordance weights for every object, grounded in scene graph context
    ↓
Priority-aware constraint relaxation loop
    → plan → fail → relax lowest-priority soft constraint → replan → ...
    ↓
Trajectory execution
```

**The key insight:** the LLM doesn't decide what to do — it decides what the robot *can* do. The planner does the rest.

---

## Architecture

```
argus_scene_graph/
│
├── OrchestratorNode.py     # Brain: prompt → YOLO → 3D → scene graph → LLM → weights
├── scene_graph.py          # Scene graph engine: nodes, edges, spatial relations, query API
├── SceneGraphHTML.py       # Live HTML visualizer: top-down map, graph, relations table
└── GraphViz.py             # Matplotlib visualization (offline)

argus_v1/
│
├── VisualBlock.py          # Perception: YOLO → 3D projection → collision objects → goal
└── PlannerClient.py        # Planning: MoveIt2 client + ACM relaxation loop

worlds/
└── table_scene_shapes.sdf  # Gazebo scene: UR5e + RGBD camera + tabletop objects
```

### Two-node perception-planning split

**`VisualBlock` (perception node)** runs continuously. It subscribes to `/argus/weights` from the orchestrator, filters YOLO detections to only known labels, publishes collision objects and goal pose to MoveIt2.

**`PlannerClient` (planning node)** receives the goal pose and runs the ACM relaxation loop:
1. Attempt motion planning with full constraint set
2. On failure, identify the soft obstacle with the lowest LLM-assigned weight
3. Allow collision with that object via ACM diff, replan
4. Restore ACM post-execution
5. Repeat until a trajectory is found or constraint set is exhausted

### Scene graph

Built from `Detection` objects (label + 3D position + confidence). For every pair of objects, 12+ spatial relation types are computed:

| Family | Relations |
|---|---|
| Proximity | `near`, `close`, `moderate_distance`, `far_from` |
| Lateral | `left_of`, `right_of`, `directly_left_of`, `directly_right_of` |
| Depth | `in_front_of`, `behind`, `directly_ahead`, `directly_behind` |
| Vertical | `above`, `below`, `same_level`, `stacked_on` |
| Path | `on_path_to_goal`, `blocking_goal`, `clear_of_path` |
| Occlusion | `may_occlude` |
| Robot-centric | `closer_to_robot_than`, `further_from_robot_than`, `more_reachable_than` |

The graph also encodes semantic knowledge — fragility scores, object categories — so the LLM receives context it doesn't have to infer from scratch.

---

## Environment

| Component | Version |
|---|---|
| OS | macOS (Apple M2) |
| ROS2 | Jazzy (via Pixi / RoboStack) |
| Simulator | Gazebo Harmonic |
| Robot | UR5e (simulation) |
| Motion planning | MoveIt2 |
| Perception | YOLO v8l (Ultralytics) |
| LLM | Claude (Anthropic API) |
| Python | 3.11 |

---

## Setup

```bash
# 1. Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/yourname/argus.git

# 2. Install Python dependencies
pixi run pip install ultralytics opencv-python anthropic scipy --break-system-packages

# 3. Build
cd ~/ros2_ws
colcon build --packages-select argus_v1 argus_scene_graph
source install/setup.bash

# 4. Set your API key
export ANTHROPIC_API_KEY="sk-ant-..."
```

---

## Running

```bash
# Terminal 1 — Gazebo + MoveIt2
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py \
  world_file:=~/ros2_ws/worlds/table_scene_shapes.sdf \
  gazebo_gui:=false

# Terminal 2 — Gazebo GUI
pixi run gz sim -g

# Terminal 3 — Image bridge
ros2 run ros_gz_image image_bridge \
  /rgbd_camera/image /rgbd_camera/depth_image

# Terminal 4 — Perception node (waits for weights)
ros2 run argus_v1 perception_node

# Terminal 5 — Planning node
ros2 run argus_v1 planning_node

# Terminal 6 — Run ARGUS
ros2 run argus_scene_graph orchestrator_node
```

The orchestrator grabs one RGBD frame, runs the full pipeline, and opens the scene graph visualizer in your browser automatically. If `ANTHROPIC_API_KEY` is set, affordance weights are computed by the LLM and published to `/argus/weights` — the perception node picks these up and the planning loop begins.

---

## Scene Graph Visualizer

The HTML visualizer opens automatically after each orchestrator run.

```
┌──────────────────────┬──────────────────────┐
│  Top-Down Spatial    │  Detected Objects    │
│  Map                 │  Table               │
├──────────────────────┼──────────────────────┤
│  Scene Graph         │  Spatial Relations   │
│  (knowledge graph)   │  Table               │
└──────────────────────┴──────────────────────┘
```

- **Top-down map** — objects at real (x, y) world positions, robot reach circle, path-to-goal arrow
- **Scene graph** — red nodes = objects, blue nodes = attributes, arrows = spatial relations
- **Spatial relations** — every pairwise relation, colour-coded by family
- **Detected objects** — confidence, fragility, reachability, affordance weight

---

## ROS2 Topics

| Topic | Type | Direction |
|---|---|---|
| `/rgbd_camera/image` | `sensor_msgs/Image` | Camera → nodes |
| `/rgbd_camera/depth_image` | `sensor_msgs/Image` | Camera → nodes |
| `/argus/weights` | `std_msgs/String` | Orchestrator → VisualBlock |
| `/collision_object` | `moveit_msgs/CollisionObject` | VisualBlock → MoveIt2 |
| `/argus/goal_pose` | `geometry_msgs/PoseStamped` | VisualBlock → PlannerClient |
| `/argus/soft_obstacles` | `std_msgs/String` | VisualBlock → PlannerClient |

---

## Roadmap

- [x] YOLO-based open-vocabulary detection
- [x] Depth projection to world frame
- [x] Scene graph construction (12+ relation types)
- [x] LLM affordance reasoning with scene graph context
- [x] ACM-based priority-aware constraint relaxation loop
- [x] Live HTML scene graph visualizer
- [ ] Closed-loop replanning on scene change
- [ ] Grasping (ARGUS v2)
- [ ] Scene graph delta as LLM context for multi-step tasks
- [ ] Real hardware deployment

---

## Citation

If you use ARGUS in your work:

```bibtex
@misc{argus2025,
  author    = {Sumukha Srivatsa},
  title     = {ARGUS: Real-Time Open-Vocabulary Affordance Grounding for Reactive Manipulation},
  year      = {2025},
  publisher = {GitHub},
  url       = {https://github.com/yourname/argus}
}
```

---

## Acknowledgements

Built as part of preparation for graduate research in robot learning at Georgia Tech.

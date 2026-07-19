# ARGUS — Part 1: The Scene Graph

> Real-Time Open-Vocabulary Affordance Grounding for Reactive Manipulation

<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/303849b4-d3a3-44ed-bf84-eba548496271" />
<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/e7fd1bc6-b1fc-4f12-8ff1-4fd474033fb5" />
<img width="800" height="600" alt="image" src="https://github.com/user-attachments/assets/25b4497e-88a4-4d21-a320-8a166364146a" />

---

This is the first part of a series documenting ARGUS. Before the LLM reasons about what to do, it needs to know what's actually there — and where. That's what the scene graph does.

---

## What this part covers

Given a live RGBD stream, ARGUS:

1. Runs **VLM** on the current frame — detects every object in the scene, not just the ones in the user's prompt
2. Projects each detection into **3D world coordinates** using depth + TF
3. Constructs a **scene graph** — every object becomes a node, every pair of objects gets a set of spatial relations as edges
4. Renders a **live HTML visualizer** that opens in the browser the moment it runs

The scene graph is what gets handed to the LLM in Part 2. The richer the graph, the better the affordance reasoning.

---

## Scene graph structure

Each object node stores:
- 3D world position `(x, y, z)`
- VLM detection confidence
- Fragility score (from a semantic knowledge base)
- Reachability (within UR5e workspace limits)
- Distance to robot base
- Semantic categories (`fragile`, `drinkware`, `container`, ...)

Each edge between two objects stores the full set of spatial relations:

| Family | Relations |
|---|---|
| Proximity | `near` · `close` · `moderate_distance` · `far_from` |
| Lateral | `left_of` · `right_of` · `directly_left_of` · `directly_right_of` |
| Depth | `in_front_of` · `behind` · `directly_ahead` · `directly_behind` |
| Vertical | `above` · `below` · `same_level` · `stacked_on` |
| Path | `on_path_to_goal` · `blocking_goal` · `clear_of_path` |
| Occlusion | `may_occlude` |
| Robot-centric | `closer_to_robot_than` · `further_from_robot_than` |

---

## Visualizer

The HTML visualizer opens automatically after each run.

```
┌──────────────────────┬──────────────────────┐
│  Top-Down Spatial    │  Detected Objects    │
│  Map                 │                      │
├──────────────────────┼──────────────────────┤
│  Scene Graph         │  Spatial Relations   │
│  (knowledge graph    │  (colour-coded       │
│   style)             │   by relation type)  │
└──────────────────────┴──────────────────────┘
```![Uploading image.png…]()


- **Top-down map** — objects plotted at real (x, y) positions, robot reach circle, path-to-goal arrow, goal highlighted
- **Scene graph** — red nodes = objects, blue attribute nodes = conf / fragility / reachability / weight, arrows = spatial relations coloured by family
- **Spatial relations table** — every pairwise relation with colour-coded badges per relation type
- **Detected objects table** — confidence, fragility, reachability, distance, affordance weight

---

## Running

```bash
# Gazebo + MoveIt2 stack already running, image bridge active

ros2 run argus_scene_graph orchestrator_node
```

Browser opens automatically. No API key needed for this part — the scene graph runs entirely from perception.

---
## JSON as input given to a reasoning LLM:
     
You are the affordance reasoning module for a reactive robot manipulation system. Your job is to assign a numerical affordance weight to every object in the scene.

╔══════════════════════════════╗
║     3D SCENE GRAPH           ║
╚══════════════════════════════╝

Robot base: (-0.50, 0.00, 0.00)m  |  Max reach: 0.85m
Camera:     (0.50, -1.40, 0.82)m
Goal:       sports ball at (1.00, 0.21, 0.07)m

── OBJECTS (5 detected) ──
  bowl                 pos=(+0.10,-0.24,+0.12)m  dist=0.65m  conf=0.88  fragility=0.5  [container]  ✓ reachable
  cup                  pos=(+0.36,-0.17,+0.06)m  dist=0.88m  conf=0.95  fragility=0.6  [drinkware/container]  ✗ unreachable
  bottle               pos=(+0.50,+0.01,+0.15)m  dist=1.01m  conf=0.54  fragility=0.8  [drinkware/fragile]  ✗ unreachable
  cake                 pos=(+0.76,-0.41,+0.15)m  dist=1.33m  conf=0.94  fragility=0.9  [fragile/food]  ✗ unreachable
  sports ball          pos=(+1.00,+0.21,+0.07)m  dist=1.52m  conf=0.82  fragility=0.1  [ball]  ✗ unreachable

── SPATIAL RELATIONS ──
  cup                  → cake                  [far_from, left_of, behind, same_level, closer_to_robot_than, clear_of_path]  dist=0.46m
  cup                  → bowl                  [moderate_distance, right_of, same_level, further_from_robot_than, less_reachable_than, clear_of_path]  dist=0.27m
  cup                  → sports ball           [far_from, left_of, in_front_of, same_level, closer_to_robot_than]  dist=0.74m
  cup                  → bottle                [close, left_of, in_front_of, same_level, closer_to_robot_than, target_on_path_to_goal, may_occlude]  dist=0.22m
  cake                 → bowl                  [far_from, right_of, in_front_of, same_level, further_from_robot_than, less_reachable_than, clear_of_path]  dist=0.68m
  cake                 → sports ball           [far_from, left_of, in_front_of, same_level, closer_to_robot_than, may_occlude]  dist=0.66m
  cake                 → bottle                [far_from, right_of, in_front_of, same_level, further_from_robot_than, target_on_path_to_goal, may_occlude]  dist=0.49m
  bowl                 → sports ball           [far_from, left_of, in_front_of, same_level, closer_to_robot_than, more_reachable_than]  dist=1.01m
  bowl                 → bottle                [far_from, left_of, in_front_of, same_level, closer_to_robot_than, more_reachable_than, target_on_path_to_goal]  dist=0.47m
  sports ball          → bottle                [far_from, right_of, behind, same_level, further_from_robot_than]  dist=0.54m

── FRAGILE OBJECTS (handle with care) ──
  cup: fragility=0.6
  cake: fragility=0.9
  bowl: fragility=0.5
  bottle: fragility=0.8

══════════════════════════════
User instruction: "move the cup to the ball"

── EXPLICITLY ASSIGNED WEIGHTS (do not change these) ──
  sports ball         : +200  [GOAL (move toward)]

── OBJECTS NOT MENTIONED BY USER (you must assign these) ──
  Use the scene graph to reason about each one.
  cup                   suggested=-100  (reason: out of reach | nearest: bottle(0.24m), bowl(0.28m))
  cake                  suggested=-100  (reason: fragile (0.9) | out of reach | nearest: cup(0.47m), bottle(0.49m))
  bowl                  suggested=-500  (reason: nearest: cup(0.28m), bottle(0.47m))
  bottle                suggested=-100  (reason: fragile (0.8) | out of reach | nearest: cup(0.24m), bowl(0.47m))

── OUTPUT FORMAT ──
Return ONLY a valid JSON object with a weight for EVERY object.
Weight scale:
  +1000 = highest priority goal (move here)
   +200 = secondary goal
   -100 = low priority obstacle (can be relaxed)
   -500 = moderate obstacle (avoid if possible)
  -1000 = hard obstacle (never touch)

Example output:
{
  "cup": -100,
  "cake": -100,
  "bowl": -500,
  "sports ball": 200,
  "bottle": -100
}%

## Stack

`ROS2 Jazzy` · `Gazebo Harmonic` · `UR5e` · `YOLO v8l` · `Python 3.11` · `Pixi / RoboStack`

---

**Part 2 →** The LLM receives this graph and reasons about affordances across every object in the scene.

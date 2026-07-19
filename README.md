# ARGUS — Part 1: The Scene Graph

> Real-Time Open-Vocabulary Affordance Grounding for Reactive Manipulation

<img width="600" height="400" alt="image" src="https://github.com/user-attachments/assets/303849b4-d3a3-44ed-bf84-eba548496271" />
<img width="600" height="400" alt="image" src="https://github.com/user-attachments/assets/e7fd1bc6-b1fc-4f12-8ff1-4fd474033fb5" />
<img width="600" height="400" alt="image" src="https://github.com/user-attachments/assets/25b4497e-88a4-4d21-a320-8a166364146a" />

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

## Stack

`ROS2 Jazzy` · `Gazebo Harmonic` · `UR5e` · `YOLO v8l` · `Python 3.11` · `Pixi / RoboStack`

---

**Part 2 →** The LLM receives this graph and reasons about affordances across every object in the scene.

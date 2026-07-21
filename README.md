# ARGUS — Part 1: Scene Graph

> **Real-Time Open-Vocabulary Scene Understanding for Reactive Robot Manipulation**

This repository implements the perception layer of **ARGUS**. Starting from a live RGB-D stream, it constructs a rich 3D scene graph that is later used by an LLM for affordance reasoning.

---

# Demo

## Scene Graph

<p align="center">
<img src="https://github.com/user-attachments/assets/cdd3ee91-8c05-4625-b26f-809415929f89" width="900">
</p>

---

## Automatically Generated Relationships

<p align="center">
<img src="https://github.com/user-attachments/assets/db2dd598-bce8-4f2a-a40c-210f85bade22" width="900">
</p>

---

## Top-Down Scene Visualization

<p align="center">
<img src="https://github.com/user-attachments/assets/3045c3fc-0b9a-410f-9561-850731ac247e" width="900">
</p>

---

# Pipeline

Given a live RGB-D stream, ARGUS:

1. Detects **every visible object** using an open-vocabulary vision model.
2. Projects detections into **3D world coordinates** using depth and TF.
3. Builds a scene graph where
   - every object is a node
   - every pair of objects is connected through semantic spatial relations.
4. Generates an interactive HTML visualizer.

This graph becomes the input to the reasoning module in **Part 2**.

---

# Scene Graph

Each node stores

- World position `(x,y,z)`
- Detection confidence
- Fragility score
- Reachability
- Distance from robot
- Semantic categories

Each edge stores spatial relationships.

| Category | Relations |
|-----------|-----------|
| Proximity | `near` · `close` · `moderate_distance` · `far_from` |
| Lateral | `left_of` · `right_of` · `directly_left_of` · `directly_right_of` |
| Depth | `in_front_of` · `behind` · `directly_ahead` · `directly_behind` |
| Vertical | `above` · `below` · `same_level` · `stacked_on` |
| Path | `on_path_to_goal` · `blocking_goal` · `clear_of_path` |
| Occlusion | `may_occlude` |
| Robot-centric | `closer_to_robot_than` · `further_from_robot_than` |

---

# HTML Visualizer

The generated dashboard contains

- Top-down spatial map
- Interactive scene graph
- Pairwise spatial relationship table
- Object summary table

```
┌──────────────────────┬──────────────────────┐
│ Top-Down Map         │ Object Summary       │
├──────────────────────┼──────────────────────┤
│ Scene Graph          │ Spatial Relations    │
└──────────────────────┴──────────────────────┘
```

---

# Running

```bash
# Gazebo + MoveIt2 already running

ros2 run argus_scene_graph orchestrator_node
```

The browser opens automatically.

---

# Prompt Sent to the LLM

The generated scene graph is serialized into a structured prompt.

```text
You are the affordance reasoning module for a reactive robot manipulation system.

═══════════════════════════════
3D SCENE GRAPH
═══════════════════════════════

Robot base: (-0.50, 0.00, 0.00)
Camera:     (0.50, -1.40, 0.82)
Goal:       sports ball

OBJECTS (5)

• bowl
• cup
• bottle
• cake
• sports ball

...
```

See `/JSONpromptToLLM/` for the complete prompt.

---

# Example LLM Output

```json
{
  "bottle": -950,
  "cake": -900,
  "cup": -150,
  "bowl": -300,
  "sports ball": 200
}
```
These values reflect relative importance rather than snapping to the predefined levels:

* Bottle (-950): Explicitly mentioned by the user (“be careful”), fragile (0.8), close to the cup, and lies on the path toward the goal (target_on_path_to_goal), making it the highest-risk obstacle.
* Cake (-900): Also explicitly mentioned, extremely fragile (0.9), and on the approach to the goal, but slightly farther from the direct interaction than the bottle.
* Bowl (-300): Not mentioned, but reachable and located earlier in the workspace. While not highly fragile, accidental contact is plausible, so it receives a moderate penalty.
* Cup (-150): Not part of the task, not particularly hazardous, and merely near the bottle. It should be avoided if convenient but is not a significant obstacle.
* Sports ball (+200): Fixed goal object as specified by the prompt.
---

# Applying the Affordances

## Gazebo

<p align="center">
<img src="https://github.com/user-attachments/assets/6574280a-e5c5-45eb-a513-44bada37602e" width="700">
</p>

---

## RViz

<p align="center">
<img src="https://github.com/user-attachments/assets/b0c3466c-bd06-4c76-aeb2-b8c513d96502" width="700">
</p>

<p align="center">
<img src="https://github.com/user-attachments/assets/eb792b3f-e2c5-4307-854e-4b23bf04da37" width="700">
</p>

---

# Technology Stack

- ROS2 Jazzy
- Gazebo Harmonic
- UR5e
- YOLOv8-L
- Python 3.11
- Pixi
- RoboStack

---

## Next

**Part 2** uses this scene graph to perform affordance reasoning with an LLM.

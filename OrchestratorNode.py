#!/usr/bin/env python3
"""
orchestrator_node.py — ARGUS v2

LinkedIn demo version.

Run with:
    python3 orchestrator_node.py "move the cup to the ball"

What it does:
  1. Waits for one RGBD frame from the camera
  2. Runs YOLO on the RGB image → detections + bounding boxes
  3. Projects each detection to 3D world frame (depth + TF)
  4. Infers goal object from the prompt (simple keyword match)
  5. Builds a SceneGraph from all detections
  6. Generates scene_graph.html → opens in browser automatically
  7. Gets affordance weights, in priority order:
       a. /tmp/argus_manual_weights.json, if present  (hand-crafted / pasted
          from an LLM you ran yourself — no API key needed)
       b. Live LLM call, if ANTHROPIC_API_KEY is set
       c. Geometric fallback (SceneGraph.suggest_default_weights())
     Either way, the full LLM prompt (built via scene_graph.py's
     LLMPromptBuilder) is ALWAYS written to /tmp/argus_llm_prompt.txt so you
     can paste it into any LLM by hand and use the result via (a).

For the LinkedIn demo, steps 1-6 are what matter.
Step 7 is bonus — works with or without ANTHROPIC_API_KEY.
"""

import json
import math
import os
import re
import subprocess
import sys

import cv2
import numpy as np
import rclpy
import rclpy.duration
import rclpy.time
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from ultralytics import YOLO
import tf2_geometry_msgs

# ── camera constants — must match table_scene_shapes.sdf ──────────────────────
CAMERA_TRANSLATION = (0.0 - (-0.5), -1.4, 0.825)
CAMERA_EULER_RPY   = (0.0, 0.5, 1.5708)
CAMERA_FOV_RAD     = 1.20
IMAGE_WIDTH_PX     = 640
IMAGE_HEIGHT_PX    = 480

ANTHROPIC_API_KEY = os.environ.get("ANTHROPIC_API_KEY", "")
ANTHROPIC_MODEL   = "claude-sonnet-4-6"

LLM_PROMPT_PATH    = '/tmp/argus_llm_prompt.txt'
MANUAL_WEIGHTS_PATH = '/tmp/argus_manual_weights.json'
WEIGHTS_JSON_PATH   = '/tmp/argus_weights.json'

# ── simple keyword → label mapping for goal inference ─────────────────────────
# "move the cup to the ball" → goal = "cup"
# edit this to match whatever YOLO calls your objects
GOAL_KEYWORDS = {
    "cup":          "cup",
    "mug":          "cup",
    "ball":         "sports ball",
    "red ball":     "sports ball",
    "sports ball":  "sports ball",
    "bottle":       "bottle",
    "vase":         "vase",
    "boot":         "boot",
    "shoe":         "boot",
}


def infer_goal_from_prompt(prompt: str) -> str | None:
    """
    Very simple: look for known object keywords in the prompt.
    The FIRST match is treated as the goal (the thing being picked up).
    e.g. "move the cup to the ball" → "cup"
    """
    prompt_lower = prompt.lower()
    for keyword, label in GOAL_KEYWORDS.items():
        if keyword in prompt_lower:
            return label
    return None


# Matches GOAL=[whatever is in here], case-insensitive on the "GOAL=" part.
# e.g. "avoid the vase GOAL=[cup]" -> "cup"
#      "GOAL=[red ball], be careful"      -> "red ball" -> resolved via
#      GOAL_KEYWORDS to "sports ball" below, same as the old keyword-inference
#      path did, so existing aliases keep working.
GOAL_PATTERN = re.compile(r"GOAL\s*=\s*\[([^\]]*)\]", re.IGNORECASE)


def parse_goal_from_prompt(prompt: str) -> str | None:
    """
    Explicit goal override: everything between the '[' and ']' after
    'GOAL=' in the prompt is taken as the goal label, e.g.
        "pick this up carefully GOAL=[bottle]" -> "bottle"
    This takes priority over infer_goal_from_prompt's keyword matching,
    since it's an explicit instruction rather than a guess. Falls through
    GOAL_KEYWORDS so aliases (e.g. "red ball", "mug") still resolve to the
    label the detector actually uses.
    Returns None if no GOAL=[...] is present, or if it's present but empty
    (e.g. "GOAL=[]"), so callers can fall back cleanly.
    """
    match = GOAL_PATTERN.search(prompt)
    if not match:
        return None
    raw = match.group(1).strip().lower()
    if not raw:
        return None
    return GOAL_KEYWORDS.get(raw, raw)


class OrchestratorNode(Node):

    def __init__(self, prompt: str):
        super().__init__('orchestrator_node')

        self.prompt    = prompt
        self.rgb_image = None
        self.depth_image = None
        self.bridge    = CvBridge()

        # Change-detection state — kept across ticks so we skip the
        # expensive rebuild (YOLO -> scene graph -> LLM/manual-weights ->
        # JSON write -> republish) on ticks where nothing actually changed.
        # Previously the pipeline was one-shot (self.done = True after the
        # first frame ever), so a suitcase that appeared later would never
        # end up in /tmp/argus_weights.json. Now: re-run each tick, but only
        # rebuild when the SET of detected labels changes OR an existing
        # object has moved more than POS_CHANGE_THRESHOLD_M. Weights are
        # cached per label-set signature so a static scene doesn't re-hit
        # the LLM or overwrite the manual weights file on every tick.
        self._prev_label_set: set[str] = set()
        self._prev_positions: dict[str, tuple] = {}
        self._weights_cache: dict[frozenset, dict] = {}

        self.get_logger().info(f'Prompt: "{self.prompt}"')

        # camera intrinsics
        self.cx = IMAGE_WIDTH_PX  / 2.0
        self.cy = IMAGE_HEIGHT_PX / 2.0
        self.fx = self.cx / math.tan(CAMERA_FOV_RAD / 2.0)
        self.fy = self.fx

        # TF
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self._broadcast_camera_transform()
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO
        self.get_logger().info('Loading YOLO...')
        self.model = YOLO("yolov8l.pt")
        self.get_logger().info('YOLO ready.')

        # subscribers
        self.create_subscription(Image, '/rgbd_camera/image',
                                 self._rgb_cb, 10)
        self.create_subscription(Image, '/rgbd_camera/depth_image',
                                 self._depth_cb, 10)

        # weights publisher (for VisualBlock if running)
        self.weights_pub = self.create_publisher(String, '/argus/weights', 10)

        # Slower than the old 0.5s: YOLO + scene-graph + potential LLM call
        # is not a hot loop, and the change-detection gate below means most
        # ticks will short-circuit anyway. 2s is fast enough to notice a
        # new object showing up (like the suitcase) without pointlessly
        # re-running YOLO 20x for the same frame.
        self.create_timer(2.0, self._try_run)
        self.get_logger().info('Waiting for RGBD frame...')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough').astype(np.float32)

    # ── main pipeline ─────────────────────────────────────────────────────────

    def _try_run(self):
        # No more one-shot latch: this tick runs every 2s (see the timer in
        # __init__), and the change-detection gate below decides whether to
        # do the expensive downstream work or bail early.
        if self.rgb_image is None or self.depth_image is None:
            return

        # save raw frame — cheap, do it every tick so the last snapshot on
        # disk always matches whatever the node most-recently looked at
        cv2.imwrite('/tmp/argus_frame.png', self.rgb_image)

        # ── step 1: YOLO ──────────────────────────────────────────────────────
        detections_raw = self._run_yolo()

        # NOTE: suitcase was previously missing from this set, which is the
        # actual reason it never showed up in /tmp/argus_weights.json — YOLO
        # was detecting it fine (see /tmp/argus_detections.png), but this
        # filter dropped it before it ever reached the scene graph.
        KNOWN_LABELS = {"cup", "bottle", "cake", "bowl", "sports ball", "suitcase", "toaster"}
        detections_raw = [d for d in detections_raw if d["label"] in KNOWN_LABELS]

        # ── change-detection gate ─────────────────────────────────────────────
        # Bail early if nothing meaningful changed since the last tick, so
        # we don't rebuild the scene graph / re-query the LLM / rewrite the
        # JSON on every tick for a static scene.
        current_labels = frozenset(d["label"] for d in detections_raw)
        labels_changed = current_labels != frozenset(self._prev_label_set)

        POS_CHANGE_THRESHOLD_M = 0.03   # 3cm — same order as VisualBlock's
        pos_changed = False
        # NOTE: we don't have projected 3D positions yet at this point (that
        # happens in step 2 below), so this per-object move check uses the
        # 2D pixel centroid as a cheap proxy — if pixel centres moved, the
        # 3D positions will too. Real move detection could be layered in
        # after _project_to_3d if we care about sub-pixel-but-large-3D moves.
        curr_pix = {d["label"]: (d["cx_px"], d["cy_px"]) for d in detections_raw}
        for lb, (cx, cy) in curr_pix.items():
            if lb in self._prev_positions:
                px, py = self._prev_positions[lb]
                if ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5 > 20:  # ~20px
                    pos_changed = True
                    break

        if not labels_changed and not pos_changed and self._prev_label_set:
            self.get_logger().info(
                f'No change ({sorted(current_labels)}) — skipping rebuild.',
                throttle_duration_sec=10.0)
            return

        self.get_logger().info(
            f'Scene changed (labels_changed={labels_changed}, '
            f'pos_changed={pos_changed}) — running pipeline...')
        self._prev_label_set = set(current_labels)
        self._prev_positions = curr_pix

        if not detections_raw:
            self.get_logger().error('No detections. Check /tmp/argus_frame.png')
            return

        self.get_logger().info(
            f'Detected: {[d["label"] for d in detections_raw]}')

        # ── step 2: 3D projection ─────────────────────────────────────────────
        detections_3d = self._project_to_3d(detections_raw)
        self.get_logger().info(
            f'3D positions: {[(d["label"], d["pos"]) for d in detections_3d]}')

        # ── step 3: goal comes from the prompt, not a hardcoded constant ────
        # Priority: explicit "GOAL=[label]" in the prompt > keyword inference
        # (infer_goal_from_prompt) > "sports ball" as a last-resort default so
        # the demo never crashes on a prompt with no goal info at all.
        detected_labels = {d["label"] for d in detections_3d}
        goal_label = (
            parse_goal_from_prompt(self.prompt)
            or infer_goal_from_prompt(self.prompt)
            or "sports ball"
        )
        if goal_label not in detected_labels:
            self.get_logger().warn(
                f'Goal "{goal_label}" not among detected objects '
                f'{sorted(detected_labels)} — falling back to keyword '
                f'inference / default instead.')
            goal_label = infer_goal_from_prompt(self.prompt) or "sports ball"
        self.get_logger().info(f'Goal: {goal_label} (from prompt: "{self.prompt}")')

        # ── step 4: build SceneGraph ──────────────────────────────────────────
        graph, diff = None, None
        try:
            from argus_scene_graph.scene_graph import SceneGraphBuilder, Detection as SGDetection
            from argus_scene_graph.SceneGraphHTML import generate_html
            builder = SceneGraphBuilder(
                robot_pos  = (-0.5, 0.0, 0.0),
                camera_pos = CAMERA_TRANSLATION,
            )
            sg_dets = [
                SGDetection(
                    label    = d["label"],
                    pos      = d["pos"],
                    conf     = d["conf"],
                    size_est = d["size"],
                )
                for d in detections_3d
            ]
            graph, diff = builder.build(sg_dets, goal_label=goal_label)
            self.get_logger().info('Scene graph built.')

            # ── step 5: generate HTML and open in browser ─────────────────────
            try:
                from argus_scene_graph.SceneGraphHTML import generate_html
                html_path = "/tmp/scene_graph.html"
                generate_html(graph, diff=diff, save_path=html_path)
                # open browser automatically
                subprocess.Popen(["open", html_path])
                self.get_logger().info(
                    f'Scene graph HTML opened: {html_path}')
            except Exception as e:
                self.get_logger().error(f'HTML generation failed: {e}')

        except ImportError as e:
            self.get_logger().warn(f'scene_graph.py not found: {e}')

        if graph is None:
            self.get_logger().error('No scene graph — cannot compute weights.')
            return

        # ── step 6: build + save the real LLM prompt, ALWAYS ────────────────
        # Previously this only ever ran (and only ever existed as a duplicate,
        # inferior copy) inside the API-key branch below, so if you didn't
        # have a key set, no prompt file was ever written. Now it's built via
        # scene_graph.py's own LLMPromptBuilder (the in-depth one, with scene
        # relations / fragility / path-obstacle reasoning per object) and
        # saved unconditionally, so you can always paste it into an LLM
        # yourself even with zero API usage.
        prompt_text = None
        try:
            from argus_scene_graph.scene_graph import LLMPromptBuilder
            prompt_text = LLMPromptBuilder.build_affordance_prompt(
                graph,
                user_instruction=self.prompt,
                known_weights={goal_label: 200},
                diff=diff,
            )
            with open(LLM_PROMPT_PATH, 'w') as f:
                f.write(prompt_text)
            self.get_logger().info(f'LLM prompt → {LLM_PROMPT_PATH}')
        except Exception as e:
            self.get_logger().error(f'Prompt build/save failed: {e}')

        # ── step 7: get weights — manual override > live LLM > geometric ────
        weights = None

        # (a) manual override: paste an LLM's response (or your own numbers)
        # into /tmp/argus_manual_weights.json and it's used as-is, no API
        # call needed. Any object missing from that file gets the geometric
        # suggestion instead of being silently dropped.
        manual_weights = self._load_manual_weights(graph)
        if manual_weights is not None:
            weights = manual_weights
            self.get_logger().info(f'Using manual weights: {weights}')

        # (b) live LLM call, only if no manual override and a key is present
        if weights is None and ANTHROPIC_API_KEY and prompt_text:
            self.get_logger().info('API key found — calling LLM...')
            weights = self._call_llm(prompt_text)
            if weights:
                self.get_logger().info(f'LLM weights: {weights}')
            else:
                self.get_logger().warn('LLM failed — falling back to geometric weights')

        # (c) geometric fallback — last resort
        if weights is None:
            weights = graph.suggest_default_weights()
            weights[goal_label] = 200
            self.get_logger().info(f'Using geometric fallback weights: {weights}')

        # ── step 8: save + publish weights to all subscribers ─────────────────
        positions  = {lb: list(node.pos)  for lb, node in graph.nodes.items()}
        # Per-object size (w, h, d) in metres, straight off SceneNode.size —
        # perception_node/SceneGraphBuilder already populated this from the
        # YOLO bbox + depth back-projection, so no new perception. Added
        # here so voxel_publisher can render each object filling its actual
        # detected extent instead of a fixed-sigma point-Gaussian at the
        # centroid (a suitcase filling its bbox instead of a 9cm halo).
        sizes      = {lb: list(node.size) for lb, node in graph.nodes.items()}
        voxel_data = {"weights": weights, "positions": positions, "sizes": sizes}

        # save for voxel_publisher to read
        with open(WEIGHTS_JSON_PATH, 'w') as f:
            json.dump(voxel_data, f, indent=2)
        self.get_logger().info(
            f'Weights + positions + sizes → {WEIGHTS_JSON_PATH}')

        # publish to /argus/weights
        # → VisualBlock starts publishing collision objects + goal
        # → VoxelPublisher triggers voxel map
        msg      = String()
        msg.data = json.dumps(weights)
        self.weights_pub.publish(msg)
        self.get_logger().info(f'Published to /argus/weights → VisualBlock + VoxelPublisher')

    # ── YOLO ──────────────────────────────────────────────────────────────────

    def _run_yolo(self) -> list[dict]:
        results = self.model(self.rgb_image, verbose=False, conf=0.50, iou=0.5)

        # save annotated image for debugging / linkedin screenshot
        annotated = results[0].plot()
        cv2.imwrite('/tmp/argus_detections.png', annotated)
        self.get_logger().info(
            'Annotated detections → /tmp/argus_detections.png')

        detections = []
        for box in results[0].boxes:
            label_name = results[0].names[int(box.cls[0])]
            conf       = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append({
                "label":    label_name,
                "conf":     conf,
                "cx_px":    (x1 + x2) / 2.0,
                "cy_px":    (y1 + y2) / 2.0,
                "box_w_px": x2 - x1,
                "box_h_px": y2 - y1,
            })
        return detections

    # ── 3D projection ─────────────────────────────────────────────────────────

    def _project_to_3d(self, detections_raw: list[dict]) -> list[dict]:
        results  = []
        TABLE_Z  = 0.0

        for det in detections_raw:
            cx_px = det["cx_px"]
            cy_px = det["cy_px"]
            depth = self._get_depth(cx_px, cy_px)

            if depth is None:
                self.get_logger().warn(
                    f'  {det["label"]}: no valid depth — skipping')
                continue

            size_x = max((det["box_w_px"] * depth) / self.fx, 0.05)
            size_y = size_x
            size_z = max((det["box_h_px"] * depth) / self.fy, 0.05)

            cam_x, cam_y, cam_z = self._pixel_to_camera(cx_px, cy_px, depth)
            world = self._camera_to_world(cam_z, -cam_x, -cam_y)

            if world is None:
                continue

            world_x, world_y, _ = world
            world_z = TABLE_Z + size_z / 2.0

            results.append({
                "label": det["label"],
                "conf":  det["conf"],
                "pos":   (round(world_x, 3),
                          round(world_y, 3),
                          round(world_z, 3)),
                "size":  (round(size_x, 3),
                          round(size_y, 3),
                          round(size_z, 3)),
            })

        return results

    # ── LLM ───────────────────────────────────────────────────────────────────

    def _load_manual_weights(self, graph) -> dict | None:
        """
        Manual override so you can drive the whole pipeline without an API
        key: run the node once, it saves /tmp/argus_llm_prompt.txt, paste
        that into any LLM (or reason about it yourself), save the JSON it
        gives you to /tmp/argus_manual_weights.json, and every subsequent
        run picks it up here instead of hitting the API or the geometric
        fallback.

        The prompt itself asks for a weight for EVERY object in the scene
        (see LLMPromptBuilder's "OUTPUT FORMAT" section), so a correctly
        pasted response should already cover every label — geometric
        fallback here was previously silent, so a typo/missing key in your
        pasted JSON would quietly get overwritten with a geometric guess
        and you'd have no way to tell your manual weights weren't actually
        the ones driving the voxel field. Now: any label missing from the
        manual file is loudly warned about (so you notice and fix the
        JSON), and only THEN does it fall back to the geometric suggestion
        rather than leaving that object unweighted entirely.
        """
        if not os.path.exists(MANUAL_WEIGHTS_PATH):
            return None
        try:
            with open(MANUAL_WEIGHTS_PATH, 'r') as f:
                manual = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Could not parse {MANUAL_WEIGHTS_PATH}: {e}')
            return None

        missing = [lb for lb in graph.nodes if lb not in manual]
        if missing:
            self.get_logger().warn(
                f'{MANUAL_WEIGHTS_PATH} is missing weights for {missing} — '
                f'these will use the geometric fallback instead of your '
                f'pasted values. Add them explicitly if that\'s not what '
                f'you want.')

        suggested = graph.suggest_default_weights() if missing else {}
        return {lb: manual.get(lb, suggested.get(lb, -200)) for lb in graph.nodes}

    def _call_llm(self, prompt_text: str) -> dict | None:
        """
        Calls the live API with the prompt already built (and saved) in
        step 6 — this used to rebuild its own separate, worse prompt and
        was also being called with a mismatched number of arguments
        (`_call_llm(graph, diff, goal_label)` against a 2-arg signature),
        which would have raised a TypeError the moment ANTHROPIC_API_KEY was
        ever set. Fixed: single prompt_text argument, matches the call site.
        """
        import requests

        try:
            resp = requests.post(
                "https://api.anthropic.com/v1/messages",
                headers={
                    "x-api-key":         ANTHROPIC_API_KEY,
                    "anthropic-version": "2023-06-01",
                    "content-type":      "application/json",
                },
                json={
                    "model":      ANTHROPIC_MODEL,
                    "max_tokens": 512,
                    "messages":   [{"role": "user", "content": prompt_text}],
                },
                timeout=30,
            )
            resp.raise_for_status()
            text = resp.json()["content"][0]["text"].strip()
            if text.startswith("```"):
                text = text.split("```")[1]
                if text.startswith("json"):
                    text = text[4:]
            return json.loads(text)
        except Exception as e:
            self.get_logger().error(f'LLM failed: {e}')
            return None

    # ── geometry helpers ──────────────────────────────────────────────────────

    def _broadcast_camera_transform(self):
        t                         = TransformStamped()
        t.header.stamp            = self.get_clock().now().to_msg()
        t.header.frame_id         = 'world'
        t.child_frame_id          = 'camera_link'
        tx, ty, tz                = CAMERA_TRANSLATION
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        r = Rotation.from_euler('xyz', CAMERA_EULER_RPY)
        qx, qy, qz, qw           = r.as_quat()
        t.transform.rotation.x    = qx
        t.transform.rotation.y    = qy
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_static_broadcaster.sendTransform(t)

    def _get_depth(self, cx_px, cy_px):
        row, col = int(cy_px), int(cx_px)
        if not (0 <= row < self.depth_image.shape[0]): return None
        if not (0 <= col < self.depth_image.shape[1]): return None
        d = float(self.depth_image[row, col])
        if d <= 0.0 or np.isnan(d) or np.isinf(d): return None
        return d

    def _pixel_to_camera(self, cx_px, cy_px, depth):
        return (
            (cx_px - self.cx) * depth / self.fx,
            (cy_px - self.cy) * depth / self.fy,
            depth,
        )

    def _camera_to_world(self, gz_x, gz_y, gz_z):
        pt                 = PointStamped()
        pt.header.frame_id = 'camera_link'
        pt.header.stamp    = rclpy.time.Time().to_msg()
        pt.point.x         = gz_x
        pt.point.y         = gz_y
        pt.point.z         = gz_z
        try:
            t = self.tf_buffer.transform(
                pt, 'world',
                timeout=rclpy.duration.Duration(seconds=1.0))
            return t.point.x, t.point.y, t.point.z
        except Exception as e:
            self.get_logger().warn(f'TF failed: {e}')
            return None


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    # get prompt — either as ROS param or plain sys.argv
    prompt = "move the cup to the ball"   # default
    for i, arg in enumerate(sys.argv[1:], 1):
        if not arg.startswith("--"):
            prompt = arg
            break
        if arg.startswith("--prompt="):
            prompt = arg.split("=", 1)[1]
            break

    print(f'\nARGUS Orchestrator\nPrompt: "{prompt}"\n')
    node = OrchestratorNode(prompt=prompt)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
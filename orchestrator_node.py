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
  7. (Optional) Calls LLM for affordance weights → publishes to /argus/weights

For the LinkedIn demo, steps 1-6 are what matter.
Step 7 is bonus — works if ANTHROPIC_API_KEY is set.
"""

import json
import math
import os
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

# ── camera constants — must match table_scene_shapes.sdf ──────────────────────
CAMERA_TRANSLATION = (0.0 - (-0.5), -1.4, 0.825)
CAMERA_EULER_RPY   = (0.0, 0.5, 1.5708)
CAMERA_FOV_RAD     = 1.20
IMAGE_WIDTH_PX     = 640
IMAGE_HEIGHT_PX    = 480

ANTHROPIC_API_KEY = os.environ.get("ANTHROPIC_API_KEY", "")
ANTHROPIC_MODEL   = "claude-sonnet-4-6"

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


class OrchestratorNode(Node):

    def __init__(self, prompt: str):
        super().__init__('orchestrator_node')

        self.prompt    = prompt
        self.rgb_image = None
        self.depth_image = None
        self.done      = False
        self.bridge    = CvBridge()

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

        self.create_timer(0.5, self._try_run)
        self.get_logger().info('Waiting for RGBD frame...')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _rgb_cb(self, msg: Image):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _depth_cb(self, msg: Image):
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough').astype(np.float32)

    # ── main pipeline ─────────────────────────────────────────────────────────

    def _try_run(self):
        if self.done:
            return
        if self.rgb_image is None or self.depth_image is None:
            return

        self.done = True
        self.get_logger().info('Frame received. Running pipeline...')

        # save raw frame
        cv2.imwrite('/tmp/argus_frame.png', self.rgb_image)

        # ── step 1: YOLO ──────────────────────────────────────────────────────
        detections_raw = self._run_yolo()
        if not detections_raw:
            self.get_logger().error('No detections. Check /tmp/argus_frame.png')
            return

        self.get_logger().info(
            f'Detected: {[d["label"] for d in detections_raw]}')

        # ── step 2: 3D projection ─────────────────────────────────────────────
        detections_3d = self._project_to_3d(detections_raw)
        self.get_logger().info(
            f'3D positions: {[(d["label"], d["pos"]) for d in detections_3d]}')

        # ── step 3: goal is hardcoded — sports ball is always the target ────
        goal_label = "sports ball"
        self.get_logger().info(f'Goal: {goal_label}')

        # ── step 4: build SceneGraph ──────────────────────────────────────────
        try:
            from scene_graph import SceneGraphBuilder, Detection as SGDetection
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
                from SceneGraphHTML import generate_html
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

        # ── step 6: (optional) call LLM + publish weights ─────────────────────
        if ANTHROPIC_API_KEY:
            self.get_logger().info('API key found — calling LLM...')
            weights = self._call_llm(detections_3d, goal_label)
            if weights:
                msg      = String()
                msg.data = json.dumps(weights)
                self.weights_pub.publish(msg)
                self.get_logger().info(f'Weights published: {weights}')
        else:
            self.get_logger().info(
                'No ANTHROPIC_API_KEY set — skipping LLM. '
                'Set it to also trigger robot movement.')

    # ── YOLO ──────────────────────────────────────────────────────────────────

    def _run_yolo(self) -> list[dict]:
        results = self.model(self.rgb_image, verbose=False, conf=0.59, iou=0.5)

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

    def _call_llm(self, detections_3d, goal_label) -> dict | None:
        import requests

        all_labels = [d["label"] for d in detections_3d]
        prompt = f"""You are a robot manipulation planner.

User instruction: "{self.prompt}"
Goal object (inferred): {goal_label}

Detected objects: {all_labels}

Assign an affordance weight to every object:
  positive (+200)   → GOAL (robot picks this up)
  -100 to -599      → SOFT obstacle (can be relaxed if stuck)
  -600 to -1000     → HARD obstacle (never touch)

Consider: fragility, proximity to goal, whether it blocks the path.
Return ONLY valid JSON, no explanation:
{{"label": weight, ...}}"""

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
                    "max_tokens": 256,
                    "messages":   [{"role": "user", "content": prompt}],
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

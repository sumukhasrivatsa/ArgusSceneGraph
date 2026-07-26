#!/usr/bin/env python3
"""
voxel_publisher.py — ARGUS VoxPoser-style affordance map in RViz

Subscribes to /argus/weights. When weights arrive from the orchestrator,
reads positions from /tmp/argus_weights.json and publishes a MarkerArray
of coloured voxels to /argus/voxel_map.

Triggers automatically — no manual run needed after orchestrator.

In RViz:
    Add → By Topic → /argus/voxel_map → MarkerArray
    Fixed Frame: world
"""

import math
import json
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

WEIGHTS_JSON_PATH   = '/tmp/argus_weights.json'
# Optional hand-crafted override — if this file exists and changes, its
# contents are used as the weights dict instead of whatever came in on
# /argus/weights, with no orchestrator re-run or extra node needed. Just
# edit/save this file and the field updates on the next poll.
MANUAL_WEIGHTS_PATH = '/tmp/argus_manual_weights.json'
MANUAL_POLL_PERIOD_S = 1.5

WS_X = (-0.70, 1.10)
WS_Y = (-0.55, 0.55)
# world origin (0,0,0) = robot base per TF/MoveIt/Gazebo/RViz, BUT the SDF's
# absolute Gazebo z (table top = 0.775) does NOT carry over into this frame —
# the collision-object positions (real detected objects) sit near z≈0-0.2,
# so the robot base is effectively already at ~table height here, not at
# Gazebo ground level. 0.90 capped around the elbow, still short of the
# end-effector — raised further. Still a rough guess without live TF data;
# nudge again based on where it lands relative to the robot in RViz.
WS_Z = (0.00, 1.40)

GRID_NX    = 30
GRID_NY    = 24
GRID_NZ    = 37   # keeps ~same voxel spacing as before over the taller range
# Fill fraction < 1 leaves visible air gaps between cubes. RViz's Marker
# renderer does NOT sort/composite overlapping transparent triangles
# correctly (no order-independent transparency), so a tightly packed grid of
# semi-transparent cubes looks like a solid, dark, opaque block once enough
# of them stack along the camera ray — regardless of how low alpha is set.
# Leaving gaps is what actually reads as "translucent fog" instead of "wall".
VOXEL_SIZE = (WS_X[1] - WS_X[0]) / GRID_NX * 0.55
# MODE SWITCH: this used to be 0.30 for a full-volume VoxPoser-style haze
# across the whole workspace. For diagnosing whether the scene-graph/LLM
# weight logic is actually doing the right thing, that's the wrong tool —
# a diffuse haze makes it impossible to tell "is this dense near the bottle"
# from "is this just ambient fog everywhere." Pulled tight instead: at
# SIGMA=0.09, contribution from an object is ~0.17 of peak at 0.15m away and
# ~0.008 by 0.25m — i.e. a halo that hugs each collision box and fades out
# within about a decimeter, not a field that reaches across the table.
# If you want the full-coverage look back later (e.g. for a demo video),
# this is the one constant to revert to ~0.30.
SIGMA      = 0.09

# Same mode switch on opacity: BASE_ALPHA near zero means empty space is
# genuinely invisible now (no ambient fog floor), and PEAK_ALPHA can go
# higher than before because tight SIGMA means far fewer voxels hit high
# |t| simultaneously along any camera ray — the RViz alpha-stacking-into-a-
# solid-wall problem from earlier was specifically a wide-SIGMA symptom.
#
# BASE_ALPHA is 0.0, not a small positive floor: giving literally every one
# of the ~30*24*37≈26.6k voxels a nonzero alpha (even ones with ~zero signal,
# nowhere near any object) compounds when enough stack along one camera ray
# — 1-(1-a)^N approaches solid opacity fast even for tiny a. Combined with
# MIN_RENDERED_ALPHA below (which skips those voxels from the marker
# entirely instead of drawing them faint), true empty space contributes
# ZERO points, which is what keeps free space actually looking like free
# space instead of a flat sheet or solid block. Do not reintroduce a
# BASE_ALPHA floor without also reconsidering MIN_RENDERED_ALPHA.
BASE_ALPHA = 0.0
PEAK_ALPHA = 0.55
MIN_RENDERED_ALPHA = 0.02   # voxels below this are skipped, not just faint

# ── Positive side (goal) — unchanged continuous ramp, normalized against
# whatever the strongest positive weight in the current scene is. ──────────
_POS_COLOR_STOPS = [
    (0.000, (0.30, 0.75, 0.35)),   # green   — neutral / no affordance
    (0.333, (0.90, 0.85, 0.20)),   # yellow
    (0.667, (0.95, 0.55, 0.10)),   # orange
    (1.000, (0.85, 0.10, 0.10)),   # red     — goal
]

# ── Negative side (obstacles) — single monochromatic blue ramp: harder
# obstacle = darker/more-navy, softer obstacle = lighter blue. One
# continuous hue family, no hard cutoff/band — this replaces an earlier
# attempt at a hard -600 threshold split, which wasn't what was wanted here;
# this is just "harder = darker," smoothly.
#
# Normalized against a FIXED reference (-1000) rather than the current
# scene's own worst obstacle: this is the same -1000 "hard obstacle, never
# touch" floor already used everywhere else in the system (the clamp in
# scene_graph.py's suggest_default_weights, and the scale documented in
# LLMPromptBuilder's own output-format spec). Fixed reference means an
# object weighted -900 always renders the same shade regardless of what
# else happens to be in the scene, instead of shifting around based on the
# current scene's max_neg — e.g. for {"cup": -300, "bowl": -900,
# "sports ball": -800, "bottle": -600}: bowl and sports ball both land in
# the dark-blue end (t=0.9 and 0.8) and read as close to the same dark
# blue, cup (t=0.3) reads as clearly lighter, exactly as wanted.
NEG_REFERENCE = -1000.0

_NEG_COLOR_STOPS = [
    (0.00, (0.70, 0.85, 0.95)),   # near-white pale blue — barely-negative
    (0.30, (0.45, 0.65, 0.88)),   # light-medium blue
    (0.60, (0.20, 0.40, 0.75)),   # medium blue
    (0.85, (0.06, 0.15, 0.50)),   # dark blue
    (1.00, (0.01, 0.02, 0.12)),   # near-black navy — hardest possible obstacle
]


def _linspace(start, stop, n):
    return [start + (stop - start) * i / max(n - 1, 1) for i in range(n)]


def _lerp_stops(t, stops):
    """t in [0,1] → (r,g,b) via piecewise-linear interpolation over a stop list."""
    t = max(0.0, min(1.0, t))
    for (t0, c0), (t1, c1) in zip(stops, stops[1:]):
        if t0 <= t <= t1:
            f = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return tuple(c0[k] + (c1[k] - c0[k]) * f for k in range(3))
    return stops[-1][1]


def _value_to_color(v, max_pos, max_neg):
    """Map a (possibly negative) affordance value to an RGBA colour.

    Positive side: unchanged, normalized against max_pos (the current
    scene's strongest positive/goal weight).

    Negative side: single continuous blue ramp normalized against the
    FIXED NEG_REFERENCE (-1000), not the scene's own max_neg — see the
    comment above _NEG_COLOR_STOPS for why. Density/opacity/falloff are
    untouched: alpha still scales the same way with |t| as before, and the
    MIN_RENDERED_ALPHA skip in _build_and_publish (free-space behaviour) is
    completely independent of this colour choice.
    """
    if v >= 0:
        t = 0.0 if max_pos < 1e-6 else min(1.0, v / max_pos)
        r, g, b = _lerp_stops(t, _POS_COLOR_STOPS)
    else:
        t = min(1.0, abs(v) / abs(NEG_REFERENCE))
        r, g, b = _lerp_stops(t, _NEG_COLOR_STOPS)

    a = BASE_ALPHA + (PEAK_ALPHA - BASE_ALPHA) * abs(t)
    return (r, g, b, a)


class VoxelPublisher(Node):

    def __init__(self):
        super().__init__('voxel_publisher')

        self.voxel_pub = self.create_publisher(
            MarkerArray, '/argus/voxel_map', 10)

        # re-published when a manual weights override is detected, so
        # VisualBlock's collision objects stay consistent with what the
        # voxel field is showing — not just the visualisation updates.
        self.weights_pub = self.create_publisher(String, '/argus/weights', 10)

        # subscribe to weights — triggers automatically when orchestrator publishes
        self.create_subscription(
            String, '/argus/weights', self._weights_cb, 10)

        # poll for a hand-crafted weights override — lets you skip the LLM
        # API and the orchestrator's camera/YOLO pipeline entirely: edit
        # /tmp/argus_manual_weights.json, save, and the field rebuilds using
        # the positions from the last real orchestrator run.
        self._manual_mtime = None
        self.create_timer(MANUAL_POLL_PERIOD_S, self._check_manual_weights)

        self.get_logger().info(
            'Voxel publisher ready. Waiting for weights on /argus/weights '
            f'(or edits to {MANUAL_WEIGHTS_PATH})...')

    def _load_positions(self):
        """Positions are always read from disk — orchestrator_node saves them
        alongside weights every real run, and they don't change unless the
        scene does, so both the live and manual paths share this."""
        with open(WEIGHTS_JSON_PATH, 'r') as f:
            data = json.load(f)
        positions = {k: tuple(v) for k, v in data['positions'].items()}
        # Sizes are optional — older JSON files (before OrchestratorNode
        # started writing this field) won't have them, so fall back to an
        # empty dict and the per-object sigma calculation below will use
        # the fixed SIGMA fallback for anything missing.
        sizes = {k: tuple(v) for k, v in data.get('sizes', {}).items()}
        return positions, sizes

    def _weights_cb(self, msg: String):
        """Called automatically when orchestrator publishes weights."""
        try:
            weights = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to parse weights: {e}')
            return

        self.get_logger().info(
            f'Weights received: {weights}. Building voxel map...')

        try:
            positions, sizes = self._load_positions()
        except Exception as e:
            self.get_logger().error(
                f'Could not load positions from {WEIGHTS_JSON_PATH}: {e}')
            return

        self._build_and_publish(weights, positions, sizes)

    def _check_manual_weights(self):
        """Polls MANUAL_WEIGHTS_PATH; rebuilds the field when it changes.

        No ROS message needed on this path — just write the file. Requires
        WEIGHTS_JSON_PATH to already exist (i.e. orchestrator_node has run
        at least once, so real object positions are known)."""
        if not os.path.exists(MANUAL_WEIGHTS_PATH):
            return

        mtime = os.path.getmtime(MANUAL_WEIGHTS_PATH)
        if mtime == self._manual_mtime:
            return   # unchanged since last poll
        self._manual_mtime = mtime

        try:
            with open(MANUAL_WEIGHTS_PATH, 'r') as f:
                weights = json.load(f)
        except Exception as e:
            self.get_logger().error(f'Could not parse {MANUAL_WEIGHTS_PATH}: {e}')
            return

        self.get_logger().info(f'Manual weights changed: {weights}.')

        # Publish on the same topic orchestrator_node uses — _weights_cb
        # above does the actual rebuild, so this also keeps VisualBlock's
        # collision objects in sync with whatever the field is showing,
        # not just the RViz visualisation.
        msg      = String()
        msg.data = json.dumps(weights)
        self.weights_pub.publish(msg)

    def _build_and_publish(self, weights, positions, sizes):
        xs = _linspace(*WS_X, GRID_NX)
        ys = _linspace(*WS_Y, GRID_NY)
        zs = _linspace(*WS_Z, GRID_NZ)

        # Per-object tuple: (x, y, z, weight, sx, sy, sz) where sx/sy/sz are
        # the per-axis Gaussian sigmas for this object.
        #
        # Old behaviour: fixed SIGMA=0.09 point-mass at the centroid for
        # EVERY object, so a marble and a suitcase both rendered as the same
        # ~9cm blob. Fix: derive each object's sigma from its own detected
        # bbox HALF-EXTENT (size/2). A Gaussian with sigma = half-extent
        # gives ~61% of its peak at the bbox edge and drops off smoothly
        # outside — i.e. the halo fills the bbox instead of being a point.
        # Anisotropic (per-axis) so a long-thin object like a bottle doesn't
        # get a spherical halo the size of its longest dimension.
        # SIGMA_FLOOR keeps a very small object (or a missing size entry)
        # from collapsing to zero — anything below the fallback stays at
        # the original fixed SIGMA so it still shows up visibly.
        SIGMA_FLOOR = SIGMA
        objects = []
        for lb, w in weights.items():
            if lb not in positions:
                continue
            px, py, pz = positions[lb]
            if lb in sizes and sizes[lb]:
                sx, sy, sz = sizes[lb]
                sig_x = max(sx * 0.5, SIGMA_FLOOR)
                sig_y = max(sy * 0.5, SIGMA_FLOOR)
                sig_z = max(sz * 0.5, SIGMA_FLOOR)
            else:
                sig_x = sig_y = sig_z = SIGMA_FLOOR
            objects.append((px, py, pz, float(w), sig_x, sig_y, sig_z))

        if not objects:
            self.get_logger().error('No objects with known positions — cannot build voxel map')
            return

        self.get_logger().info(
            f'Computing {GRID_NX}x{GRID_NY}x{GRID_NZ} = '
            f'{GRID_NX*GRID_NY*GRID_NZ} voxels...')

        voxels = []
        for x in xs:
            for y in ys:
                for z in zs:
                    val = 0.0
                    for (ox, oy, oz, ow, sx, sy, sz) in objects:
                        val += ow * math.exp(
                            -((x-ox)**2 / (2 * sx * sx)
                              + (y-oy)**2 / (2 * sy * sy)
                              + (z-oz)**2 / (2 * sz * sz))
                        )
                    voxels.append((x, y, z, val))

        pos_vals = [v[3] for v in voxels if v[3] > 0]
        neg_vals = [v[3] for v in voxels if v[3] < 0]
        max_pos  = max(pos_vals) if pos_vals else 1.0
        max_neg  = abs(min(neg_vals)) if neg_vals else 1.0

        now = self.get_clock().now().to_msg()

        ma         = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        # Single CUBE_LIST marker holding EVERY rendered voxel — this is
        # what actually gives the VoxPoser look: a continuous, filled 3D
        # field rather than isolated high-opacity blobs. Using one CUBE_LIST
        # instead of ~10k individual Marker messages also keeps RViz fast.
        field = Marker()
        field.header.frame_id  = 'world'
        field.header.stamp     = now
        field.ns               = 'argus_voxels'
        field.id                = 0
        field.type              = Marker.CUBE_LIST
        field.action            = Marker.ADD
        field.lifetime.sec      = 0
        field.pose.orientation.w = 1.0
        field.scale.x           = VOXEL_SIZE
        field.scale.y           = VOXEL_SIZE
        field.scale.z           = VOXEL_SIZE * 0.7

        skipped = 0
        for (x, y, z, val) in voxels:
            r, g, b, a = _value_to_color(val, max_pos, max_neg)

            # Skip near-zero-signal voxels entirely instead of adding a
            # point with a tiny alpha — this is what keeps free space
            # genuinely empty (zero points) instead of a huge number of
            # individually-faint points that compound into visible haze
            # once enough stack along a camera ray. Untouched by, and
            # independent of, the colour choice above.
            if a < MIN_RENDERED_ALPHA:
                skipped += 1
                continue

            p = Point()
            p.x, p.y, p.z = x, y, z
            field.points.append(p)

            c = ColorRGBA()
            c.r, c.g, c.b, c.a = r, g, b, a
            field.colors.append(c)

        ma.markers.append(field)

        self.voxel_pub.publish(ma)
        self.get_logger().info(
            f'Published {len(field.points)} voxels '
            f'({skipped} skipped as below-threshold empty space) '
            f'to /argus/voxel_map')


def main(args=None):
    rclpy.init(args=args)
    node = VoxelPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
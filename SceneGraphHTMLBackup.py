"""
SceneGraphHTML.py — ARGUS Scene Graph → HTML Report

Static SVG graph (no JS, no vis.js, no interactivity) + full data tables.
Open in any browser.
"""

import json
import math
from datetime import datetime

EDGE_FAMILY_COLOUR = {
    "on_path_to_goal":          "#E63946",
    "blocking_goal":            "#C1121F",
    "near":                     "#4361EE",
    "close":                    "#4CC9F0",
    "moderate_distance":        "#ADB5BD",
    "far_from":                 "#CED4DA",
    "left_of":                  "#2DC653",
    "right_of":                 "#2DC653",
    "directly_left_of":         "#1A7431",
    "directly_right_of":        "#1A7431",
    "in_front_of":              "#0096C7",
    "behind":                   "#0096C7",
    "directly_ahead":           "#023E8A",
    "directly_behind":          "#023E8A",
    "above":                    "#FB8500",
    "below":                    "#FB8500",
    "same_level":               "#FFB703",
    "stacked_on":               "#E85D04",
    "may_occlude":              "#7B2D8B",
    "closer_to_robot_than":     "#6D6875",
    "further_from_robot_than":  "#B5838D",
    "more_reachable_than":      "#457B9D",
    "less_reachable_than":      "#A8DADC",
}

NODE_PALETTE = [
    "#FFB3C6","#FFD6A5","#FDFFB6","#CAFFBF",
    "#9BF6FF","#BDB2FF","#FFC6FF","#A0C4FF",
    "#FFADAD","#D4E09B",
]

PRIORITY_RELATIONS = [
    "blocking_goal","on_path_to_goal","stacked_on","may_occlude",
    "near","close","directly_left_of","directly_right_of",
    "directly_ahead","directly_behind","left_of","right_of",
    "in_front_of","behind","above","below","same_level",
    "closer_to_robot_than","further_from_robot_than","far_from",
]

def _top_relations(rels, n=2):
    out = []
    for p in PRIORITY_RELATIONS:
        if p in rels:
            out.append(p)
        if len(out) == n:
            break
    return out or list(rels[:n])


def _spring_layout(labels, edges, W=820, H=560, iterations=200):
    """Simple force-directed layout — runs in Python, outputs pixel positions."""
    n = len(labels)
    if n == 0:
        return {}

    # seed: circle
    pos = {}
    for i, lb in enumerate(labels):
        a = 2 * math.pi * i / n - math.pi / 2
        pos[lb] = [W/2 + (W/3.2) * math.cos(a),
                   H/2 + (H/3.2) * math.sin(a)]

    k   = math.sqrt(W * H / max(n, 1)) * 0.9
    edg = [(e.source, e.target) for e in edges
           if e.source in pos and e.target in pos]

    for step in range(iterations):
        temp = 30 * (1 - step / iterations) + 5
        forces = {lb: [0.0, 0.0] for lb in labels}

        # repulsion
        lbs = list(labels)
        for i in range(len(lbs)):
            for j in range(i+1, len(lbs)):
                a, b = lbs[i], lbs[j]
                dx = pos[a][0] - pos[b][0]
                dy = pos[a][1] - pos[b][1]
                d  = math.sqrt(dx*dx + dy*dy) + 0.01
                f  = k*k / d
                forces[a][0] += f * dx / d
                forces[a][1] += f * dy / d
                forces[b][0] -= f * dx / d
                forces[b][1] -= f * dy / d

        # attraction
        for src, tgt in edg:
            dx = pos[tgt][0] - pos[src][0]
            dy = pos[tgt][1] - pos[src][1]
            d  = math.sqrt(dx*dx + dy*dy) + 0.01
            f  = d*d / k
            forces[src][0] += f * dx / d
            forces[src][1] += f * dy / d
            forces[tgt][0] -= f * dx / d
            forces[tgt][1] -= f * dy / d

        # apply
        for lb in labels:
            fx, fy = forces[lb]
            d = math.sqrt(fx*fx + fy*fy) + 0.01
            move = min(d, temp)
            pos[lb][0] += move * fx / d
            pos[lb][1] += move * fy / d
            pos[lb][0] = max(70, min(W-70, pos[lb][0]))
            pos[lb][1] = max(70, min(H-70, pos[lb][1]))

    return pos


def _svg_topdown(graph) -> str:
    """
    Top-down 2D table view — objects at their actual (x,y) world positions.
    Table in world coords: x ∈ [-0.7, 0.7], y ∈ [-0.45, 0.45]
    """
    W, H   = 560, 400
    PAD    = 48      # padding around table in SVG pixels
    NODE_R = 14

    # world → SVG coordinate mapping
    WX_MIN, WX_MAX = -0.90, 1.20
    WY_MIN, WY_MAX = -0.70, 0.70

    def wx(x): return PAD + (x - WX_MIN) / (WX_MAX - WX_MIN) * (W - 2*PAD)
    def wy(y): return (H - PAD) - (y - WY_MIN) / (WY_MAX - WY_MIN) * (H - 2*PAD)

    path_obs = set(graph.get_path_obstacles())
    labels   = list(graph.nodes.keys())
    label_idx = {lb: i for i, lb in enumerate(labels)}

    parts = []

    # background
    parts.append(
        f'<rect width="{W}" height="{H}" fill="#1C1C2E" rx="6"/>'
    )

    # grid lines
    for gx in [-0.5, -0.25, 0.0, 0.25, 0.5]:
        px = wx(gx)
        parts.append(
            f'<line x1="{px:.1f}" y1="{PAD}" x2="{px:.1f}" y2="{H-PAD}" '
            f'stroke="white" stroke-width="0.4" opacity="0.07"/>'
        )
    for gy in [-0.3, -0.15, 0.0, 0.15, 0.3]:
        py = wy(gy)
        parts.append(
            f'<line x1="{PAD}" y1="{py:.1f}" x2="{W-PAD}" y2="{py:.1f}" '
            f'stroke="white" stroke-width="0.4" opacity="0.07"/>'
        )

    # table outline
    tx1, ty1 = wx(-0.70), wy(0.45)
    tx2, ty2 = wx(1.10),  wy(-0.45)
    parts.append(
        f'<rect x="{tx1:.1f}" y="{ty1:.1f}" '
        f'width="{tx2-tx1:.1f}" height="{ty2-ty1:.1f}" '
        f'rx="4" fill="#2E1F0F" stroke="#D4A843" stroke-width="2"/>'
    )

    # axis labels
    for v, label in [(-0.5,"−0.5"), (0.0,"0"), (0.5,"+0.5")]:
        parts.append(
            f'<text x="{wx(v):.1f}" y="{H-6}" text-anchor="middle" '
            f'font-family="monospace" font-size="9" fill="#888">{label}</text>'
        )
    for v, label in [(-0.3,"−0.3"), (0.0,"0"), (0.3,"+0.3")]:
        parts.append(
            f'<text x="6" y="{wy(v)+3:.1f}" text-anchor="start" '
            f'font-family="monospace" font-size="9" fill="#888">{label}</text>'
        )
    parts.append(
        f'<text x="{W//2}" y="{H-1}" text-anchor="middle" '
        f'font-family="monospace" font-size="8" fill="#666">x (m)</text>'
    )

    # robot reach circle
    rx, ry = wx(graph.robot_pos[0]), wy(graph.robot_pos[1])
    from argus_scene_graph.scene_graph import ROBOT_REACH_RADIUS
    reach_px = ROBOT_REACH_RADIUS * (W - 2*PAD) / (WX_MAX - WX_MIN)
    parts.append(
        f'<circle cx="{rx:.1f}" cy="{ry:.1f}" r="{reach_px:.1f}" '
        f'fill="none" stroke="#FFD166" stroke-width="0.8" '
        f'stroke-dasharray="5,4" opacity="0.3"/>'
    )

    # path arrow: robot → goal
    goal_node = graph.nodes.get(graph.goal_label)
    if goal_node:
        gx2 = wx(goal_node.pos[0])
        gy2 = wy(goal_node.pos[1])
        dx  = gx2 - rx; dy = gy2 - ry
        d   = math.sqrt(dx*dx + dy*dy) + 0.001
        # shorten to node edge
        ax0 = rx + dx/d * NODE_R
        ay0 = ry + dy/d * NODE_R
        ax1 = gx2 - dx/d * (NODE_R + 6)
        ay1 = gy2 - dy/d * (NODE_R + 6)
        parts.append(
            '<defs><marker id="td-arrow" markerWidth="8" markerHeight="6" '
            'refX="8" refY="3" orient="auto">'
            '<polygon points="0 0,8 3,0 6" fill="#FFB703"/></marker></defs>'
        )
        parts.append(
            f'<line x1="{ax0:.1f}" y1="{ay0:.1f}" x2="{ax1:.1f}" y2="{ay1:.1f}" '
            f'stroke="#FFB703" stroke-width="1.5" stroke-dasharray="6,3" '
            f'marker-end="url(#td-arrow)" opacity="0.8"/>'
        )

    # faint edges between objects
    for edge in graph.edges:
        if edge.source not in graph.nodes or edge.target not in graph.nodes:
            continue
        ax = wx(graph.nodes[edge.source].pos[0])
        ay = wy(graph.nodes[edge.source].pos[1])
        bx = wx(graph.nodes[edge.target].pos[0])
        by = wy(graph.nodes[edge.target].pos[1])
        parts.append(
            f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" y2="{by:.1f}" '
            f'stroke="white" stroke-width="0.6" opacity="0.08"/>'
        )

    # object nodes
    for lb in labels:
        node    = graph.nodes[lb]
        x, y    = wx(node.pos[0]), wy(node.pos[1])
        i       = label_idx[lb]
        is_goal = lb == graph.goal_label
        is_path = lb in path_obs
        fr      = node.fragility
        fr_r    = min(255, int(fr * 2 * 255))
        fr_g    = min(255, int((1-fr) * 2 * 255))
        fill    = f"rgb({fr_r},{fr_g},50)"
        size    = NODE_R + int(node.conf * 8)

        # red halo for path obstacles
        if is_path:
            parts.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{size*1.9:.1f}" '
                f'fill="#E63946" opacity="0.25"/>'
            )
        # gold star halo for goal
        if is_goal:
            parts.append(
                f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{size*2.2:.1f}" '
                f'fill="#FFD166" opacity="0.3"/>'
            )

        parts.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{size}" '
            f'fill="{fill}" stroke="white" stroke-width="1.2" opacity="0.9"/>'
        )

        # label
        parts.append(
            f'<text x="{x:.1f}" y="{y - size - 5:.1f}" '
            f'text-anchor="middle" font-family="monospace" font-size="9" '
            f'fill="white">{lb}</text>'
        )
        # reach indicator
        reach_ch = "✓" if node.reachable else "✗"
        parts.append(
            f'<text x="{x:.1f}" y="{y + size + 11:.1f}" '
            f'text-anchor="middle" font-family="monospace" font-size="8" '
            f'fill="#aaa">{reach_ch} f={node.fragility:.1f}</text>'
        )

    # robot marker (star shape via circle + label)
    parts.append(
        f'<circle cx="{rx:.1f}" cy="{ry:.1f}" r="9" '
        f'fill="#FFD166" stroke="white" stroke-width="1.5"/>'
    )
    parts.append(
        f'<text x="{rx:.1f}" y="{ry - 13:.1f}" text-anchor="middle" '
        f'font-family="monospace" font-size="9" fill="#FFD166">robot</text>'
    )

    # camera marker
    cx2 = wx(graph.camera_pos[0])
    cy2 = wy(graph.camera_pos[1])
    parts.append(
        f'<rect x="{cx2-7:.1f}" y="{cy2-7:.1f}" width="14" height="14" '
        f'fill="#06D6A0" stroke="white" stroke-width="1.2" rx="2"/>'
    )
    parts.append(
        f'<text x="{cx2:.1f}" y="{cy2 - 12:.1f}" text-anchor="middle" '
        f'font-family="monospace" font-size="9" fill="#06D6A0">cam</text>'
    )

    # legend
    legend_y = H - 2
    parts.append(
        f'<text x="{PAD}" y="{legend_y}" font-family="monospace" '
        f'font-size="8" fill="#666">'
        f'green=robust · red=fragile · gold halo=goal · red halo=on path · '
        f'size=confidence · dashed=reach limit</text>'
    )

    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="100%" height="100%" viewBox="0 0 {W} {H}" '
        f'preserveAspectRatio="xMidYMid meet">'
        + "".join(parts)
        + "</svg>"
    )


def _svg_graph(graph) -> str:
    """
    Technical research-style scene graph SVG.
    Nodes are rounded rectangles (not cartoonish circles).
    Each node shows: label, conf, fragility, reachable status.
    Edges are thin precise arrows with italic relation labels.
    Color scheme: dark on white, muted role-based accents.
    """
    W, H  = 860, 580
    NW    = 110   # node width
    NH    = 62    # node height
    labels = list(graph.nodes.keys())
    n      = len(labels)
    if n == 0:
        return "<svg></svg>"

    pos      = _spring_layout(labels, graph.edges, W, H)

    # normalize positions to fill the viewbox with padding
    # spring layout often clusters in center — this stretches it out
    PAD_X, PAD_Y = 80, 70
    if len(pos) > 1:
        min_x = min(p[0] for p in pos.values())
        max_x = max(p[0] for p in pos.values())
        min_y = min(p[1] for p in pos.values())
        max_y = max(p[1] for p in pos.values())
        rx = max(max_x - min_x, 1)
        ry = max(max_y - min_y, 1)
        for lb in pos:
            pos[lb][0] = PAD_X + (pos[lb][0] - min_x) / rx * (W - 2*PAD_X)
            pos[lb][1] = PAD_Y + (pos[lb][1] - min_y) / ry * (H - 2*PAD_Y)

    path_obs = set(graph.get_path_obstacles())

    # ── role-based node styling ───────────────────────────────────────────────
    # No bright pastels — use very muted fills with precise borders
    def node_style(lb, node):
        is_goal = lb == graph.goal_label
        is_path = lb in path_obs
        fr      = node.fragility
        if is_goal:
            return "#FFF3CD", "#C77B00", 2.5   # pale amber fill, amber border
        if is_path:
            return "#FFF0F0", "#CC3333", 2.0   # pale red fill, red border
        if fr >= 0.7:
            return "#F8F0FF", "#6B3FA0", 1.5   # pale violet, violet border
        return "#F4F6F8", "#445566", 1.2        # cool grey fill, slate border

    # ── arrowhead markers — one per edge family colour ────────────────────────
    seen_cols = set()
    for edge in graph.edges:
        top = _top_relations(edge.relations, 1)
        c   = EDGE_FAMILY_COLOUR.get(top[0] if top else "", "#777")
        seen_cols.add(c)
    # always include default
    seen_cols.add("#888888")

    markers = ""
    for c in seen_cols:
        cid = c.replace("#", "a")
        markers += (
            f'<marker id="{cid}" markerWidth="7" markerHeight="5" '
            f'refX="7" refY="2.5" orient="auto">'
            f'<polygon points="0 0,7 2.5,0 5" fill="{c}" opacity="0.9"/></marker>'
        )

    parts = [f'<defs>{markers}</defs>']

    # ── background: white with faint grid ────────────────────────────────────
    parts.append(f'<rect width="{W}" height="{H}" fill="#ffffff"/>')
    # faint grid
    for gx in range(0, W, 40):
        parts.append(
            f'<line x1="{gx}" y1="0" x2="{gx}" y2="{H}" '
            f'stroke="#F0F0F0" stroke-width="1"/>'
        )
    for gy in range(0, H, 40):
        parts.append(
            f'<line x1="0" y1="{gy}" x2="{W}" y2="{gy}" '
            f'stroke="#F0F0F0" stroke-width="1"/>'
        )
    # border
    parts.append(
        f'<rect width="{W}" height="{H}" fill="none" '
        f'stroke="#CCCCCC" stroke-width="1"/>'
    )

    # ── edges first (drawn behind nodes) ─────────────────────────────────────
    def rect_intercept(ox, oy, px, py, w, h):
        """Nearest point on rectangle (px,py,w,h) toward (ox,oy)."""
        dx = ox - px; dy = oy - py
        d  = math.sqrt(dx*dx + dy*dy) + 0.001
        # scale to hit the rectangle edge
        sx = (w/2) / (abs(dx)/d + 1e-9)
        sy = (h/2) / (abs(dy)/d + 1e-9)
        t  = min(sx, sy)
        return px + dx/d*t, py + dy/d*t

    for edge in graph.edges:
        if edge.source not in pos or edge.target not in pos:
            continue
        sx, sy = pos[edge.source]
        tx, ty = pos[edge.target]
        top2   = _top_relations(edge.relations, 2)
        pcol   = EDGE_FAMILY_COLOUR.get(top2[0] if top2 else "", "#888888")
        cid    = pcol.replace("#", "a")

        dx = tx - sx; dy = ty - sy
        d  = math.sqrt(dx*dx + dy*dy) + 0.001
        ux, uy = dx/d, dy/d

        has_rev = any(e.source == edge.target and e.target == edge.source
                      for e in graph.edges)
        bend = 0.18 if has_rev else 0.06

        mx  = (sx + tx) / 2
        my  = (sy + ty) / 2
        cpx = mx - uy * d * bend
        cpy = my + ux * d * bend

        # intercept rectangle edges
        ax0, ay0 = rect_intercept(cpx, cpy, sx, sy, NW, NH)
        ax1, ay1 = rect_intercept(cpx, cpy, tx, ty, NW+8, NH+8)

        # edge line — thin, precise
        parts.append(
            f'<path d="M{ax0:.1f},{ay0:.1f} Q{cpx:.1f},{cpy:.1f} {ax1:.1f},{ay1:.1f}" '
            f'fill="none" stroke="{pcol}" stroke-width="1.2" opacity="0.75" '
            f'marker-end="url(#{cid})"/>'
        )

        # relation labels — small italic, no background box
        t    = 0.5
        lx   = (1-t)**2*ax0 + 2*(1-t)*t*cpx + t**2*ax1
        ly   = (1-t)**2*ay0 + 2*(1-t)*t*cpy + t**2*ay1
        tx2  = 2*(1-t)*(cpx-ax0) + 2*t*(ax1-cpx)
        ty2  = 2*(1-t)*(cpy-ay0) + 2*t*(ay1-cpy)
        td   = math.sqrt(tx2*tx2 + ty2*ty2) + 0.001
        perpx = -ty2/td;  perpy = tx2/td
        lx += perpx * 12; ly += perpy * 12

        for k2, rel in enumerate(top2):
            rcol = EDGE_FAMILY_COLOUR.get(rel, "#666")
            txt  = rel.replace("_", " ")
            oy   = k2 * 12
            # hairline underline instead of background box — looks more technical
            tw = len(txt) * 5.4
            parts.append(
                f'<line x1="{lx-tw/2:.1f}" y1="{ly+oy+1:.1f}" '
                f'x2="{lx+tw/2:.1f}" y2="{ly+oy+1:.1f}" '
                f'stroke="{rcol}" stroke-width="0.6" opacity="0.5"/>'
            )
            parts.append(
                f'<text x="{lx:.1f}" y="{ly+oy:.1f}" '
                f'text-anchor="middle" font-family="monospace" font-size="9" '
                f'font-style="italic" fill="{rcol}" opacity="0.9">{txt}</text>'
            )

    # ── nodes — rounded rectangles ────────────────────────────────────────────
    for lb in labels:
        x, y  = pos[lb]
        node  = graph.nodes[lb]
        fill, stroke, sw = node_style(lb, node)
        is_goal = lb == graph.goal_label
        is_path = lb in path_obs
        nx = x - NW/2; ny = y - NH/2

        # drop shadow — very subtle
        parts.append(
            f'<rect x="{nx+2:.1f}" y="{ny+2:.1f}" width="{NW}" height="{NH}" '
            f'rx="4" fill="#CCCCCC" opacity="0.25"/>'
        )
        # node box
        parts.append(
            f'<rect x="{nx:.1f}" y="{ny:.1f}" width="{NW}" height="{NH}" '
            f'rx="4" fill="{fill}" stroke="{stroke}" stroke-width="{sw:.1f}"/>'
        )

        # top accent bar — role colour, thin
        bar_col = stroke
        parts.append(
            f'<rect x="{nx:.1f}" y="{ny:.1f}" width="{NW}" height="3" '
            f'rx="4" fill="{bar_col}" opacity="0.7"/>'
        )

        # label — primary text
        parts.append(
            f'<text x="{x:.1f}" y="{ny+16:.1f}" '
            f'text-anchor="middle" font-family="monospace" font-size="11" '
            f'font-weight="600" fill="#111111">{lb}</text>'
        )

        # secondary data lines — small monospace
        conf_txt = f"conf={node.conf:.2f}"
        frag_txt = f"frag={node.fragility:.1f}"
        reach_ch = "reach=Y" if node.reachable else "reach=N"
        reach_col = "#2D7A2D" if node.reachable else "#AA2222"

        parts.append(
            f'<text x="{x:.1f}" y="{ny+29:.1f}" '
            f'text-anchor="middle" font-family="monospace" font-size="8" '
            f'fill="#555555">{conf_txt}  {frag_txt}</text>'
        )
        parts.append(
            f'<text x="{x:.1f}" y="{ny+41:.1f}" '
            f'text-anchor="middle" font-family="monospace" font-size="8" '
            f'fill="{reach_col}">{reach_ch}</text>'
        )

        # GOAL / PATH badge — small pill top-right corner
        if is_goal:
            parts.append(
                f'<rect x="{nx+NW-32:.1f}" y="{ny-8:.1f}" width="30" height="12" '
                f'rx="6" fill="#C77B00"/>'
            )
            parts.append(
                f'<text x="{nx+NW-17:.1f}" y="{ny+0:.1f}" '
                f'text-anchor="middle" font-family="monospace" font-size="7" '
                f'fill="white" font-weight="bold">GOAL</text>'
            )
        elif is_path:
            parts.append(
                f'<rect x="{nx+NW-34:.1f}" y="{ny-8:.1f}" width="32" height="12" '
                f'rx="6" fill="#CC3333"/>'
            )
            parts.append(
                f'<text x="{nx+NW-18:.1f}" y="{ny+0:.1f}" '
                f'text-anchor="middle" font-family="monospace" font-size="7" '
                f'fill="white" font-weight="bold">PATH</text>'
            )

    # ── watermark / caption ───────────────────────────────────────────────────
    parts.append(
        f'<text x="{W-8}" y="{H-6}" text-anchor="end" '
        f'font-family="monospace" font-size="8" fill="#BBBBBB" '
        f'font-style="italic">ARGUS · scene graph</text>'
    )

    return (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="100%" height="100%" viewBox="0 0 {W} {H}" '
        f'preserveAspectRatio="xMidYMid meet">'
        + "".join(parts)
        + "</svg>"
    )


# ── HTML helpers ──────────────────────────────────────────────────────────────

def _fragility_bar(f):
    pct   = int(f * 100)
    r     = min(255, int(f * 2 * 255))
    g     = min(255, int((1-f) * 2 * 255))
    col   = f"rgb({r},{g},50)"
    return (
        f'<div style="display:inline-flex;align-items:center;gap:6px">'
        f'<div style="width:80px;background:#eee;border-radius:3px;'
        f'overflow:hidden;border:1px solid #ccc">'
        f'<div style="width:{pct}%;height:10px;background:{col}"></div>'
        f'</div><span style="font-size:11px;color:#555">{f:.2f}</span></div>'
    )

def _weight_bar(w):
    pct   = min(100, int(abs(w) / 10))
    col   = "#FFD166" if w > 0 else ("#E63946" if w <= -600 else "#4CC9F0")
    label = f"{int(w):+d}"
    return (
        f'<div style="display:flex;align-items:center;gap:6px">'
        f'<span style="font-family:monospace;font-size:12px;'
        f'width:52px;text-align:right">{label}</span>'
        f'<div style="width:100px;background:#eee;border-radius:3px;'
        f'overflow:hidden;border:1px solid #ccc">'
        f'<div style="width:{pct}%;height:12px;background:{col}"></div>'
        f'</div></div>'
    )

def _rel_badge(rel):
    col  = EDGE_FAMILY_COLOUR.get(rel, "#888")
    text = rel.replace("_"," ")
    return (
        f'<span style="display:inline-block;margin:2px;padding:2px 7px;'
        f'border-radius:10px;font-size:11px;color:white;background:{col};'
        f'white-space:nowrap">{text}</span>'
    )


def generate_html(
    graph,
    diff=None,
    save_path="/tmp/scene_graph.html",
    auto_refresh_sec=0,
) -> str:

    labels      = list(graph.nodes.keys())
    path_obs    = set(graph.get_path_obstacles())
    fragile_obs = set(graph.get_fragile_objects(0.5))
    suggested   = graph.suggest_default_weights()
    if graph.goal_label and graph.goal_label in graph.nodes:
        suggested[graph.goal_label] = 200.0
    clusters    = graph.get_cluster_summary()
    n_obj       = len(graph.nodes)
    n_edges     = len(graph.edges)
    n_rels      = sum(len(e.relations) for e in graph.edges)
    ts          = datetime.fromtimestamp(graph.timestamp).strftime("%H:%M:%S")

    # ── static SVG graph ──────────────────────────────────────────────────────
    topdown_svg = _svg_topdown(graph)
    svg_graph   = _svg_graph(graph)

    # ── objects table ─────────────────────────────────────────────────────────
    obj_rows = ""
    for label, node in sorted(graph.nodes.items(),
                               key=lambda x: x[1].dist_robot):
        x, y, z   = node.pos
        is_goal   = label == graph.goal_label
        is_path   = label in path_obs
        is_frag   = label in fragile_obs
        row_bg    = "#fff9e6" if is_goal else (
                    "#fff0f0" if is_path or is_frag else "#ffffff")
        badges    = ""
        if is_goal: badges += '<span class="badge goal">GOAL</span>'
        if is_path: badges += '<span class="badge path">ON PATH</span>'
        if is_frag: badges += '<span class="badge frag">FRAGILE</span>'
        cats      = ", ".join(node.categories) if node.categories else "—"
        w         = int(suggested.get(label, 0))
        obj_rows += f"""
        <tr style="background:{row_bg}">
          <td><b>{label}</b>{badges}</td>
          <td style="font-family:monospace">{x:+.3f}, {y:+.3f}, {z:+.3f}</td>
          <td>{node.conf:.2f}</td>
          <td>{_fragility_bar(node.fragility)}</td>
          <td style="text-align:center">{"✓" if node.reachable else "✗"}</td>
          <td>{node.dist_robot:.2f}m</td>
          <td style="font-size:11px;color:#555">{cats}</td>
          <td>{_weight_bar(w)}</td>
        </tr>"""

    # ── relations table ───────────────────────────────────────────────────────
    rel_rows = ""
    for edge in sorted(graph.edges, key=lambda e: e.dist_2d):
        badges  = "".join(_rel_badge(r) for r in edge.relations)
        is_crit = any(r in ("on_path_to_goal","blocking_goal","may_occlude")
                      for r in edge.relations)
        rel_rows += f"""
        <tr style="background:{"#fff0f0" if is_crit else "#fff"}">
          <td><b>{edge.source}</b></td>
          <td style="text-align:center;color:#aaa">→</td>
          <td><b>{edge.target}</b></td>
          <td>{badges}</td>
          <td style="font-family:monospace">{edge.dist_2d:.3f}m</td>
          <td style="font-family:monospace">{edge.dist_3d:.3f}m</td>
        </tr>"""

    # ── path panel ────────────────────────────────────────────────────────────
    path_html = ""
    if path_obs:
        items = "".join(
            f'<li><b>{p}</b> — fragility {graph.nodes[p].fragility:.2f}, '
            f'dist to robot {graph.nodes[p].dist_robot:.2f}m</li>'
            for p in path_obs
        )
        path_html = f"""
        <div class="panel" style="border-left:4px solid #E63946">
          <h2 style="color:#E63946">Path Obstacles</h2>
          <p style="color:#666;font-size:12px;margin-bottom:8px">
            Objects between robot and goal. Planner must avoid or relax these.
          </p>
          <ul style="margin:0;padding-left:20px;line-height:2">{items}</ul>
        </div>"""

    # ── diff panel ────────────────────────────────────────────────────────────
    diff_html = ""
    if diff is not None:
        items = ""
        for lb in diff.appeared:
            items += f'<li style="color:#2DC653">+ {lb} appeared</li>'
        for lb in diff.disappeared:
            items += f'<li style="color:#E63946">− {lb} disappeared</li>'
        for lb, (old, new, delta) in diff.moved.items():
            items += (f'<li style="color:#FB8500">~ {lb} moved '
                      f'{delta*100:.1f}cm: '
                      f'({old[0]:.2f},{old[1]:.2f}) → '
                      f'({new[0]:.2f},{new[1]:.2f})</li>')
        if not items:
            items = '<li style="color:#aaa">No significant changes</li>'
        diff_html = f"""
        <div class="panel">
          <h2>Scene Diff</h2>
          <ul style="margin:0;padding-left:20px;line-height:2">{items}</ul>
        </div>"""

    # ── cluster panel ─────────────────────────────────────────────────────────
    cluster_html = ""
    if clusters:
        items = "".join(
            f'<li>{", ".join(c)} ({len(c)} objects within 15cm)</li>'
            for c in clusters
        )
        cluster_html = f"""
        <div class="panel">
          <h2>Object Clusters</h2>
          <ul style="margin:0;padding-left:20px;line-height:2">{items}</ul>
        </div>"""

    # ── raw JSON ──────────────────────────────────────────────────────────────
    raw_json = json.dumps(graph.to_dict(), indent=2)

    refresh = f'<meta http-equiv="refresh" content="{auto_refresh_sec}">' \
              if auto_refresh_sec else ""

    # ── legend entries ────────────────────────────────────────────────────────
    legend_items = "".join(
        f'<span style="display:inline-flex;align-items:center;gap:4px;'
        f'font-size:11px;margin-right:12px">'
        f'<span style="width:12px;height:12px;border-radius:50%;'
        f'background:{c};display:inline-block"></span>{fam}</span>'
        for fam, c in [
            ("path/blocking","#E63946"),("near/close","#4361EE"),
            ("left/right","#2DC653"),("front/behind","#0096C7"),
            ("above/below","#FB8500"),("may occlude","#7B2D8B"),
            ("robot dist","#6D6875"),
        ]
    )

    # ══════════════════════════════════════════════════════════════════════════
    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">{refresh}
<title>ARGUS Scene Graph — {ts}</title>
<style>
  *{{box-sizing:border-box;margin:0;padding:0}}
  body{{font-family:"Courier New",monospace;font-size:13px;
        background:#f5f5f5;color:#222;overflow-x:hidden}}
  header{{background:#1a1a2e;color:white;padding:10px 22px;height:52px;
          display:flex;justify-content:space-between;align-items:center}}
  header h1{{font-size:17px;letter-spacing:2px}}
  header .meta{{font-size:11px;color:#aaa;text-align:right}}

  /* ── hero grid: fills viewport below header ── */
  /* left half: top-down (top) + scene graph (bottom), each 50% */
  /* right half: spatial relations (75%) + detected objects (25%) */
  .hero{{
    display:grid;
    grid-template-columns:1fr 1fr;
    grid-template-rows:1fr;
    height:calc(100vh - 52px);
    gap:8px;
    padding:8px;
    background:#f0f0f0;
  }}
  .hero-left{{
    display:grid;
    grid-template-rows:1fr 1fr;
    gap:8px;
    min-height:0;
  }}
  .hero-right{{
    display:grid;
    grid-template-rows:3fr 1fr;
    gap:8px;
    min-height:0;
  }}
  .hero-panel{{
    background:white;
    border:1px solid #ddd;
    border-radius:4px;
    display:flex;
    flex-direction:column;
    overflow:hidden;
    min-height:0;
  }}
  .hero-panel h2{{
    flex-shrink:0;
    font-size:11px;letter-spacing:1px;text-transform:uppercase;
    color:#333;border-bottom:1px solid #eee;
    padding:10px 14px 8px;margin:0;
  }}
  .hero-panel .body{{
    flex:1;overflow:auto;min-height:0;padding:0 14px 14px;
  }}
  .hero-panel .svg-body{{
    flex:1;overflow:hidden;min-height:0;
    display:flex;align-items:stretch;
  }}
  .hero-panel .svg-body > svg{{width:100%;height:100%}}

  /* ── rest of page (scroll to reveal) ── */
  .rest{{
    display:grid;
    grid-template-columns:1fr 1fr;
    gap:14px;
    padding:14px;
  }}
  .full{{grid-column:1/-1}}
  .panel{{background:white;border:1px solid #ddd;
          border-radius:4px;padding:14px}}
  h2{{font-size:12px;letter-spacing:1px;text-transform:uppercase;
      color:#333;border-bottom:1px solid #eee;padding-bottom:7px;
      margin-bottom:11px}}
  table{{width:100%;border-collapse:collapse;font-size:12px}}
  th{{background:#f0f0f0;padding:5px 9px;text-align:left;
      border-bottom:2px solid #ddd;font-size:10px;
      letter-spacing:.5px;text-transform:uppercase;color:#555}}
  td{{padding:5px 9px;border-bottom:1px solid #f0f0f0;
      vertical-align:middle}}
  tr:last-child td{{border-bottom:none}}
  .badge{{display:inline-block;padding:1px 5px;border-radius:3px;
          font-size:10px;font-weight:bold;margin-left:3px}}
  .badge.goal{{background:#FFD166;color:#333}}
  .badge.path{{background:#E63946;color:white}}
  .badge.frag{{background:#7B2D8B;color:white}}
  pre{{background:#1a1a2e;color:#a8ff78;padding:14px;border-radius:4px;
       overflow:auto;font-size:11px;max-height:380px}}
  .legend{{margin-top:8px;line-height:2;font-size:11px}}
  .scroll-hint{{text-align:center;padding:6px;font-size:11px;
                color:#aaa;background:#f0f0f0;letter-spacing:1px}}
</style>
</head>
<body>

<header>
  <div>
    <h1>ARGUS — SCENE GRAPH</h1>
    <div style="font-size:11px;color:#888;margin-top:3px">
      goal: <b style="color:#FFD166">{graph.goal_label or "none"}</b>
      &nbsp;|&nbsp; robot: ({graph.robot_pos[0]:.2f}, {graph.robot_pos[1]:.2f})
      &nbsp;|&nbsp; camera: ({graph.camera_pos[0]:.2f}, {graph.camera_pos[1]:.2f}, {graph.camera_pos[2]:.2f})
    </div>
  </div>
  <div class="meta">
    {n_obj} objects &nbsp;·&nbsp; {n_edges} edges &nbsp;·&nbsp; {n_rels} relations<br>
    {ts}{f" &nbsp;·&nbsp; auto-refresh {auto_refresh_sec}s" if auto_refresh_sec else ""}
  </div>
</header>

<!-- ══ HERO: fills viewport on landing ══ -->
<div class="hero">

  <!-- LEFT COLUMN: top-down (50%) + scene graph (50%) -->
  <div class="hero-left">

    <div class="hero-panel">
      <h2>Table — Top Down View</h2>
      <div class="svg-body">{topdown_svg}</div>
    </div>

    <div class="hero-panel">
      <h2>Scene Graph &nbsp;<span style="font-weight:normal;color:#999;font-size:10px">dot=fragility · size=confidence · border=fragility</span></h2>
      <div class="svg-body">{svg_graph}</div>
    </div>

  </div>

  <!-- RIGHT COLUMN: spatial relations (75%) + detected objects (25%) -->
  <div class="hero-right">

    <div class="hero-panel">
      <h2>Spatial Relations &nbsp;<span style="font-weight:normal;color:#999;font-size:10px">{n_rels} total</span></h2>
      <div class="body">
        <table>
          <thead><tr>
            <th>From</th><th></th><th>To</th>
            <th>Relations</th><th>2D</th><th>3D</th>
          </tr></thead>
          <tbody>{rel_rows}</tbody>
        </table>
      </div>
    </div>

    <div class="hero-panel">
      <h2>Detected Objects &nbsp;<span style="font-weight:normal;color:#999;font-size:10px">{n_obj} detected</span></h2>
      <div class="body">
        <table>
          <thead><tr>
            <th>Label</th><th>Pos (x,y,z)</th><th>Conf</th>
            <th>Fragility</th><th>Reach</th><th>Dist</th>
            <th>Categories</th><th>Weight</th>
          </tr></thead>
          <tbody>{obj_rows}</tbody>
        </table>
      </div>
    </div>

  </div>

</div>

<!-- scroll hint -->
<div class="scroll-hint">▼ &nbsp; scroll for path analysis · clusters · diff · raw json &nbsp; ▼</div>

<!-- ══ REST: revealed on scroll ══ -->
<div class="rest">

  <!-- PATH + DIFF + CLUSTERS -->
  <div>{path_html}</div>
  <div>{diff_html}{cluster_html}</div>

  <!-- EDGE LEGEND -->
  <div class="panel full">
    <h2>Edge Colour Legend</h2>
    <div class="legend">{legend_items}</div>
  </div>

  <!-- RAW JSON -->
  <div class="panel full">
    <h2>Raw JSON</h2>
    <pre>{raw_json}</pre>
  </div>

</div>
</body>
</html>"""

    with open(save_path, "w") as f:
        f.write(html)
    print(f"Saved → {save_path}  |  open {save_path}")
    return save_path


# ─────────────────────────────────────────────────────────────────────────────
# Demo — run directly to generate the HTML from hardcoded positions
# Positions match the actual SDF scene (table_scene_shapes.sdf)
# In the full pipeline, these come from orchestrator_node → YOLO → depth
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":

    from argus_scene_graph.scene_graph import SceneGraphBuilder, Detection

    builder = SceneGraphBuilder(
        robot_pos  = (-0.5, 0.0, 0.0),
        camera_pos = (0.5, -1.4, 0.825),
    )

    # positions match the SDF — left to right: bottle, vase, cup, boot, ball
    detections = [
        Detection(label="bottle",      pos=(-0.38, 0.18, 0.15), conf=0.85),
        Detection(label="vase",        pos=(-0.14, 0.26, 0.10), conf=0.72),
        Detection(label="cup",         pos=(0.10,  0.22, 0.09), conf=0.52),
        Detection(label="boot",        pos=(0.42,  0.18, 0.12), conf=0.07),
        Detection(label="sports ball", pos=(0.55,  0.28, 0.08), conf=0.79),
    ]

    graph, diff = builder.build(
        detections = detections,
        goal_label = "sports ball",
    )

    generate_html(
        graph     = graph,
        diff      = diff,
        save_path = "/tmp/scene_graph.html",
    )
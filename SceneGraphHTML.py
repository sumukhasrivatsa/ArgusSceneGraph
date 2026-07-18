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
    Research paper style top-down view.
    White background, minimal black/grey, clean lines.
    Looks like a robotics paper figure.
    """
    W, H     = 560, 400
    PAD      = 50
    NODE_R   = 12
    WX_MIN, WX_MAX = -0.90, 1.20
    WY_MIN, WY_MAX = -0.70, 0.70

    def wx(x): return PAD + (x-WX_MIN)/(WX_MAX-WX_MIN)*(W-2*PAD)
    def wy(y): return (H-PAD) - (y-WY_MIN)/(WY_MAX-WY_MIN)*(H-2*PAD)

    path_obs  = set(graph.get_path_obstacles())
    labels    = list(graph.nodes.keys())
    label_idx = {lb:i for i,lb in enumerate(labels)}

    parts = []
    # white background
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')
    # axis tick labels
    for v,lbl in [(-0.5,"−0.5m"),(0.0,"0"),(0.5,"+0.5m"),(1.0,"+1.0m")]:
        if WX_MIN<=v<=WX_MAX:
            parts.append(f'<text x="{wx(v):.1f}" y="{H-4}" text-anchor="middle" font-family="monospace" font-size="8" fill="#999">{lbl}</text>')
    for v,lbl in [(-0.5,"−0.5"),(0.0,"0"),(0.3,"+0.3")]:
        if WY_MIN<=v<=WY_MAX:
            parts.append(f'<text x="4" y="{wy(v)+3:.1f}" text-anchor="start" font-family="monospace" font-size="8" fill="#999">{lbl}</text>')
    # axis lines
    parts.append(f'<line x1="{PAD}" y1="{H-PAD}" x2="{W-PAD}" y2="{H-PAD}" stroke="#CCCCCC" stroke-width="0.8"/>')
    parts.append(f'<line x1="{PAD}" y1="{PAD}" x2="{PAD}" y2="{H-PAD}" stroke="#CCCCCC" stroke-width="0.8"/>')

    # table outline — thin dashed rectangle
    tx1,ty1=wx(-0.70),wy(0.45); tx2,ty2=wx(1.10),wy(-0.45)
    parts.append(f'<rect x="{tx1:.1f}" y="{ty1:.1f}" width="{tx2-tx1:.1f}" height="{ty2-ty1:.1f}" fill="#F9F9F9" stroke="#AAAAAA" stroke-width="1" stroke-dasharray="6,3" rx="3"/>')
    parts.append(f'<text x="{tx1+4:.1f}" y="{ty1+12:.1f}" font-family="monospace" font-size="8" fill="#AAAAAA">table</text>')

    # robot reach circle — thin dashed
    rx_,ry_=wx(graph.robot_pos[0]),wy(graph.robot_pos[1])
    ROBOT_REACH_RADIUS = 0.85
    reach_px=ROBOT_REACH_RADIUS*(W-2*PAD)/(WX_MAX-WX_MIN)
    parts.append(f'<circle cx="{rx_:.1f}" cy="{ry_:.1f}" r="{reach_px:.1f}" fill="none" stroke="#AAAAAA" stroke-width="0.7" stroke-dasharray="4,3"/>')

    # path robot→goal — thin dashed arrow
    goal_node=graph.nodes.get(graph.goal_label)
    if goal_node:
        gx2=wx(goal_node.pos[0]); gy2=wy(goal_node.pos[1])
        dx=gx2-rx_;dy=gy2-ry_;d=math.sqrt(dx*dx+dy*dy)+0.001
        ax0=rx_+dx/d*NODE_R; ay0=ry_+dy/d*NODE_R
        ax1=gx2-dx/d*(NODE_R+6); ay1=gy2-dy/d*(NODE_R+6)
        parts.append(f'<defs><marker id="td-a" markerWidth="7" markerHeight="5" refX="7" refY="2.5" orient="auto"><polygon points="0 0,7 2.5,0 5" fill="#333"/></marker></defs>')
        parts.append(f'<line x1="{ax0:.1f}" y1="{ay0:.1f}" x2="{ax1:.1f}" y2="{ay1:.1f}" stroke="#333" stroke-width="1" stroke-dasharray="5,3" marker-end="url(#td-a)" opacity="0.6"/>')

    # faint edges between objects
    for edge in graph.edges:
        if edge.source not in graph.nodes or edge.target not in graph.nodes: continue
        ap=graph.nodes[edge.source].pos; bp=graph.nodes[edge.target].pos
        parts.append(f'<line x1="{wx(ap[0]):.1f}" y1="{wy(ap[1]):.1f}" x2="{wx(bp[0]):.1f}" y2="{wy(bp[1]):.1f}" stroke="#DDDDDD" stroke-width="0.6"/>')

    # object nodes — clean circles, minimal color
    for lb in labels:
        node=graph.nodes[lb]; x,y=wx(node.pos[0]),wy(node.pos[1])
        is_goal=lb==graph.goal_label; is_path=lb in path_obs
        size=NODE_R+int(node.conf*5)

        if is_goal:
            fill="#222222"; stroke="#000000"; tc="white"; sw=2
        elif is_path:
            fill="#666666"; stroke="#333333"; tc="white"; sw=1.5
        else:
            fill="white"; stroke="#555555"; tc="#222"; sw=1.2

        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{size}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')
        # label above
        parts.append(f'<text x="{x:.1f}" y="{y-size-4:.1f}" text-anchor="middle" font-family="monospace" font-size="9" fill="#222">{lb}</text>')
        # small reach tick
        reach_ch="✓" if node.reachable else "✗"
        parts.append(f'<text x="{x:.1f}" y="{y+size+10:.1f}" text-anchor="middle" font-family="monospace" font-size="7" fill="#888">{reach_ch}</text>')

    # robot marker — small filled circle with label
    parts.append(f'<circle cx="{rx_:.1f}" cy="{ry_:.1f}" r="7" fill="#333" stroke="#000" stroke-width="1.5"/>')
    parts.append(f'<text x="{rx_:.1f}" y="{ry_-11:.1f}" text-anchor="middle" font-family="monospace" font-size="8" fill="#333">robot</text>')

    # camera marker — small square
    cx2=wx(graph.camera_pos[0]); cy2=wy(graph.camera_pos[1])
    parts.append(f'<rect x="{cx2-5:.1f}" y="{cy2-5:.1f}" width="10" height="10" fill="#888" stroke="#555" stroke-width="1"/>')
    parts.append(f'<text x="{cx2:.1f}" y="{cy2-9:.1f}" text-anchor="middle" font-family="monospace" font-size="8" fill="#888">cam</text>')

    # legend — bottom right
    parts.append(f'<text x="{W-PAD}" y="{H-PAD+18}" text-anchor="end" font-family="monospace" font-size="7.5" fill="#AAA">● goal (black)  ○ obstacle  ✓ reachable  - - reach limit</text>')

    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="100%" height="100%" viewBox="0 0 {W} {H}" preserveAspectRatio="xMidYMid meet">'
            + "".join(parts) + "</svg>")



def _svg_graph(graph) -> str:
    """
    Research paper style scene graph.
    RED rounded rectangles  = object nodes
    BLUE rounded rectangles = attribute + relation nodes
    BLACK arrows            = connections
    WHITE background        = clean
    """
    W, H = 1000, 660
    labels = list(graph.nodes.keys())
    n = len(labels)
    if n == 0:
        return "<svg></svg>"

    pos = _spring_layout(labels, graph.edges, W, H)
    PAD_X, PAD_Y = 130, 110
    if len(pos) > 1:
        min_x=min(p[0] for p in pos.values()); max_x=max(p[0] for p in pos.values())
        min_y=min(p[1] for p in pos.values()); max_y=max(p[1] for p in pos.values())
        rx=max(max_x-min_x,1); ry=max(max_y-min_y,1)
        for lb in pos:
            pos[lb][0]=PAD_X+(pos[lb][0]-min_x)/rx*(W-2*PAD_X)
            pos[lb][1]=PAD_Y+(pos[lb][1]-min_y)/ry*(H-2*PAD_Y)

    path_obs  = set(graph.get_path_obstacles())
    suggested = graph.suggest_default_weights()
    if graph.goal_label and graph.goal_label in graph.nodes:
        suggested[graph.goal_label] = 200.0

    cx_all = sum(pos[lb][0] for lb in labels)/max(n,1)
    cy_all = sum(pos[lb][1] for lb in labels)/max(n,1)

    OBJ_W, OBJ_H = 88, 28
    ATR_W, ATR_H = 80, 22

    # colours
    OBJ_FILL   = "#E63946"; OBJ_STROKE = "#9B1B24"; OBJ_TC = "white"
    GOAL_FILL  = "#C1121F"; GOAL_STROKE= "#7A0000"; GOAL_TC = "white"
    ATR_FILL   = "#4895EF"; ATR_STROKE = "#1D5CA6"; ATR_TC  = "white"
    REL_FILL   = "#4361EE"; REL_STROKE = "#1B3BA0"; REL_TC  = "white"
    ARROW_COL  = "#222222"

    def attrs_for(lb, node):
        w    = int(suggested.get(lb, 0))
        items = [
            f"conf={node.conf:.2f}",
            f"frag={node.fragility:.2f}",
            f"dist={node.dist_robot:.2f}m",
            f"reach={'Y' if node.reachable else 'N'}",
        ]
        if node.categories:
            items.append(node.categories[0])
        items.append(f"w={w:+d}")
        return items

    def attr_positions(ox, oy, count):
        dx=ox-cx_all; dy=oy-cy_all
        d=math.sqrt(dx*dx+dy*dy)+0.001
        ux,uy=dx/d,dy/d
        base=math.atan2(uy,ux)
        spread=math.pi*0.75
        out=[]
        for i in range(count):
            a=base-spread/2+spread*i/max(count-1,1) if count>1 else base
            r=78+(i%2)*16
            ax=ox+math.cos(a)*r; ay=oy+math.sin(a)*r
            ax=max(ATR_W/2+4,min(W-ATR_W/2-4,ax))
            ay=max(ATR_H/2+4,min(H-ATR_H/2-4,ay))
            out.append((ax,ay))
        return out

    def box_intercept(ox,oy,tx,ty,bw,bh):
        dx=tx-ox;dy=ty-oy;d=math.sqrt(dx*dx+dy*dy)+0.001
        sx=(bw/2)/(abs(dx)/d+1e-9); sy=(bh/2)/(abs(dy)/d+1e-9)
        t=min(sx,sy)
        return ox+dx/d*t, oy+dy/d*t

    parts = []
    # white background
    parts.append(f'<rect width="{W}" height="{H}" fill="white"/>')
    # light grey grid
    for gx in range(0,W,40):
        parts.append(f'<line x1="{gx}" y1="0" x2="{gx}" y2="{H}" stroke="#F2F2F2" stroke-width="1"/>')
    for gy in range(0,H,40):
        parts.append(f'<line x1="0" y1="{gy}" x2="{W}" y2="{gy}" stroke="#F2F2F2" stroke-width="1"/>')
    parts.append(f'<rect width="{W}" height="{H}" fill="none" stroke="#DDDDDD" stroke-width="1"/>')

    # arrowhead marker
    parts.append(f'<defs><marker id="arr" markerWidth="8" markerHeight="6" refX="8" refY="3" orient="auto"><polygon points="0 0,8 3,0 6" fill="{ARROW_COL}"/></marker></defs>')

    # STEP 1 — attribute connector lines
    for lb in labels:
        ox,oy=pos[lb]; node=graph.nodes[lb]
        attrs=attrs_for(lb,node); apos=attr_positions(ox,oy,len(attrs))
        for (ax,ay) in apos:
            parts.append(f'<line x1="{ox:.1f}" y1="{oy:.1f}" x2="{ax:.1f}" y2="{ay:.1f}" stroke="#BBBBBB" stroke-width="0.9" stroke-dasharray="3,2"/>')

    # STEP 2 — relation arrows between objects
    for edge in graph.edges:
        if edge.source not in pos or edge.target not in pos: continue
        sx,sy=pos[edge.source]; tx,ty=pos[edge.target]
        top_rels=_top_relations(edge.relations,3)
        if not top_rels: continue
        for k,rel in enumerate(top_rels):
            has_rev=any(e.source==edge.target and e.target==edge.source for e in graph.edges)
            bend=(0.22 if has_rev else 0.09)+k*0.07
            dx=tx-sx;dy=ty-sy;d=math.sqrt(dx*dx+dy*dy)+0.001
            ux,uy=dx/d,dy/d; px2,py2=-uy,ux
            offset=(k-len(top_rels)/2)*16
            mx=(sx+tx)/2+px2*d*bend+px2*offset
            my=(sy+ty)/2+py2*d*bend+py2*offset
            ax0,ay0=box_intercept(sx,sy,mx,my,OBJ_W,OBJ_H)
            ax1,ay1=box_intercept(tx,ty,mx,my,OBJ_W,OBJ_H)
            dist_end=math.sqrt((mx-ax1)**2+(my-ay1)**2)+0.001
            ex=ax1+(mx-ax1)/dist_end*10; ey=ay1+(my-ay1)/dist_end*10
            parts.append(f'<path d="M{ax0:.1f},{ay0:.1f} Q{mx:.1f},{my:.1f} {ex:.1f},{ey:.1f}" fill="none" stroke="{ARROW_COL}" stroke-width="1.2" opacity="0.7" marker-end="url(#arr)"/>')
            # relation label — BLUE box on the arrow
            t=0.5; lx=(1-t)**2*ax0+2*(1-t)*t*mx+t**2*ex; ly=(1-t)**2*ay0+2*(1-t)*t*my+t**2*ey
            txt=rel.replace("_"," "); tw=len(txt)*5.4+10
            parts.append(f'<rect x="{lx-tw/2:.1f}" y="{ly-ATR_H/2:.1f}" width="{tw:.1f}" height="{ATR_H}" rx="11" fill="{REL_FILL}" stroke="{REL_STROKE}" stroke-width="0.8"/>')
            parts.append(f'<text x="{lx:.1f}" y="{ly+4:.1f}" text-anchor="middle" font-family="monospace" font-size="8.5" fill="{REL_TC}" font-style="italic">{txt}</text>')

    # STEP 3 — attribute boxes (BLUE)
    for lb in labels:
        ox,oy=pos[lb]; node=graph.nodes[lb]
        attrs=attrs_for(lb,node); apos=attr_positions(ox,oy,len(attrs))
        for (txt,(ax,ay)) in zip(attrs,apos):
            tw=max(ATR_W,len(txt)*6+10)
            parts.append(f'<rect x="{ax-tw/2:.1f}" y="{ay-ATR_H/2:.1f}" width="{tw:.1f}" height="{ATR_H}" rx="11" fill="{ATR_FILL}" stroke="{ATR_STROKE}" stroke-width="0.8"/>')
            parts.append(f'<text x="{ax:.1f}" y="{ay+4:.1f}" text-anchor="middle" font-family="monospace" font-size="9" fill="{ATR_TC}">{txt}</text>')

    # STEP 4 — object boxes (RED, on top)
    for lb in labels:
        ox,oy=pos[lb]; node=graph.nodes[lb]
        is_goal=lb==graph.goal_label
        fill=GOAL_FILL if is_goal else OBJ_FILL
        stroke=GOAL_STROKE if is_goal else OBJ_STROKE
        tc=GOAL_TC
        parts.append(f'<rect x="{ox-OBJ_W/2+2:.1f}" y="{oy-OBJ_H/2+2:.1f}" width="{OBJ_W}" height="{OBJ_H}" rx="14" fill="#999" opacity="0.18"/>')
        parts.append(f'<rect x="{ox-OBJ_W/2:.1f}" y="{oy-OBJ_H/2:.1f}" width="{OBJ_W}" height="{OBJ_H}" rx="14" fill="{fill}" stroke="{stroke}" stroke-width="2"/>')
        parts.append(f'<text x="{ox:.1f}" y="{oy+5:.1f}" text-anchor="middle" font-family="monospace" font-size="12" font-weight="bold" fill="{tc}">{lb}</text>')

    parts.append(f'<text x="{W-6}" y="{H-5}" text-anchor="end" font-family="monospace" font-size="8" fill="#CCCCCC" font-style="italic">ARGUS · scene graph</text>')

    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="100%" height="100%" viewBox="0 0 {W} {H}" preserveAspectRatio="xMidYMid meet">'
            + "".join(parts) + "</svg>")




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
        x, y, z = node.pos
        is_goal  = label == graph.goal_label
        is_path  = label in path_obs
        is_frag  = label in fragile_obs
        tags = ""
        if is_goal: tags += '<span class="tag goal">goal</span>'
        if is_path: tags += '<span class="tag path">path</span>'
        if is_frag: tags += '<span class="tag frag">fragile</span>'
        cats = ", ".join(node.categories) if node.categories else "—"
        w    = int(suggested.get(label, 0))
        w_str = f"{w:+d}"
        row_cls = 'goal-row' if is_goal else ('crit' if is_path else '')
        obj_rows += f"""
        <tr class="{row_cls}">
          <td><b>{label}</b>{tags}</td>
          <td>({x:+.3f},&thinsp;{y:+.3f},&thinsp;{z:+.3f})</td>
          <td>{node.conf:.3f}</td>
          <td>{node.fragility:.2f}</td>
          <td>{"Y" if node.reachable else "N"}</td>
          <td>{node.dist_robot:.3f}</td>
          <td>{cats}</td>
          <td>{w_str}</td>
        </tr>"""

    # ── relations table ───────────────────────────────────────────────────────
    REL_COLORS = {
        # path — soft rose
        "on_path_to_goal":          ("#FFE4E6", "#9F1239"),
        "blocking_goal":            ("#FFE4E6", "#9F1239"),
        "target_on_path_to_goal":   ("#FFE4E6", "#9F1239"),
        # proximity — soft violet
        "near":                     ("#EDE9FE", "#5B21B6"),
        "close":                    ("#EDE9FE", "#5B21B6"),
        "moderate_distance":        ("#F5F3FF", "#6D28D9"),
        "far_from":                 ("#F5F3FF", "#6D28D9"),
        # lateral — soft green
        "left_of":                  ("#DCFCE7", "#166534"),
        "right_of":                 ("#DCFCE7", "#166534"),
        "directly_left_of":         ("#DCFCE7", "#166534"),
        "directly_right_of":        ("#DCFCE7", "#166534"),
        # depth — soft sky
        "in_front_of":              ("#E0F2FE", "#0C4A6E"),
        "behind":                   ("#E0F2FE", "#0C4A6E"),
        "directly_ahead":           ("#E0F2FE", "#0C4A6E"),
        "directly_behind":          ("#E0F2FE", "#0C4A6E"),
        # vertical — soft amber
        "above":                    ("#FEF3C7", "#92400E"),
        "below":                    ("#FEF3C7", "#92400E"),
        "same_level":               ("#FEF3C7", "#92400E"),
        "stacked_on":               ("#FEF3C7", "#92400E"),
        # occlusion — soft purple
        "may_occlude":              ("#F3E8FF", "#6B21A8"),
        # robot-centric — soft slate
        "closer_to_robot_than":     ("#F1F5F9", "#334155"),
        "further_from_robot_than":  ("#F1F5F9", "#334155"),
        "more_reachable_than":      ("#F1F5F9", "#334155"),
        "less_reachable_than":      ("#F1F5F9", "#334155"),
        "clear_of_path":            ("#F1F5F9", "#334155"),
    }
    rel_rows = ""
    for edge in sorted(graph.edges, key=lambda e: e.dist_2d):
        is_crit = any(r in ("on_path_to_goal","blocking_goal","may_occlude")
                      for r in edge.relations)
        badges = ""
        for r in edge.relations:
            txt = r.replace("_"," ")
            bg, tc = REL_COLORS.get(r, ("#F1F5F9", "#334155"))
            badges += (f'<span style="display:inline-block;margin:2px 3px;'
                       f'padding:2px 7px;border-radius:4px;font-size:9px;'
                       f'font-weight:600;background:{bg};color:{tc};'
                       f'white-space:nowrap">{txt}</span>')
        row_cls2 = "crit" if is_crit else ""
        rel_rows += f"""
        <tr class="{row_cls2}">
          <td><b>{edge.source}</b></td>
          <td style="color:var(--muted);text-align:center">→</td>
          <td><b>{edge.target}</b></td>
          <td style="line-height:2.2">{badges}</td>
          <td>{edge.dist_2d:.3f}</td>
          <td>{edge.dist_3d:.3f}</td>
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
  :root{{
    --bg:       #FFFFFF;
    --surface:  #F9FAFB;
    --border:   #E5E7EB;
    --ink:      #111827;
    --body:     #374151;
    --muted:    #9CA3AF;
    --indigo:   #4F46E5;
    --indigo-l: #EEF2FF;
    --teal:     #0F766E;
    --teal-l:   #F0FDFA;
    --amber:    #B45309;
    --amber-l:  #FFFBEB;
  }}
  html, body{{height:100%}}
  body{{
    font-family: -apple-system, "Segoe UI", Helvetica, Arial, sans-serif;
    font-size:13px;background:var(--surface);color:var(--body);
    overflow-x:hidden;
  }}
  header{{
    background:var(--ink);color:white;
    padding:0 28px;height:56px;
    display:flex;justify-content:space-between;align-items:center;
  }}
  header h1{{
    font-size:12px;letter-spacing:5px;font-weight:700;
    text-transform:uppercase;color:white;font-family:"Courier New",monospace;
  }}
  header .pill{{
    display:inline-block;background:var(--indigo);
    color:white;font-size:10px;font-weight:600;
    padding:2px 10px;border-radius:20px;margin-left:12px;
    letter-spacing:0.5px;
  }}
  header .meta{{
    font-size:11px;color:var(--muted);text-align:right;line-height:1.8;
    font-family:"Courier New",monospace;
  }}
  .hero{{
    display:grid;grid-template-columns:1fr 1fr;
    grid-template-rows:1fr;
    height:calc(100vh - 56px);
    gap:10px;padding:10px;background:var(--surface);
  }}
  .hero-left{{display:grid;grid-template-rows:1fr 1fr;gap:10px;min-height:0}}
  .hero-right{{display:grid;grid-template-rows:3fr 1fr;gap:10px;min-height:0}}
  .hero-panel{{
    background:var(--bg);border:1px solid var(--border);
    border-radius:10px;display:flex;flex-direction:column;
    overflow:hidden;min-height:0;
  }}
  .hero-panel h2{{
    flex-shrink:0;font-size:9px;letter-spacing:1.5px;
    text-transform:uppercase;color:var(--muted);font-weight:700;
    background:var(--bg);border-bottom:1px solid var(--border);
    padding:6px 14px;margin:0;
  }}
  .hero-panel .body{{flex:1;overflow:auto;min-height:0;padding:0}}
  .hero-panel .svg-body{{
    flex:1;overflow:hidden;min-height:0;
    display:flex;align-items:stretch;
  }}
  .hero-panel .svg-body > svg{{width:100%;height:100%}}
  .rest{{
    display:grid;grid-template-columns:1fr 1fr;
    gap:12px;padding:12px;
  }}
  .full{{grid-column:1/-1}}
  .panel{{
    background:var(--bg);border:1px solid var(--border);
    border-radius:10px;padding:16px;
  }}
  h2{{
    font-size:10px;letter-spacing:1.5px;text-transform:uppercase;
    color:var(--muted);font-weight:700;
    border-bottom:1px solid var(--border);
    padding-bottom:8px;margin-bottom:12px;
  }}
  table{{width:100%;border-collapse:collapse;font-size:11px}}
  th{{
    padding:5px 10px;text-align:left;
    font-size:9px;letter-spacing:0.8px;text-transform:uppercase;
    color:var(--muted);font-weight:700;
    border-bottom:1px solid var(--border);
    background:var(--bg);
  }}
  td{{
    padding:4px 10px;border-bottom:1px solid var(--border);
    vertical-align:middle;color:var(--body);line-height:1.4;
  }}
  tbody tr:hover td{{background:var(--surface)}}
  tr.crit td:first-child{{border-left:3px solid var(--indigo)}}
  tr.goal-row td:first-child{{border-left:3px solid var(--teal)}}
  tr:last-child td{{border-bottom:none}}
  .tag{{
    display:inline-block;padding:2px 8px;
    border-radius:5px;font-size:10px;font-weight:600;
    margin-left:5px;letter-spacing:0.3px;
  }}
  .tag.goal{{background:var(--teal-l);color:var(--teal)}}
  .tag.path{{background:var(--indigo-l);color:var(--indigo)}}
  .tag.frag{{background:var(--amber-l);color:var(--amber)}}
  .badge{{
    display:inline-block;margin:2px 3px;padding:3px 10px;
    border-radius:5px;font-size:10px;font-weight:500;
    background:var(--indigo);color:white;white-space:nowrap;
  }}
  .badge.dark{{background:#3730A3}}
  .badge.mid{{background:#4F46E5}}
  .badge.light{{background:#6366F1}}
  pre{{
    background:var(--ink);color:#A5B4FC;
    padding:16px;border-radius:8px;
    overflow:auto;font-size:11px;max-height:380px;line-height:1.7;
    font-family:"Courier New",monospace;
  }}
  .scroll-hint{{
    text-align:center;padding:6px;font-size:10px;letter-spacing:3px;
    color:var(--muted);background:var(--surface);
    text-transform:uppercase;
    border-top:1px solid var(--border);
  }}
</style>
</head>
<body>

<header>
  <div>
    <h1>ARGUS — SCENE GRAPH</h1>
    <div style="font-size:10px;color:#A0AEC0;margin-top:3px;letter-spacing:1px">
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

    from scene_graph import SceneGraphBuilder, Detection

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
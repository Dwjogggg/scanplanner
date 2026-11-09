"""Shared known-floorplan global routing; local SCAN uses live ray observations."""
import heapq
import json
import math
import random
import hashlib
from pathlib import Path

SCENE_FILE = Path(__file__).with_name('go2_cluttered_scene.json')
PATH_OBSTACLE_SCENE = Path(__file__).with_name('go2_path_obstacles_scene.json')


def load_scene():
    return json.loads(SCENE_FILE.read_text())


def fixed_episode(scene_file=PATH_OBSTACLE_SCENE):
    """One fixed scene, with physical on-path blockers omitted from its reference."""
    scene = json.loads(Path(scene_file).read_text())
    background = dict(scene,obstacles=[o for o in scene['obstacles'] if not o.get('on_reference')])
    reference = global_route(background)
    feasible = global_route(scene)
    scene['layout_hash'] = hashlib.sha256(json.dumps(scene['obstacles'],sort_keys=True).encode()).hexdigest()[:16]
    scene['on_path_blocker_count'] = sum(bool(o.get('on_reference')) for o in scene['obstacles'])
    return scene, reference, feasible


def randomized_episode(seed):
    """Random warehouse clutter plus three unannounced blockers on its reference.

    Background clutter is known to global A*. Blockers are added afterwards:
    only live LiDAR tells SCAN about them. The final A* is a feasibility check,
    never the reference sent to SCAN. Seeds reproduce all geometry and routes.
    """
    rng = random.Random(seed)
    scene = load_scene()
    scene.update(name='go2_randomized_warehouse', seed=seed,
                 description='Random warehouse clutter and three on-reference blockers; live LiDAR avoidance.',
                 start=[0.0, rng.uniform(-0.35, 0.35)],
                 goal=[9.8, rng.uniform(-0.45, 0.45)], route_clearance=0.65)
    walls = [o for o in scene['obstacles'] if o['name'].startswith('wall')]
    clutter = []
    count = rng.randint(14, 20)
    for attempt in range(2000):
        if len(clutter) == count:
            break
        shape = rng.choice(['box', 'box', 'cylinder'])
        o = dict(name=f'clutter_{len(clutter)+1}', shape=shape,
                 xy=[rng.uniform(0.75, 9.4), rng.choice([-1, 1])*rng.uniform(2.05, 3.0)],
                 height=rng.uniform(0.55, 1.25))
        if shape == 'box':
            o['half'] = [rng.uniform(0.22, 0.55), rng.uniform(0.20, 0.40)]
            extent = math.hypot(*o['half'])
        else:
            o['radius'] = rng.uniform(0.20, 0.36)
            extent = o['radius']
        if any(obstacle_distance(o['xy'], w) < extent+0.10 for w in walls):
            continue
        if any(obstacle_distance(o['xy'], other) < extent+0.16 for other in clutter):
            continue
        clutter.append(o)
    if len(clutter) != count:
        raise RuntimeError('Could not place warehouse clutter without overlaps')
    scene['obstacles'] = walls+clutter
    reference = global_route(scene)
    blockers = []
    for i, bounds in enumerate([(2.0, 2.8), (4.5, 5.3), (7.1, 8.0)]):
        # Interpolate an actual reference segment, rather than assuming y=0.
        x = rng.uniform(*bounds)
        a, b = next((a, b) for a, b in zip(reference[:-1], reference[1:])
                    if a[0] <= x <= b[0])
        y = a[1]+(b[1]-a[1])*(x-a[0])/(b[0]-a[0])
        shape = rng.choice(['box', 'cylinder'])
        o = dict(name=f'path_blocker_{i+1}', shape=shape, xy=[x, y],
                 height=rng.uniform(0.65, 1.10), on_reference=True)
        if shape == 'box':
            o['half'] = [rng.uniform(0.24, 0.36), rng.uniform(0.24, 0.36)]
        else:
            o['radius'] = rng.uniform(0.26, 0.36)
        blockers.append(o)
    scene['obstacles'].extend(blockers)
    feasible = global_route(scene)  # Raise instead of launching an impossible test.
    scene['layout_hash'] = hashlib.sha256(json.dumps(scene['obstacles'], sort_keys=True).encode()).hexdigest()[:16]
    scene['on_path_blocker_count'] = len(blockers)
    return scene, reference, feasible


def obstacle_distance(xy, obstacle):
    dx = abs(xy[0]-obstacle['xy'][0])
    dy = abs(xy[1]-obstacle['xy'][1])
    if obstacle['shape'] == 'cylinder':
        return math.hypot(dx, dy)-obstacle['radius']
    hx, hy = obstacle['half']
    # Signed distance to the rectangular footprint.
    dx, dy = dx-hx, dy-hy
    return math.hypot(max(dx, 0), max(dy, 0))+min(max(dx, dy), 0)


def route_distance(xy, route):
    """Shortest horizontal point-to-polyline distance, including segment interiors."""
    best = float('inf')
    for a, b in zip(route[:-1], route[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        length2 = dx*dx+dy*dy
        t = max(0.0, min(1.0, ((xy[0]-a[0])*dx+(xy[1]-a[1])*dy)/length2)) if length2 else 0.0
        best = min(best, math.hypot(xy[0]-a[0]-t*dx, xy[1]-a[1]-t*dy))
    return best


def global_route(scene, resolution=0.1):
    """Eight-connected A*, with swept-radius collision checks and line simplification."""
    xmin, xmax, ymin, ymax = scene['bounds']
    radius = scene['route_clearance']
    nx, ny = round((xmax-xmin)/resolution)+1, round((ymax-ymin)/resolution)+1
    def world(k):return (xmin+k[0]*resolution, ymin+k[1]*resolution)
    def grid(p):return (round((p[0]-xmin)/resolution), round((p[1]-ymin)/resolution))
    cache = {}
    def free(k):
        if k not in cache:
            xy = world(k)
            cache[k] = (0 <= k[0] < nx and 0 <= k[1] < ny and
                        all(obstacle_distance(xy, o) >= radius for o in scene['obstacles']))
        return cache[k]
    source, target = grid(scene['start']), grid(scene['goal'])
    queue = [(0, source)]
    cost, parent = {source: 0}, {}
    while queue:
        _, k = heapq.heappop(queue)
        if k == target:break
        for dx, dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            n = (k[0]+dx, k[1]+dy)
            if not free(n) or (dx and dy and (not free((k[0]+dx,k[1])) or not free((k[0],k[1]+dy)))):continue
            c = cost[k]+math.hypot(dx, dy)
            if c >= cost.get(n, float('inf')):continue
            cost[n], parent[n] = c, k
            heuristic = math.hypot(n[0]-target[0], n[1]-target[1])
            heapq.heappush(queue, (c+heuristic,n))
    if target not in cost:raise RuntimeError('No collision-free global route in the selected scene')
    route = [target]
    while route[-1] != source:route.append(parent[route[-1]])
    route = [world(k) for k in reversed(route)]
    def visible(a,b):
        count = max(1, math.ceil(math.dist(a,b)/(resolution/2)))
        return all(all(obstacle_distance((a[0]+(b[0]-a[0])*i/count,
                                         a[1]+(b[1]-a[1])*i/count),o) >= radius
                       for o in scene['obstacles']) for i in range(count+1))
    simplified, i = [route[0]], 0
    while i < len(route)-1:
        j = len(route)-1
        while j > i+1 and not visible(route[i],route[j]):j -= 1
        simplified.append(route[j]);i = j
    # Keep enough samples for the planner's reference-progress projection.
    dense = [simplified[0]]
    for a,b in zip(simplified[:-1],simplified[1:]):
        count = math.ceil(math.dist(a,b)/0.3)
        dense.extend([(a[0]+(b[0]-a[0])*i/count,a[1]+(b[1]-a[1])*i/count)
                      for i in range(1,count+1)])
    return dense

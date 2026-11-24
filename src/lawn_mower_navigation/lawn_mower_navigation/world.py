"""Build the Gazebo world of a lawn: ground, obstacles and the sensor systems.

The obstacles are the holes of the lawn polygon, the same polygon the planner
covers, so the simulated garden always matches the planned one. The lawn
boundary is drawn on the ground; it has no collision, like the edge of a real
lawn.
"""

from shapely.geometry import LineString, Polygon

from lawn_mower_electronics.sensors import world_sdf

DEFAULT_STEP = 0.004       # s, physics step
UNTHROTTLED = 0.0          # real_time_factor: as fast as the computer can
OBSTACLE_HEIGHT = 0.6      # m
BOUNDARY_WIDTH = 0.06      # m, painted line
BOUNDARY_HEIGHT = 0.004    # m above the ground, so it is not hidden by the plane
GROUND_MARGIN = 20.0       # m of ground around the lawn


def _polyline(points, height):
    pts = ''.join(f'<point>{x:.4f} {y:.4f}</point>' for x, y in points)
    return f'<polyline>{pts}<height>{height:.4g}</height></polyline>'


def _material(rgb):
    return (f'<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse>'
            '<specular>0.1 0.1 0.1 1</specular></material>')


def _obstacle(index, ring):
    geometry = _polyline(list(ring.coords)[:-1], OBSTACLE_HEIGHT)
    return f"""
    <model name="obstacle_{index}">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry>{geometry}</geometry></collision>
        <visual name="visual"><geometry>{geometry}</geometry>{_material('0.45 0.3 0.2')}</visual>
      </link>
    </model>"""


def _boundary(lawn):
    """The lawn edge as flat strips, one per side, with no collision."""
    visuals = []
    coords = list(lawn.exterior.coords)
    for i, (p, q) in enumerate(zip(coords, coords[1:])):
        strip = LineString([p, q]).buffer(BOUNDARY_WIDTH / 2, cap_style='square')
        geometry = _polyline(list(Polygon(strip.exterior).exterior.coords)[:-1], BOUNDARY_HEIGHT)
        visuals.append(f'<visual name="edge_{i}"><geometry>{geometry}</geometry>'
                       f'{_material("0.95 0.95 0.95")}</visual>')
    return f"""
    <model name="lawn_boundary">
      <static>true</static>
      <link name="link">{''.join(visuals)}</link>
    </model>"""


def lawn_world_sdf(lawn, name='lawn', step=DEFAULT_STEP, real_time_factor=UNTHROTTLED):
    """SDF world of a lawn polygon (obstacles are its holes)."""
    xmin, ymin, xmax, ymax = lawn.bounds
    size = f'{xmax - xmin + 2 * GROUND_MARGIN:.4g} {ymax - ymin + 2 * GROUND_MARGIN:.4g}'
    centre = f'{(xmin + xmax) / 2:.4g} {(ymin + ymax) / 2:.4g} 0 0 0 0'
    plane = f'<plane><normal>0 0 1</normal><size>{size}</size></plane>'
    obstacles = ''.join(_obstacle(i, ring) for i, ring in enumerate(lawn.interiors))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="physics" type="ignored">
      <max_step_size>{step:.6g}</max_step_size>
      <real_time_factor>{real_time_factor:.6g}</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    {world_sdf()}

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <pose>{centre}</pose>
      <link name="link">
        <collision name="collision"><geometry>{plane}</geometry></collision>
        <visual name="visual"><geometry>{plane}</geometry>{_material('0.3 0.5 0.3')}</visual>
      </link>
    </model>
    {_boundary(lawn)}
    {obstacles}
  </world>
</sdf>
"""


def main(argv=None):
    """Print the world of a lawn: lawn_world demo [-o world.sdf]."""
    import argparse

    from .lawns import find_lawn

    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('lawn', help='lawn name: demo, or one of lawn_mower_planning.maps')
    parser.add_argument('-o', '--output', help='write to this file instead of stdout')
    args = parser.parse_args(argv)
    sdf = lawn_world_sdf(find_lawn(args.lawn).area)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(sdf)
    else:
        print(sdf, end='')


if __name__ == '__main__':
    main()

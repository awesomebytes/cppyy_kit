# Small home world candidate: TurtleBot3 House

Recommended quick asset for the existing Humble / Gazebo Classic playground:
the official ROBOTIS **TurtleBot3 House**, pinned to the Humble branch commit in
`sources.json`. The world and all referenced models, meshes and textures are
downloaded here, including the standard ground and sun models. No model
database download is needed at runtime.

The house is a simple roofless multi-room layout, approximately **15.15 ×
10.70 m** across its wall collision bounds. It includes bookcases, cabinets,
tables, bins and a mailbox; it is lightly furnished rather than a photorealistic
home. Its overall model is static, keeping the environment inexpensive and
avoiding loose furniture dynamics. Asset size is approximately 3 MB excluding
temporary metadata. Door clearance for our 1.10 × 1.00 m robot envelope has
**not** been validated.

Files:

- `turtlebot3_house.world`: unmodified official SDF 1.6 Gazebo Classic world.
- `models/turtlebot3_house/`: house geometry and model metadata.
- `models/{cafe_table,first_2015_trash_can,mailbox,table_marble}`: required
  furniture assets from the official Gazebo model database.
- `models/{ground_plane,sun}`: bundled standard world dependencies.
- `sources.json`: pinned upstream commits.

`home.world` is the integration world: it preserves the robot playground's
existing `empty.world` physics, ground and light, adds the house model, and
sets a camera overlooking the rooms. The original upstream
`turtlebot3_house.world` is also retained unchanged for reference.

Add this directory's `models` to `GAZEBO_MODEL_PATH`, disable the online model
database (`GAZEBO_MODEL_DATABASE_URI=''`), and select `home.world`. The selected
spawn is **x=-2.65728, y=1.35, yaw=π/2**, facing the existing house table from
the south. Its near edge starts approximately 0.674 m ahead of the base.
Native planar odometry reports world coordinates, so
identity `map -> odom` remains correct at this nonzero spawn.

The matching `map.yaml` / `map.pgm` uses 0.05 m cells and origin `(-9,-7)`.
`map.png` previews the projected walls and furniture primitives. The raster
includes 82 box/cylinder collision geometries between heights 0.08 and 1.6 m,
including the existing house table and its legs;
mesh furniture is added by the live lidar costmaps, rather than guessed from
COLLADA vertices. This is an initial navigation map, not a complete clearance
validation. Select it with `TIAGO_PRO_NAV_MAP` when starting navigation; the
old empty-space map must not be used in the house.

Regenerate the merged world and map with `pixi run --locked python
worlds/home/prepare.py` from the parent playground. This preparation script
does not start a simulator or move the robot. The primary playground launcher
owns the actual world switch.

The integrated launch is now available from the parent playground:

```bash
pixi run --locked playground-home
```

The first integrated home session opened successfully with the robot visible
inside the rooms: `../../artifacts/home_running_gazebo.png`. Existing native
joint initialization is retained; the world switch adds no homing motions.
The merged world passed the Gazebo Classic SDF check. No autonomous house
navigation goal was sent during this initial visual check.

## Brighter home and manipulation props

The ground visual uses a plain light warm neutral matte material, with its
original collision plane preserved. Balanced ambient light, a warm directional
key and a softer cool fill brighten the interior without extra runtime assets.

The existing `turtlebot3_house::table` has a 1.40 × 0.80 m collision top matching the visual surface at
height 1.020 m, centered at world `(-2.65728,2.42376)`, with four cylindrical
legs. Its nested model is explicitly static to prevent furniture bouncing. No additional table is spawned. Three native dynamic objects start
3 mm above its collision surface:

| Gazebo model | Shape | Mass | Initial center (world metres) |
|---|---|---|---|
| `red_cube` | 45 mm cube | 80 g | `(-2.85,2.23,1.0455)` |
| `blue_cylinder` | diameter 40 mm, height 70 mm | 100 g | `(-2.65728,2.23,1.058)` |
| `yellow_cube` | 50 mm cube | 100 g | `(-2.46,2.23,1.048)` |

These use native gravity, collision, friction and computed solid-body inertia;
they are not static visual props. MoveIt observes the environment through the
robot's depth point cloud rather than manually injected table/object primitives.
Grasping is not demonstrated by adding these physical objects.

## Alternatives considered

| World | Why use it | Qualification |
|---|---|---|
| **TurtleBot3 House** | Small download; simple static rooms and furniture; official Humble/Classic asset. | Moderate 15 × 11 m layout; doorway and spawn checks pending. |
| **PAL `home.world`** | Already present at `../../../tiago_dual/vendor/pal_gazebo_worlds/worlds/home.world`; richer domestic objects. | Approximately 17.5 × 15.25 m wall dimensions; many dynamic cabinets/objects; missing `bowl` and `cafe_table` meshes in the local dependency set. |
| **AWS Small House** | Richer residential furnishings and an upstream screenshot. | More assets and roughly 20 m-wide layout; archived upstream. Original `ros2` branch remains available. |

## Sources and attribution

ROBOTIS world/model: [official Humble source](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble/turtlebot3_gazebo),
[official simulation guide and house screenshot](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/).
Repository license: **Apache-2.0**, included in `LICENSE`.

Furniture/ground/sun: [OSRF Gazebo model database](https://github.com/osrf/gazebo_models).
Repository license: **Creative Commons Attribution 3.0 Unported**, included in
`GAZEBO_MODELS_LICENSE`; model metadata retains original authors.

Alternatives: [PAL worlds](https://github.com/pal-robotics/pal_gazebo_worlds/tree/humble-devel/worlds),
[AWS Small House original ROS 2 branch and screenshot](https://github.com/aws-robotics/aws-robomaker-small-house-world/tree/ros2).

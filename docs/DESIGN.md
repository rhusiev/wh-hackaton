# Design and reference

The long version of the README: why Gazebo, what is modelled, topics and the
search-and-AR loop. To launch the sim, see the [README](../README.md)

Gazebo Harmonic + ArduPilot SITL simulation of a Y3 tricopter with a forward
RGBD camera, flying inside a generated warehouse. One container, one launch
file, worlds kept as data.

---

## Is Gazebo the right choice here?

Yes, for autonomy work. No, if you want to learn anything about the airframe.

The deciding fact is that your flight controller is a Matek H743 running
ArduPilot. ArduPilot ships SITL, which compiles the *actual firmware* — the same
`AP_MotorsTri` mixer, the same EKF3, the same GUIDED mode — as a host binary and
talks to an external physics engine over a JSON socket. Gazebo plugs straight
into that socket via `ardupilot_gazebo`. So the thing flying in the sim runs the
code that will run on the real board, and a MAVLink command that works in the
sim works on the bench. Nothing else gives you that for a tricopter.

The alternatives, and why they lose:

- **PX4 + gz**: better out-of-the-box ROS 2 story, but PX4's tricopter airframe
  is deprecated, and your FC is not running PX4. Wrong firmware, wrong mixer.
- **Isaac Sim**: much better depth/RGB realism, and it would be the right answer
  for training a perception model. It needs an RTX card. This machine has a
  GTX 1050, so it is not an option.
- **AirSim / Colosseum**: unmaintained; the ArduPilot bridge rots.
- **Webots**: has an ArduPilot bridge, but the depth-camera and ROS 2 tooling
  around it is thinner than `ros_gz_*`.
- **ArduPilot SITL alone, no 3D**: the *right* choice if the question is tuning,
  mixer behaviour or failsafes. It runs 10x real time with no GPU. Use this when
  you do not need pixels. `sim_vehicle.py -v ArduCopter -f tri --console --map`.

What Gazebo will *not* tell you, and what you should not spend hackathon hours
on inside it:

- Whether the 3115/1050 combination actually makes its thrust, or how hot the
  Aikon ESC gets. The rotor model is a fitted curve (see `docs/NOTES.md`), not
  a propeller.
- Anything about the DJI O4 video link or the ELRS control link. Both are
  modelled as "perfect and absent".
- Prop wash, ground effect near shelving, or the vortex ring state you will hit
  descending in an aisle. The lift-drag model has none of it.

And one that matters specifically for this idea: **Gazebo only half tells you
whether your detector works.** The people are real scanned meshes, so a network
trained on photos does find them and their heads here. But the rendering is
clean and evenly lit, and there are only three static people in fixed poses. A
detector that works here can still fail on real footage with people sitting,
moving or half hidden. `detector:=truth` swaps the network for
`scripts/spatial_detector.py`, which reads the true positions and drops what the
camera could not see. Use it to test tracking and AR without the network's
misses.

Check the detector on real OAK-D footage. Develop everything downstream of it
here.

So: build the perception → planning → MAVLink loop here, then move it to a
companion computer and re-test the flight envelope on the real machine.

---

## What is modelled, against your parts list

| Real part | In the sim |
| --- | --- |
| NEXERO 3115 900KV, 6S, 1620 W max | rotor joint driven 0–1000 rad/s, 31.4 N and ~840 W of shaft power at the top |
| Aikon AKE 80A ESC (Bluejay) | velocity PID on the rotor joint, torque capped at 2 N·m → roughly 90 ms from idle to full |
| Gemfan 1050 tri-blade, 10" | `LiftDrag` disc, r = 0.127 m, pressure centre at 0.089 m, blade area 0.00878 m² |
| Matek H743-SLIM V4 (ArduPilot) | ArduCopter SITL, `FRAME_CLASS=7` (Tri), outputs 1/2/4 + tail servo on 7 |
| tail tilt servo | revolute joint, ±45° mechanical, ±30° commanded (`MOT_YAW_SV_ANGLE`) |
| Matek M10Q-5883 GNSS + compass | SITL's own simulated GPS and compass; world origin 49.8397 N, 24.0297 E, 296 m |
| PDB-HEX + Dogcom 6S 5200 mAh | `MOT_BAT_VOLT_MIN/MAX` 19.8/25.2 V only. No discharge curve, no current draw |
| DJI O4 Air Unit Pro | **not modelled.** It is a pilot video downlink and carries nothing autonomy needs |
| RadioMaster RP3 V2 ELRS | **not modelled.** SITL RC comes from MAVProxy's `rc` command |
| Luxonis OAK-D | Gazebo `rgbd_camera`, 640×400 @ 15 Hz, 68.8° HFOV, depth 0.7–30 m (12 m reliable) |
| OAK-D on-device YOLO | `scripts/person_detector.py`, YOLO11n-pose on the CPU, same `vision_msgs/Detection3DArray` contract |
| Raspberry Pi 5 8 GB | **not modelled as a bottleneck.** Everything runs on the desktop; the rates are picked to fit the Pi |
| Spectacles 2024 | `scripts/ar_bridge.py` — a WebSocket of JSON the Lens draws |

One Gazebo sensor cannot have separate colour and stereo optics, so it carries
the colour FOV (68.8°): depthai aligns depth into the colour frame, and that
aligned pair is what the Pi 5 actually receives.

Airframe totals: 3.136 kg AUW (OAK-D 91 g on the nose, Pi 5 and cooler 110 g on
the frame), 0.27 m arms (equilateral Y3, front rotors at ±60° from the nose),
thrust-to-weight 3.1, hover at 32% of full throttle.

---

## Topics

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image`, `rgb8` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/image_raw` | `sensor_msgs/Image`, `32FC1`, metres, with stereo noise from `scripts/depth_noise.py` |
| `/camera/depth/ideal/image_raw` | the same without noise, straight from Gazebo |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/points` | `sensor_msgs/PointCloud2`, rebuilt by `depth_image_proc` from depth decimated 4x (160x100) |
| `/ground_truth/odom` | `nav_msgs/Odometry`, exact pose straight from Gazebo |
| `/tf` | `map` → `base_link`, also straight from Gazebo |
| `/oak/spatial_detections` | `vision_msgs/Detection3DArray` in `camera_optical_frame` |
| `/oak/detections_2d` | `vision_msgs/Detection2DArray`, `person` and `head` boxes in colour pixels, a person and their head share an id |
| `/oak/detection_markers` | `visualization_msgs/MarkerArray`, for RViz |
| `/scan` | `sensor_msgs/LaserScan` from the depth, relayed to `/mavros/obstacle/send` for ArduPilot |
| `/map` | `nav_msgs/OccupancyGrid` at 0.2 m, from `scripts/grid_mapper.py` using the true pose, or from RTAB-Map with `slam:=true` |
| `ws://<host>:8790` | the AR payload, 10 Hz JSON |
| `/mavros/state`, `/mavros/local_position/pose`, … | the usual MAVROS surface |

Frames: `map` → `base_link` (ground truth, bridged from Gazebo) → `camera_link` →
`camera_optical_frame`. MAVROS does not publish TF here - see `docs/NOTES.md`
The two camera transforms are static publishers in the launch file because there
is no URDF, so no `robot_state_publisher`.

## The search-and-AR loop

This is the part the idea actually needs, and it is four pieces:

1. **Depth → obstacles.** `pointcloud_to_laserscan` flattens the cloud into a
   scan, and `scripts/scan_relay.py` hands it to mavros's `obstacle_distance`. That
   is what makes `PRX1_TYPE 2`, `AVOID_ENABLE 7` and `OA_TYPE 1` in
   `config/tricopter.parm` do anything - without the scan they are inert.
2. **Colour + depth → targets.** `scripts/person_detector.py` stands in for the
   OAK-D's on-device network. It runs YOLO11n-pose (`models/detector/`) at up
   to 5 Hz. The network gives a person box and 17 body points. The head box is
   built from the face points, or from the shoulders when the person faces
   away. The person's distance is the median depth over their torso plus
   0.15 m, because the depth sees the front of the body. The 3D position is
   skipped, and only the 2D box sent, in two cases. One is a person past 12 m,
   where the stereo error is over 0.5 m. The other is a torso whose depth
   spread (interquartile range) is over 0.4 m plus the stereo noise. That is a
   box over two people, or over a person and a rack. With
   `detector:=truth`, `scripts/spatial_detector.py` reads
   `worlds/warehouse_targets.json` instead and keeps what the camera could see.
3. **Targets → tracks.** `scripts/ar_bridge.py` transforms detections into the
   map frame and hands them to a tracker (`scripts/tracker.py`). Each sighting
   joins the nearest track of its label within 1 m, or starts a new one. Two
   tracks that drift within 1 m of each other are merged. A track is only sent
   after 6 sightings, and a "person" whose top is above 2.3 m is dropped. This is the point of the
   whole thing: a person seen once down an aisle stays on the minimap after the
   drone has flown past, which is what "бачити людину за стінкою" means.
4. **Tracks → Spectacles.** The same node serves a WebSocket on port 8790 at
   10 Hz:

   ```json
   {"t": 41.2,
    "drone": {"x": -13.4, "y": 0.1, "z": 2.5, "yaw": 0.02},
    "targets": [{"id": 0, "label": "person", "x": -9.0, "y": 0.0, "z": 0.8,
                 "h": 1.6, "score": 0.78, "age": 1.3, "hits": 12,
                 "head": {"x": -9.0, "y": 0.0, "z": 1.5, "size": 0.25}}],
    "map": {"res": 0.2, "w": 200, "h": 200, "x0": -20.0, "y0": -20.0,
            "cells": "<base64, 0 free / 1 occupied / 2 unknown>"}}
   ```

   Metres in the map frame, so the Lens only has to scale and rotate. `"head"`
   is missing when no head was seen. `./run.sh preview` draws this payload the
   way the glasses would.

Fly with:

```bash
./run.sh explore --altitude 2.5        # lawnmower over all five lanes
./run.sh explore --lanes 2             # just the first two, for a quick demo
./run.sh explore --strategy frontier   # no known layout, explore the map's unknown edges
./run.sh score                         # found / missed / false against the true positions
```

Frontier exploration (`scripts/frontier.py`) works like this. A frontier is a
known free cell next to an unknown one. The planner keeps 0.8 m from obstacles,
finds the nearest reachable group of frontier cells by breadth-first search, and
flies there in legs of at most 3 m, facing the direction of travel. The scan only
covers what the camera faces. At each frontier it turns toward the unknown
space. A frontier it has visited or failed to reach is skipped within 1.5 m. It
stops when no frontier is left or after `--max-time` (600 s). In test runs it
flew 34-42 legs in about 4 min and found all 6 people, each within 0.25 m and
with a head. False people went away in steps:

1. With a 2-sighting minimum there were 8 false tracks, with 2-5 sightings
   each. Each was a real person's noisy depth splitting off ~1 m away. Real
   people had 18 or more
2. With a 6-sighting minimum, 2 remained. Both were person_5 seen from
   25-30 m, so 3D positions past 12 m were dropped
3. Then 2 remained again. One was a second track on person_5 that started
   before either position settled. The other was a person box whose torso
   depth mixed near and far
4. With track merging and the torso spread check, 0 remained. person_3 had
   only 6 sightings, exactly the minimum, so a shorter look would miss them. The
   next run did: it flew 26 legs, saw person_3 too briefly and found 5 of 6

To move this to the real aircraft, run with `detector:=none` and start
`depthai_ros_driver` instead. It publishes the same `Detection3DArray`, so
`ar_bridge.py` and the Lens do not change.

## Swapping a part

Every part talks to the next only through topics or a small Python interface, so
each one can be replaced without touching the others.

| Part | Contract | Built in | Replace with |
|---|---|---|---|
| Detector node | publishes `/oak/spatial_detections` (and `/oak/detections_2d`), person and head share an id | `person_detector.py`, `spatial_detector.py` | `detector:=none` and run your own node, e.g. `depthai_ros_driver` |
| Detector network | `person_network.PersonNetwork`: called with an RGB image, returns `Person(box, score, head)` | `person_network:PoseNetwork` | parameter `network` (built with `model`, `threads`, `min_score`, `min_keypoint`), or only `model` for other YOLO-pose weights |
| Mapper | publishes `/map` as `nav_msgs/OccupancyGrid` in `map` | `grid_mapper.py`, RTAB-Map with `slam:=true` | `mapper:=false` and run your own node |
| Tracker | `tracker.Tracker`: `update(sightings, now)` and `targets(now)` | `tracker:NearestTracker` | parameter `tracker` (built with `merge_radius`, `min_hits`, `max_top`, `timeout`) |
| AR server | serves the JSON above on port 8790 | `ar_bridge.py` | `ar:=false` and serve your own |
| Exploration | a class built from the parsed arguments with `run(flight)`, optional static `add_arguments(parser)` | `sweep`, `frontier` | `./run.sh explore --strategy my_search.py:MySearch` |
| Flight | `flight.Flight`: `here`, `heading`, `grid`, `fly_to`, `turn`, over MAVROS in GUIDED | `flight.py` | subclass it for another autopilot link; strategies do not change |

Parameters are set by starting the node yourself with its launch part off:

```bash
./run.sh sim sitl:=true detector:=none
./run.sh python3 scripts/person_detector.py --ros-args -p network:=my_net.py:MyNet
```

`module:Class` names an importable module in `scripts/`; `path/to/file.py:Class`
loads a file from anywhere (`scripts/plugin.py`). The drone's pose comes from the
`map` -> `base_link` transform everywhere, which is ground truth in the sim and
SLAM on the aircraft.

## Multi-robot and GPS-denied

Both follow from the same two swaps:

- **Shared map** (idea 1): `ar_bridge.py` tracks are already in the `map` frame,
  so a second robot publishing into the same frame just appears. What is missing
  is frame alignment between the two - with `slam:=true` each robot has its own
  RTAB-Map origin, and merging them needs `rtabmap` in multi-session mode or a
  common fiducial.
- **Indoor, no GNSS** (the honest search-and-rescue case):
  `config/gps_denied.parm` switches the EKF to `EK3_SRC1_*=6` (ExternalNav) and
  turns GPS off entirely, so the EKF cannot silently fall back to it. Nothing
  arms until visual odometry is actually publishing:

  ```bash
  ./run.sh sim slam:=true
  ./run.sh sitl -- --add-param-file=config/gps_denied.parm
  ```

## The world

`worlds/warehouse.sdf` is generated, not hand-written — regenerate it with a
different layout any time:

```bash
python3 scripts/gen_warehouse.py --seed 7
```

32 × 20 × 8 m hall, four rack rows at y = ±2.5 and ±7.5, five clear lanes at
y = ±9.3, ±5 and 0. The structure is primitives. The people are three
Gazebo Fuel meshes (Nurse, FemaleVisitor, Scrubs, CC BY 4.0) vendored in
`models/people/`, so it still works offline. Racks carry randomised cargo boxes and the walls have painted
bands — both are there so visual odometry has something to track, which a bare
white warehouse would not give it.

Six standing people are the search targets. Two stand in open aisles;
three are in the 2.6 m gaps between racks, visible only from the neighbouring
lane; one is in a corner. The generator writes their true positions to
`worlds/warehouse_targets.json`, which is what the detector stand-in reads.

## Performance on a GTX 1050

Physics runs at 500 Hz (2 ms steps) and the plugin is in lock-step, so if the GPU cannot
keep up the whole sim slows down together and ArduPilot stays in sync — it does
not fall over, it just runs below real time. If that bothers you, drop the
camera in `models/tricopter/model.sdf` to 424×240, and/or run
`./run.sh sim gui:=false` and watch through RViz or Foxglove instead.

See [NOTES.md](NOTES.md) for the rotor-model derivation and the external-behaviour
findings that are easy to get wrong.

# wh-hackaton — warehouse tricopter simulation

Gazebo Harmonic + ArduPilot SITL simulation of a Y3 tricopter with a forward
RGBD camera, flying inside a generated warehouse. Same shape as the `husky-sim`
setup in the diploma: one container, one launch file, worlds kept as data.

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

And one that matters specifically for this idea: **Gazebo will not tell you
whether your detector works.** Its rendering is clean, evenly lit and untextured
compared with a real warehouse, so a network trained or tuned on these images
will not transfer. That is why `scripts/spatial_detector.py` does not run a
network at all - it reads the true target positions and then throws away
everything the camera could not have seen. You get the geometry, the occlusion
and the range limits honestly, and you develop the tracking and AR layers
against a real message stream, without pretending the perception is solved.

Train the detector on real OAK-D footage. Develop everything downstream of it
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
| Luxonis OAK-D | Gazebo `rgbd_camera`, 640×400 @ 15 Hz, 68.8° HFOV, depth 0.7–12 m |
| OAK-D on-device YOLO | `scripts/spatial_detector.py` — same `vision_msgs/Detection3DArray` contract |
| Raspberry Pi 5 8 GB | **not modelled as a bottleneck.** Everything runs on the desktop; the rates are picked to fit the Pi |
| Spectacles 2024 | `scripts/ar_bridge.py` — a WebSocket of JSON the Lens draws |

One Gazebo sensor cannot have separate colour and stereo optics, so it carries
the colour FOV (68.8°): depthai aligns depth into the colour frame, and that
aligned pair is what the Pi 5 actually receives.

Airframe totals: 3.136 kg AUW (OAK-D 91 g on the nose, Pi 5 and cooler 110 g on
the frame), 0.27 m arms (equilateral Y3, front rotors at ±60° from the nose),
thrust-to-weight 3.1, hover at 32% of full throttle.

---

## Quick start

```bash
./run.sh build          # slow once: clones and waf-builds ArduPilot
./run.sh up
```

Two terminals from here on.

```bash
# terminal 1 — Gazebo, the gz↔ROS bridges and MAVROS
./run.sh sim

# terminal 2 — ArduPilot SITL with the MAVProxy console
./run.sh sitl
```

In the MAVProxy console, once EKF settles:

```
mode guided
arm throttle
takeoff 2
```

Or from ROS:

```bash
./run.sh "./scripts/takeoff_hover.py --altitude 2.0"
```

Useful launch arguments:

```bash
./run.sh sim rviz:=true foxglove:=true          # viewers
./run.sh sim gui:=false                         # headless Gazebo
./run.sh sim x:=-13.5 y:=5.0 z:=0.2 yaw:=0.0    # spawn pose
./run.sh sim sitl:=true                         # start SITL inside the launch too
./run.sh sim depth_decimation:=1                # full 640x400 depth into the cloud and scan (default 4)
```

```bash
./run.sh sim slam:=true                         # RTAB-Map visual odometry + /map
./run.sh sim perception:=false                  # bare flight sim, no detector or AR
./run.sh explore --altitude 2.5                 # fly the search pattern
./run.sh smoke                                  # check the topic contract
./run.sh demo                                   # colour and false-coloured depth side by side
./run.sh demo --save frame.png                  # same, one frame to a file on a headless host
```

`WIPE=1 ./run.sh sitl` resets the simulated EEPROM, which you want after editing
`config/tricopter.parm`.

## Topics

| Topic | Type |
| --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image`, `rgb8` |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/image_raw` | `sensor_msgs/Image`, `32FC1`, metres |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` |
| `/camera/depth/points` | `sensor_msgs/PointCloud2`, rebuilt by `depth_image_proc` from depth decimated 4x (160x100) |
| `/ground_truth/odom` | `nav_msgs/Odometry`, exact pose straight from Gazebo |
| `/tf` | `map` → `base_link`, also straight from Gazebo |
| `/oak/spatial_detections` | `vision_msgs/Detection3DArray` in `camera_optical_frame` |
| `/oak/detection_markers` | `visualization_msgs/MarkerArray` in `map`, for RViz |
| `/scan` | `sensor_msgs/LaserScan` from the depth, relayed to `/mavros/obstacle/send` for ArduPilot |
| `/map` | `nav_msgs/OccupancyGrid`, only with `slam:=true` |
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
2. **Depth → targets.** `scripts/spatial_detector.py` stands in for the OAK-D's
   on-device network. It knows where the people are from
   `worlds/warehouse_targets.json`, and then decides what the camera could
   actually see: inside the frustum, 0.7–12 m, at least 24 px across, and not
   hidden behind a rack. The occlusion test compares the predicted range with
   the measured depth at that pixel, so it fails where the real one fails.
3. **Targets → tracks.** `scripts/ar_bridge.py` transforms detections into the
   map frame and fuses them into persistent tracks. This is the point of the
   whole thing: a person seen once down an aisle stays on the minimap after the
   drone has flown past, which is what "бачити людину за стінкою" means.
4. **Tracks → Spectacles.** The same node serves a WebSocket on port 8790 at
   10 Hz:

   ```json
   {"t": 41.2,
    "drone": {"x": -13.4, "y": 0.1, "z": 2.5, "yaw": 0.02},
    "targets": [{"id": 0, "label": "person", "x": -9.0, "y": 0.0,
                 "score": 0.78, "age": 1.3, "hits": 12}],
    "map": {"res": 0.1, "w": 320, "h": 200, "x0": -16.0, "y0": -10.0,
            "cells": "<base64, 0 free / 1 occupied / 2 unknown>"}}
   ```

   Metres in the map frame, so the Lens only has to scale and rotate. `"map"`
   appears once something publishes `/map`, i.e. with `slam:=true`.

Fly the pattern with:

```bash
./run.sh explore --altitude 2.5        # lawnmower over all five lanes
./run.sh explore --lanes 2             # just the first two, for a quick demo
```

To move this to the real aircraft, delete `spatial_detector.py` and run
`depthai_ros_driver` instead. It publishes the same `Detection3DArray`, so
`ar_bridge.py` and the Lens do not change.

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
python3 scripts/gen_warehouse.py --seed 42
```

32 × 20 × 8 m hall, four rack rows at y = ±2.5 and ±7.5, five clear lanes at
y = ±9.3, ±5 and 0. Everything is a primitive, so there is no Fuel download and
it works offline. Racks carry randomised cargo boxes and the walls have painted
bands — both are there so visual odometry has something to track, which a bare
white warehouse would not give it.

Six figures in hi-vis vests are the search targets. Two stand in open aisles;
three are in the 2.6 m gaps between racks, visible only from the neighbouring
lane; one is in a corner. The generator writes their true positions to
`worlds/warehouse_targets.json`, which is what the detector stand-in reads.

## Performance on a GTX 1050

Physics runs at 1000 Hz and the plugin is in lock-step, so if the GPU cannot
keep up the whole sim slows down together and ArduPilot stays in sync — it does
not fall over, it just runs below real time. If that bothers you, drop the
camera in `models/tricopter/model.sdf` to 424×240, and/or run
`./run.sh sim gui:=false` and watch through RViz or Foxglove instead.

See `docs/NOTES.md` for the rotor-model derivation and the external-behaviour
findings that are easy to get wrong.

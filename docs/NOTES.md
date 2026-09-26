# Findings

Things about ArduPilot, `ardupilot_gazebo` and Gazebo that are not obvious from
their docs, and that this setup depends on. Verified against ArduPilot
`Copter-4.6.3` and `ardupilot_gazebo` `main` on 2026-09-16.

## ArduPilot tricopter mixing

From `libraries/AP_Motors/AP_MotorsTri.cpp`:

- Motors are on outputs **1 (front right), 2 (front left), 4 (rear)**. Output 3
  is skipped. The tail servo is `SRV_Channel::k_motor7`, i.e. `SERVO7_FUNCTION 39`.
  In the plugin these are channel indices 0, 1, 3 and 6.
- The mixer is `right = -0.5·roll + 0.5·pitch`, `left = 0.5·roll + 0.5·pitch`,
  `rear = -0.5·pitch`. The equal 0.5 pitch weights are only correct for an
  equilateral Y3: front rotors at 0.5·L forward, rear at 1.0·L back. The model
  uses exactly that (front at ±60° from the nose, L = 0.27 m), so the mixer
  matches the geometry.
- Roll authority is 1.73× pitch authority for that geometry and the mixer does
  not equalise it. That is true of every real tricopter; ArduPilot's stock gains
  already assume it.
- `_pivot_angle = asin(yaw_thrust)` and then `_thrust_rear /= cos(_pivot_angle)`.
  Tilting the rear rotor is therefore modelled as vectoring, which is what the
  SDF does by rotating the whole rotor link.

## The tail servo full scale is MOT_YAW_SV_ANGLE, not 45°

`init()` calls `SRV_Channels::set_angle(..., _yaw_servo_angle_max_deg * 100)`.
So the **whole** `SERVO7_MIN..SERVO7_MAX` span maps to ±`MOT_YAW_SV_ANGLE`, not
to some fixed ±45° scale. With `MOT_YAW_SV_ANGLE = 30`:

```
joint_angle = multiplier · (raw_cmd + offset),  raw_cmd ∈ [0, 1]
offset      = -0.5
multiplier  = -2 · radians(30) = -1.0472
```

**If you change `MOT_YAW_SV_ANGLE`, change `<multiplier>` on channel 6 to match**,
or ArduPilot's yaw authority estimate and the real tilt will disagree.

The parameter is `MOT_YAW_SV_ANGLE`. The source calls it `YAW_SV_ANGLE`, because
that is the name *within* the `MOT_` parameter group.

The sign is negative, derived as follows. Positive yaw demand means nose right.
That drives PWM above trim, so `raw_cmd > 0.5`. With a negative multiplier the
joint rotates **-θ** about body X, which points the rear thrust to +Y (left).
Applied at x = -0.27 m that is a -Z torque, i.e. clockwise seen from above, i.e.
nose right. Correct. A positive multiplier yaws the wrong way and the controller
diverges within a second of arming.

## The rear rotor needs a standing tail trim

Front rotors counter-rotate and cancel. The rear rotor's reaction torque does
not, so the tail servo carries a constant offset — roughly 6° at hover here. The
yaw I-term absorbs it, which is why `ATC_RAT_YAW_I` is raised to 0.060 with
`IMAX 0.50`. This is exactly the behaviour of a real tricopter, not a sim
artefact. Do not "fix" it with `SERVO7_TRIM`; the required trim scales with
throttle.

## ardupilot_gazebo plugin gotchas

- The element is `<rotorVelocitySlowdownSim>`, **not** `controlVelocitySlowdownSim`.
  Misspelling it is silent — the plugin just uses the default.
- Command math is `cmd = multiplier · (raw_cmd + offset)` where
  `raw_cmd = (pwm - servo_min) / (servo_max - servo_min)`, clamped by nothing.
  Keep `servo_min`/`servo_max` equal to `MOT_PWM_MIN`/`MOT_PWM_MAX`.
- With `<useForce>1</useForce>` the VELOCITY and POSITION types both drive
  `JointForceCmd` through a `gz::math::PID`, whose output is clamped to
  `cmd_max`/`cmd_min`. Those are therefore **torques in N·m**, not velocities.
  Setting `cmd_max` too low silently caps the motor.
- `<imuName>` is looked up as a scoped sensor name inside the model. The
  upstream `iris_with_standoffs` hangs its IMU off a zero-limit *revolute* joint
  to dodge fixed-joint merging, which was a Gazebo Classic behaviour. gz-sim
  keeps fixed-joint links separate, and on a light IMU link the zero limit is
  soft: `imu_joint` wobbled at ~1.4 rad/s about Z, the gyro reported yaw the
  airframe did not have, arming said `Gyros inconsistent`, and the controller
  spun the drone up after takeoff. The joint here is `fixed`.
- The IMU sensor needs `<pose degrees="true">0 0 0 180 0 0</pose>`, as upstream.
  The plugin forwards the gyro and accel in the sensor frame, so without the roll
  pitch and yaw rates reach ArduPilot sign-inverted and it flips within 0.4 s.
- `gazeboXYZToNED` is `0 0 0 180 0 90`, not `180 0 0`: Gazebo's world is ENU,
  so reaching NED takes the 90° yaw as well.
- The plugins install to `lib/ardupilot_gazebo/`, not `lib/`, so that subdirectory
  is what `GZ_SIM_SYSTEM_PLUGIN_PATH` must name. A wrong path fails silently: the
  model spawns and SITL just waits for JSON that never comes.
- Do not drive a light joint with a POSITION channel and `useForce`. With the tail
  pivot at ~3.7e-4 kg·m², `p 25 d 1.5` chattered at ~12 rad/s, and `p 5 d 0.09`
  crept toward a target with a 10 s time constant on the ground. In the air it ran
  past a -0.52 rad target and sat on the -0.785 rad limit for seconds, so yaw spun
  up right after takeoff. The tail now uses a COMMAND channel. The plugin only
  publishes the angle, and gz-sim's `JointPositionController` with
  `use_velocity_commands` tracks it. POSITION without `useForce` is not
  implemented upstream: it only logs a warning.
- A VELOCITY channel with `useForce` is also explicit at the physics step, so its
  speed error shrinks by `P·dt/I` per step. It rings or diverges unless that is
  below 2. With the rotor's 1.3e-4 kg·m² at a 2 ms step, `p_gain 0.2` gives 3.1.
  `0.05` gives 0.77.
- `<lock_step>1</lock_step>` makes Gazebo and SITL step together. On a slow GPU
  the whole sim drops below real time instead of the flight controller
  desyncing. Keep it on.

## SITL frame selection

There is a `tri` frame and a `gazebo-iris` frame, and you need bits of both.
Only frames marked `"external": True` in `Tools/autotest/pysim/vehicleinfo.py`
speak the JSON protocol Gazebo needs, and `tri` is not one of them. So SITL runs
as `-f gazebo-iris --model JSON` and `config/tricopter.parm` is applied on top,
which is what actually makes it a tricopter. Later default-param files win, so
`FRAME_CLASS 7` overrides the iris `FRAME_CLASS 1`.

`gazebo-iris.parm` also enables a fake sonar (`RNGFND1_TYPE 1`, `SIM_SONAR_SCALE`)
and precision landing. Both are turned back off in `tricopter.parm` — they are
iris test fixtures, not part of this airframe.

## Rotor model derivation

`LiftDrag` computes `L = 0.5·ρ·A·(cla·α)·v²` with `v = ω·r_cp`, so thrust is
`k·ω²` with `k = 0.5·ρ·A·cla·a0·r_cp²`. Targets and the values that hit them:

| Target | Value |
| --- | --- |
| max rotor speed | 1000 rad/s (≈ 9550 rpm; a 900 KV motor on 6S loaded) |
| thrust at max | 31.4 N (3.2 kgf), so 94 N total against 29.5 N of weight |
| shaft power at max | 839 W — from ideal induced power 504 W at a figure of merit of 0.6 |
| `r_cp` | 0.089 m = 0.7 × the 0.127 m prop radius |
| `a0`, `cla` | 0.15 rad, 5.0 |
| `area` | 0.00878 m² — solves `k = 3.14e-5`; a real 10×5 tri-blade is ≈ 0.0095 m² |
| `cd0`, `cda` | 0.02, 1.37 — gives `cd = 0.225` at α = 0.15, hence 0.839 N·m |

Consequences worth knowing:

- Thrust is *exactly* quadratic in the PWM command, so `MOT_THST_EXPO` is set to
  **1.0**. The stock 0.65 assumes a real prop's partial linearisation and would
  mistune the throttle response here.
- `MOT_THST_HOVER 0.320` is 10.3 N of 31.4 N at the 3.136 kg AUW. Recompute it if you change the
  mass or the rotor constants.
- `alpha_stall` is 0.35, above the 0.15 operating α, so the rotor never leaves
  the linear part of the curve. The model has no stall, no prop wash, no ground
  effect and no translational lift.
- Each rotor gets **two** `LiftDrag` plugins, with pressure centres at ±`r_cp` and
  opposite `forward` vectors, each at half of the `area` above. With one plugin the
  whole thrust acts 0.089 m off the hub, a ~0.9 N·m moment that rotates with the
  rotor. At ~600-1000 rad/s it turns 1-2 rad per 2 ms physics step. That sampling
  does not average the moment to zero: it leaves a steady torque on the tail tilt
  joint. Upstream iris uses the same mirrored pair.

## Gazebo publishes an X-forward point cloud under an optical frame_id

This one costs an afternoon if you do not know it. Confirmed by reading
`gz-sensors8/src/RgbdCameraSensor.cc` and the ogre2 depth shader:

- `<optical_frame_id>` sets the `frame_id` on the image, the depth image, the
  `camera_info` **and** the point cloud (`InitPointCloudPacked(..., OpticalFrameId(), ...)`).
- The point data itself is copied straight out of the render buffer with
  `memcpy`, no axis reordering. The shader
  (`depth_camera_final_fs.glsl`) clamps `point.x` against near/far, so **depth
  runs along X** — Gazebo's camera convention, not ROS's Z-forward optical one.

So `/rgbd/points` is X-forward data wearing an optical-frame label, and anything
that trusts the label puts it 90° out. The setup here therefore:

- keeps `<optical_frame_id>camera_optical_frame</optical_frame_id>`, which *is*
  correct for the images and `camera_info`, since those are what
  `depth_image_proc` and RTAB-Map project through;
- does not bridge Gazebo's `/rgbd/points` at all, since a subscriber makes
  Gazebo build and serialize a cloud nobody should use;
- rebuilds the real cloud with `depth_image_proc::PointCloudXyzNode` from the
  depth image + `camera_info`, publishing `/camera/depth/points` genuinely in
  `camera_optical_frame`. `image_proc::CropDecimateNode` first takes the depth to
  160x100 with nearest-neighbour sampling. That is still finer than the scan's 139
  bins, and it took `/scan` from ~5 Hz to the full 15 Hz on a busy host. The cloud
  is uncoloured and sparse in RViz as a result. `depth_decimation:=1` restores full
  resolution and still holds 15 Hz. It costs ~0.3 core more, in the scan slicer.

Use `/camera/depth/points`.

Related: the sensor advertises only **one** `camera_info`, at
`/rgbd/camera_info`. There is no `/rgbd/depth_image/camera_info`. The bridge maps
that single topic to both `/camera/color/camera_info` and
`/camera/depth/camera_info`.

## Everything runs on sim time

The bridge publishes `/clock` from Gazebo. Nodes that stamp their own messages
— MAVROS, `depth_image_proc`, RViz — run with `use_sim_time: True`. The two
bridges do not, and must not: they only forward Gazebo's stamps, and the
`parameter_bridge` is the thing publishing `/clock` in the first place.

This matters because MAVROS stamps with `now()` rather than FCU time. Leave one
stamping node on wall time and, the moment the real-time factor drops below 1 on
a weak GPU, its transforms extrapolate against everyone else's.

The Python nodes are the exception. Gazebo publishes `/clock` once per
physics step, and with `use_sim_time` rclpy handles every one of those messages
in Python: `scan_relay.py` alone sat at 44% CPU relaying 2 Hz of scans. They
stamp outputs with the incoming message's header and never with `now()`, so they
run on wall time. The AR bridge's track `age` and `t` are therefore wall seconds.

## MAVROS 2 parameters live on the plugin sub-nodes, not on mavros_node

`mavros_node` starts each plugin as its own sub-node, so a plugin parameter is
addressed as `<plugin>.<param>` under a wildcard node key. That is why
`apm_config.yaml` looks like

```yaml
/**/local_position:
  ros__parameters:
    frame_id: "map"
```

and not like a flat dict on `mavros`. Passing `local_position.tf.send: True` in
a `Node(parameters=[{...}])` silently does nothing - the parameter is declared
on the parent node and no plugin ever reads it. The same trap applies to the
connection parameters: they are `tgt_system` and `tgt_component`, not
`target_system_id` / `target_component_id`.

The launch file therefore includes mavros's own `node.launch`
(`share/mavros/launch/node.launch`, an XML launch file - use
`AnyLaunchDescriptionSource`) the way `apm.launch` does, with the same
`apm_config.yaml`. The plugin list is `config/mavros_plugins.yaml` instead of
`apm_pluginlists.yaml`. That stock list is a short denylist and loads ~40
plugins, which cost ~0.9 core against SITL. The allowlist of the six plugins
the scripts use takes it off the CPU chart.

## TF comes from Gazebo, not from MAVROS

`map` → `base_link` is published by the `OdometryPublisher` plugin in
`models/tricopter/model.sdf` via `<tf_topic>/model/tricopter/pose</tf_topic>`,
bridged as `gz.msgs.Pose_V` → `tf2_msgs/msg/TFMessage` on `/tf`.

MAVROS could publish the same edge from `local_position`, but its pose is in the
EKF origin frame, which is wherever the vehicle happened to be when the EKF
initialised - not the Gazebo world origin. Two publishers of the same parent →
child edge make TF non-deterministic, so only one of them may own it, and the
ground-truth one is the useful one for evaluating whatever estimator you put in
later. When you do add a visual odometry estimator, stop bridging `/tf` and let
it own `map` → `odom` instead.

## `docker compose exec` skips the entrypoint, and Ubuntu's .bashrc skips itself

Two container gotchas that together make `./run.sh sim` silently fail with
`ros2: command not found`:

1. `docker compose exec` runs the command directly - it does not go through
   `ENTRYPOINT`. Anything the entrypoint sets up (sourcing ROS, the uid remap)
   is absent, and the command runs as the image's default user, which is `root`
   unless `-u` says otherwise.
2. Ubuntu's stock `/home/ubuntu/.bashrc` starts with
   `case $- in *i*) ;; *) return;; esac`, so appending `source setup.bash` to it
   does nothing for `bash -lc "..."`.

So the environment goes in `/etc/profile.d/10-sim-env.sh`, which every login
shell reads with no interactivity guard, and `run.sh` passes `-u ubuntu`.

The entrypoint also runs `usermod -o -u <host uid> ubuntu` when the bind mount
is owned by a different uid. Recursively chowning the mount instead would
rewrite the ownership of the host's own files, which is the wrong direction to
fix the mismatch in.

## The OAK-D cannot see anything closer than 0.7 m

Stereo disparity saturates: at 800p with a 7.5 cm baseline the nearest
resolvable depth is about 0.7 m, and closer than that the camera returns
nothing at all - not a wrong number, an empty pixel. `<depth_camera><clip>
<near>0.7</near>` in `models/tricopter/model.sdf` reproduces that, deliberately.

This sets the avoidance geometry. At `WPNAV_SPEED 300` (3 m/s) and
`WPNAV_ACCEL 250` (2.5 m/s²) the braking distance is v²/2a = 1.8 m, so the last
useful warning arrives 0.7 m before contact and the drone needs 2.5 m of notice
in total. That is where `AVOID_MARGIN 2.5` comes from. Lower the margin and the
aircraft will stop inside things.

Extended disparity brings the minimum to ~0.35 m at the cost of the far range;
it is a depthai pipeline setting, not something the sim models.

## Detection range is limited by pixels, not by the depth sensor

The stereo pair ranges to 12 m reliably and 30 m usably, so it is tempting to
plan the search pattern around that. A person detector cannot use that range.

With `horizontal_fov` 1.2008 rad over 640 px the focal length is
fx = 320 / tan(0.6004) = 466 px. A person is about 0.42 m across the shoulders,
so they subtend 466 × 0.42 / z = 196 / z pixels. At 12 m that is 16 px wide,
below what any of the usual MobileNet/YOLO input sizes will fire on. The
practical floor is about 24 px, which puts the real detection horizon at

    z = 466 × 0.42 / 24 = 8.2 m

`scripts/spatial_detector.py` enforces this with its `min_pixel_width`
parameter, so the simulated detector goes quiet at the same range the real one
does. Aisle spacing in the search pattern should be sized off 8 m, not 12 m.
Only larger targets benefit from the 12-30 m depth.

## Gazebo's depth noise is the wrong shape for stereo

`<noise><stddev>` on a depth camera is a constant in metres. Real stereo error
grows with the square of range, because depth is inversely proportional to
disparity: σ_z ≈ z²·σ_d/(b·f). For a 7.5 cm baseline, fx 466 px and 1/8 px
disparity resolution that is 9 cm at 5 m, 0.5 m at 12 m and 3.2 m at 30 m.

So the model has no Gazebo noise. Gazebo publishes clean depth to
`/camera/depth/ideal/image_raw`, and `scripts/depth_noise.py` republishes it on
`/camera/depth/image_raw` with the z² error, correlated over 8×8 px patches like
stereo matching errors. Past 12 m patches also drop out, up to 50 % at 30 m.
The constants live in `scripts/stereo.py`, shared with the detector and demo.
It costs about 0.3 core at 15 Hz.

Everything downstream sees 0.7-30 m depth, but the scan for avoidance is still
capped at 12 m in `launch/perception.launch.py`. Past that the error is bigger
than the avoidance margin.

## Remaps do not reach mavros plugin sub-nodes

`IncludeLaunchDescription` has no `remappings` argument, and the usual
workaround - wrapping the include in a `GroupAction` with
`SetRemap(src="/mavros/obstacle/send", dst="/scan")` - launches fine and does
nothing. Verified at runtime: `/scan` had 0 subscribers and the plugin still
listened on `/mavros/obstacle/send`. The remap is attached to `mavros_node`, but
each plugin is its own sub-node and does not apply it.

Publishing the scan directly onto `/mavros/obstacle/send` is not enough either:
`pointcloud_to_laserscan` publishes BEST_EFFORT, the plugin subscribes RELIABLE,
and ROS 2 never matches a RELIABLE subscriber to a BEST_EFFORT publisher.
Neither node exposes a `qos_overrides` parameter for that topic. ArduPilot shows
it as `Arm: PRX1: No Data`. `scripts/scan_relay.py` bridges the two QoS
profiles. Without it `PRX1_TYPE 2` in `config/tricopter.parm` is inert and, worse,
blocks arming.

Related: `pointcloud_to_laserscan` and `depth_image_proc` both subscribe lazily,
only while their output has a subscriber. With nobody on the scan, no cloud is
computed at all, which is why `/camera/depth/points` read 0 Hz in the smoke test
until something actually subscribed.

## libgz-sim8-dev is not in the ROS apt repo

`ros-jazzy-ros-gz` installs from `packages.ros.org`, which makes it look like
Gazebo Harmonic is fully covered there. The development packages are not:
`libgz-sim8-dev` and `gz-tools2` - both needed to compile `ardupilot_gazebo` -
live only on `packages.osrfoundation.org/gazebo/ubuntu-stable`. Building without
that repo fails with a bare `E: Unable to locate package libgz-sim8-dev`.

## ArduPilot has no Copter-4.6 branch, only Copter-4.6.x tags

`git clone --branch Copter-4.6` fails with `Remote branch Copter-4.6 not found`.
The stable line exists as tags - `Copter-4.6.0` through `Copter-4.6.3` - while
the newest branch head is `Copter-4.5`. `ARDUPILOT_REF` in the Dockerfile is
pinned to a tag for that reason.

Editing that `ARG` line invalidates every layer below it, including the ~5 min
apt install, because an `ARG` instruction is itself a cache step. Override it at
build time instead when experimenting:

```bash
ARDUPILOT_REF=Copter-4.6.2 docker compose build
```

## ardupilot_gazebo requires GStreamer even when nothing streams video

`CMakeLists.txt:88` is `pkg_check_modules(GST REQUIRED gstreamer-1.0
gstreamer-app-1.0)` - unconditional, with no option to turn it off. Only
`GstCameraPlugin` links against it, and this project never loads that plugin,
but cmake configure still aborts with:

```
Package 'gstreamer-1.0', required by 'virtual:world', not found
```

So `libgstreamer1.0-dev` and `libgstreamer-plugins-base1.0-dev` have to be in
the image. They sit in their own layer *after* the ArduPilot waf build so that
adding them does not invalidate the slowest layer.

## SITL simulates a 3S battery unless told otherwise

`SIM_BATT_VOLTAGE` defaults to 12.6 V. With the 6S `BATT_LOW_VOLT` settings the
first arm attempt fails with `Battery 1 low voltage failsafe`. The parm file sets
`SIM_BATT_VOLTAGE 25.2` and `SIM_BATT_CAP_AH 5.2`.

## Pruning the BuildKit cache makes every later Dockerfile edit a full rebuild

`docker builder prune -af` frees the space, but the next change - even one `ENV`
line at the end - rebuilds ArduPilot and ardupilot_gazebo from scratch. On a
nearly full disk that rebuild is what runs out of space, and a killed build
leaves its cache records "in use" and unprunable until the daemon restarts. For
a metadata-only fix, layer it on the existing image instead:
`FROM wh-hackathon-sim:latest` plus the `ENV`, tagged back to the same name.

## Gazebo's default collision checker is most of the warehouse's cost

The real-time factor of the full stack at a 1 ms step was ~0.4. Measured one
change at a time on this 8-core host:

1. The empty warehouse alone ran at RTF 0.54. DART's default FCL collision
   detector tests the static racks, walls and boxes against each other every
   step. `<dart><collision_detector>bullet</collision_detector></dart>` in the
   world's `<physics>` gives 1.01.
2. The `gz-sim-contact-system` plugin cost 0.54 → 0.68 on its own. Nothing read
   contacts, so it is gone.
3. Every rclpy node with `use_sim_time` burns ~0.5 core on `/clock`. The rest of
   that story is in "Everything runs on sim time".
4. The step went from 1 ms to 2 ms: RTF 0.93 with everything running. Lock-step
   makes each step one flight-controller tick, so SITL sees a 500 Hz gyro.
   ArduPilot refuses to arm if the gyro rate is below 1.8x `SCHED_LOOP_RATE`
   (`Gyro 0 rate 500Hz < loop ratex1.8 720Hz`). `config/sitl.parm` therefore sets
   the loop to 250 Hz.

Rendering was not the cost. Gazebo does use the GPU: `nvidia-smi` shows gz sim
holding GPU memory. The `libEGL warning: pci id ... driver (null)` line at
startup is Mesa probing devices and is harmless.

Nothing else left in the stack can move to the GPU. Gazebo's physics is CPU-only,
and ROS message transport has no GPU path. What remains is gz sim at ~1.7 cores
and ~0.2 core for each of the other processes.

## FastDDS sends a 1 MB image over UDP unless its shared-memory segment is bigger

FastDDS's built-in shared-memory transport uses 512 KB segments. A 640x400
`32FC1` depth frame is 1 MB and a colour frame 768 KB, so both fell back to
fragmented UDP over loopback. The C++ depth pipeline kept up. rclpy
subscribers got 5-11 Hz of the 15 Hz: the smoke test, and `spatial_detector.py`
with it. `config/fastdds.xml` sets 16 MB segments, and `docker-compose.yaml`
points `FASTRTPS_DEFAULT_PROFILES_FILE` at it. Every topic now arrives at 15 Hz.
This works because the container uses `ipc: host` and has a 16 GB `/dev/shm`.
A process outside the container falls back to UDP, which is still correct, only
slower.

## MAVROS local position has no orientation here

`/mavros/local_position/pose` always carries the orientation 0, 0, 0, 1. MAVROS
fills it from the imu plugin, and that plugin is not in the plugin allowlist.
The position is correct. `scripts/flight.py` reads yaw from the `map` ->
`base_link` transform instead. Before this, turning in place waited forever for a yaw that never came

## ArduCopter will not arm without the proximity scan

With `PRX1_TYPE 2`, arming fails with `PRX1: No Data` until
`/mavros/obstacle/send` is publishing. The scan comes from the perception
launch, so start the sim with perception on. A takeoff command sent while
already flying is rejected with MAV_RESULT 4

## YOLO11-pose ONNX keypoint confidences are already probabilities

The export `yolo export model=yolo11n-pose.pt format=onnx imgsz=416,640` has one
output of shape (1, 56, 5460). Each column is cx, cy, w, h and the person score,
then 17 x, y, confidence triples. Both the score and the keypoint confidences
have the sigmoid applied already, so thresholding them at 0.5 is correct. The
boxes come before NMS. On this machine's CPU, onnxruntime with 2 threads takes
~86 ms a frame while the sim runs, so the detector reaches ~3.5-4 Hz of its 5 Hz

## Fuel models download from the version-less URL

`https://fuel.gazebosim.org/1.0/OpenRobotics/models/<name>.zip` works. The
`/tip/files.zip` form returns 404. `MaleVisitorPhone` is a skinned DAE and
does not render as a static include, so it is not used

## A shelf deck at flight altitude is missing from the 2D map

The rack decks are 7 cm plates, the third one at 2.45-2.52 m. At 2.5 m the camera
is at 2.49 m, level with that deck, so it only ever sees the deck's front edge.
Flying along an aisle, that edge is side-on and gives no depth points. So
`pointcloud_to_laserscan` has nothing there, and the grid mapper marks the rack
face free wherever a slot has no cargo. In a 2.5 m test flight, the ground-truth
clearance check (`./run.sh clearance`) found the planner had put a waypoint
0.25 m from a deck edge. The prop disc crossed a deck edge 4 times, by up to 5 cm.
At 2.8 m the camera looks down onto the deck's top surface, which does give
points. The scan band goes down to -0.5 m, so that surface is inside it. The
same holds for any thin horizontal part level with the camera, such as a real
rack's beams

## Gazebo's set_pose service takes about half a second per call

`gz service -s /world/warehouse/set_pose` from the CLI returns after ~0.5 s,
most of it process start-up and discovery. A person walked in 0.2 s steps
therefore moves at a third of the intended speed. `walk_person.py` moves by
elapsed time instead of by step count

## A rack upright is cleared from a 2D grid by beams passing beside it

At 2.8 m the scan slice (2.3-3.25 m) passes through the open shelves between
decks. A 0.1 m upright fills half of a 0.2 m cell. Each scan hits it once but
also sends beams past it through the same cell, and those count as misses. So
the cell kept dropping back to free, and the planner flew within 0.3 m of
uprights: 5 contacts in one run. Making cells above a log-odds of 2.5 clear ten
times slower fixed it. Making them never clear did not: people do show up in the
slice now and then, and a person who walked away left a permanent obstacle that
blocked the view of their old spot

## A 2D map cannot say whether the camera could see a person

The occupancy grid is one slice at flight height. A warehouse shelf below the
slice, or a 1.8 m garden hedge, never reaches it, so the map reports a clear
line where the view is solid. People standing still behind one were marked lost
for it. The frame's own depth image answers the question directly: sample the
pixels where the person should be and compare with their distance. Nothing to
accumulate, nothing to age, and it fails exactly where the detector fails


## A mapped place is not a searched place

Frontier exploration of the warehouse ran out of frontiers 78 s after take-off,
reported the map complete, and had found 2 of the 6 people. It was right about
the map: 14071 of the grid's cells were known free, the whole 32 x 20 m hall,
and every unknown cell left was outside the walls. A level laser slice sees a
hall from a handful of positions, because each scan reaches to the far wall,
while the camera that finds people covers a 0.6 rad cone about 7 m deep. The two
finish at completely different times, and nothing in the occupancy grid records
the difference. Coverage (`scripts/coverage.py`) is the second grid that does,
and the explorer only stops once that one is finished too


## The escape hatch out of the clearance margin was a hole in it

The drone may end a leg inside the 1.0 m margin the planner keeps around
obstacles, so `passable_cells()` marked a square around its current cell
passable to let it leave. The square was unconditional and 2 x margin wide - at
0.2 m cells, 2.2 x 2.2 m of margin switched off wherever the drone happened to
be. Next to a rack that is a hole the path planner routes straight through: run
8 touched `rack_2_1/upright_3_c` twice, -0.21 m at (-3.9, 3.2), while flying to
a watch station 1.3 m from the post. Runs 3 to 7 never hit anything only because
they never planned from that close to a rack. An escape has to be relative, not
absolute: inside the box the drone may only cross cells whose distance to the
nearest obstacle is at least the distance it already has, so it can always get
out and can never get closer

Closing the hole moved the cost elsewhere: run 9 had 0 contacts but 13 "could
not reach station" warnings against run 8's 1. Watch stations were picked from
every cell the breadth-first search could reach, and that search starts inside
the escape box, so spots only reachable through it were chosen. By the time the
drone had flown to the previous station the box was somewhere else and the goal
was unreachable. An escape is for crossing, not for aiming at, so watch
stations are now restricted to cells clear on their own (`clear_cells()`) while
paths may still cross the box. Restricting frontier goals the same way was a
mistake and was reverted: a frontier cell is by definition next to unknown
space, unknown space is usually right behind an obstacle, and demanding a full
1.0 m of room discards most real frontiers. Run 10 then never exhausted them at
all - 585 legs in its 600 s budget, so the coverage phase never started and the
score fell to 1 of 6


## Exploration was chasing frontiers on the other side of the wall

Runs 9 to 13 never finished exploring: all 600 s went on frontier legs, so the
coverage phase never started and the score fell to 4 of 6. The goals tell the
story - (-7.1, 10.5), (7.7, 11.5), (-0.5, -13.1), all outside a hall that ends
at y = +-10, while the drone itself never left it. A level laser slice maps the
ground outside through every gap in a wall, those cells are free and next to
unknown, and the breadth-first search calls them reachable, so the planner kept
picking them. The map therefore kept growing and no frontier ever ran out. Run 8
only escaped it because the old oversized escape box let the drone squeeze
through and clear them.

Two things were missing. The explore loop wrote a goal off only when a leg
outright failed, so a goal it could approach but never arrive at was chased for
ever - it now gives up after GIVE_UP legs on the same one, the rule travel()
already used. And "still exploring" was tested as "did the known-cell count go
up at all", which map noise guarantees for ever; it now takes GROWTH cells of
real growth within STALE seconds


## A weak detection is worth more than no detection

The detector threw away every box under 0.4, so a person at range whose box
faded produced nothing at all, and their track decayed into "lost" for want of
evidence. Keeping boxes down to 0.25 and letting them extend an existing track,
while still requiring 0.6 to start one, is the ByteTrack idea and it is worth
more here than anything in the search logic: people found while exploring went
from 22 of 30 to 23 of 24 across whole runs, and the garden's far-east person at
(9.0, -1.0), whom no earlier run had ever found, came in with 200 hits.

Measuring the detector first mattered. An earlier gate at 0.5 was set from the
range curve in spatial_detector.py, which is the ground-truth stand-in and was
not the detector running - the real YOLO scores are 0.85 median and 0.61 at the
5th percentile, so that gate would have blocked nothing at all


## Watching from farther away does not kill ghosts, it just sees less

A miss needs the whole body in frame, and from 2.8 m up the feet leave the frame
closer than about 4.2 m, so a station inside that ought never to be able to
charge one - a ghost watched from close up looked unkillable. Raising the watch
station floor from 1.5 m to 4.5 m tested that and made things worse: 5/6 and 3/6
found against 6/6 and 5/6, with the ghosts still there (121 and 862 hits). The
stations lose detections faster than they gain the ability to time out a track.

Looking at where the surviving false targets actually are says the premise was
wrong anyway. Garden target 28 at (-9.2, -1.5) with 862 hits sits 1.0 m from
person_3, who never moved, and warehouse target 20 at (-7.0, -2.5) sits 1.4 m
from person_4. They are not ghosts of people who walked away - they are
duplicate tracks of people we already have, born just outside the 1 m
merge_radius. Admitting weak boxes made more of them, because a weak box has the
worst stereo depth. The fix to try is a birth exclusion: a sighting too far to
join a track of the same label, but still close to one, should be ignored rather
than allowed to start a rival track

## The GPS-denied path needed four separate things before it would arm

`config/gps_denied.parm` existed but had never been run, and every step of
getting it airborne turned up something not in our code:

- a defaults file cannot override a value already in the SITL EEPROM, which
  persists in `.sitl/eeprom.bin` between runs. `EK3_SRC1_POSXY` stayed at 3
  (GPS) through several boots with 6 in the file. Wipe the EEPROM when switching
  in or out of the file
- an unknown parameter name in a defaults file swallows the line after it as
  well. `SIM_GPS1_ENABLE` does not exist in this build - it is `SIM_GPS_DISABLE`
  - and its presence is what silently dropped `EK3_SRC1_POSXY`
- `VISO_TYPE 0` is not "no extra hardware". It disables the visual odometry
  backend that ExternalNav reads from, so the prearm says `VisOdom: not healthy`
  no matter how fast the vision pose arrives. MAVLink is 1
- mavros's plugin is called `vision_pose`, not `vision_pose_estimate`. A name
  the allowlist does not recognise is not an error, the plugin just never loads
  and the topic has no subscriber. `ros2 topic info` showing a subscription
  count of 0 is the tell

Nothing set the EKF origin or home either, since without GNSS nothing can.
`scripts/vision_relay.py` does both, and home has to be asked for repeatedly:
it is refused until the EKF has an origin and a position it trusts, which is a
moment after the origin lands.

## Visual odometry alone does not hold the warehouse

Once it armed it took off, flew one leg to (-13.7, -4.3), and then hit `EKF
variance`, failsafed into LAND and disarmed, about 3 minutes in. The EKF held
its estimate at the origin while the airframe actually slid 4.6 m in y, so the
error is not a slow drift - the estimate stopped tracking altogether. Blank
warehouse walls give rgbd_odometry almost nothing to hold on to.

The frame question resolved itself: with ExternalNav the EKF's local frame
starts at the drone, not at the world origin, and explore.py already measures
the difference ("map -> local offset 13.46 0.04"). What breaks is the flying,
not the bookkeeping.

## A blank wall was our own doing, and texturing it bought most of the flight

The worlds were built entirely from flat-coloured boxes - no texture anywhere -
so a wall filled the frame as a single uniform grey. That is harsher than
reality: a real warehouse wall has grain, stains and scuffs. The only concession
was painted bands on the north and south walls every 4 m, which is close to the
worst possible help, because identical marks at a regular spacing are what makes
a feature matcher pair the wrong two.

scripts/gen_textures.py now generates the surfaces procedurally. Two details
mattered more than expected:

- noise amplitude has to fall as the square root of the octave scale, not as the
  scale. The usual 1/scale weighting is what stone looks like, but it leaves
  nearly all the energy in the coarsest octave, and a smooth gradient has no
  corners to track
- seams have to be segments, not lines spanning the surface. Full lines cross
  into a lattice and one intersection of a lattice looks exactly like the next

Measured against ground truth, with the constant frame offset removed:

| | before texture | after |
| --- | --- | --- |
| legs flown before failsafe | 0, timed out on the first | 5 and still going |
| median drift | estimate never left the origin while the airframe slid 4.6 m | 0.09 m over the first quarter, 0.19 m overall |
| worst single-step jump | - | 5.21 m |

So visual odometry is accurate to about 10-20 cm right up until it fails, and
then it fails all at once. That shape matters for what to build next: the
failure is not noise to be filtered, it is a discrete jump of the whole frame,
and a per-track constant-velocity filter would read it as every person in the
building accelerating at once.

## The drone flew for weeks without being in the picture

`ros_gz_sim create` puts a model into the running server, and physics, sensors
and the ArduPilot plugin all pick it up at once - `/model/tricopter/pose` and the
IMU topic appear, MAVROS arms, the camera publishes. The GUI is the one consumer
that does not. It builds its scene once from a snapshot of
`/world/<name>/scene/info` taken at startup, and afterwards learns about new
models only from the periodic `/world/<name>/state` message, which it drops when
it is loaded. So the drone was in the simulation and absent from the Entity Tree,
which reads as a spawn failure and is not one.

The launch fired the spawn on a 4 s timer, which is a guess about how long a
world of textured walls, racks and three animated meshes takes to build. The
guess loses on a slower machine, and nothing reports that it lost.

The aircraft is now part of the generated world, in `worldgen.py`, so it is in
the first snapshot and the race cannot happen.

## Half the camera rate was 2500 dead files in /dev/shm

For two sessions the wall textures were blamed for the sensor rates: colour and
depth ran at 5-7 Hz against a nominal 15, and turning the textures off appeared
to help. It was never the textures. With the textures on and nothing else
changed, the smoke test now reports 0 failures at 14-15 Hz on every sensor topic
and RTF 0.92.

The tell was `/camera/camera_info` at 15.1 Hz while `/camera/image_raw` from the
same sensor managed 5.6 Hz. Both are published by one Gazebo sensor in one step,
so the renderer was keeping up and the images were being lost after it - in
transport, not in rendering.

Fast DDS carries them over shared memory, and `config/fastdds.xml` asks for a
16 MB segment per participant because the 512 KB default is smaller than one
640x400 depth frame. A process that is killed rather than shut down never
unlinks its segment, and `docker-compose.yml` runs the container with
`ipc: host`, so every one of those orphans lands in the host's `/dev/shm` and
stays there across runs, reboot to reboot. There were 2526 of them holding
13 GB of 16 GB, the oldest a year old. With no room for a new segment Fast DDS
does not fail - it falls back to fragmented UDP, where a frame split across
dozens of datagrams loses one and is discarded whole.

Deleting only the `fastrtps_*` files took `/dev/shm` to 2.3 MB and the rates
back to nominal. `./run.sh sim` now sweeps them first, and it decides what is an
orphan by asking whether any sim is running at all, because neither side can see
the other's processes: the host's `fuser` cannot look into the container's PID
namespace, and the container cannot look out.

The sweep also has to take the `sem.fastrtps_*` files, the mutexes beside each
port. After killing a sim and deleting only `fastrtps_*`, every ROS process of the
next run logged `RTPS_TRANSPORT_SHM Error ... Failed init_port fastrtps_port7000:
open_and_lock_file failed` for the ports whose mutex had survived, and ArduCopter
refused to arm with `PRX1: No Data`. Deleting both and starting a fresh container
cleared the errors and it armed at once. Which of the two did it was not
separated.

## The arrow keys reach OpenCV from one window and not the other

`ar_preview.py` opens two windows and reads keys with `cv2.waitKeyEx`, which
returns the whole key code - plain `waitKey` keeps only the low byte, so GTK's
65361 for Left arrives as 113, which is `q`, and quits.

Even with the full code, the arrows only arrive from the minimap window. This
build of OpenCV is Qt5 (`cv2.getBuildInformation()`, `GUI: QT5`), and its image
widget scrolls itself with the arrows once the picture is larger than the
window, which the 640x400 glasses view usually is and the small minimap is not.
Nothing in our code sees those presses. I J K L does the same thing and is never
swallowed, so that is the pair to document.

## A gz-transport request times out on a node that is receiving images

`ar_preview.py` moves the wearer's camera with the `/world/<name>/set_pose`
service and reads `/wearer/image`, both through the gz-transport 13 Python
bindings. Made from the node that holds the image subscription, a request
succeeds only if no image arrives while it waits. Once one does, the request
returns `(False, )` after the full timeout, every time: with 15 images a second
that is nearly always. The same request from a second `Node` in the same process
comes back in under a millisecond while the images keep flowing, so the camera
gets two nodes, one to listen and one to ask.

The very first request from a fresh node can still miss, because service
discovery takes about 0.1 s and sometimes more. A failed move is therefore just
tried again on the next frame.

A static model moved with `set_pose` takes its sensors with it: the camera
renders from the new pose from the next frame on. Its first frame after the move
may still show the old one, so the preview discards one image after each move.

The same bindings crash the interpreter on exit about one run in six: an image
delivered on gz-transport's own thread while Python is finalizing segfaults it
(exit code 139, after the script's last line has run, so `faulthandler` catches
nothing). Unsubscribing before exit stopped it - 0 crashes in 55 runs - so
`WearerCamera.close()` does that and the preview calls it on the way out.

## Spectacles run WebXR pages, not only Lenses

Lens Studio, the only way to build a native Lens, has no Linux build. The
glasses' Browser, since the November 2025 Snap OS update (tested on
5.064.0453), supports WebXR, and a page from a secure origin reports
`immersive-ar` and `immersive-vr` as supported. Browser is a Lens in Lens
Explorer, but Lens Explorer's search does not find it - scroll to it.

In an `immersive-ar` session the Browser window stays in front of the wearer, it
is not hidden as WebXR describes. Our content has to sit where the window is not.

three.js asks for the `local-floor` reference space by default, whose origin is
on the floor: a cube at (0, 0, -1) sat at the wearer's feet. `local` puts the
origin at the head when the session starts.

## A reverse ssh tunnel is only reachable from the server itself

`ssh -R 8790:...` binds the server's 127.0.0.1 unless sshd has `GatewayPorts`,
which the netcup server leaves off, and changing it needs root. Caddy runs in a
container and reaches the host as `host.docker.internal`, the docker bridge
172.17.0.1, not the host's loopback. `web/server/docker-compose.yml` runs socat
on the host network to relay 172.17.0.1:8791 to 127.0.0.1:8790, which keeps the
feed off the public interface.

## Palm-down hands track poorly on the Spectacles

A hand held palm down, with the back facing the glasses, was often lost or given a wrong
wrist orientation, and a minimap tied to it jumped back into the air. A palm held up
toward the glasses tracks much better, so the hand minimap uses the right palm up,
reads the orientation from the middle-finger metacarpal rather than the wrist, and
rides out short dropouts

## `pgrep -f` inside `bash -c` matches its own shell

`docker compose exec sim bash -c 'pgrep -f "gz sim" || ...'` always finds a
match: the `bash -c` stays alive for the `||`, and its own command line holds the
pattern. `run.sh` brackets one letter of each pattern (`[g]z sim`), which still
matches `gz sim` but not the text `[g]z sim`.

## `docker compose exec` fails silently in a script without `-T`

With no terminal on stdin, `docker compose exec` without `-T` exits with "the
input device is not a TTY", and with stderr dropped that looks like an empty
answer. `run.sh` adds `-T` whenever stdin is not a terminal.

## `ros2 topic echo --field` prints Python's `True`

`ros2 topic echo --once --field connected /mavros/state` prints `True`, not
`true`, followed by `---`.

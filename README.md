# wh-hackaton - wallhack hackathon

A wallhack for AR glasses: a scout drone looks behind the walls and racks you
can't see past, and Spectacles draw the people it found on a minimap.

This repo is its simulation. A tricopter with an OAK-D depth camera explores
a place it is given no layout of - a warehouse or a garden behind a house -
finds people and their heads with a real neural network, and
streams where they are to the glasses. The autopilot is real
ArduPilot firmware (SITL), the world is Gazebo Harmonic, the glue is ROS 2 Jazzy.
Everything runs inside one Docker container

## What you need

- Linux with an NVIDIA GPU and its driver
- Docker with the Compose plugin
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html), so the container can use the GPU
- ~7 GB of disk for the image

## Launch it

```bash
git clone https://github.com/rhusiev/wh-hackaton.git
cd wh-hackaton
./run.sh pull     # once, downloads the ready image (~7 GB)
./run.sh up       # start the container in the background
```

Then open two terminals in the project folder:

```bash
# terminal 1 - the simulator, autopilot and the perception stack
./run.sh sim sitl:=true

# terminal 2 - once terminal 1 has settled (~30 s), fly the search pattern
./run.sh explore
```

The Gazebo window shows the drone take off and sweep all five aisles.
`./run.sh explore --strategy frontier` instead explores with no prior layout: it flies
toward whatever part of the map is still unknown. `--strategy watch` does the same,
then keeps flying between a few spots that together keep every person found in
view. The same commands fly the garden instead:

```bash
./run.sh sim world:=garden
WORLD=garden ./run.sh explore --strategy watch
WORLD=garden ./run.sh score
```

To check that everything works, or to see what the drone and the glasses see:

```bash
./run.sh smoke    # prints PASS/FAIL for every topic and link
./run.sh demo     # colour with people and head boxes, and depth, one window each
./run.sh preview  # the through-the-wall view the glasses get, minimap inset
./run.sh score    # which of the 6 hidden people were found, and how accurately
```

To build the image yourself instead of pulling, run `./run.sh build` (20-40 min)

Each part - detector network, tracker, mapper, exploration strategy - can be
replaced with your own, see "Swapping a part" in `docs/DESIGN.md`

Stop with Ctrl+C in terminal 1, and `./run.sh down` to remove the container

## Common options

```bash
./run.sh sim sitl:=true gui:=false          # no Gazebo window, faster
./run.sh sim sitl:=true rviz:=true          # RViz view of the camera, scan and detections
./run.sh sim sitl:=true depth_decimation:=1 # full-resolution depth (default 4 = 160x100)
./run.sh sim sitl:=true slam:=true          # RTAB-Map builds /map instead of the known-pose mapper
./run.sh sim sitl:=true detector:=truth     # read true positions instead of running YOLO, saves ~1 core
./run.sh sim sitl:=true detector:=none      # no detector, run your own (also mapper:=false, ar:=false)
./run.sh explore --lanes 2                  # shorter demo flight
./run.sh demo --save frame.png              # one frame to a file, no window needed
./run.sh preview --viewer -15 0 0           # glasses view from x, y, yaw; WASD walks and turns
CAMERA=/dev/video0 ./run.sh up              # pass a camera in, then:
./run.sh preview --camera 0                 # the overlay on a real camera instead of the wireframe
./run.sh preview --camera 0 --hfov 78       # match the camera's real field of view
./run.sh shell                              # a shell inside the container
```

To fly by hand instead of `explore`, run `./run.sh sim` and then `./run.sh sitl`
in the second terminal. In its MAVProxy console type `mode guided`,
`arm throttle`, `takeoff 2`

## Where things are

| Path | What |
| --- | --- |
| `models/tricopter/` | the drone and its camera |
| `models/people/`, `models/detector/` | the people to find and the YOLO11n-pose network |
| `worlds/` | the two worlds, regenerated with `python3 scripts/gen_warehouse.py` or `gen_garden.py` |
| `config/tricopter.parm` | ArduPilot parameters. After editing run `WIPE=1 ./run.sh sitl` |
| `launch/` | what `./run.sh sim` starts |
| `scripts/` | detector, mapper, AR bridge (WebSocket JSON on port 8790), explore, tests |

The AR app connects to `ws://<host>:8790`

## Troubleshooting

- **No Gazebo window** - `./run.sh up` allows X access, so run it again after a reboot or re-login, or use `gui:=false`
- **GPU errors on start** - the NVIDIA Container Toolkit is missing or Docker was not restarted after installing it
- **Drone does not arm** - wait until terminal 1 stops printing startup messages, then retry `explore`
- **Sim runs slow** - use `gui:=false`. The sim slows down as a whole, the autopilot stays in sync

## More

- [docs/DESIGN.md](docs/DESIGN.md) - why Gazebo, what is modelled, topics, the search-and-AR loop
- [docs/NOTES.md](docs/NOTES.md) - non-obvious findings about Gazebo, ArduPilot and MAVROS

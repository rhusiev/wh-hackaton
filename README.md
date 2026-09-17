# wh-hackaton - wallhack hackathon

A wallhack for AR glasses: a scout drone looks behind the walls and racks you
can't see past, and Spectacles draw the people it found on a minimap.

This repo is its simulation. A tricopter with an OAK-D depth camera explores
a warehouse, finds people and their heads with a real neural network, and
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
`./run.sh explore --frontier` instead explores with no prior layout: it flies
toward whatever part of the map is still unknown

To check that everything works, or to see what the drone and the glasses see:

```bash
./run.sh smoke    # prints PASS/FAIL for every topic and link
./run.sh demo     # colour with people and head boxes, next to depth
./run.sh preview  # the minimap and the through-the-wall view the glasses get
./run.sh score    # which of the 6 hidden people were found, and how accurately
```

To build the image yourself instead of pulling, run `./run.sh build` (20-40 min)

Stop with Ctrl+C in terminal 1, and `./run.sh down` to remove the container

## Common options

```bash
./run.sh sim sitl:=true gui:=false          # no Gazebo window, faster
./run.sh sim sitl:=true rviz:=true          # RViz view of the camera, scan and detections
./run.sh sim sitl:=true depth_decimation:=1 # full-resolution depth (default 4 = 160x100)
./run.sh sim sitl:=true slam:=true          # RTAB-Map builds /map instead of the known-pose mapper
./run.sh sim sitl:=true detector:=truth     # read true positions instead of running YOLO, saves ~1 core
./run.sh explore --lanes 2                  # shorter demo flight
./run.sh demo --save frame.png              # one frame to a file, no window needed
./run.sh preview --viewer -15 0 0           # glasses view from x, y, yaw
./run.sh shell                              # a shell inside the container
```

To fly by hand instead of `explore`, run `./run.sh sim` and then `./run.sh sitl`
in the second terminal. In its MAVProxy console type `mode guided`,
`arm throttle`, `takeoff 2`

## Where things are

| Path | What |
| --- | --- |
| `models/tricopter/` | the drone and its camera |
| `models/people/`, `models/detector/` | the people in the warehouse and the YOLO11n-pose network |
| `worlds/` | the warehouse, regenerated with `python3 scripts/gen_warehouse.py` |
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

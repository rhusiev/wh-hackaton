# wh-hackathon - wallhack hackathon

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
git clone https://github.com/UC-UFO/wh-hackathon.git
cd wh-hackathon
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
./run.sh preview  # the glasses view: the sim rendered from where you walk, people boxed
                  # through the walls, plus a minimap window
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
./run.sh preview --viewer -15 0 0           # glasses view from x, y, yaw; WASD walks, IJKL looks
./run.sh preview --camera none              # a wireframe of the mapped room behind the overlay instead
CAMERA=/dev/video1 ./run.sh up              # pass a camera in (not every node captures), then:
./run.sh preview --camera 0                 # the overlay on a real camera instead of the wireframe
./run.sh preview --camera drone             # the overlay on what the drone sees
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
| `models/wearer/` | the glasses wearer's eyes, a camera the preview walks around |
| `models/people/`, `models/detector/` | the people to find and the YOLO11n-pose network |
| `worlds/` | the two worlds, regenerated with `python3 scripts/gen_warehouse.py` or `gen_garden.py` |
| `config/tricopter.parm` | ArduPilot parameters. After editing run `WIPE=1 ./run.sh sitl` |
| `launch/` | what `./run.sh sim` starts |
| `scripts/` | detector, mapper, AR bridge (WebSocket JSON on port 8790), explore, tests |
| `web/` | the glasses' page, and the server side that serves it with the feed |

## On the Spectacles

The glasses run no Lens of ours: `web/index.html` is a WebXR page opened in the
Spectacles Browser. It draws the AR feed twice - a minimap, and the people
life-size where they are. For the life-size part the wearer is placed at the
sim's wearer start (x -14.5, y 0, facing +x) when entering AR, and walking the
room walks the sim; `?at=x,y,yaw_deg` and `?eye=` change that. In AR and VR the
minimap, 25 cm wide, shows only on a right hand held palm up, just above the
palm; on a desktop it is 0.6 m wide below eye level. Around the wearer the sim
itself is drawn too, as outlines in AR and solid in VR, from
`web/worlds/<world>.json`, which `scripts/world_layout.py` writes whenever a
world is generated. The people are the sim's own meshes, served through the
`web/people` link to `models/people`, in both modes. The drone is the sim's
model, with its edges and a cone above it drawn through walls. The world is the
one the feed names; `?world=` overrides it.

The page needs HTTPS and the glasses need to reach the feed, so both go through a
server: Caddy serves `web/` and proxies `/feed` to port 8790, which
`./run.sh glasses` forwards from this machine over ssh.

```bash
./run.sh sim sitl:=true gui:=false ar_video:=false   # the drone's image is 1 MB/s nobody looks at
./run.sh explore --strategy watch
XR_SSH="-i ~/.ssh/key user@server" ./run.sh glasses
```

Then Browser on the glasses (Lens Explorer, it does not come up in search),
open the site and tap Enter AR. A desktop browser shows the same scene without
AR, and `?feed=ws://localhost:8790` points it at a local sim.

Server side, once: `web/server/Caddyfile` goes into Caddy's config with `web/`
copied to the site root, following the `web/people` link
(`tar -h -C web -cf - index.html worlds people`), and `web/server/docker-compose.yml` runs next to it.

## Troubleshooting

- **No Gazebo window** - `./run.sh up` allows X access, so run it again after a reboot or re-login, or use `gui:=false`
- **GPU errors on start** - the NVIDIA Container Toolkit is missing or Docker was not restarted after installing it
- **Drone does not arm** - wait until terminal 1 stops printing startup messages, then retry `explore`
- **Sim runs slow** - use `gui:=false`. The sim slows down as a whole, the autopilot stays in sync

## More

- [docs/DESIGN.md](docs/DESIGN.md) - why Gazebo, what is modelled, topics, the search-and-AR loop
- [docs/NOTES.md](docs/NOTES.md) - non-obvious findings about Gazebo, ArduPilot and MAVROS

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
./run.sh pull           # once, downloads the ready image (~7 GB)
./run.sh start          # local: Gazebo, the glasses preview and the drone's camera, one window each
./run.sh start xr       # or: no Gazebo window, the AR feed goes to https://xr.r1a.nl for the Spectacles
./run.sh stop           # stops either
```

`start` starts the container, the simulator, autopilot and perception, waits for
the autopilot (~30 s), and flies the `watch` search: it explores the place with no
prior layout, then keeps flying between a few spots that together keep every
person found in view. Everything runs in the background, with its output in
`.run/<part>.log`; `./run.sh logs explore` follows one (`sim`, `explore`,
`preview`, `demo`, `glasses`). A second `start` replaces the running one.

`WORLD=garden ./run.sh start` flies the garden behind a house instead of the
warehouse. Anything after the mode goes to the sim's launch, e.g.
`./run.sh start local detector:=truth`. `./run.sh down` also removes the container.

### Step by step

`start` is these commands, which can be run by hand in separate terminals for
more control:

```bash
./run.sh up                          # the container
./run.sh sim sitl:=true              # simulator, autopilot and perception
./run.sh explore --strategy watch    # once the sim has settled
./run.sh preview                     # the glasses view: people boxed through the walls, plus a minimap
./run.sh demo                        # colour with people and head boxes, and depth
./run.sh glasses                     # or: the AR feed to xr.r1a.nl
```

`./run.sh explore` alone sweeps all five aisles on a known layout, and
`--strategy frontier` explores with no prior layout without the watching after.
With `./run.sh sim world:=garden`, give the tools `WORLD=garden` too.

To check that everything works:

```bash
./run.sh smoke    # prints PASS/FAIL for every topic and link
./run.sh score    # which of the 6 hidden people were found, and how accurately
```

To build the image yourself instead of pulling, run `./run.sh build` (20-40 min)

Each part - detector network, tracker, mapper, exploration strategy - can be
replaced with your own, see "Swapping a part" in `docs/DESIGN.md`

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
life-size where they are. A person is yellow while the drone sees them, red once
it has not for 1.5 s, and grey when lost; life-size, they show through walls.
For the life-size part the wearer is placed at the sim's wearer start (x -14.5,
y 0, facing +x) when entering AR, and walking the room walks the sim;
`?at=x,y,yaw_deg` and `?eye=` change that. `?scale=0.7` renders at
that fraction of the display's resolution (on Snap OS 5.064 the session may then
not start), and `?figures=0` leaves out the people's meshes. In AR and VR the minimap, 25 cm wide,
shows only on a right hand held palm up, just above the palm; on a desktop it is
0.6 m wide below eye level. A left hand held upright, palm away, puts the
drone's camera image with the detector's boxes in front of it, 24 cm wide; on a
desktop it stands to the right. Around the wearer the sim itself is drawn too,
as outlines in AR and solid in VR, from `web/worlds/<world>.json`, which
`scripts/world_layout.py` writes whenever a world is generated. The people are
the sim's own meshes, served through the `web/people` link to `models/people`,
in both modes. The drone is the sim's model, with its edges and a see-through
view cone from it drawn through walls. The world is the one the feed names;
`?world=` overrides it.

The page needs HTTPS and the glasses need to reach the feed, so both go through a
server: Caddy serves `web/` and proxies `/feed` to port 8790, which
`./run.sh start xr` (or `./run.sh glasses` alone) forwards from this machine
over ssh to the server. The ssh arguments go into `XR_SSH` in `.env`, which
git ignores: `XR_SSH="-i ~/.ssh/key user@server"`.

Then Browser on the glasses (Lens Explorer, it does not come up in search),
open the site and tap Enter AR. A desktop browser shows the same scene without
AR, and `?feed=ws://localhost:8790` points it at a local sim.

Server side, once: `web/server/Caddyfile` goes into Caddy's config with `web/`
copied to the site root, following the `web/people` link
(`tar -h -C web -cf - index.html worlds people`), and `web/server/docker-compose.yml` runs next to it.

## Troubleshooting

- **No Gazebo window** - `./run.sh up` allows X access, so run it again after a reboot or re-login, or use `gui:=false`
- **GPU errors on start** - the NVIDIA Container Toolkit is missing or Docker was not restarted after installing it
- **Drone does not arm** - `start` waits for this; by hand, wait until the sim stops printing startup messages, then retry `explore`
- **`start` says the autopilot is not up** - see `./run.sh logs sim`
- **Sim runs slow** - use `gui:=false`. The sim slows down as a whole, the autopilot stays in sync

## More

- [docs/DESIGN.md](docs/DESIGN.md) - why Gazebo, what is modelled, topics, the search-and-AR loop
- [docs/NOTES.md](docs/NOTES.md) - non-obvious findings about Gazebo, ArduPilot and MAVROS

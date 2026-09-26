#!/usr/bin/env bash
# Thin wrapper around the sim container.
#
#   ./run.sh start [local|xr] [launch args]
#                           everything in one go, in the background: container, sim,
#                           autopilot and the watch flight, then either the preview
#                           and camera windows (local) or the feed to xr.r1a.nl (xr)
#   ./run.sh stop           stop what start started, the container keeps running
#   ./run.sh logs [name]    follow a started part's log: sim, explore, preview, demo, glasses
#
#   ./run.sh pull           download the prebuilt image
#   ./run.sh build          build the image locally instead (slow: ArduPilot waf)
#   ./run.sh push           publish a locally built image
#   ./run.sh up             start the container in the background
#   ./run.sh sim [args]     Gazebo + bridges + MAVROS
#   ./run.sh sitl [args]    ArduPilot SITL with the MAVProxy console
#   ./run.sh explore [args] arm, take off and search the world
#   ./run.sh world [args]   regenerate a world: warehouse or garden
#   ./run.sh score [args]   compare the people found with where they really are
#   ./run.sh smoke          check a running sim against the topic contract
#   ./run.sh clearance      closest the drone gets to anything, from ground truth
#   ./run.sh walk [args]    walk a person to a new spot in a running sim
#   ./run.sh demo [args]    colour and depth side by side from a running sim
#   ./run.sh preview [args] what the AR glasses would draw, from the AR feed
#   ./run.sh glasses        tunnel the AR feed to $XR_SSH (default xr.r1a.nl), where web/ is served
#   ./run.sh shell          interactive shell
#   ./run.sh down           stop and remove the container
#
# WORLD=garden picks the world the tools read the truth of; ./run.sh sim takes
# world:=garden to fly it.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

LOGS=.run

# A killed ROS process leaves its 16 MB shared-memory segment behind, and with
# ipc: host they pile up across runs until /dev/shm is full. Fast DDS then quietly
# falls back to fragmented UDP, which drops most of a 640x400 image, so the sim
# looks slow rather than broken - 2500 orphans filling 13 GB cost us half the
# camera rate before we found them.
#
# Whether a segment is live cannot be answered from the host, whose fuser cannot
# see into the container, nor from inside it, which cannot see the host. So the
# question asked instead is whether any sim is running at all: if none is, every
# segment is an orphan. Run ROS on the host as well and this would clear it too.
sweep_shm() {
    docker compose exec -u root -T sim bash -lc \
        'pgrep -f "[g]z sim|[m]avros_node|[p]arameter_bridge" >/dev/null || rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*' \
        2>/dev/null || true
}

exec_in() {
    docker compose exec -u ubuntu -e WIPE="${WIPE:-0}" -e CONSOLE="${CONSOLE:-1}" -e MAP="${MAP:-0}" \
        -e WORLD="${WORLD:-warehouse}" ${DETACH:+-d} $([ -t 0 ] || echo -T) sim bash -lc "$1"
}

# Runs a part in the background, its output in .run/<name>.log.
spawn() {
    DETACH=1 exec_in "exec $2 > ${LOGS}/$1.log 2>&1"
}

up() {
    xhost +local:docker >/dev/null 2>&1 || true
    # Whoever owns the camera on this host, so the container's ubuntu can read it.
    [ -e "${CAMERA:-}" ] && export VIDEO_GID=$(stat -c %g "${CAMERA}")
    docker compose up -d "$@"
}

stop() {
    [ -f "${LOGS}/glasses.pid" ] && kill "$(cat "${LOGS}/glasses.pid")" 2>/dev/null || true
    rm -f "${LOGS}/glasses.pid"
    [ -n "$(docker compose ps -q sim 2>/dev/null)" ] || return 0
    # Bracketed, so the patterns do not match the bash -c that carries them.
    local parts='[r]os2 launch|[e]xplore.py|[a]r_preview.py|[c]amera_demo.py'
    local rest="${parts}|[g]z sim|[a]rducopter|[m]avproxy|[m]avros_node|[p]arameter_bridge|[s]cripts/"
    docker compose exec -u root -T sim bash -c "
        pkill -INT -f '${parts}'
        for _ in \$(seq 20); do pgrep -f '${rest}' >/dev/null || break; sleep 0.5; done
        pkill -KILL -f '${rest}'" || true
    sweep_shm
}

# On the host: the tunnel needs the user's ssh keys, and the feed is on host networking anyway.
tunnel() {
    exec ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -R 8790:localhost:8790 ${XR_SSH:-xr.r1a.nl}
}

start() {
    local mode=local
    case "${1:-}" in local|xr) mode=$1; shift ;; esac
    local world=${WORLD:-warehouse}
    up
    stop
    mkdir -p "${LOGS}"
    CONSOLE=0 spawn sim "ros2 launch launch/tricopter_sim.launch.py sitl:=true world:=${world} \
        gui:=$([ "${mode}" = local ] && echo true || echo false) $*"
    echo -n "waiting for the autopilot"
    local i
    for i in $(seq 90); do
        exec_in "timeout 3 ros2 topic echo --once --field connected /mavros/state" 2>/dev/null \
            | grep -qi true && break
        [ "${i}" = 90 ] && { echo; echo "not up after 3 min, see ./run.sh logs sim"; exit 1; }
        echo -n .
    done
    echo
    spawn explore "./scripts/explore.py --strategy watch"
    if [ "${mode}" = local ]; then
        spawn preview ./scripts/ar_preview.py
        spawn demo ./scripts/camera_demo.py
    else
        tunnel > "${LOGS}/glasses.log" 2>&1 &
        echo $! > "${LOGS}/glasses.pid"
        sleep 5
        kill -0 $! 2>/dev/null || { echo "the tunnel failed:"; cat "${LOGS}/glasses.log"; stop; exit 1; }
        echo "open https://xr.r1a.nl in the Spectacles Browser"
    fi
    echo "running ${world} in ${mode} mode, ./run.sh stop to stop it"
}

cmd=${1:-shell}
shift || true

case "${cmd}" in
    pull)  docker compose pull "$@" ;;
    build) docker compose build "$@" ;;
    push)  docker compose push "$@" ;;
    start) start "$@" ;;
    stop)  stop ;;
    logs)  tail -f "${LOGS}/${1:-sim}.log" ;;
    up)    up "$@" ;;
    down)  stop; docker compose down "$@" ;;
    sim)   sweep_shm
           exec_in "ros2 launch launch/tricopter_sim.launch.py $*" ;;
    sitl)  exec_in "./scripts/run_sitl.sh $*" ;;
    explore) exec_in "./scripts/explore.py $*" ;;
    score) exec_in "./scripts/score_search.py $*" ;;
    smoke) exec_in "./scripts/smoke_test.py $*" ;;
    clearance) exec_in "./scripts/clearance.py $*" ;;
    walk)  exec_in "./scripts/walk_person.py $*" ;;
    world) exec_in "./scripts/gen_${1:-warehouse}.py ${*:2}" ;;
    demo)  exec_in "./scripts/camera_demo.py $*" ;;
    preview) exec_in "./scripts/ar_preview.py $*" ;;
    glasses) echo "AR feed on the server's 127.0.0.1:8790, Ctrl-C to stop"
           tunnel ;;
    shell) docker compose exec -u ubuntu sim bash -l ;;
    *)     exec_in "${cmd} $*" ;;
esac

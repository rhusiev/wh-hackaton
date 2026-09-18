#!/usr/bin/env bash
# Thin wrapper around the sim container.
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
#   ./run.sh shell          interactive shell
#   ./run.sh down           stop and remove the container
#
# WORLD=garden picks the world the tools read the truth of; ./run.sh sim takes
# world:=garden to fly it.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

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
        'pgrep -f "gz sim|mavros_node|parameter_bridge" >/dev/null || rm -f /dev/shm/fastrtps_*' \
        2>/dev/null || true
}

exec_in() {
    docker compose exec -u ubuntu -e WIPE="${WIPE:-0}" -e CONSOLE="${CONSOLE:-1}" -e MAP="${MAP:-0}" \
        -e WORLD="${WORLD:-warehouse}" sim bash -lc "$1"
}

cmd=${1:-shell}
shift || true

case "${cmd}" in
    pull)  docker compose pull "$@" ;;
    build) docker compose build "$@" ;;
    push)  docker compose push "$@" ;;
    up)    xhost +local:docker >/dev/null 2>&1 || true
           docker compose up -d "$@" ;;
    down)  docker compose down "$@" ;;
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
    shell) docker compose exec -u ubuntu sim bash -l ;;
    *)     exec_in "${cmd} $*" ;;
esac

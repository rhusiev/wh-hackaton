#!/usr/bin/env bash
# Thin wrapper around the sim container.
#
#   ./run.sh pull           download the prebuilt image
#   ./run.sh build          build the image locally instead (slow: ArduPilot waf)
#   ./run.sh push           publish a locally built image
#   ./run.sh up             start the container in the background
#   ./run.sh sim [args]     Gazebo + bridges + MAVROS
#   ./run.sh sitl [args]    ArduPilot SITL with the MAVProxy console
#   ./run.sh explore [args] arm, take off and sweep the aisles
#   ./run.sh smoke          check a running sim against the topic contract
#   ./run.sh demo [args]    colour and depth side by side from a running sim
#   ./run.sh shell          interactive shell
#   ./run.sh down           stop and remove the container
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

exec_in() {
    docker compose exec -u ubuntu sim bash -lc "$1"
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
    sim)   exec_in "ros2 launch launch/tricopter_sim.launch.py $*" ;;
    sitl)  exec_in "./scripts/run_sitl.sh $*" ;;
    explore) exec_in "./scripts/explore.py $*" ;;
    smoke) exec_in "./scripts/smoke_test.py $*" ;;
    demo)  exec_in "./scripts/camera_demo.py $*" ;;
    shell) docker compose exec -u ubuntu sim bash -l ;;
    *)     exec_in "${cmd} $*" ;;
esac

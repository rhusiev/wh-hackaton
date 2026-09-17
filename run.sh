#!/usr/bin/env bash
# Thin wrapper around the sim container.
#
#   ./run.sh build          build the image (slow the first time: ArduPilot waf)
#   ./run.sh up             start the container in the background
#   ./run.sh sim [args]     Gazebo + bridges + MAVROS
#   ./run.sh sitl [args]    ArduPilot SITL with the MAVProxy console
#   ./run.sh explore [args] arm, take off and sweep the aisles
#   ./run.sh smoke          check a running sim against the topic contract
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
    build) docker compose build "$@" ;;
    up)    xhost +local:docker >/dev/null 2>&1 || true
           docker compose up -d "$@" ;;
    down)  docker compose down "$@" ;;
    sim)   exec_in "ros2 launch launch/tricopter_sim.launch.py $*" ;;
    sitl)  exec_in "./scripts/run_sitl.sh $*" ;;
    explore) exec_in "./scripts/explore.py $*" ;;
    smoke) exec_in "./scripts/smoke_test.py $*" ;;
    shell) docker compose exec -u ubuntu sim bash -l ;;
    *)     exec_in "${cmd} $*" ;;
esac

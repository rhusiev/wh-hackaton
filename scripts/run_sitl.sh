#!/usr/bin/env bash
# Start ArduCopter SITL against the Gazebo tricopter.
#
#   WIPE=1     reset the simulated EEPROM before loading tricopter.parm
#   CONSOLE=0  no MAVProxy console
#   MAP=1      MAVProxy map window
#   GPS_DENIED=1  load config/gps_denied.parm on top, so the EKF takes its position
#                 from /mavros/vision_pose/pose instead of the simulated GNSS
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${ARDUPILOT_HOME:=/opt/ardupilot}"

# sim_vehicle.py writes eeprom.bin, logs/, terrain/ and mav.tlog into the cwd.
mkdir -p "${ROOT_DIR}/.sitl"
cd "${ROOT_DIR}/.sitl"

extra=()
[[ "${WIPE:-0}" == "1" ]] && extra+=(-w)
[[ "${CONSOLE:-1}" == "1" ]] && extra+=(--console)
[[ "${MAP:-0}" == "1" ]] && extra+=(--map)
case "${GPS_DENIED:-0}" in 1|true) extra+=(--add-param-file="${ROOT_DIR}/config/gps_denied.parm") ;; esac

# gazebo-iris only supplies the JSON backend defaults; tricopter.parm is applied
# after it and is what actually defines the airframe, then sitl.parm on top.
exec "${ARDUPILOT_HOME}/Tools/autotest/sim_vehicle.py" \
    -v ArduCopter \
    -f gazebo-iris \
    --model JSON \
    --add-param-file="${ROOT_DIR}/config/tricopter.parm" \
    --add-param-file="${ROOT_DIR}/config/sitl.parm" \
    --no-rebuild \
    --out=udp:127.0.0.1:14550 \
    --out=udp:127.0.0.1:14551 \
    "${extra[@]}" "$@"

#!/usr/bin/env bash
set -e

# Give the container user the uid that owns the bind mount, so SITL logs and
# regenerated worlds stay editable on the host.
host_uid=$(stat -c %u "${WORKSPACE}" 2>/dev/null || echo "")
if [[ -n "${host_uid}" && "${host_uid}" != "0" && "${host_uid}" != "$(id -u ubuntu)" ]]; then
    usermod -o -u "${host_uid}" ubuntu
    chown -R "${host_uid}" /home/ubuntu
fi

source /opt/ros/"${ROS_DISTRO}"/setup.bash

echo "ROS ${ROS_DISTRO} + Gazebo ${GZ_VERSION} + ArduPilot SITL ready at ${WORKSPACE}"

exec sudo -u ubuntu -E -H "$@"

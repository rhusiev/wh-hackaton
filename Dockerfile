FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# NVIDIA container toolkit injects the host driver; the image only needs the
# GL/EGL loaders for ogre2 to find it.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display

ARG ARDUPILOT_REF=Copter-4.6.3
ARG ARDUPILOT_GAZEBO_REF=main
ARG WITH_RTABMAP=1

ENV ARDUPILOT_HOME=/opt/ardupilot
ENV ARDUPILOT_GAZEBO_HOME=/opt/ardupilot_gazebo
ENV GZ_VERSION=harmonic

# ros-jazzy-ros-gz comes from the ROS repo, but the Gazebo dev packages that
# ardupilot_gazebo compiles against (libgz-sim8-dev, gz-tools2) only exist in
# the OSRF repo.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential ccache cmake curl g++ gdb git ninja-build pkg-config sudo \
      libeigen3-dev libgz-sim8-dev libopencv-dev libtool rapidjson-dev \
      libegl1 libgl1 libgles2 libglvnd0 libglx0 libxext6 libx11-6 gz-tools2 \
      python3-pip python3-venv python3-dev python3-setuptools python3-wxgtk4.0 \
      python3-matplotlib \
      python3-opencv python3-yaml python3-scipy \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      ros-${ROS_DISTRO}-ros-gz-image \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-mavros \
      ros-${ROS_DISTRO}-mavros-extras \
      ros-${ROS_DISTRO}-mavros-msgs \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-foxglove-bridge \
      ros-${ROS_DISTRO}-image-transport-plugins \
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-image-pipeline \
      ros-${ROS_DISTRO}-pointcloud-to-laserscan \
      ros-${ROS_DISTRO}-tf2-tools \
      ros-${ROS_DISTRO}-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "${WITH_RTABMAP}" = "1" ]; then \
      apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-rtabmap-ros ros-${ROS_DISTRO}-octomap-ros \
      && rm -rf /var/lib/apt/lists/*; \
    fi

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

RUN pip3 install --no-cache-dir --break-system-packages \
      "empy==3.3.4" pexpect future lxml pymavlink MAVProxy dronecan websockets

# ArduPilot SITL. The waf build is the slow layer, so keep it last-but-one.
RUN git clone --depth 1 --branch ${ARDUPILOT_REF} --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git ${ARDUPILOT_HOME} \
    && cd ${ARDUPILOT_HOME} \
    && ./waf configure --board sitl \
    && ./waf copter \
    && rm -rf ${ARDUPILOT_HOME}/build/sitl/*.o

# ardupilot_gazebo's CMakeLists calls pkg_check_modules(GST REQUIRED ...)
# unconditionally, even though only its camera-streaming plugin uses GStreamer.
# Kept in its own layer so it does not invalidate the ArduPilot waf build above.
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 --branch ${ARDUPILOT_GAZEBO_REF} \
      https://github.com/ArduPilot/ardupilot_gazebo.git /tmp/ardupilot_gazebo \
    && cmake -S /tmp/ardupilot_gazebo -B /tmp/ardupilot_gazebo/build -G Ninja \
         -DCMAKE_BUILD_TYPE=RelWithDebInfo \
         -DCMAKE_INSTALL_PREFIX=${ARDUPILOT_GAZEBO_HOME} \
    && cmake --build /tmp/ardupilot_gazebo/build --target install \
    && rm -rf /tmp/ardupilot_gazebo

# The person detector's network runtime. Its own layer, so adding it did not
# invalidate the ArduPilot build above.
RUN pip3 install --no-cache-dir --break-system-packages onnxruntime

ENV PATH=${ARDUPILOT_HOME}/Tools/autotest:${ARDUPILOT_HOME}/build/sitl/bin:$PATH
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=${ARDUPILOT_GAZEBO_HOME}/lib/ardupilot_gazebo
ENV GZ_SIM_RESOURCE_PATH=${ARDUPILOT_GAZEBO_HOME}/share/ardupilot_gazebo/models:${ARDUPILOT_GAZEBO_HOME}/share/ardupilot_gazebo/worlds
ENV WORKSPACE=/workspace/wh-hackathon

# profile.d, not ~/.bashrc: Ubuntu's stock .bashrc returns early for
# non-interactive shells, so `bash -lc "ros2 ..."` would never see ROS.
RUN printf 'source /opt/ros/%s/setup.bash\ncd %s\n' "${ROS_DISTRO}" "${WORKSPACE}" \
      > /etc/profile.d/10-sim-env.sh

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR ${WORKSPACE}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]

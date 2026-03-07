FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# NVIDIA runtime env vars — ensures GPU is visible for OpenGL
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

# System dependencies for Stonefish + VirtualGL + VNC + SSH
RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential \
    libglm-dev libsdl2-dev libfreetype6-dev \
    libglew-dev libglu1-mesa-dev freeglut3-dev \
    ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    # VNC stack
    x11vnc xvfb novnc websockify \
    # SSH server (RunPod expects this)
    openssh-server \
    # Utils
    python3-pip python3-colcon-common-extensions \
    wget curl mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install VirtualGL (routes OpenGL calls to the GPU on headless servers)
RUN wget -q https://github.com/VirtualGL/virtualgl/releases/download/3.1.1/virtualgl_3.1.1_amd64.deb && \
    dpkg -i virtualgl_3.1.1_amd64.deb || apt-get install -f -y && \
    rm virtualgl_3.1.1_amd64.deb && \
    /opt/VirtualGL/bin/vglserver_config -config +s +f -t

# Configure SSH for RunPod
RUN mkdir -p /var/run/sshd && \
    echo "PermitRootLogin yes" >> /etc/ssh/sshd_config && \
    echo "PasswordAuthentication yes" >> /etc/ssh/sshd_config && \
    echo "root:chatboat" | chpasswd

# Build Stonefish from source
WORKDIR /opt
RUN git clone --depth 1 https://github.com/patrykcieslak/stonefish.git && \
    cd stonefish && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# Create ROS2 workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Clone and build stonefish_ros2
RUN cd src && \
    git clone --depth 1 https://github.com/patrykcieslak/stonefish_ros2.git && \
    cd /ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select stonefish_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# Copy project source
COPY src/ /ros2_ws/src/

# Build project packages
RUN source /opt/ros/humble/setup.bash && \
    source /ros2_ws/install/setup.bash && \
    colcon build --packages-select \
      chatboat_description \
      chatboat_stonefish \
      chatboat_control \
      chatboat_bringup \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source ROS2 on login
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

ENV DISPLAY=:99
EXPOSE 6080 22

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "chatboat_bringup", "bringup.launch.py"]

# Base image: ROS2-Humble distribution, Ubuntu 22.04
FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

# Base necessities, used to build environment 
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Simulation dependencies: Ignition Fortress, Ignition Launch, and ROS-Ignition bridge
RUN apt-get update \
    && apt-get install -y lsb-release gnupg \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y ignition-fortress libgz-launch5 ros-humble-ros-ign-bridge\
    && rm -rf /var/lib/apt/lists/*
    
# Copy in SDF files for premade simulations
COPY gaz_worlds_files /root/gaz_worlds_files

# Set up websocket server
COPY steam-websocket /root/steam-websocket
RUN cd /root/steam-websocket \
    && npm install \
    && npm run build 

# Copy in the entrypoint script
COPY entrypoint.sh /root/
RUN chmod +x /root/entrypoint.sh

# Expose the necessary ports
EXPOSE 8002 9002
# Terminal websocket server: 8002
# Gazebo websocket server: 9002

# Run the entrypoint script
ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]
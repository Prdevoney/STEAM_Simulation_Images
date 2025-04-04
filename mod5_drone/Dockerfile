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

# Copy in python scripts 
COPY mod5_drone/python_scripts /root/python_scripts

# Install Node.js 
RUN apt-get update && apt-get install -y ca-certificates gnupg \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_22.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list \
    && apt-get update \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*

# Set up websocket server
COPY steam-websocket /root/steam-websocket
RUN cd /root/steam-websocket \
    && npm install \
    && npm run build 

# Health check for load balancer 
COPY steam-healthcheck /root/steam-healthcheck
RUN cd /root/steam-healthcheck \
    && npm install \
    && npm run build

# Copy in the entrypoint script
COPY mod5_drone/entrypoint.sh /root/
RUN chmod +x /root/entrypoint.sh

# Expose the necessary ports
EXPOSE 7002 8002 9002 
# 7002: HTTP health check for load balancer
# 8002: Terminal websocket server
# 9002: Gazebo websocket server

# Run the entrypoint script
ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]
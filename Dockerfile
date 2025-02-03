# Base image: ROS2-Humble distribution, Ubuntu 22.04
FROM ros:humble-ros-base-jammy


ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

# Base necessities, used to build. 
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    git \
    && pip3 install flask \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js as well as angular
RUN apt-get update && apt-get install -y ca-certificates gnupg \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg \
    && echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_18.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list \
    && apt-get update \
    && apt-get install -y nodejs \
    && npm install -g @angular/cli@16.0.1


# Gazebo Web dependencies: angular app, gazebo launch, & gazebo garden
RUN apt-get update \
    && npm install -y gzweb \
    && git clone https://github.com/german-e-mas/angular-gzweb.git \
    && apt-get install -y libgz-launch5 lsb-release gnupg \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && sudo apt-get update \
    && sudo apt-get install -y ignition-fortress

# Clone in Husky & Turtlebot3 simulations for ROS2 humble 
RUN git clone https://github.com/husky/husky.git \
    && cd husky \
    && git checkout humble-devel \
    && cd .. \
    && git clone https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && cd turtlebot3 \
    && git checkout humble \
    && cd .. \
    && git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git \
    && cd turtlebot3_simulations \
    && git checkout humble 

# # Install Turtlebot3 for ROS1 Noetic
# RUN apt-get update && apt-get install -y \
#     ros-noetic-turtlebot3 \
#     ros-noetic-turtlebot3-msgs \
#     ros-noetic-turtlebot3-gazebo \
#     ros-noetic-turtlebot3-simulations \
#     && rm -rf /var/lib/apt/lists/*

# Set up entrypoint
COPY app.py /root/
# COPY entrypoint.sh /root/
# RUN chmod +x /root/entrypoint.sh

EXPOSE 5000 8080 4200 9002

WORKDIR /root

# ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]
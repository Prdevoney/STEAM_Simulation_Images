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
    && apt-get update \
    && apt-get install -y ignition-fortress \
    && apt-get install ros-humble-ros-ign-bridge -y 

# Clone in Husky, Turtlebot3, and ur5 packages for ROS2 humble 
RUN mkdir -p ~/ros2_ws/src \
    && cd ~/ros2_ws/src \
    && git clone https://github.com/husky/husky.git \
    && cd husky \
    && git checkout humble-devel \
    && cd .. \
    && git clone https://github.com/ROBOTIS-GIT/turtlebot3.git \
    && cd turtlebot3 \
    && git checkout humble \
    && cd .. \
    && git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git \
    && cd Universal_Robots_ROS2_Driver \
    && git checkout humble \
    && cd .. \ 
    && rosdep install --from-paths . --ignore-src -r -y \
    && colcon build 



# Set up entrypoint
COPY app.py /root/
# COPY entrypoint.sh /root/
# RUN chmod +x /root/entrypoint.sh

EXPOSE 5000 8080 4200 9002

WORKDIR /root

# ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /root

RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    && pip3 install flask \
    && rm -rf /var/lib/apt/lists/*

# Add gazebo and turtlebot 
RUN apt-get update && apt-get install -y \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    libgazebo11-dev \
    libjansson-dev \
    libboost-dev \
    imagemagick \
    libtinyxml-dev \
    mercurial \
    cmake \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install nvm and Node.js
ENV NVM_DIR=/root/.nvm
RUN curl -fsSL https://deb.nodesource.com/setup_18.x | bash - && \
    apt-get install -y nodejs

# Add node and npm to path so the commands are available
ENV NODE_PATH=$NVM_DIR/v18.x/lib/node_modules
ENV PATH=$NVM_DIR/versions/node/v18.x/bin:$PATH


ENV GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
ENV GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
ENV TURTLEBOT3_MODEL=burger


# Set up entrypoint
COPY app.py /root/
COPY entrypoint.sh /root/
RUN chmod +x /root/entrypoint.sh

EXPOSE 5000 8080

WORKDIR /root

ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]
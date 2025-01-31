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

# Install nvm and Node.js as well as angular
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash \ 
    && export NVM_DIR="$HOME/.nvm" \
    && [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm \
    && nvm install 18 \ 
    && npm install -g @angular/cli@16.0.1

# Gazebo Web dependencies: angular app, gazebo launch, & gazebo garden
RUN apt-get update \
    && npm install gzweb \
    && git clone https://github.com/german-e-mas/angular-gzweb.git \
    && apt-get install libgz-launch5 \
    && apt-get install lsb-release gnupg \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get install gz-garden


# Add gazebo and turtlebot 
RUN apt-get update && apt-get install -y \
    && rm -rf /var/lib/apt/lists/*


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

EXPOSE 5000 8080 4200 9002

WORKDIR /root

ENTRYPOINT ["/root/entrypoint.sh"]

CMD ["bash"]
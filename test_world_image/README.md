To run this container image: <br>
    `docker run --name test_world_<n> -p <host_port>:9002 -p <host_port>:8002 -it test_world_image:<tag>` <br>
    If you want to see the gazebo sim start in the actual gazebo ignition app then add `-e DISPLAY=host.docker.internal:0` brfore the -it, <br>
    you will also need to install https://sourceforge.net/projects/x-launcher/
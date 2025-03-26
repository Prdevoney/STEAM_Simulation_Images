To run this container image: <br>
    docker run --name test_world_<n> -p <host_port>:9002 -p <host_port>:8002 -e DISPLAY=host.docker.internal:0 -it test_world_image:<tag>
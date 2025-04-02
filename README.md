# STEAM_Simulation_Images
Repository holds the code for the container images and the server code used for the STEAM RE-RASSOR Education program. 


Each folder that starts with mod and test contains a unique image configuration, even though the differences may only be slight. <br>

The `steam-websocket` directory holds the `pty` websocket app that allows requests to be sent to a users container. We didn't duplicate <br> 
that folder; so, if the code ever needed to be edited it can be edited once and remain constant across all container images. To build an <br>
image you have to be in the `STEAM_Simulation_Images` directory and run the command: `docker build -t <image_name>:<tag> -f ./<directory_name>/Dockerfile .` <br>
That sets the build directory to `STEAM_Simulation_Images` so you can include `steam-websocket` in the image successfully. 

## Endpoints: 
`ws://<gateway_ip_address>/<user_id>/<module_id>/simulation `<br>
This is used to access the simulation through Gazebo web viewer <br><br>

`ws://<gateway_ip_address>/<user_id>/<module_id>/command` <br>
This is used to send commands to be executed in the users container
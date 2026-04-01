To set up docker:

FIRST:
Check the docker-compose.yml, and comment out lines that have comments "#added LINUX" if you are not on a linux system. For reference, the compose should for Arch linux x86-64 with Wayland. I'm not sure if there are differences with ubuntu. The changes made are related to wayland/x11 functionality, so ubuntu Wayland should work the same, but no clue if WSL even includes x11/wayland.

From the main directory (docker_miniproject):
THEN RUN:
``` bash
echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > mia_hand_ros2_pkgs/docker-deployment/.env
```
For Linux Wayland, also run:
``` bash
echo "XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}" >> mia_hand_ros2_pkgs/docker-deployment/.env
```

THEN cd to the docker-deployment directory:
``` bash
cd mia_hand_ros2_pkgs/docker-deployment
```
THEN to build the simulation:
``` bash
scene=custom docker compose run --build --rm miahand_mujoco
```
It will show a wrong simulation for some reason. So, use ctrl+c to stop the sim, type "exit" to leave the container shell, and then run the container again (without "--build"):
``` bash
scene=custom docker compose run --rm miahand_mujoco
```
Now it should show the simulation with the hand and a red ball in front of it.

THEN in a different terminal (also in docker-deployment folder), to start grasp script run:
``` bash
docker compose run --build --rm miahand_ros2
```
and in the shell run:
``` bash
cd src/dev/grasp_preshaping && cargo run -r -- --mode ros --pointcloud-topic /segmented_object_cloud --pointcloud-scale 1.0 --iterations 1 --publish-commands --command-backend pos_ff
```

Multiview:
The multiview system presumes launch on the Nvidia Jetson, and is not containerized-- This will be harder to set up to run on your own systems.
For using the launch script in the multiview folder, change the directory path in the .sh file as: RVIZ_CONFIG.

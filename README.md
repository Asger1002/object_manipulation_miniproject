To set up docker:

From the main directory (docker_miniproject):
FIRST RUN:

echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > mia_hand_ros2_pkgs/docker-deployment/.env

For Linux Wayland, run:

echo "XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}" >> mia_hand_ros2_pkgs/docker-deployment/.env


THEN cd to the docker-deployment directory:

cd mia_hand_ros2_pkgs/docker-deployment

THEN to run the simulation:

scene=custom docker compose run --build --rm miahand_mujoco

THEN in a different terminal (also in docker-deployment folder), to start grasp script run:

docker compose run --build --rm miahand_ros2

and in the shell run:
cd src/dev/grasp_preshaping && cargo run -- --mode ros --pointcloud-topic /segmented_object_cloud --pointcloud-scale 1.0 --iterations 1 --publish-commands --command-backend pos_ff

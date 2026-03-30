To set up docker:
FIRST RUN:

echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > mia_hand_ros2_pkgs/docker-deployment/.env

For Linux Wayland, run:

echo "XAUTHORITY=${XAUTHORITY:-$HOME/.Xauthority}" >> mia_hand_ros2_pkgs/docker-deployment/.env

THEN to run all docker services:

docker compose -f 'mia_hand_ros2_pkgs/docker-deployment/docker-compose.yml' up -d --build

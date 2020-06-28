#!/bin/bash

YEL='\033[1;33m'
RED='\033[0;31m'
GRN='\033[0;32m'
NC='\033[0m'

VIDEO_DEVICE=/dev/video0

# Get latest image
printf "${YEL}Ensuring image is up to date.${NC}\n"
docker pull auvic/polaris:polaris_shell

# Check if the container has been created
if [ ! "$( docker container ls -a | grep polaris_interactive)" ]; then
	printf "${YEL}Docker container ${RED}not found${YEL}, creating new container.${NC}\n"
	docker run -a stdin -a stdout -a stderr -i -t --name polaris_interactive --mount src="$(pwd)"/..,target=/var/polaris,type=bind --device=${VIDEO_DEVICE}:/dev/video auvic/polaris:polaris_shell
else
	printf "${YEL}Docker container ${GRN}found${YEL}.${NC}\n"
	docker start -i -a polaris_interactive
fi
printf "${YEL}Done.${NC}\n"

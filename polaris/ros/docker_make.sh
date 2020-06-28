#!/bin/bash

YEL='\033[1;33m'
RED='\033[0;31m'
GRN='\033[0;32m'
NC='\033[0m'

# Get latest image
printf "${YEL}Ensuring image is up to date.${NC}\n"
docker pull auvic/polaris:polaris_dev

# Check if the container has been created
if [ ! "$( docker container ls -a | grep polaris_development)" ]; then
	printf "${YEL}Docker container ${RED}not found${YEL}, creating new container.${NC}\n"
	docker run -it --name polaris_development --mount src=$(pwd)/..,target=/var/polaris,type=bind auvic/polaris:polaris_dev
else
	printf "${YEL}Docker container ${GRN}found${YEL}.${NC}\n"
	docker start -a polaris_development
fi
printf "${YEL}Done.${NC}\n"

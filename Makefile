.PHONY: bash roscore roslaunch image

USE_GPUS_FLAG := $(shell command -v nvidia-container-toolkit 2> /dev/null)

FS_VOLUME = kthfsdv
DOCKER_RUN_OPTIONS = \
	-it --rm \
	-p 5901:5901 -p 6901:6901 \
	--name=arcs \
	--volume="${FS_VOLUME}:/home/fs_workspace" \
	--volume="${CURDIR}:/home/fs_workspace/src"
DOCKER_IMAGE_NAME = kthfsdv/arcs:latest

UNIX_DISPLAY_CONFIG = \
	--env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="$(XAUTHORITY):/root/.Xauthority" \
	--privileged

ifdef USE_GPUS_FLAG
	UNIX_DISPLAY_CONFIG += --gpus=all
endif

bash:
	docker run $(DOCKER_RUN_OPTIONS) \
			$(if $(X),$(UNIX_DISPLAY_CONFIG)) \
			$(DOCKER_IMAGE_NAME) bash

roslaunch:
	docker run $(DOCKER_RUN_OPTIONS) \
			$(if $(X),$(UNIX_DISPLAY_CONFIG)) \
			$(DOCKER_IMAGE_NAME) bash -c \
			"roslaunch $(ARGS)"

roscore:
	docker run $(DOCKER_RUN_OPTIONS) \
			$(if $(X),$(UNIX_DISPLAY_CONFIG)) \
			$(DOCKER_IMAGE_NAME) bash -c \
			"roscore"

# Do not run this, you probably don't need it
image:
	docker build -t kthfsdv/arcs:latest -f docker/Dockerfile .
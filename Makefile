.PHONY: bash test image


FS_VOLUME = kthfsdv
CONTAINER_NAME = arcs
DOCKER_RUN_OPTIONS = \
	-p 5901:5901 -p 6901:6901 \
	--name=${CONTAINER_NAME} \
	--volume="${FS_VOLUME}:/home/fs_workspace" \
	--volume="${CURDIR}:/home/fs_workspace/src"
DOCKER_IMAGE_NAME = kthfsdv/arcs:latest

UNIX_DISPLAY_CONFIG = \
	--env="DISPLAY:${DISPLAY}" \
	--env="IS_X11_USED=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${XAUTHORITY}:/root/.Xauthority" \
	--privileged

ifdef X
	USE_GPUS_FLAG := $(shell command -v nvidia-container-toolkit 2> /dev/null)
	ifdef UNIX_DISPLAY_CONFIG
		UNIX_DISPLAY_CONFIG += --gpus=all
	endif
endif


bash:
	@docker run -it --rm $(DOCKER_RUN_OPTIONS) \
			$(if $(X),$(UNIX_DISPLAY_CONFIG)) \
			$(DOCKER_IMAGE_NAME) bash || \
	(echo Ignore the above error. Connecting to the existing container .. && \
	 docker exec -it ${CONTAINER_NAME} bash)

test:
	docker run --rm $(DOCKER_RUN_OPTIONS) \
			$(DOCKER_IMAGE_NAME) bash

# Do not run this, you probably don't need it
image:
	docker build -t kthfsdv/arcs:latest -f docker/Dockerfile .
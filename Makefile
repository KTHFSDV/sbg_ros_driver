.PHONY: bash test image integration_test


FS_WORKSPACE_VOLUME = kthfsdv
FS_PROFILE_VOLUME = kthfsdvpf
CONTAINER_NAME = arcs
DOCKER_RUN_OPTIONS = \
	-p 5901:5901 -p 6901:6901 \
	--name=${CONTAINER_NAME} \
	--volume="${FS_PROFILE_VOLUME}:/root" \
	--volume="${FS_WORKSPACE_VOLUME}:/home/fs_workspace" \
	--volume="${CURDIR}:/home/fs_workspace/src"
DOCKER_IMAGE_NAME = kthfsdv/arcs:latest

UNIX_DISPLAY_CONFIG = \
	--env="DISPLAY=${DISPLAY}" \
	--env="IS_X11_USED=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="${XAUTHORITY}:/root/.Xauthority" \
	--privileged

# X is a boolean that indicates whether we'll be using the X server
ifdef X
	USE_GPUS_FLAG := $(shell command -v nvidia-container-toolkit 2> /dev/null)
	ifdef USE_GPUS_FLAG
		UNIX_DISPLAY_CONFIG += --gpus=all
	endif
endif


bash:
	@docker run -it --rm $(DOCKER_RUN_OPTIONS) \
			$(if $(X),$(UNIX_DISPLAY_CONFIG)) \
			$(DOCKER_IMAGE_NAME) bash || \
	(echo Ignore the above error. Connecting to the existing container .. && \
	 docker exec -it ${CONTAINER_NAME} bash)

update:
	@docker pull $(DOCKER_IMAGE_NAME)

test:
	docker run --rm \
		   $(DOCKER_RUN_OPTIONS) \
		   $(DOCKER_IMAGE_NAME) \
		   bash -c "source /opt/ros/melodic/setup.bash && catkin build"

integration_test:
	docker run --rm \
		   $(DOCKER_RUN_OPTIONS) \
		   $(DOCKER_IMAGE_NAME) \
		   bash -c "source /opt/ros/melodic/setup.bash && catkin build && source devel/setup.bash && cd src/test/ && make run_tests"

# Only run this if your really need to build the image on your local computer
image:
	docker build -t $(DOCKER_IMAGE_NAME) -f .docker/Dockerfile .

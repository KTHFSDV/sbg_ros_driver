.PHONY: build, roscore, roslaunch, rqt, rviz

USE_GPUS_FLAG := $(shell command -v nvidia-container-toolkit 2> /dev/null)

DISPLAY_CONFIG = \
	--env="DISPLAY" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="$(XAUTHORITY):/root/.Xauthority" \
	--privileged

ifdef USE_GPUS_FLAG
	DISPLAY_CONFIG += --gpus=all
endif

build:
	docker build -t as2021:vnc -f docker/Dockerfile .

roscore:
	docker run -it --rm --net=host --name=roscore as2021 bash -c "roscore"

bash:
	docker run -it --rm \
		-p 5901:5901 -p 6901:6901 \
		--name=roslaunch \
		--volume="${CURDIR}:/home/fs_workspace/src" \
		kthfsdv/as2021:vnc bash

roslaunch:
	docker run -it --rm \
		-p 5901:5901 -p 6901:6901 \
		--name=roslaunch \
		--volume="${CURDIR}:/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && roslaunch $(ARGS)"

rqt:
	docker run -it --rm \
		-p 5901:5901 -p 6901:6901 \
		--name=rqt \
		--volume="${CURDIR}:/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && rqt"

rviz:
	docker run -it --rm \
		-p 5901:5901 -p 6901:6901 \
		--name=rviz \
		--volume="${CURDIR}:/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && rviz"

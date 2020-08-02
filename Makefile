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
	docker build -t as2021 -f docker/Dockerfile .

roscore:
	docker run -it --rm --net=host --name=as2021 as2021 bash -c "roscore"

roslaunch:
	docker run -it --rm \
		--net=host \
		--name=as2021 \
		$(DISPLAY_CONFIG) \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && roslaunch $(ARGS)"

rqt:
	docker run -it --rm \
        --net=host \
		--name=as2021-rqt \
		$(DISPLAY_CONFIG) \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && rqt"

rviz:
	docker run -it --rm \
        --net=host \
		--name=as2021-rviz \
		$(DISPLAY_CONFIG) \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && rviz"

.PHONY: build, roscore, roslaunch

build:
	docker build -t as2021 -f docker/Dockerfile .

roscore:
	docker run -it --name=as2021 --rm --net=host as2021 bash -c "roscore"

roslaunch:
	docker run -it \
		--gpus all \
		--name=as2021 \
		--user=$(id -u $USER):$(id -g $USER) \
    	--env="DISPLAY" \
		--env="LIBGL_ALWAYS_INDIRECT=0" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="LIBGL_ALWAYS_SOFTWARE=1" \
		--ipc=host \
    	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(XAUTHORITY):/root/.Xauthority" \
		--rm \
		--privileged \
		--net=host \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && roslaunch $(ARGS)"

rqt:
	docker run -it \
		--gpus all \
		--name=as2021 \
    	--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="LIBGL_ALWAYS_INDIRECT=0" \
    	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(XAUTHORITY):/root/.Xauthority" \
		--ipc=host \
		--rm \
		--privileged \
		--net=host \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && rqt"

rviz:
	docker run -it \
		--gpus all \
		--name=as2021-rviz \
    	--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--env="LIBGL_ALWAYS_INDIRECT=0" \
    	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume="$(XAUTHORITY):/root/.Xauthority" \
		--ipc=host \
		--rm \
		--privileged \
		--net=host \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && rviz"

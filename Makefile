.PHONY: init, roscore, roslaunch

init:
	docker build -t as2021 -f docker/Dockerfile .

roscore: init
	docker run -it --name=as2021 --rm --net=host as2021 bash -c "roscore"

roslaunch: init
	export LIBGL_ALWAYS_SOFTWARE=1
	docker run -it \
		--name=as2021 \
		--rm \
		--net=host \
		--volume="$(PWD):/home/fs_workspace/src" \
		as2021 bash -c \
		"source /home/fs_workspace/devel/setup.bash && roslaunch $(ARGS)"

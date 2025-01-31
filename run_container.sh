docker run -it --rm \
  --privileged \
  --network=host \
  --cap-add=SYS_PTRACE \
  --security-opt=seccomp:unconfined \
  --security-opt=apparmor:unconfined \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  --volume=/mnt/wslg:/mnt/wslg \
  --ipc=host \
  --device=/dev/bus/usb:/dev/bus/usb \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e PULSE_SERVER=$PULSE_SERVER \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  --volume=/home/redhairedlynx/gitClones/humble_docker/workspace/:/home/ros/workspace \
  pi-humble


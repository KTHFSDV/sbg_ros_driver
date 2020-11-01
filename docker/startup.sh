#!/bin/bash
set -e

vncserver -kill $DISPLAY &> $STARTUPDIR/vnc_startup.log || rm -rfv /tmp/.X*-lock /tmp/.X11-unix &> $STARTUPDIR/vnc_startup.log

# Set vnc password
echo | vncpasswd -f > /root/.vnc/passwd

vncserver $DISPLAY -geometry $VNC_RESOLUTION -passwd /root/.vnc/passwd &> $STARTUPDIR/vnc_startup.log

$NO_VNC_HOME/utils/launch.sh --vnc localhost:$VNC_PORT --listen $NO_VNC_PORT &> $STARTUPDIR/no_vnc_startup.log &

# Set vnc desktop name
vncconfig -set desktop="kthfsdv:1" &> $STARTUPDIR/vnc_startup.log

exec "$@"

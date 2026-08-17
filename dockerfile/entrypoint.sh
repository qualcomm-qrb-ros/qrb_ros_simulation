#!/bin/bash
set -e

VNC_PASSWORD="${VNC_PASSWORD:-qrbrossim}"

mkdir -p /root/.vnc

echo "${VNC_PASSWORD}" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd

# clean up any stale lock/socket files from a previous container run
vncserver -kill :1 >/dev/null 2>&1 || true
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1

vncserver :1 -geometry "${RESOLUTION}" -depth 24

echo "noVNC web UI: http://<host_ip>:6080/vnc.html"

exec websockify --web=/usr/share/novnc/ 0.0.0.0:6080 localhost:5901

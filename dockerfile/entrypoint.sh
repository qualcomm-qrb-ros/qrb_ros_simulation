#!/bin/bash
set -e

VNC_PASSWORD="${VNC_PASSWORD:-qrbrossim}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
VNC_DISPLAY_START="${VNC_DISPLAY_START:-1}"
VNC_DISPLAY_END="${VNC_DISPLAY_END:-99}"

echo_error() {
    echo "ERROR: $*" >&2
}

is_uint() {
    [[ "$1" =~ ^[0-9]+$ ]]
}

validate_port() {
    local name="$1"
    local port="$2"

    if ! is_uint "${port}" || [ "${port}" -lt 1 ] || [ "${port}" -gt 65535 ]; then
        echo_error "${name} must be a TCP port in the range 1-65535, got '${port}'"
        exit 1
    fi
}

validate_display() {
    local name="$1"
    local display="$2"

    if ! is_uint "${display}" || [ "${display}" -lt 1 ] || [ "${display}" -gt 99 ]; then
        echo_error "${name} must be an X display number in the range 1-99, got '${display}'"
        exit 1
    fi
}

is_tcp_port_in_use() {
    local port="$1"
    local port_hex

    if command -v ss >/dev/null 2>&1; then
        ss -H -ltn 2>/dev/null | awk -v port="${port}" '
            {
                n = split($4, parts, ":")
                if (parts[n] == port) {
                    found = 1
                }
            }
            END { exit found ? 0 : 1 }
        '
        return $?
    fi

    port_hex="$(printf '%04X' "$((10#${port}))")"
    awk -v port=":${port_hex}" '
        $4 == "0A" && $2 ~ port "$" { found = 1 }
        END { exit found ? 0 : 1 }
    ' /proc/net/tcp /proc/net/tcp6 2>/dev/null
}

is_unix_x_socket_in_use() {
    local display=$((10#$1))
    local socket_path="/tmp/.X11-unix/X${display}"

    command -v ss >/dev/null 2>&1 || return 1

    ss -H -xl 2>/dev/null | awk -v socket_path="${socket_path}" '
        {
            for (i = 1; i <= NF; i++) {
                if ($i == socket_path || $i == "@" socket_path) {
                    found = 1
                }
            }
        }
        END { exit found ? 0 : 1 }
    '
}

display_conflict_reasons() {
    local display=$((10#$1))
    local x_port=$((6000 + display))
    local vnc_port=$((5900 + display))
    local reasons=()

    [ -e "/tmp/.X${display}-lock" ] && reasons+=("/tmp/.X${display}-lock exists")
    [ -e "/tmp/.X11-unix/X${display}" ] && reasons+=("/tmp/.X11-unix/X${display} exists")
    is_unix_x_socket_in_use "${display}" && reasons+=("X Unix socket for :${display} is listening")
    is_tcp_port_in_use "${x_port}" && reasons+=("X TCP port ${x_port} is in use")
    is_tcp_port_in_use "${vnc_port}" && reasons+=("VNC TCP port ${vnc_port} is in use")

    if [ "${#reasons[@]}" -eq 0 ]; then
        echo "no conflicts detected"
    else
        local reason
        printf '%s' "${reasons[0]}"
        for reason in "${reasons[@]:1}"; do
            printf ', %s' "${reason}"
        done
        printf '\n'
    fi
}

is_display_available() {
    local display=$((10#$1))
    local x_port=$((6000 + display))
    local vnc_port=$((5900 + display))

    [ -e "/tmp/.X${display}-lock" ] && return 1
    [ -e "/tmp/.X11-unix/X${display}" ] && return 1
    is_unix_x_socket_in_use "${display}" && return 1
    is_tcp_port_in_use "${x_port}" && return 1
    is_tcp_port_in_use "${vnc_port}" && return 1

    return 0
}

select_vnc_display() {
    local display

    if [ -n "${VNC_DISPLAY:-}" ]; then
        validate_display "VNC_DISPLAY" "${VNC_DISPLAY}"
        if ! is_display_available "${VNC_DISPLAY}"; then
            echo_error "X display :${VNC_DISPLAY} is already in use: $(display_conflict_reasons "${VNC_DISPLAY}")"
            exit 1
        fi
        echo "$((10#${VNC_DISPLAY}))"
        return 0
    fi

    validate_display "VNC_DISPLAY_START" "${VNC_DISPLAY_START}"
    validate_display "VNC_DISPLAY_END" "${VNC_DISPLAY_END}"
    if [ "${VNC_DISPLAY_START}" -gt "${VNC_DISPLAY_END}" ]; then
        echo_error "VNC_DISPLAY_START must be less than or equal to VNC_DISPLAY_END"
        exit 1
    fi

    for ((display = 10#${VNC_DISPLAY_START}; display <= 10#${VNC_DISPLAY_END}; display++)); do
        if is_display_available "${display}"; then
            echo "${display}"
            return 0
        fi
    done

    echo_error "No available X display found in :${VNC_DISPLAY_START}-:${VNC_DISPLAY_END}"
    exit 1
}

mkdir -p /root/.vnc
touch /root/.Xauthority

echo "${VNC_PASSWORD}" | vncpasswd -f > /root/.vnc/passwd
chmod 600 /root/.vnc/passwd

validate_port "NOVNC_PORT" "${NOVNC_PORT}"
if is_tcp_port_in_use "${NOVNC_PORT}"; then
    echo_error "Required noVNC port ${NOVNC_PORT} is already in use."
    echo "Please choose an available host port with NOVNC_PORT, for example:" >&2
    echo "  NOVNC_PORT=6081 ./scripts/docker_run.sh" >&2
    echo "  docker run --network host -e NOVNC_PORT=6081 qrb_ros_simulation:latest" >&2
    exit 1
fi

VNC_DISPLAY="$(select_vnc_display)"
VNC_PORT=$((5900 + VNC_DISPLAY))
X_PORT=$((6000 + VNC_DISPLAY))

vncserver ":${VNC_DISPLAY}" -geometry "${RESOLUTION}" -depth 24

echo "Using X display :${VNC_DISPLAY} (X TCP port ${X_PORT}, VNC port ${VNC_PORT})"
echo "noVNC web UI: http://<host_ip>:${NOVNC_PORT}/vnc.html"

exec websockify --web=/usr/share/novnc/ "0.0.0.0:${NOVNC_PORT}" "localhost:${VNC_PORT}"

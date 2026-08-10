#!/usr/bin/env bash
# start_vnc.sh — share the machine-local X display :1 over VNC so the Gazebo GUI
# can be viewed remotely, WITHOUT falling back to X11 forwarding (which has no
# direct GLX and would tunnel every rendered camera frame over the network).
#
# x11vnc attaches to the EXISTING :1 (the real Xorg on the NVIDIA card), so the
# sim keeps rendering with direct GPU acceleration and you just watch the pixels.
# Do NOT use `vncserver`/Xvnc instead: that creates a separate virtual display
# with software (llvmpipe) rendering, where ogre2 would be unusably slow.
#
# SECURITY: binds to 127.0.0.1 only (-localhost) and requires a VNC password, so
# nothing is exposed on the network. You reach it through an SSH tunnel:
#
#     # on your laptop
#     ssh -L 5900:localhost:5900 haozewang@ecen-000429919
#     # then point any VNC client at  localhost:5900
#
# RESOLUTION: the desktop is two 2560x1440 monitors side by side (5120x1440).
# Serving all of it scaled to 1920 wide would give 1920x540 -- the ultrawide
# aspect can't become 16:9. So the DEFAULT is true 1080p: clip to one 2560x1440
# monitor and scale x0.75, which is exactly 1920x1080 with no distortion.
#
# Usage:
#   ./start_vnc.sh                 # 1080p (primary monitor)  [default]
#   ./start_vnc.sh --monitor right # 1080p of the other monitor
#   ./start_vnc.sh --full          # both monitors, 1920x540
#   ./start_vnc.sh --native        # untouched 5120x1440
#   ./start_vnc.sh --scale 0.5     # override the scale factor
#   ./start_vnc.sh --stop
#
# Env: DISPLAY_NUM (:1), VNC_PORT (5900), PASSWD_FILE (~/.vnc/passwd)
set -uo pipefail

DISPLAY_NUM=${DISPLAY_NUM:-:1}
VNC_PORT=${VNC_PORT:-5900}
PASSWD_FILE=${PASSWD_FILE:-$HOME/.vnc/passwd}
MODE=1080p
MONITOR=primary
SCALE_OVERRIDE=""

while [ $# -gt 0 ]; do
  case "$1" in
    --stop)    pkill -f "x11vnc.*-display $DISPLAY_NUM" && echo "stopped x11vnc on $DISPLAY_NUM" \
                 || echo "no x11vnc running on $DISPLAY_NUM"; exit 0 ;;
    --full)    MODE=full; shift ;;
    --native)  MODE=native; shift ;;
    --monitor) MONITOR="$2"; shift 2 ;;
    --scale)   SCALE_OVERRIDE="$2"; shift 2 ;;
    -h|--help) sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1 (try --help)"; exit 1 ;;
  esac
done

command -v x11vnc >/dev/null || {
  echo "ERROR: x11vnc is not installed. Install it with:"
  echo "    sudo apt install -y x11vnc"
  exit 1; }

# The X server must already exist; x11vnc only mirrors it.
if ! DISPLAY=$DISPLAY_NUM timeout 5 xdpyinfo >/dev/null 2>&1; then
  echo "ERROR: cannot reach display $DISPLAY_NUM."
  echo "  If it exists but rejects you, grant local access from a shell with the cookie:"
  echo "    DISPLAY=$DISPLAY_NUM XAUTHORITY=/run/user/\$(id -u)/gdm/Xauthority xhost +local:"
  exit 1
fi

# ---- work out the clip/scale that yields the requested resolution ------------
# Ask xrandr which physical output covers which part of the root window, so the
# 1080p clip follows the actual monitor layout instead of hard-coded numbers.
geom_for() {   # $1 = primary|right|left  -> "WxH+X+Y" of that output, or empty
  DISPLAY=$DISPLAY_NUM XAUTHORITY=${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority} \
    xrandr --query 2>/dev/null | awk -v want="$1" '
      / connected/ {
        geo=""; for (i=1;i<=NF;i++) if ($i ~ /^[0-9]+x[0-9]+\+[0-9]+\+[0-9]+$/) geo=$i
        if (geo=="") next
        split(geo, g, "+"); x=g[2]+0
        if (want=="primary" && $0 ~ / primary /) { print geo; exit }
        if (want!="primary") { if (best=="" || (want=="right" && x>bx) || (want=="left" && x<bx)) { best=geo; bx=x } }
      }
      END { if (want!="primary" && best!="") print best }'
}

CLIP=""; SCALE=""
case "$MODE" in
  1080p)
    g=$(geom_for "$MONITOR")
    if [ -n "$g" ]; then
      w=${g%%x*}
      CLIP="-clip $g"
      # scale that monitor's width down to 1920 (2560 -> 0.75 -> 1920x1080)
      SCALE="-scale $(awk -v w="$w" 'BEGIN{printf "%.6g", 1920/w}')"
    else
      echo "WARNING: could not read monitor geometry; serving the full desktop."
    fi ;;
  full)
    fw=$(DISPLAY=$DISPLAY_NUM xdpyinfo 2>/dev/null | awk '/dimensions/{split($2,d,"x"); print d[1]}')
    [ -n "$fw" ] && SCALE="-scale $(awk -v w="$fw" 'BEGIN{printf "%.6g", 1920/w}')" ;;
  native) ;;   # no clip, no scale
esac
[ -n "$SCALE_OVERRIDE" ] && SCALE="-scale $SCALE_OVERRIDE"

# A valid VNC password file is exactly 8 bytes (DES-obfuscated) and must be
# written by `x11vnc -storepasswd`. A plaintext file (e.g. `echo pw > passwd`)
# is silently non-empty but unreadable to x11vnc, which then fails EVERY login
# with "Couldn't read password file" -- so validate the size, not just presence.
if [ ! -s "$PASSWD_FILE" ] || [ "$(stat -c %s "$PASSWD_FILE" 2>/dev/null)" != 8 ]; then
  if [ -s "$PASSWD_FILE" ]; then
    echo "WARNING: $PASSWD_FILE is $(stat -c %s "$PASSWD_FILE") bytes, not the required 8."
    echo "         It was probably written as plaintext; x11vnc cannot read it and every"
    echo "         login would fail. Regenerating it properly now."
    mv -f "$PASSWD_FILE" "$PASSWD_FILE.bad.$(date +%s)"
  else
    echo "No VNC password set yet. Create one now (stored in $PASSWD_FILE):"
  fi
  mkdir -p "$(dirname "$PASSWD_FILE")"
  x11vnc -storepasswd "$PASSWD_FILE" || exit 1
  [ "$(stat -c %s "$PASSWD_FILE" 2>/dev/null)" = 8 ] || {
    echo "ERROR: password file still malformed; aborting."; exit 1; }
fi
chmod 600 "$PASSWD_FILE" 2>/dev/null

if pgrep -f "x11vnc.*-display $DISPLAY_NUM" >/dev/null; then
  echo "x11vnc is already running on $DISPLAY_NUM (port $VNC_PORT). Use --stop to restart."
  exit 0
fi

# -noxdamage: GL/Gazebo windows often don't report damage rectangles, so poll the
#             framebuffer instead or the view freezes on a stale frame.
# -forever/-shared: survive client disconnects; allow more than one viewer.
# -localhost + -rfbauth: loopback only, password required.
# shellcheck disable=SC2086
x11vnc -display "$DISPLAY_NUM" -rfbport "$VNC_PORT" -rfbauth "$PASSWD_FILE" \
       -localhost -forever -shared -noxdamage -ncache 0 $CLIP $SCALE \
       -o "$HOME/.vnc/x11vnc.log" -bg

sleep 1
if pgrep -f "x11vnc.*-display $DISPLAY_NUM" >/dev/null; then
  cat <<EOF
x11vnc is serving $DISPLAY_NUM on 127.0.0.1:$VNC_PORT  (log: ~/.vnc/x11vnc.log)

To view from your laptop:
  1) ssh -L $VNC_PORT:localhost:$VNC_PORT $(id -un)@$(hostname -s)
  2) open a VNC client on  localhost:$VNC_PORT  (use the password you set)

Serving: ${CLIP:-full desktop} ${SCALE:-(unscaled)}  [mode: $MODE]
Root display is $(DISPLAY=$DISPLAY_NUM xdpyinfo 2>/dev/null | awk '/dimensions/{print $2}').

Other views:  --monitor right | --full (1920x540) | --native (5120x1440) | --scale F
Stop with:    ./start_vnc.sh --stop
EOF
else
  echo "ERROR: x11vnc failed to start. Last lines of ~/.vnc/x11vnc.log:"
  tail -n 15 "$HOME/.vnc/x11vnc.log" 2>/dev/null
  exit 1
fi

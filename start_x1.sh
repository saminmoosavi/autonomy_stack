#!/usr/bin/env bash
# start_x1.sh — bring up a headless GPU X server on display :1 for Gazebo/ogre2
# rendering used by run_ablation.sh / run_experiments.sh (SIM_DISPLAY defaults :1).
#
#   Run as root:   sudo bash /planning/autonomy_stack/start_x1.sh
#
# -ac  disables X access control so the sim container (XAUTHORITY blanked,
#      connects over the /tmp/.X11-unix socket) gets in with no xhost/cookie.
# vt4  a free virtual terminal (gdm holds vt1, another user vt3).
# setsid + </dev/null detaches X into its own session so it survives this script.
set -u

echo "running as: $(id -un) (uid $(id -u))"

if [ -S /tmp/.X11-unix/X1 ]; then
  echo "X1 already present — display :1 is already up. Nothing to do."
  exit 0
fi

# Log to /var/log (root-owned, always writable by root) rather than /tmp, which
# rejected root writes on this host. X writes its own detailed log via -logfile.
LAUNCH_LOG=/var/log/x1.log
XORG_LOG=/var/log/Xorg.1.log
: > "$LAUNCH_LOG" 2>/dev/null || { echo "cannot write $LAUNCH_LOG — are you root? (use: sudo bash $0)"; exit 1; }

setsid X :1 vt4 -nolisten tcp -ac -noreset -logfile "$XORG_LOG" \
       >"$LAUNCH_LOG" 2>&1 </dev/null &
pid=$!
echo "started X :1 (pid $pid) — waiting for the socket... (logs: $XORG_LOG)"

for i in $(seq 1 8); do
  sleep 1
  if [ -S /tmp/.X11-unix/X1 ]; then
    echo "X1 UP — display :1 is ready (pid $pid)."
    exit 0
  fi
  kill -0 "$pid" 2>/dev/null || break   # X died early
done

echo "X1 did NOT come up. Diagnostics:"
echo "=== launcher output ($LAUNCH_LOG) ==="
tail -n 15 "$LAUNCH_LOG" 2>/dev/null
echo "=== Xorg log ($XORG_LOG) ==="
tail -n 30 "$XORG_LOG" 2>/dev/null
echo "======================================"
exit 1

#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Push the GB10's clock to the Jackal, so ROS timestamps from both machines
# agree and AMCL stops dropping every scan.
#
#   ./fix_clock_sync.sh              # measure, fix, verify
#   ./fix_clock_sync.sh --check      # measure only, change nothing
#
# WHY: the GB10 is NTP-synced (systemd-timesyncd -> ntp.ubuntu.com). The Jackal
# is on the isolated robot network with no time source, so it free-runs. Measured
# 2026-08-10: the Jackal was 0.727 s BEHIND, which makes every TF lookup at a scan
# timestamp fail with "extrapolation into the future" and fills AMCL's message
# filter queue -> "discarding message because the queue is full".
#
# Use 192.168.131.1 (the wired robot link), NOT 172.20.0.101 from ssh_101.sh —
# that address routes over wifi and currently times out.
#
# This is a one-shot step. It does NOT survive a robot reboot; see PERMANENT
# below, printed at the end.
# ---------------------------------------------------------------------------
set -uo pipefail

ROBOT_USER="${ROBOT_USER:-administrator}"
ROBOT="${ROBOT:-192.168.131.1}"
CHECK_ONLY=0
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=1

CTL="/tmp/.jackal-ssh-%r@%h-%p"
SSH="ssh -o ControlPath=$CTL"

cleanup() { ssh -o ControlPath="$CTL" -O exit "$ROBOT_USER@$ROBOT" 2>/dev/null; }
trap cleanup EXIT

echo "=== opening a shared connection to $ROBOT_USER@$ROBOT ==="
# ControlMaster: every later command reuses this connection, so `date -s` pays a
# few ms of round trip instead of a full ssh handshake. That difference IS the
# accuracy of the fix.
ssh -o ControlMaster=yes -o ControlPath="$CTL" -o ControlPersist=120 \
    -o ConnectTimeout=8 -Nf "$ROBOT_USER@$ROBOT" || {
  echo "ERROR: cannot reach $ROBOT_USER@$ROBOT." >&2
  echo "  ping $ROBOT   # is the wired link up?" >&2
  echo "  ROBOT_USER=<name> $0   # if the account is not '$ROBOT_USER'" >&2
  exit 2
}

measure() {
  # Offset = robot_clock - midpoint(before, after), which cancels the round trip.
  local t0 t1 rt
  t0=$(date +%s.%N)
  rt=$($SSH "$ROBOT_USER@$ROBOT" 'date +%s.%N' 2>/dev/null)
  t1=$(date +%s.%N)
  [[ -n "$rt" ]] || { echo "n/a"; return 1; }
  python3 -c "print(f'{$rt - ($t0 + $t1)/2:+.3f}')"
}

echo "=== robot clock state ==="
$SSH "$ROBOT_USER@$ROBOT" 'timedatectl 2>/dev/null | grep -iE "Local time|synchronized|NTP service"
  if command -v chronyc >/dev/null; then
    echo "--- chrony ---"; chronyc tracking 2>/dev/null | head -4
    chronyc sources 2>/dev/null | head -5
  else echo "chrony not installed on the robot"; fi'

OFFSET_BEFORE=$(measure)
echo
echo "=== offset before: ${OFFSET_BEFORE}s (robot minus GB10) ==="
echo "    anything past about +/-0.05 s will break AMCL"

if [[ $CHECK_ONLY -eq 1 ]]; then
  echo "--check: stopping here, nothing changed."
  exit 0
fi

echo
echo "=== stepping the robot's clock (sudo on the robot — expect a prompt) ==="
# Two things here are deliberate, and getting either wrong is what broke the
# first version of this script (it pushed the clock 8 s, then 21 s, out):
#
#   1. `sudo -v` FIRST, as a separate command. If the timestamp is computed in
#      the same command line as the sudo that prompts, the shell expands it
#      BEFORE you type the password — so however long you take typing is baked
#      in as error. Authenticating first means the value is computed after.
#   2. A RELATIVE correction, evaluated on the robot from the robot's own clock:
#      new = robot_now - offset. Because both the read and the set happen on the
#      robot at execution time, network delay cannot leak into the result. The
#      only error left is the offset measurement itself, which is RTT-cancelled.
#
# awk, not python3, does the float arithmetic — one less thing to depend on.
step_clock() {
  local off="$1" remote
  remote=$(printf 'sudo -v; sudo -n date -s @$(date +%%s.%%N | awk -v o=%s "{printf \\"%%.6f\\", \\$1 - o}")' "$off")
  ssh -t -o ControlPath="$CTL" "$ROBOT_USER@$ROBOT" "$remote"
}

OFFSET_AFTER="$OFFSET_BEFORE"
for attempt in 1 2 3; do
  step_clock "$OFFSET_AFTER" || { echo "step failed"; break; }
  sleep 1
  OFFSET_AFTER=$(measure)
  echo "    after attempt $attempt: ${OFFSET_AFTER}s"
  awk -v o="$OFFSET_AFTER" 'BEGIN{exit !(o < 0.05 && o > -0.05)}' && break
done

echo
echo "=== offset after: ${OFFSET_AFTER}s  (was ${OFFSET_BEFORE}s) ==="

awk -v o="$OFFSET_AFTER" 'BEGIN{
  a = (o < 0) ? -o : o
  if (a <= 0.05) print "OK — the clocks agree closely enough for TF."
  else {
    printf "STILL OFF by %.3fs.\n", a
    print "  - a time daemon on the robot may be slewing it back:"
    print "      sudo systemctl stop systemd-timesyncd    # then re-run"
    print "  - or sudo re-prompted mid-step; re-run now that credentials are cached"
  }
}'

echo
echo "NOTE: a large clock jump can wedge ROS nodes that were already running on the"
echo "      robot (timers, TF buffers). If odom or TF look stale afterwards, restart"
echo "      the robot's ROS stack before starting loc."

cat <<'EOF'

=== VERIFY (on the GB10, with the robot stack up) ===
  python3 -u ~/autonomy_stack_ros_humble/clock_probe.py \
    --ros-args -r /tf:=/j100_0612/tf -r /tf_static:=/j100_0612/tf_static
  # scan / odom_msg / tf columns should all be within a few tens of ms

  python3 -u ~/autonomy_stack_ros_humble/tf_probe.py \
    --ros-args -r /tf:=/j100_0612/tf -r /tf_static:=/j100_0612/tf_static
  # expect "20/20 OK" and "VERDICT: AMCL's lookup succeeds"

=== PERMANENT (this step does not survive a robot reboot) ===
The Jackal has no internet and no chrony, so it cannot fetch time and cannot
install a client. It does not need to: the GB10 has internet, and Ubuntu already
ships systemd-timesyncd on both machines. Make the GB10 the time SERVER for the
robot network, and point the robot's existing timesyncd at it — nothing gets
installed on the robot.

  # on the GB10 — it has internet, so this works. chrony replaces timesyncd here
  # and keeps ntp.ubuntu.com upstream, while also serving the robot subnet.
  sudo apt install chrony
  echo 'allow 192.168.131.0/16' | sudo tee -a /etc/chrony/chrony.conf
  sudo systemctl restart chrony
  sudo ss -ulnp | grep :123          # confirm chrony is listening

  # on the Jackal — no install, just point the built-in client at the GB10:
  sudo mkdir -p /etc/systemd/timesyncd.conf.d
  printf '[Time]\nNTP=192.168.131.10\n' | \
    sudo tee /etc/systemd/timesyncd.conf.d/gb10.conf
  sudo timedatectl set-ntp true
  sudo systemctl restart systemd-timesyncd
  timedatectl timesync-status        # Server: 192.168.131.10, Packet count rising

timesyncd SLEWS rather than steps, so it closes a large gap slowly. Run this
script once first to get inside a second, then let timesyncd hold it there.

Until that is set up, run this script after every Jackal reboot, before `loc`.
EOF

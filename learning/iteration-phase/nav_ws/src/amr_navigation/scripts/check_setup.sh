#!/usr/bin/env bash
# Pre-flight check for amr_navigation: run AFTER the robot bringup is up,
# BEFORE starting SLAM / Nav2.
#   ros2 run amr_navigation check_setup.sh
#   ros2 run amr_navigation check_setup.sh /my_odom /my_scan /my_cmd_vel

ODOM_TOPIC="${1:-/odom}"
SCAN_TOPIC="${2:-/scan}"
CMD_TOPIC="${3:-/cmd_vel}"
WAIT=8            # seconds; DDS discovery on a Pi can take a few seconds
PASS=0; FAIL=0

ok()   { echo "  [ OK ] $1"; PASS=$((PASS+1)); }
bad()  { echo "  [FAIL] $1"; FAIL=$((FAIL+1)); }
info() { echo "  [info] $1"; }

echo "== amr_navigation pre-flight check =="
ros2 daemon start >/dev/null 2>&1

echo "-- Topics"
TOPICS=$(ros2 topic list 2>/dev/null)
for t in "$SCAN_TOPIC" "$ODOM_TOPIC" "$CMD_TOPIC" /tf /tf_static; do
  if echo "$TOPICS" | grep -qx "$t"; then ok "$t exists"; else bad "$t missing"; fi
done

echo "-- Data actually flowing (up to ${WAIT} s each)"
for t in "$SCAN_TOPIC" "$ODOM_TOPIC"; do
  if timeout "$WAIT" ros2 topic echo --once "$t" >/dev/null 2>&1; then
    ok "$t is publishing"
  else
    bad "$t published nothing in ${WAIT} s"
  fi
done

echo "-- Velocity command input"
SUBS=$(ros2 topic info "$CMD_TOPIC" 2>/dev/null | awk '/Subscription count/ {print $3}')
if [ -n "$SUBS" ] && [ "$SUBS" -ge 1 ]; then
  ok "$CMD_TOPIC has $SUBS subscriber(s) (your diff drive controller)"
else
  bad "nobody subscribes to $CMD_TOPIC: Nav2's commands would go nowhere"
fi

echo "-- TF: odom -> base_link"
if timeout "$WAIT" ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation"; then
  ok "odom -> base_link available"
else
  bad "odom -> base_link NOT available (check odometry node / frame names)"
fi

echo "-- TF: base_link -> LiDAR frame"
SCAN_FRAME=$(timeout "$WAIT" ros2 topic echo --once "$SCAN_TOPIC" 2>/dev/null | awk '/frame_id/ {print $2; exit}' | tr -d "'\"")
if [ -n "$SCAN_FRAME" ]; then
  info "scan frame_id = $SCAN_FRAME"
  if timeout "$WAIT" ros2 run tf2_ros tf2_echo base_link "$SCAN_FRAME" 2>&1 | grep -q "Translation"; then
    ok "base_link -> $SCAN_FRAME available"
  else
    bad "base_link -> $SCAN_FRAME NOT available (static TF / URDF missing)"
  fi
else
  bad "could not read frame_id from $SCAN_TOPIC"
fi

echo "-- Nodes PUBLISHING /tf (subscribers are not listed)"
ros2 topic info /tf -v 2>/dev/null | awk '
  /Node name:/      { n = $3 }
  /Endpoint type:/  { if ($3 == "PUBLISHER") print "  [info] " n }'

echo "-- Who publishes odom -> base_link (must be exactly ONE node)"
MON=$(timeout "$WAIT" ros2 run tf2_ros tf2_monitor odom base_link 2>/dev/null | grep "Node:" | sort -u)
if [ -z "$MON" ]; then
  info "tf2_monitor gave no output (not fatal)"
else
  echo "$MON" | sed 's/^/  [info] /'
  N=$(echo "$MON" | awk '{print $2}' | sort -u | wc -l)
  if [ "$N" -gt 1 ]; then
    bad "more than one node publishes TF in the odom->base_link chain: check the list above"
  fi
fi

echo
echo "== Result: $PASS passed, $FAIL failed =="
[ "$FAIL" -eq 0 ] && echo "Ready for SLAM / Nav2." || echo "Fix the FAIL items first (README section 11)."

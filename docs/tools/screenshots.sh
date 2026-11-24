#!/bin/bash
# Camera and screenshot helpers for a running coverage_sim launch with the Gazebo GUI.
# Needs xdotool and ImageMagick, and DISPLAY set to the display Gazebo runs on.
#
#   screenshots.sh camera X Y Z ROLL PITCH YAW   move the Gazebo camera (m, rad)
#   screenshots.sh top X Y Z                     look straight down on (X, Y) from height Z
#   screenshots.sh follow DIST HEIGHT ANGLE      camera DIST m from the robot, ANGLE rad off its rear
#   screenshots.sh raise gazebo|rviz             bring a window to the front
#   screenshots.sh shot FILE [scene]             save the screen; with scene, only the 3D view
set -e
source /opt/ros/${ROS_DISTRO:-lyrical}/setup.bash

camera() {
  read -r qw qx qy qz < <(python3 -c "
import math, sys
r, p, y = map(float, sys.argv[1:4])
cr, sr, cp, sp = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2)
cy, sy = math.cos(y / 2), math.sin(y / 2)
print(cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy)" "$4" "$5" "$6")
  gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
    --timeout 3000 --req "pose: {position: {x: $1, y: $2, z: $3}, orientation: {w: $qw, x: $qx, y: $qy, z: $qz}}" > /dev/null
}

robot_pose() {
  python3 - <<'PY'
import math
import rclpy
from nav_msgs.msg import Odometry
rclpy.init()
node = rclpy.create_node('screenshot_pose')
got = []
node.create_subscription(Odometry, '/ground_truth/odom', got.append, 1)
while not got:
    rclpy.spin_once(node, timeout_sec=1.0)
p = got[0].pose.pose
q = p.orientation
yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
print(p.position.x, p.position.y, yaw)
PY
}

case $1 in
  camera) camera "$2" "$3" "$4" "$5" "$6" "$7" ;;
  top) camera "$2" "$3" "$4" 0 1.5707 1.5708 ;;
  follow)
    read -r x y yaw < <(robot_pose)
    read -r cx cy pitch cyaw < <(python3 -c "
import math
x, y, yaw, d, h, s = $x, $y, $yaw, $2, $3, $4
a = yaw + math.pi + s
cx, cy = x + d * math.cos(a), y + d * math.sin(a)
print(cx, cy, math.atan2(h - 0.2, d), math.atan2(y - cy, x - cx))")
    camera "$cx" "$cy" "$3" 0 "$pitch" "$cyaw" ;;
  raise)
    pattern=$([ "$2" = rviz ] && echo RViz || echo Gazebo)
    for w in $(xdotool search --name "$pattern"); do xdotool windowraise "$w"; done
    sleep 1 ;;
  shot)
    import -window root "$2"
    # The 3D view of a 1600 x 1000 Gazebo window, without the toolbars and the side panel
    if [ "$3" = scene ]; then mogrify -crop 1176x760+0+150 +repage "$2"; fi ;;
  *) sed -n '2,10p' "$0"; exit 1 ;;
esac

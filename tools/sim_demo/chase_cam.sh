#!/usr/bin/env bash
# Keep the Gazebo GUI camera behind the robot. /gui/follow lags by metres when
# the GUI renders slowly, so the camera is placed directly instead.
export IGN_IP=127.0.0.1
BACK=${1:-3.5}; UP=${2:-1.8}
while true; do
  read -r x y yaw < <(ign model -m j100_0000/robot -p 2>/dev/null | tr -d '[]' | awk '/XYZ/{getline; x=$1; y=$2; getline; print x, y, $3}')
  [ -z "$yaw" ] && { sleep 1; continue; }
  req=$(python3 -c "
import math
x,y,yaw,b,u=$x,$y,$yaw,$BACK,$UP
cx,cy=x-b*math.cos(yaw),y-b*math.sin(yaw)
pitch=math.atan2(u-0.3,b)
cr,sr=1,0; cp,sp=math.cos(pitch/2),math.sin(pitch/2); cz,sz=math.cos(yaw/2),math.sin(yaw/2)
print('pose: {position: {x: %.3f, y: %.3f, z: %.3f}, orientation: {x: %.5f, y: %.5f, z: %.5f, w: %.5f}}'%(cx,cy,u,-sp*sz,sp*cz,cp*sz,cp*cz))")
  ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera --reptype ignition.msgs.Boolean --timeout 500 --req "$req" >/dev/null 2>&1
done

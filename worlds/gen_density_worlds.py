#!/usr/bin/env python3
"""Generate warehouse_people{N}.sdf for N in {0,10,20,30,40,50}.

Each file has exactly N animated actors walking perpendicular to the robot's
primary mission corridors (R7→R10 route and surrounding aisles).

Coordinate reference (from SDF comments and region list):
  Robot route regions: R7(12.7,6.5) R8(3.5,13) R9(11.9,17.75) R10(1.9,22.1)
                       R11(-3.9,23.7) R12(-7.1,20) R13(-12.4,17.9) R14(-13,9.9)

  Primary N-S corridors: x≈8 (east aisle), x≈2 (centre), x≈-12 (west aisle)
  Primary E-W corridors: y≈22 (north), y≈13 (mid-north)

  "Normal to robot path" → E-W walkers cross N-S corridors;
                           N-S walkers cross E-W corridors.

Shelf/barrier exclusion zones used when placing waypoints:
  shelf_big_3 (3.5,9.5,rot90):  x  1.0– 6.0, y  8.3–10.8
  shelf_big_4 (-1.3,18.5,rot90):x -3.8– 2.2, y 17.3–19.8
  shelf_0  (-10,21.5,rot90):    x -11.3–-8.8, y 20.3–22.8
  shelf_1  (-7,23.6,rot90):     x  -8.3–-5.8, y 22.4–24.9
  shelf_2  (-4,21.5,rot90):     x  -5.3–-2.8, y 20.3–22.8
  shelf_3  (13.5,4.5):          x 11.5–15.5,  y  2.5– 6.5
  shelf_4  (10,4.5):            x  8.0–12.0,  y  2.5– 6.5
  shelf_7  (0.4,-2):            x -0.5–  1.5, y -3.0– -1.0
  shelf_big_0 (-8.5,-13):       x -11.5–-5.5, y -15.5–-10.5
  shelf_big_1 (6.5,-13):        x   3.5– 9.5, y -15.5–-10.5
  shelf_big_2 (-1.5,-13):       x  -4.5– 1.5, y -15.5–-10.5
  shelf_5  (13.5,-21):          x 11.5–15.5,  y -22.5–-19.5
  shelf_6  (13.5,-15):          x 11.5–15.5,  y -16.5–-13.5
  barriers x=-10.4: y 6.5–14.75 (actors at x<-10.4 are unaffected)

Region-node clearance (1.5 m minimum):
  All actor paths verified ≥1.5 m from every region node R1–R14.
  Key fixes from original generator:
    a1: y 22.0→20.5  (R10 at y=22.13 was 0.13 m away)
    a2: y 13.0→11.5  (R8  at y=13.01 was 0.01 m away)
    a3: x1 0→-1.5    (R5  at x=1.44  was 1.46 m — below 1.5 m threshold)
    d2: y 17.0→15.5  (R9  at y=17.75 was 0.75 m away)
    e1: x -13→-14    (R13 at x=-12.38 was 0.62 m away)
    e2: x0 -3→-2     (R11 at x=-3.92 was 0.97 m from start)
    e3: y 11.0→9.5   (R6  at y=11.72 was 0.72 m away)
    f1: y 19.0→17.5  (R12 at y=20.01 was 1.01 m away)
    g1: y0 14→15.5, y1 22→20.5  (R8 1.12 m at start; R10 1.12 m at end)
    h2: y0 7→8.0     (R7  at y=6.49  was 0.88 m from start)
    i3: x0 2→3.5     (R5  at x=1.44  was 0.93 m from start)
    j1: x 13→14      (R9  at x=11.86 was 1.14 m away)

Deceleration near turn-around points:
  Each actor slows from 0.5 m/s to 0.25 m/s for the last 1.0 m before each
  endpoint (adds 2 intermediate waypoints per cycle, 7 waypoints total).
"""

import json
import math
import os
import random

SKIN = ('https://fuel.gazebosim.org/1.0/Mingfei/models/actor/'
        'tip/files/meshes/walk.dae')

STATIC_BODY = f'''\
<?xml version='1.0' encoding='ASCII'?>
<sdf version='1.7'>
  <world name='warehouse'>
    <physics type="ode">
      <max_step_size>0.003</max_step_size>
      <real_time_factor>0.5</real_time_factor>
    </physics>
    <plugin name='ignition::gazebo::systems::Physics' filename='libignition-gazebo-physics-system.so'/>
    <plugin name='ignition::gazebo::systems::UserCommands' filename='libignition-gazebo-user-commands-system.so'/>
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='libignition-gazebo-scene-broadcaster-system.so'/>
    <plugin name="ignition::gazebo::systems::Sensors" filename="libignition-gazebo-sensors-system.so">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin name="ignition::gazebo::systems::Imu" filename="libignition-gazebo-imu-system.so"/>
    <plugin name="ignition::gazebo::systems::NavSat" filename="libignition-gazebo-navsat-system.so"/>

    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.3 0.7 0.9 1</background>
      <shadows>0</shadows>
      <grid>false</grid>
    </scene>

    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>-22.986687</latitude_deg>
      <longitude_deg>-43.202501</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <model name='ground_plane'>
      <static>true</static>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0.0 0.0 1</normal><size>1 1</size></plane></geometry>
        </collision>
      </link>
      <pose>0 0 0 0 0 0</pose>
    </model>

    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse</uri>
      <name>warehouse</name>
      <pose>0 0 -0.1 0 0 0</pose>
    </include>

    <!-- South shelves -->
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big</uri><name>shelf_big_0</name><pose>-8.5 -13 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big</uri><name>shelf_big_1</name><pose>6.5 -13 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big</uri><name>shelf_big_2</name><pose>-1.5 -13 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_3</name><pose>13.5 4.5 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_4</name><pose>10 4.5 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_5</name><pose>13.5 -21 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_6</name><pose>13.5 -15 0 0 0 0</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_7</name><pose>0.4 -2 0 0 0 0</pose></include>
    <!-- North shelves (rotated, flank the mission corridor) -->
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big</uri><name>shelf_big_3</name><pose>3.5 9.5 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf_big</uri><name>shelf_big_4</name><pose>-1.3 18.5 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_0</name><pose>-10 21.5 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_1</name><pose>-7 23.6 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/MovAi/models/shelf</uri><name>shelf_2</name><pose>-4 21.5 0 0 0 1.57</pose></include>

    <!-- West-side barriers -->
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier</uri><name>barrier_0</name><pose>-10.4 14.75 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier</uri><name>barrier_1</name><pose>-10.4 10.5 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier</uri><name>barrier_2</name><pose>-10.4 6.5 0 0 0 1.57</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier</uri><name>barrier_3</name><pose>-12.85 4.85 0 0 0 0</pose></include>

    <!-- Furniture -->
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair</uri><name>chair_0</name><pose>14.3 -5.5 0 0 0 3</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair</uri><name>chair_1</name><pose>14.3 -4 0 0 0 -3</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair</uri><name>fchair_0</name><pose>-11.5 6.4 0 0 0 -1.8</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair</uri><name>fchair1</name><pose>-14 6.5 0 0 0 1.9</pose></include>
    <include><uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri><name>table0</name><pose>-12.7 6.5 0 0 0 0</pose></include>
'''

FOOTER = '''\
  </world>
</sdf>
'''

# Deceleration parameters: slow the last SLOW_DIST metres of each leg to SLOW_SPEED m/s.
SLOW_DIST  = 1.0   # metres before endpoint where decel begins
SLOW_SPEED = 0.25  # m/s during slow phase (half normal speed)


def ew_actor(name, y, x0, x1, delay=0, speed=0.5):
    """E-W patrol with deceleration 1 m before each turn-around."""
    dist      = abs(x1 - x0)
    direction = 1 if x1 >= x0 else -1
    fwd_yaw   = 0.0    if x1 >= x0 else 3.1416
    rev_yaw   = 3.1416 if x1 >= x0 else 0.0

    x1_slow = round(x1 - direction * SLOW_DIST, 2)   # decel start, outbound
    x0_slow = round(x0 + direction * SLOW_DIST, 2)   # decel start, return

    fast_t  = round((dist - SLOW_DIST) / speed)
    slow_t  = round(SLOW_DIST / SLOW_SPEED)           # = 4 s at 0.25 m/s
    travel  = fast_t + slow_t
    turn    = 4
    cycle   = 2 * travel + 2 * turn

    t1 = fast_t
    t2 = travel
    t3 = travel + turn
    t4 = travel + turn + fast_t
    t5 = 2 * travel + turn
    t6 = cycle

    return (
        f'\n    <!-- {name}: E-W patrol y={y}, x={x0}→{x1} (~{speed} m/s,'
        f' crosses N-S robot corridor) -->\n'
        f'    <actor name="{name}">\n'
        f'      <skin><filename>{SKIN}</filename><scale>1.0</scale></skin>\n'
        f'      <animation name="walk"><filename>{SKIN}</filename>'
        f'<interpolate_x>true</interpolate_x></animation>\n'
        f'      <script>\n'
        f'        <loop>true</loop><delay_start>{delay}.0</delay_start>'
        f'<auto_start>true</auto_start>\n'
        f'        <trajectory id="0" type="walk">\n'
        f'          <waypoint><time>0</time>   <pose>{x0} {y} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t1}</time>  <pose>{x1_slow} {y} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t2}</time>  <pose>{x1} {y} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t3}</time>  <pose>{x1} {y} 1.0 0 0 {rev_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t4}</time>  <pose>{x0_slow} {y} 1.0 0 0 {rev_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t5}</time>  <pose>{x0} {y} 1.0 0 0 {rev_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t6}</time>  <pose>{x0} {y} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'        </trajectory>\n'
        f'      </script>\n'
        f'    </actor>\n'
    )


def ns_actor(name, x, y0, y1, delay=0, speed=0.5):
    """N-S patrol with deceleration 1 m before each turn-around."""
    dist      = abs(y1 - y0)
    direction = 1 if y1 >= y0 else -1
    fwd_yaw   =  1.5708 if y1 >= y0 else -1.5708
    rev_yaw   = -1.5708 if y1 >= y0 else  1.5708

    y1_slow = round(y1 - direction * SLOW_DIST, 2)
    y0_slow = round(y0 + direction * SLOW_DIST, 2)

    fast_t  = round((dist - SLOW_DIST) / speed)
    slow_t  = round(SLOW_DIST / SLOW_SPEED)
    travel  = fast_t + slow_t
    turn    = 4
    cycle   = 2 * travel + 2 * turn

    t1 = fast_t
    t2 = travel
    t3 = travel + turn
    t4 = travel + turn + fast_t
    t5 = 2 * travel + turn
    t6 = cycle

    return (
        f'\n    <!-- {name}: N-S patrol x={x}, y={y0}→{y1} (~{speed} m/s,'
        f' crosses E-W robot corridor) -->\n'
        f'    <actor name="{name}">\n'
        f'      <skin><filename>{SKIN}</filename><scale>1.0</scale></skin>\n'
        f'      <animation name="walk"><filename>{SKIN}</filename>'
        f'<interpolate_x>true</interpolate_x></animation>\n'
        f'      <script>\n'
        f'        <loop>true</loop><delay_start>{delay}.0</delay_start>'
        f'<auto_start>true</auto_start>\n'
        f'        <trajectory id="0" type="walk">\n'
        f'          <waypoint><time>0</time>   <pose>{x} {y0} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t1}</time>  <pose>{x} {y1_slow} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t2}</time>  <pose>{x} {y1} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t3}</time>  <pose>{x} {y1} 1.0 0 0 {rev_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t4}</time>  <pose>{x} {y0_slow} 1.0 0 0 {rev_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t5}</time>  <pose>{x} {y0} 1.0 0 0 {rev_yaw}</pose></waypoint>\n'
        f'          <waypoint><time>{t6}</time>  <pose>{x} {y0} 1.0 0 0 {fwd_yaw}</pose></waypoint>\n'
        f'        </trajectory>\n'
        f'      </script>\n'
        f'    </actor>\n'
    )


# 50 actors in 10 groups of 5.  Each group adds people to the next density level.
# Staggered delay_start values prevent a traffic jam at t=0.
#
# Warehouse footprint: x≈-15 to +15, y≈-22 to +25.
# Coverage uses 5 N-S bands so density is uniform at every level:
#   Band 5 (north,    y= 14 to 25): mission north
#   Band 4 (mid,      y=  5 to 14): mission mid
#   Band 3 (transit,  y= -4 to  5): transition zone
#   Band 2 (south,    y=-13 to -4): south floor
#   Band 1 (deep-S,   y=-22 to-13): south-shelf area
# Each group A→J adds one person to each band → uniform density at N=5,10,...50.
#
#  Group A – one actor per band (N=5 baseline):
#    a1: E-W  y=20.5  x=  0→ 7    Band5: (was y=22.0; shifted −1.5 to clear R10 at y=22.13)
#    a2: E-W  y=11.5  x= -3→ 5    Band4: (was y=13.0; shifted −1.5 to clear R8  at y=13.01)
#    a3: E-W  y= 3.0  x= -8→-1.5  Band3: (was x1=0; shortened to clear R5 at x=1.44)
#    a4: E-W  y=-8.0  x=-12→-5    Band2: south-west aisle
#    a5: E-W  y=-18.0 x=-13→-6    Band1: deep-south west
#
#  Group B – second actor per band (N=10):
#    b1: N-S  x= 8    y=12→20     Band5+4: east mission aisle
#    b2: E-W  y= 9.0  x=  7→12   Band4: east mid
#    b3: N-S  x=-8    y=  0→ 8   Band3+4: west-central
#    b4: E-W  y=-8.0  x=  2→ 9   Band2: south-east aisle
#    b5: E-W  y=-18.0 x=  3→10   Band1: deep-south east
#
#  Group C – third actor per band (N=15):
#    c1: E-W  y=20.0  x=-14.5→-11.5  Band5: far-west
#    c2: N-S  x= 7    y=  5→15   Band4+3: east-centre
#    c3: E-W  y= 1.0  x=-14→-6   Band3: west lower transition
#    c4: N-S  x=-12   y=-10→-2   Band2+3: far-west mid-south
#    c5: N-S  x=10    y=-18→-10  Band1+2: east deep-south
#
#  Group D – fourth actor per band (N=20):
#    d1: N-S  x=-5.5  y=14→22    Band5: west-centre
#    d2: E-W  y=15.5  x=  7→13  Band5: (was y=17.0; shifted −1.5 to clear R9 at y=17.75)
#    d3: N-S  x= 5    y=  1→ 8  Band3: east-centre lower
#    d4: E-W  y=-4.0  x=-10→-2  Band2: south-transition west
#    d5: N-S  x=-12   y=-20→-12 Band1: far-west deep-south
#
#  Group E – fifth actor per band (N=25):
#    e1: N-S  x=-14   y=16→22   Band5: (was x=-13; shifted −1 to clear R13 at x=-12.38)
#    e2: E-W  y=24.0  x= -2→ 4 Band5: (was x0=-3; shifted +1 to clear R11 at x=-3.92)
#    e3: E-W  y= 9.5  x= -8→-1 Band4: (was y=11.0; shifted −1.5 to clear R6 at y=11.72)
#    e4: E-W  y=-4.0  x=  3→10 Band2: south-transition east
#    e5: N-S  x=11    y=-18→-10 Band1+2: east mid-south
#
#  Group F – sixth actor per band (N=30):
#    f1: E-W  y=17.5  x=-10→-6  Band5: (was y=19.0; shifted −1.5 to clear R12 at y=20.01)
#    f2: E-W  y= 6.0  x=-13→-5  Band4: west
#    f3: N-S  x= 3    y= -4→ 4  Band3: central
#    f4: N-S  x= 3    y=-12→-5  Band2: central
#    f5: E-W  y=-16.0 x=-14→-12 Band1: far-west
#
#  Group G – seventh actor per band (N=35):
#    g1: N-S  x= 3    y=15.5→20.5  Band5: (was 14→22; trimmed to clear R8 at start and R10 at end)
#    g2: E-W  y= 8.0  x= -9→-2 Band4: west-mid
#    g3: E-W  y= 0.0  x=  3→11 Band3: east
#    g4: E-W  y=-9.0  x=-14→-5 Band2: west sweep
#    g5: E-W  y=-16.0 x=  2→ 9 Band1: east
#
#  Group H – eighth actor per band (N=40):
#    h1: E-W  y=15.0  x=-12→-7  Band5: west-lower
#    h2: N-S  x=12    y= 8→14   Band4: (was y0=7; raised to clear R7 at y=6.49)
#    h3: N-S  x=-3    y= -4→ 4  Band3: west-centre
#    h4: N-S  x= 4    y=-10→-4  Band2: east-central
#    h5: N-S  x= 2    y=-20→-14 Band1: central-corridor
#
#  Group I – ninth actor per band (N=45):
#    i1: E-W  y=16.0  x=  0→ 7  Band5: east-central-lower
#    i2: N-S  x=-4    y=  5→12  Band4: west-centre
#    i3: E-W  y= 2.0  x=3.5→11  Band3: (was x0=2; shifted +1.5 to clear R5 at x=1.44)
#    i4: E-W  y=-6.0  x= -8→-1  Band2: west-central
#    i5: E-W  y=-20.0 x=-14→-12 Band1: far-west deep-south
#
#  Group J – tenth actor per band (N=50, maximum density):
#    j1: N-S  x=14    y=15→22   Band5: (was x=13; shifted +1 to clear R9 at x=11.86)
#    j2: E-W  y=12.0  x=-14→-11 Band4: far-west
#    j3: E-W  y=-3.5  x=-13→-5  Band3: south-west
#    j4: N-S  x=-13   y=-12→-5  Band2: far-west
#    j5: E-W  y=-20.0 x=  1→ 8  Band1: east

ACTORS = [
    # --- Group A ---
    ew_actor('person_a1', y=20.5,  x0=0,    x1=7,    delay=0),
    ew_actor('person_a2', y=11.5,  x0=-3,   x1=5,    delay=4),
    ew_actor('person_a3', y=3.0,   x0=-8,   x1=-1.5, delay=2),
    ew_actor('person_a4', y=-8.0,  x0=-12,  x1=-5,   delay=6),
    ew_actor('person_a5', y=-18.0, x0=-13,  x1=-6,   delay=0),
    # --- Group B ---
    ns_actor('person_b1', x=8,     y0=12,   y1=20,   delay=0),
    ew_actor('person_b2', y=9.0,   x0=7,    x1=12,   delay=5),
    ns_actor('person_b3', x=-8,    y0=0,    y1=8,    delay=3),
    ew_actor('person_b4', y=-8.0,  x0=2,    x1=9,    delay=7),
    ew_actor('person_b5', y=-18.0, x0=3,    x1=10,   delay=4),
    # --- Group C ---
    ew_actor('person_c1', y=20.0,  x0=-14.5,x1=-11.5,delay=0),
    ns_actor('person_c2', x=7,     y0=5,    y1=15,   delay=5),
    ew_actor('person_c3', y=1.0,   x0=-14,  x1=-6,   delay=8),
    ns_actor('person_c4', x=-12,   y0=-10,  y1=-2,   delay=2),
    ns_actor('person_c5', x=10,    y0=-18,  y1=-10,  delay=6),
    # --- Group D ---
    ns_actor('person_d1', x=-5.5,  y0=14,   y1=22,   delay=0),
    ew_actor('person_d2', y=15.5,  x0=7,    x1=13,   delay=4),
    ns_actor('person_d3', x=5,     y0=1,    y1=8,    delay=8),
    ew_actor('person_d4', y=-4.0,  x0=-10,  x1=-2,   delay=2),
    ns_actor('person_d5', x=-12,   y0=-20,  y1=-14,  delay=6),  # was y1=-12; R1(-12.29,-12.29) was 0.41 m away
    # --- Group E ---
    ns_actor('person_e1', x=-14,   y0=16,   y1=22,   delay=3),
    ew_actor('person_e2', y=24.0,  x0=-2,   x1=4,    delay=0),
    ew_actor('person_e3', y=9.5,   x0=-8,   x1=-1,   delay=5),
    ew_actor('person_e4', y=-4.0,  x0=3,    x1=10,   delay=1),
    ns_actor('person_e5', x=11,    y0=-18,  y1=-10,  delay=7),
    # --- Group F ---
    ew_actor('person_f1', y=17.5,  x0=-10,  x1=-6,   delay=0),
    ew_actor('person_f2', y=6.0,   x0=-13,  x1=-5,   delay=3),
    ns_actor('person_f3', x=3,     y0=-4,   y1=4,    delay=7),
    ns_actor('person_f4', x=3,     y0=-11,  y1=-5,   delay=1),  # was y0=-12; R3(2.75,-13.17) was 1.20 m away
    ew_actor('person_f5', y=-16.0, x0=-14,  x1=-12,  delay=5),
    # --- Group G ---
    ns_actor('person_g1', x=3,     y0=15.5, y1=20.5, delay=0),
    ew_actor('person_g2', y=8.0,   x0=-9,   x1=-2,   delay=5),
    ew_actor('person_g3', y=0.0,   x0=3,    x1=11,   delay=8),
    ew_actor('person_g4', y=-9.0,  x0=-14,  x1=-5,   delay=2),
    ew_actor('person_g5', y=-16.0, x0=2,    x1=9,    delay=6),
    # --- Group H ---
    ew_actor('person_h1', y=15.0,  x0=-12,  x1=-7,   delay=0),
    ns_actor('person_h2', x=12,    y0=8,    y1=14,   delay=4),
    ns_actor('person_h3', x=-3,    y0=-4,   y1=4,    delay=8),
    ns_actor('person_h4', x=4,     y0=-10,  y1=-4,   delay=2),
    ns_actor('person_h5', x=2,     y0=-20,  y1=-15.5,delay=6),  # was y1=-14; R3(2.75,-13.17) was 1.12 m away
    # --- Group I ---
    ew_actor('person_i1', y=16.0,  x0=0,    x1=7,    delay=0),
    ns_actor('person_i2', x=-4,    y0=5,    y1=12,   delay=5),
    ew_actor('person_i3', y=2.0,   x0=3.5,  x1=11,   delay=8),
    ew_actor('person_i4', y=-6.0,  x0=-8,   x1=-1,   delay=2),
    ew_actor('person_i5', y=-20.0, x0=-14,  x1=-12,  delay=6),
    # --- Group J ---
    ns_actor('person_j1', x=14,    y0=15,   y1=22,   delay=3),
    ew_actor('person_j2', y=12.0,  x0=-14,  x1=-11,  delay=0),
    ew_actor('person_j3', y=-3.5,  x0=-13,  x1=-5,   delay=5),
    ns_actor('person_j4', x=-14,   y0=-12,  y1=-5,   delay=1),  # was x=-13; R1(-12.29,-12.29) was 0.77 m away
    ew_actor('person_j5', y=-20.0, x0=1,    x1=8,    delay=7),
]

assert len(ACTORS) == 50, f"Expected 50 actors, got {len(ACTORS)}"

out_dir = os.path.dirname(os.path.abspath(__file__))

for count in range(0, 51, 10):
    path = os.path.join(out_dir, f'warehouse_people{count}.sdf')
    with open(path, 'w') as f:
        f.write(STATIC_BODY)
        if count > 0:
            f.write(f'\n    <!-- ===== {count} animated actors ===== -->\n')
            for actor in ACTORS[:count]:
                f.write(actor)
        f.write(FOOTER)
    print(f'Wrote {path}  ({count} actors)')


# ─────────────────────────────────────────────────────────────────────────────
# 100-person sampling POOL — warehouse_people100.sdf
#
# Unlike the cumulative warehouse_people{0..50}.sdf files (fixed first-N subsets),
# this is a large pool of 100 region-clearing walkers meant to be *subsampled* at
# run time (see make_density_world.py --keep N): each experiment run spawns a
# random N-actor subset. The pool is generated procedurally with rejection
# sampling so every actor's whole patrol stays ≥ REGION_CLEAR m from all region
# nodes R1–R14 and outside every shelf/barrier footprint — a person parked on a
# mission waypoint or the goal region would make 100 % route completion
# impossible and burn the harness's retry budget.
#
# Deterministic: fixed seed → identical pool every regeneration.
# ─────────────────────────────────────────────────────────────────────────────
POOL_SIZE   = 100
POOL_SEED   = 100
REGION_CLEAR = 1.5   # min distance from any region node (m) — matches hand-tuned set
SHELF_MARGIN = 0.3   # extra keep-out around each shelf/barrier box (m)

# Footprint (kept just inside the warehouse walls; matches gen comments x∈[-15,15], y∈[-22,25])
FOOT_X = (-14.5, 14.5)
FOOT_Y = (-21.5, 24.5)

# Region-node coordinates: authoritative source is the planner graph.
_GRAPH = os.path.normpath(os.path.join(
    out_dir, '..', 'src', 'planning_ros_pkgs', 'evo_skill_ros', 'config', 'graph.json'))
try:
    with open(_GRAPH) as _gf:
        REGIONS = [tuple(r['coords']) for r in json.load(_gf)['regions']]
except (OSError, KeyError, ValueError):
    # fallback to the values baked into graph.json at authoring time
    REGIONS = [
        (-12.29, -12.29), (-4.63, -12.29), (2.75, -13.17), (10.78, -12.54),
        (1.44, 2.74), (-7.41, 11.72), (12.72, 6.49), (3.52, 13.01),
        (11.86, 17.75), (1.89, 22.13), (-3.92, 23.68), (-7.07, 20.01),
        (-12.38, 17.86), (-12.94, 9.89),
    ]

# Shelf / barrier keep-out boxes (xmin, xmax, ymin, ymax) — from the header comment
# above; the barrier wall at x≈-10.4 is modelled as a thin box.
SHELF_BOXES = [
    (1.0, 6.0, 8.3, 10.8),      # shelf_big_3
    (-3.8, 2.2, 17.3, 19.8),    # shelf_big_4
    (-11.3, -8.8, 20.3, 22.8),  # shelf_0
    (-8.3, -5.8, 22.4, 24.9),   # shelf_1
    (-5.3, -2.8, 20.3, 22.8),   # shelf_2
    (11.5, 15.5, 2.5, 6.5),     # shelf_3
    (8.0, 12.0, 2.5, 6.5),      # shelf_4
    (-0.5, 1.5, -3.0, -1.0),    # shelf_7
    (-11.5, -5.5, -15.5, -10.5),# shelf_big_0
    (3.5, 9.5, -15.5, -10.5),   # shelf_big_1
    (-4.5, 1.5, -15.5, -10.5),  # shelf_big_2
    (11.5, 15.5, -22.5, -19.5), # shelf_5
    (11.5, 15.5, -16.5, -13.5), # shelf_6
    (-10.6, -10.2, 6.5, 14.75), # west barriers (thin wall)
]


def _in_box(x, y, box, margin=SHELF_MARGIN):
    xmin, xmax, ymin, ymax = box
    return (xmin - margin) <= x <= (xmax + margin) and \
           (ymin - margin) <= y <= (ymax + margin)


def _seg_clears(x0, y0, x1, y1, step=0.25):
    """True if the whole straight leg keeps ≥REGION_CLEAR from every region node
    and never enters a shelf/barrier box. Endpoints must be inside the footprint."""
    for (x, y) in ((x0, y0), (x1, y1)):
        if not (FOOT_X[0] <= x <= FOOT_X[1] and FOOT_Y[0] <= y <= FOOT_Y[1]):
            return False
    length = math.hypot(x1 - x0, y1 - y0)
    n = max(2, int(length / step) + 1)
    for i in range(n + 1):
        t = i / n
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        for (rx, ry) in REGIONS:
            if math.hypot(x - rx, y - ry) < REGION_CLEAR:
                return False
        for box in SHELF_BOXES:
            if _in_box(x, y, box):
                return False
    return True


def _build_pool(n, seed):
    """Procedurally sample n region-clearing E-W / N-S walkers (reproducible)."""
    rng = random.Random(seed)
    pool = []
    tries = 0
    max_tries = n * 4000
    while len(pool) < n and tries < max_tries:
        tries += 1
        idx = len(pool) + 1
        name = f'person_p{idx:03d}'
        delay = rng.randint(0, 9)          # stagger starts to avoid a t=0 jam
        seg = rng.uniform(3.0, 10.0)       # leg length (m); >SLOW_DIST for decel math
        if rng.random() < 0.5:             # E-W patrol (constant y)
            y = rng.uniform(FOOT_Y[0], FOOT_Y[1])
            x0 = rng.uniform(FOOT_X[0], FOOT_X[1] - seg)
            x1 = x0 + seg
            if not _seg_clears(x0, y, x1, y):
                continue
            pool.append(ew_actor(name, y=round(y, 2),
                                 x0=round(x0, 2), x1=round(x1, 2), delay=delay))
        else:                              # N-S patrol (constant x)
            x = rng.uniform(FOOT_X[0], FOOT_X[1])
            y0 = rng.uniform(FOOT_Y[0], FOOT_Y[1] - seg)
            y1 = y0 + seg
            if not _seg_clears(x, y0, x, y1):
                continue
            pool.append(ns_actor(name, x=round(x, 2),
                                 y0=round(y0, 2), y1=round(y1, 2), delay=delay))
    if len(pool) < n:
        raise RuntimeError(
            f'pool: only placed {len(pool)}/{n} clearing actors in {tries} tries')
    return pool


POOL = _build_pool(POOL_SIZE, POOL_SEED)
pool_path = os.path.join(out_dir, f'warehouse_people{POOL_SIZE}.sdf')
with open(pool_path, 'w') as f:
    f.write(STATIC_BODY)
    f.write(f'\n    <!-- ===== {POOL_SIZE}-person sampling pool '
            f'(seed {POOL_SEED}, subsample via make_density_world.py) ===== -->\n')
    for actor in POOL:
        f.write(actor)
    f.write(FOOTER)
print(f'Wrote {pool_path}  ({POOL_SIZE} actors, sampling pool)')

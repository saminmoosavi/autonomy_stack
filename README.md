# ROS 2 Jazzy Distributed Zenoh Setup

This setup connects a **Clearpath Jackal** and two **NVIDIA GB10 computers** using `rmw_zenoh_cpp`.

GB10 #1 acts as the central Zenoh router because it has connections to both the Jackal Ethernet network and the GB10 Wi-Fi network.

## Network Architecture

```text
 Clearpath Jackal
 192.168.131.1
       │
       │ Ethernet
       │
 192.168.131.10
┌──────────────────────────┐
│         GB10 #1          │
│                          │
│ Ethernet: 192.168.131.10 │
│ Wi-Fi:    192.168.50.10  │
│                          │
│       rmw_zenohd         │
│      Router :7447        │
└────────────┬─────────────┘
             │
             │ Wi-Fi
             │
      192.168.50.11
┌──────────────────────────┐
│         GB10 #2          │
│                          │
│      Zenoh Client        │
└──────────────────────────┘
```

The connection architecture is:

```text
Jackal
   │
   │ Zenoh client
   ▼
GB10 #1
Zenoh Router
   ▲
   │ Zenoh client
   │
GB10 #2
```

The important point is that **GB10 #1 is the only Zenoh router**.

---

# 1. IP Configuration

## Jackal

```text
Ethernet: 192.168.131.1
```

## GB10 #1

```text
Ethernet: 192.168.131.10
Wi-Fi:    192.168.50.10
```

## GB10 #2

```text
Wi-Fi:    192.168.50.11
```

Therefore:

```text
Jackal → GB10 #1
tcp/192.168.131.10:7447
```

and:

```text
GB10 #2 → GB10 #1
tcp/192.168.50.10:7447
```

---

# 2. Install Zenoh RMW

All computers use ROS 2 Jazzy and:

```bash
sudo apt update
sudo apt install ros-jazzy-rmw-zenoh-cpp
```

Verify:

```bash
ros2 pkg prefix rmw_zenoh_cpp
```

---

# 3. GB10 #1 — Central Zenoh Router

GB10 #1 runs the only `rmw_zenohd`.

Source ROS:

```bash
source /opt/ros/jazzy/setup.bash
```

Clear any previous client configuration:

```bash
unset ZENOH_CONFIG_OVERRIDE
unset ZENOH_SESSION_CONFIG_URI
```

Select Zenoh:

```bash
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
```

Start the router:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

Keep this terminal/process running.

## Verify the Router

```bash
ss -lntp | grep 7447
```

Expected:

```text
LISTEN 0 1024 *:7447 *:*
```

The `*:7447` listener makes the router reachable through both GB10 #1 interfaces:

```text
192.168.131.10:7447
192.168.50.10:7447
```

---

# 4. GB10 #1 Docker Networking

If ROS 2 is running inside Docker on GB10 #1, use:

```yaml
network_mode: host
```

For example:

```yaml
services:

  ros:
    network_mode: host
```

This allows the Zenoh router inside the container to access the GB10 host network interfaces directly.

---

# 5. Test Jackal → GB10 #1 Connectivity

From the Jackal:

```bash
nc -vz 192.168.131.10 7447
```

Expected:

```text
Connection to 192.168.131.10 7447 port [tcp/*] succeeded!
```

This has been successfully verified.

---

# 6. Test GB10 #2 → GB10 #1 Connectivity

From GB10 #2:

```bash
nc -vz 192.168.50.10 7447
```

Expected:

```text
Connection to 192.168.50.10 7447 port [tcp/*] succeeded!
```

This has also been successfully verified.

Therefore both networks can reach the GB10 #1 Zenoh router.

---

# 7. GB10 #2 — Zenoh Client

GB10 #2 should **not** run `rmw_zenohd`.

Configure it as a client of GB10 #1:

```bash
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.50.10:7447"]'
```

Verify:

```bash
echo $RMW_IMPLEMENTATION
echo $ZENOH_CONFIG_OVERRIDE
```

Expected:

```text
rmw_zenoh_cpp
```

and:

```text
mode="client";connect/endpoints=["tcp/192.168.50.10:7447"]
```

---

# 8. Jackal — Zenoh Client

The Jackal also connects to the GB10 #1 router, but through GB10 #1's Ethernet interface.

For a manually launched ROS 2 node:

```bash
source /opt/ros/jazzy/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/192.168.131.10:7447"]'
```

Do not start a second `rmw_zenohd` for this test.

---

# 9. Test Jackal → GB10 #1 → GB10 #2

On the Jackal:

```bash
ros2 topic pub /zenoh_test \
    std_msgs/msg/String \
    "{data: 'hello from jackal'}" \
    -r 1
```

On GB10 #1:

```bash
ros2 topic echo /zenoh_test
```

On GB10 #2:

```bash
ros2 topic echo /zenoh_test
```

The following communication has been successfully verified:

```text
                 /zenoh_test
Jackal ─────────────────────────►
             GB10 #1 Router
                    │
                    ▼
                  GB10 #2
```

Both GB10 #1 and GB10 #2 can see `/zenoh_test` published from the Jackal.

This verifies that:

- Jackal can reach the GB10 #1 router.
- GB10 #2 can reach the GB10 #1 router.
- Zenoh routes ROS 2 traffic between the two networks.
- No IP routing between `192.168.131.x` and `192.168.50.x` is required for this ROS 2 communication.

---

# 10. Clearpath Jackal System Services

The Jackal normally starts its own Zenoh router through:

```text
clearpath-zenoh-router.service
```

The service launches:

```text
/etc/clearpath/zenoh-router-start
```

The main Clearpath services include:

```text
clearpath-robot.service
clearpath-platform.service
clearpath-sensors.service
clearpath-zenoh-router.service
```

The platform service launches the main Jackal nodes, including components such as:

```text
robot_state_publisher
ros2_control_node
robot_localization
twist_mux
joy_linux
teleop_twist_joy
diagnostic_aggregator
micro_ros_agent
imu_filter_madgwick
nmea_navsat_driver
```
# Permanent Jackal Zenoh Client Configuration

This procedure changes the Clearpath Jackal so that its ROS 2 Jazzy nodes permanently use the **external Zenoh router running on GB10 #1** instead of the Jackal's local `rmw_zenohd`.

## Network Configuration

```text
Clearpath Jackal
192.168.131.1
      │
      │ Ethernet
      │
      ▼
GB10 #1
192.168.131.10
rmw_zenohd :7447
```

The Jackal connects to:

```text
tcp/192.168.131.10:7447
```

---

## 1. Verify Connectivity

Before changing the Jackal configuration, verify that the GB10 #1 Zenoh router is reachable:

```bash
nc -vz 192.168.131.10 7447
```

Expected:

```text
Connection to 192.168.131.10 7447 port [tcp/*] succeeded!
```

Do not continue if this connection fails.

---

## 2. Create the Permanent Zenoh Environment File

Create:

```bash
sudo nano /etc/clearpath/zenoh-client.env
```

Add:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["tcp/192.168.131.10:7447"]
```

Save the file.

Verify:

```bash
cat /etc/clearpath/zenoh-client.env
```

---

## 3. Configure the Clearpath Platform Service

Create a systemd override:

```bash
sudo systemctl edit clearpath-platform.service
```

Add:

```ini
[Service]
EnvironmentFile=/etc/clearpath/zenoh-client.env
```

Save and exit.

Verify:

```bash
sudo systemctl cat clearpath-platform.service
```

The output should contain the override with:

```ini
[Service]
EnvironmentFile=/etc/clearpath/zenoh-client.env
```

---

## 4. Configure the Clearpath Sensor Service

Create another systemd override:

```bash
sudo systemctl edit clearpath-sensors.service
```

Add:

```ini
[Service]
EnvironmentFile=/etc/clearpath/zenoh-client.env
```

Save and exit.

Verify:

```bash
sudo systemctl cat clearpath-sensors.service
```

---

## 5. Disable the Jackal's Local Zenoh Router

The Jackal normally starts:

```text
clearpath-zenoh-router.service
```

The external GB10 #1 router will now replace it.

Disable and stop the local router:

```bash
sudo systemctl disable --now clearpath-zenoh-router.service
```

Verify:

```bash
systemctl is-enabled clearpath-zenoh-router.service
```

Expected:

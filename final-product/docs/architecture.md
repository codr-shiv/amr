# Architecture

From the whole system down to individual functions and wires. The diagrams are Mermaid and render on GitHub.

1. [System level](#1-system-level)
2. [ROS 2 graph on the Raspberry Pi](#2-ros-2-graph-on-the-raspberry-pi)
3. [TF tree](#3-tf-tree)
4. [Startup sequence (`amr_bringup.sh`)](#4-startup-sequence-amr_bringupsh)
5. [Module level](#5-module-level): `amr_diff_drive`, Nav2, ESP32 firmware
6. [Function level](#6-function-level): kinematics, wheel control law, encoder timestamps
7. [Circuit level](#7-circuit-level): ESP32 wiring

---

## 1. System level

```mermaid
flowchart LR
    subgraph Laptop
        RVIZ["RViz<br/>map view, 2D Pose Estimate, Nav2 Goal"]
    end
    subgraph Pi["Raspberry Pi (Ubuntu 22.04, ROS 2 Humble)"]
        NAV["Nav2 + AMCL<br/>or SLAM Toolbox"]
        DD["amr_diff_drive"]
        AG["micro-ROS agent"]
        LD["rplidar_ros"]
    end
    subgraph ESP["ESP32 (micro-ROS client)"]
        FW["PID speed control<br/>+ encoder decoding"]
    end
    LIDAR["RPLIDAR A1"]
    DRV["Cytron motor driver"]
    ML["Left motor + encoder"]
    MR["Right motor + encoder"]

    RVIZ <-->|"ROS 2 DDS over Wi-Fi<br/>(ROS_DOMAIN_ID 30)"| NAV
    LIDAR -->|"USB serial /dev/ttyUSB0"| LD
    NAV --> DD
    DD <--> AG
    AG <-->|"micro-ROS, Wi-Fi UDP 8888"| FW
    FW -->|"PWM + DIR"| DRV
    DRV --> ML
    DRV --> MR
    ML -->|"encoder A/B"| FW
    MR -->|"encoder A/B"| FW
```

| Subsystem | Runs on | Job |
|---|---|---|
| Navigation / mapping | Pi | Builds the map (SLAM Toolbox) or localizes on it (AMCL) and plans + follows paths (Nav2), outputs `/cmd_vel` |
| Base controller | Pi (`amr_diff_drive`) | `/cmd_vel` → wheel speed setpoints; encoder data → odometry + `odom → base_link` |
| Bridge | Pi (micro-ROS agent) | Connects the ESP32 to the ROS 2 graph |
| Low-level control | ESP32 | Closes the wheel speed loops at 50 Hz, reads the encoders, publishes wheel state |
| Sensing | RPLIDAR A1 | 360° laser scans (`/scan`) |

---

## 2. ROS 2 graph on the Raspberry Pi

**Nav mode** (`./amr_bringup.sh`, the default):

```mermaid
flowchart LR
    ESP32(["ESP32 via micro-ROS agent"])
    LIDAR(["rplidar_node"])
    TFS(["static_transform_publisher<br/>laser_tf.launch.py"])
    DD["diff_drive_controller<br/>(amr_diff_drive)"]
    MAP["map_server"]
    AMCL["amcl"]
    NAV2["Nav2 servers<br/>bt_navigator, planner_server,<br/>controller_server, smoother_server,<br/>behavior_server, velocity_smoother"]
    RVIZ(["RViz (laptop)"])

    ESP32 -->|"/encoder_telemetry<br/>JointState, 20 Hz"| DD
    DD -->|"/left_vel, /right_vel<br/>Float64 rad/s, 50 Hz"| ESP32
    DD -->|"/odom 50 Hz<br/>TF odom→base_link"| NAV2
    DD -->|"TF odom→base_link"| AMCL
    TFS -->|"TF base_link→laser"| AMCL
    LIDAR -->|"/scan"| AMCL
    LIDAR -->|"/scan"| NAV2
    MAP -->|"/map"| AMCL
    MAP -->|"/map"| NAV2
    AMCL -->|"TF map→odom"| NAV2
    RVIZ -->|"/initialpose"| AMCL
    RVIZ -->|"navigate_to_pose action"| NAV2
    NAV2 -->|"/cmd_vel Twist"| DD
```

All Nav2 nodes, `map_server` and `amcl` run in one process (`use_composition:=True`) to save RAM on the Pi.

**Slam mode** (`./amr_bringup.sh slam`): `map_server`, `amcl` and the Nav2 servers are replaced by
`slam_toolbox` (async), which consumes `/scan` + `odom → base_link` and publishes `/map` + `map → odom`.
`/cmd_vel` comes from teleop (`amr_teleop` or `teleop_twist_keyboard`).

---

## 3. TF tree

```mermaid
flowchart TD
    MAPF["map"] -->|"AMCL (nav) or SLAM Toolbox (slam)"| ODOM["odom"]
    ODOM -->|"amr_diff_drive, 50 Hz"| BASE["base_link<br/>(middle of the wheel axle)"]
    BASE -->|"static: x 0.175, y 0, z 0.100, no rotation<br/>laser_tf.launch.py"| LASER["laser"]
```

---

## 4. Startup sequence (`amr_bringup.sh`)

```mermaid
sequenceDiagram
    participant S as amr_bringup.sh
    participant A as micro-ROS agent
    participant E as ESP32
    participant D as amr_diff_drive
    participant L as rplidar_ros
    participant N as Nav2 / SLAM
    S->>S: source ROS + both workspaces, check map, kill leftovers
    S->>A: start (udp4, port 8888)
    E-->>A: connects, publishes /encoder_telemetry
    S->>S: wait for /encoder_telemetry (60 s max)
    S->>S: start laser_tf.launch.py (base_link → laser)
    S->>D: start diff_drive.launch.py
    S->>S: wait for /odom (30 s max)
    S->>L: start rplidar_a1_launch.py
    S->>S: wait for /scan (30 s max)
    alt nav (default)
        S->>N: nav_map.launch.py map:=MAP use_composition:=True
        S->>S: wait for "lifecycle_manager_localization ... Managed nodes are active"
    else slam
        S->>N: slam.launch.py
    end
    Note over S: Ctrl-C → SIGINT to every process group, SIGKILL after 5 s
```

Each program runs in its own process group and logs to `~/amr_logs/<name>.log`.

---

## 5. Module level

### 5.1 `amr_diff_drive` (Python node `diff_drive_controller`)

```mermaid
flowchart LR
    subgraph CB["Callbacks"]
        C1["cmd_vel_cb<br/>/cmd_vel"]
        C2["encoder_cb<br/>/encoder_telemetry (best effort)"]
    end
    subgraph ST["State"]
        S1["cmd_left, cmd_right<br/>last_cmd_time"]
        S2["pose x, y, θ at sample time<br/>measured wheel velocities"]
    end
    subgraph T["update() timer, 50 Hz"]
        U1["cmd_vel watchdog (0.5 s)"]
        U2["encoder staleness check (0.5 s)"]
        U3["predict pose to now<br/>(max 0.2 s ahead)"]
        U4["publish /left_vel, /right_vel,<br/>/odom, TF"]
    end
    C1 -->|"twist_to_wheels()"| S1
    C2 -->|"select_sample_time()<br/>integrate_pose(), wheels_to_twist()"| S2
    S1 --> U1 --> U4
    S2 --> U2 --> U3 --> U4
```

Pure math lives in `amr_diff_drive/kinematics.py` (no ROS, unit tested); the node only handles ROS I/O and state.

### 5.2 Nav2 pipeline (`amr_navigation/config/nav2_params.yaml`)

```mermaid
flowchart LR
    GOAL(["goal pose"]) --> BT["bt_navigator<br/>(default behavior tree)"]
    BT --> PL["planner_server<br/>NavFn, global costmap (map frame)"]
    PL -->|"path"| CT["controller_server<br/>Regulated Pure Pursuit, local costmap (odom frame)"]
    CT -->|"cmd_vel_nav"| VS["velocity_smoother<br/>max 0.25 m/s, 1.0 rad/s"]
    VS -->|"/cmd_vel"| DD(["amr_diff_drive"])
    BT -.->|"on failure"| BH["behavior_server<br/>spin, back up, wait"]
    GC["global costmap<br/>static + obstacle + inflation"] --- PL
    LC["local costmap 3×3 m<br/>obstacle + inflation"] --- CT
```

`smoother_server` (simple smoother) is also started, but Humble's default behavior tree doesn't call it;
`waypoint_follower` handles multi-goal missions.

### 5.3 ESP32 firmware (`firmware/esp32/amr_esp32/amr_esp32.ino`)

```mermaid
flowchart LR
    subgraph Core1["controlTask: FreeRTOS, core 1, fixed 50 Hz"]
        K1["read encoders"] --> K2["velocity filter"] --> K3["setpoint ramp"] --> K4["feed-forward + PI"] --> K5["PWM + DIR out"]
    end
    subgraph Loop["loop()"]
        M1["microRosStep()<br/>connect, spin, publish 20 Hz"]
        M2["handleSerial()<br/>tuning commands"]
        M3["serialOutput()<br/>status / plot"]
    end
    SH[("sh: shared struct<br/>spinlock protected")]
    M1 -->|"targets from /left_vel, /right_vel"| SH
    M2 -->|"mode, manual targets, gains"| SH
    SH -->|"snapshot each cycle"| Core1
    Core1 -->|"position, velocity, PWM"| SH
    SH -->|"telemetry"| M1
```

micro-ROS connection state machine (motors are stopped whenever the agent isn't connected):

```mermaid
stateDiagram-v2
    [*] --> WAITING_AGENT
    WAITING_AGENT --> AGENT_AVAILABLE: ping ok (every 500 ms)
    AGENT_AVAILABLE --> AGENT_CONNECTED: node, subs, pub created + time sync
    AGENT_AVAILABLE --> WAITING_AGENT: creation failed
    AGENT_CONNECTED --> AGENT_DISCONNECTED: ping fails (checked every 1 s)
    AGENT_DISCONNECTED --> WAITING_AGENT: stop motors, destroy entities
```

---

## 6. Function level

### 6.1 Differential-drive kinematics (`kinematics.py`)

r = wheel radius (0.056 m), L = wheel separation (0.39 m), REP-103 frame (x forward, y left, yaw counter-clockwise).

| Function | Math |
|---|---|
| `twist_to_wheels(v, ω)` | ω_L = (v − ω·L/2) / r, ω_R = (v + ω·L/2) / r; if max(\|ω_L\|, \|ω_R\|) > 17 rad/s, both are scaled by the same factor (keeps the path curvature) |
| `wheels_to_twist(ω_L, ω_R)` | v = r(ω_R + ω_L)/2, ω = r(ω_R − ω_L)/L |
| `integrate_pose(Δφ_L, Δφ_R)` | Δs = r(Δφ_R + Δφ_L)/2, Δθ = r(Δφ_R − Δφ_L)/L; exact arc: x += (Δs/Δθ)(sin(θ+Δθ) − sin θ), y −= (Δs/Δθ)(cos(θ+Δθ) − cos θ); midpoint formula when \|Δθ\| < 1e-6 |
| `predict_pose(age)` | `integrate_pose` with Δφ = ω_measured · age, only if 0 < age ≤ 0.2 s; never written back into the stored pose |

### 6.2 Encoder timestamp acceptance (`select_sample_time`)

```mermaid
flowchart TD
    A["encoder message<br/>stamp from ESP32 (Pi clock via micro-ROS time sync)"] --> B{"stamp > 0?"}
    B -->|no| R["use receive time<br/>(warning, no latency compensation)"]
    B -->|yes| C{"newer than previous stamp?"}
    C -->|no| R
    C -->|yes| D{"abs(receive − stamp) ≤ 0.5 s?"}
    D -->|no| R
    D -->|yes| OK["use stamp as measurement time<br/>(latency statistics updated)"]
```

### 6.3 Per-wheel speed control (ESP32 `updateWheel`, every 20 ms)

```mermaid
flowchart LR
    T["target rad/s<br/>(ROS or serial)"] --> RP["ramp: ref moves toward target<br/>at most 40 rad/s² × dt"]
    RP --> Z{"target = 0 and ref ≈ 0?"}
    Z -->|yes| OFF["PWM 0, integrator reset"]
    Z -->|no| LAW["u = kff·ref + sign(ref)·pwmMin<br/>+ kp·(ref − meas) + I"]
    ENC["encoder counts"] --> MEAS["meas = 0.3·raw + 0.7·meas<br/>raw = Δcounts·2π / (CPR·dt)"]
    MEAS --> LAW
    LAW --> AW["anti-windup: I frozen when u is saturated<br/>and the error pushes further; I clamped ±400"]
    AW --> OUT["PWM (0..1023) + DIR pin"]
```

Gains, CPR and invert flags per wheel: [firmware README §3](../firmware/esp32/README.md#3-configuration-top-of-the-sketch).

---

## 7. Circuit level

Signal wiring as defined in the firmware pin map:

```mermaid
flowchart LR
    subgraph ESP32
        P18["GPIO 18 (PWM L)"]
        P19["GPIO 19 (DIR L)"]
        P23["GPIO 23 (PWM R)"]
        P22["GPIO 22 (DIR R)"]
        P32["GPIO 32 (ENC L A)"]
        P33["GPIO 33 (ENC L B)"]
        P25["GPIO 25 (ENC R A)"]
        P26["GPIO 26 (ENC R B)"]
    end
    subgraph CY["Cytron motor driver (PWM + DIR mode)"]
        CL["channel 1: PWM, DIR"]
        CR["channel 2: PWM, DIR"]
    end
    ML["Left motor 26.9:1<br/>encoder 752.6 counts/wheel rev"]
    MR["Right motor 19.1:1<br/>encoder 536.1 counts/wheel rev"]
    P18 --> CL
    P19 --> CL
    P23 --> CR
    P22 --> CR
    CL --> ML
    CR --> MR
    ML -->|"A"| P32
    ML -->|"B"| P33
    MR -->|"A"| P25
    MR -->|"B"| P26
```

- PWM: 20 kHz, 10-bit. Encoder inputs use the ESP32's internal pull-ups and a glitch filter.
- The Cytron channel numbering above is illustrative; check which driver channel each motor is on.
- **TODO (team): power wiring.** Battery type/voltage, fuse/switch, the supply for the motor driver, the 5 V supply
  for the Raspberry Pi, how the ESP32 is powered, and the common ground. Add a wiring photo or schematic
  here or in [hardware.md](hardware.md).

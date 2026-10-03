# AMR: Project Log
Robotics Society, IIT Jodhpur · Core Team 2026–27

---

## 1. Overview

| | |
|---|---|
| **Goal** | Build an Autonomous Mobile Robot from scratch as Inter-IIT prep: a feel for how to systematically approach a full end-to-end hardware product problem statement |
| **Scope** | Roughly the Eternal AMR, minus the scanning features |
| **Target timeline** | ~2 weeks max; stretch goal of 1 week |
| **Actual** | SLAM bot in ~3 days (26 Sep), autonomous navigation in 5 days (28 Sep) |
| **Architecture deck** | https://canva.link/pxjnqahkfg13uoa |

---

## 2. Hardware (component list, 23 Sep)

| Component | Qty |
|---|---|
| ESP32 + cable | 1 |
| Raspberry Pi + adapter + LAN cable | 1 |
| Planetary geared DC motors with encoders (12V, 262 RPM, 45 N-cm, PG36M555-19.2K, encoder ME-37 7PPR) + connecting wires | 3 |
| Cytron motor drivers | 3 |
| RPLidar | 1 |
| DC power supply | 1 |

- **Software stack:** Ubuntu 22.04 + ROS 2 Humble on the RPi, micro-ROS on the ESP32, RPLidar driver, SLAM Toolbox, Nav2, differential drive controller + hardware interface.
- **Build:** T-slot aluminium chassis, 3D-printed CAD parts (wheel mounts, LiDAR mount), castor wheels.
- A dedicated "AMR station" table was set up in Tinkering with all components.

---

## 3. Teams and Task Assignments (23 Sep)

Verticals were chosen by poll: HW/CAD (6 votes), ESP32 side (9), RPi side (23).

### Raspberry Pi side: all ROS 2 work
Setup for everyone: flash Ubuntu 22.04, install ROS 2 Humble.

| # | Team | Task | Size |
|---|---|---|---|
| R1 | SLAM | Set up RPLidar driver and SLAM Toolbox | Not posted |
| R2 | Controller & HW interface | Set up differential drive controller and hardware interface | 3 |
| R3 | micro-ROS communication | Set up micro-ROS; establish ROS communication between ESP32 and RPi | 5 |
| R4 | Nav2 | Study Nav2; analyze the config file and its main parameters | 3 |

### ESP32 side: all electronics work

| # | Team | Task | Size |
|---|---|---|---|
| E1 | Motor control | Make the motors run: left/right wheel velocity → PWM conversion, PID control loop | 4 |
| E2 | Encoder / decoder | Set up the decoder; odometry testing | 3 |

### Hardware & CAD

| # | Requirement |
|---|---|
| H1 | Robust chassis |
| H2 | Stable motor mounting |
| H3 | Wheels mounted on motors without vibration |
| H4 | LiDAR at appropriate height |
| H5 | Clean design with enough room to work on electronics |

**Mentors:** T-slots are optional. What matters is the final product's stability and ease of access to the electronics.

---

## 4. Team Updates

### R1: SLAM (RPLidar + SLAM Toolbox)
- **24 Sep, SLAM team:** SLAM Toolbox 90% done (installation mostly complete, testing pending).
  - **Mentors:** complete it today and test properly.
- **24 Sep, SLAM team:** LiDAR scan data coming through correctly in SLAM Toolbox. SLAM Toolbox, Nav2 and transform tools set up. Fake odom data plus TFs `base_link → laser` and `odom → base_link` to be finished that night.
- **26 Sep, 04:36, SLAM team:** SLAM mapping working on the bot (RViz map videos). **Challenge completed.**

### R2: Controller & HW interface (differential drive)
- **24 Sep, Diff drive team:** downstream path from RPi Nav2 → ESP32 ready and tested (hardcoded values for now). Remaining: upstream path using encoder data at integration time. PID is the main remaining piece.
- **25 Sep, Diff drive team:** code deployed on the RPi and tested. Topics publishing correctly; subscribing to wheel state also working.

### R3: micro-ROS communication
- **24 Sep, micro-ROS team:** micro-ROS set up on both RPi and ESP32; basic communication tested.
- **25 Sep, micro-ROS team:** defined proper formats and names for topics RPi → ESP32 and ESP32 → RPi. Tested with dummy data (fake left/right wheel data). Shared the topics / message-format doc covering all inputs, outputs and message formats: https://docs.google.com/document/d/1CKC5PaiLtuVq5qNwfhTToIpuMQxy-mmduqVFJ-J3ZQc/edit?usp=sharing

### R4: Nav2
- **24 Sep, Nav2 team:** reading Nav2 config files, expected to be clear by the next day.
- **26 Sep, Nav2 team:** Nav2 pushed to the next day (day lost to other commitments).
- **28 Sep, 00:41, Nav2 team:** Nav2 almost complete.
- **28 Sep, 00:47, Nav2 team:** map built with Nav2, zero odometry drift (RViz screenshot).
  - **Mentors:** mapping was already working; Nav2 is for **navigation**.
- **28 Sep, 00:49, Nav2 team:** ~6-min video of Nav2 navigation in RViz.
- **28 Sep, 02:32, Nav2 team:** **Challenge completed.** Autonomous navigation working.

### E1: Motor control
- **24 Sep, Motor control team:** motor running verified the previous day, but only one motor was tested. m/s → PWM conversion due today.
  - **Mentors:** PWM conversion must be done today. Confirm with the diff drive team whether wheel velocities are published in **m/s or rad/s**.
- **25 Sep, Motor control team:** encoder + PID work done. Motors follow any speed commanded from the serial monitor and recover under manual load. Only the motor deadband under load is left to tune.
- **28 Sep, 00:11, Motor control team:** bot driving smoothly: "PID at its peak, no filters" (video).

### E2: Encoder / decoder
- **24 Sep, Mentors:** encoder calibration done for one motor. Values vary between motors, so all motors must be calibrated. Verify whether wheel state is needed as linear or angular.
- **24 Sep, Encoder team:** both encoders working correctly.
- **25 Sep, Encoder team:** decoder/encoder + PID integration done (see E1).

### Hardware & CAD
- **24 Sep, HW/CAD team:** CAD parts printed; everything fits almost properly.
- **25 Sep, HW/CAD team:** castor wheels installed and levelled (don't disturb them). Full test planned in the evening.
- **26 Sep, HW/CAD team:** cutting T-slots and improving the bot's aesthetics.
- **27 Sep, HW/CAD team:** some parts being reprinted. A print was cancelled halfway overnight, and printer problems pushed the reprint to ~8:30 PM. Reassembly after.
- **28 Sep, 02:58, HW/CAD team:** final bot photos. T-slot frame, RPLidar on a 3D-printed mount, clean wiring.

---

## 5. Timeline

| Date | Milestone |
|---|---|
| 21 Sep | Kickoff announced: build the AMR from scratch, architecture meeting next day |
| 22 Sep | Kickoff meeting (~10 PM): system architecture presented. Codename **"Schlawg"** |
| 23 Sep, 00:07 | Vertical poll; target of 20% of the work on day 1; component list; AMR station set up |
| 23 Sep, 19:53 | Task breakdown posted (4 RPi teams, 2 ESP32 teams, HW/CAD) |
| 23 Sep, ~22:40 | Team sign-ups; work starts. Whiteboard architecture photos shared |
| 24 Sep | Individual tasks in progress. CAD printed, micro-ROS link up, SLAM Toolbox installed, Nav2 → ESP32 downstream tested |
| 25 Sep, morning | All individual tasks done. Integration starts that night |
| 25 Sep, 20:57 | **Bot drives**; integration / interconnection session |
| 26 Sep, 04:36 | **SLAM bot complete** (~3 days after work started), after an all-night session |
| 26 Sep, 09:33 | Next challenge set: finish navigation before the weekend ends |
| 27 Sep | Parts reprinted and reassembled |
| 28 Sep, 00:47 | Mapping through Nav2 with zero odom drift |
| 28 Sep, 02:32 | **Autonomous navigation complete** |
| 28 Sep, 06:50 | Robotics Society's first successful autonomous navigation, done in a record 5 days |
| 28 Sep | Next challenge: Inter-IIT, targeting more than one medal |

---

## 6. Mentors' Guidance

- **23 Sep:** the architecture doesn't need to be fully clear yet. Details come while building. Daily task targets; tasks within each vertical run in parallel.
- **24 Sep:** post all updates in the group so everyone knows the status.
- **26 Sep:** the seniors took 1.5 months to build this. The architecture being solid in your heads matters more than the end goal, or your juniors will take 1.5 months too.
- **26 Sep:** the AMR will be properly documented in the same format shared earlier.
- **26 Sep:** learn your pacing. At Inter-IIT there are no full days off, so know how much you can do in a day without needing a break the next.
- **27 Sep:** don't think this is complete. The seniors got this far last year too. You have to prove you're better.

---

## 7. Links and Media

**Docs**
- Architecture deck: https://canva.link/pxjnqahkfg13uoa
- Topics / message-format doc: https://docs.google.com/document/d/1CKC5PaiLtuVq5qNwfhTToIpuMQxy-mmduqVFJ-J3ZQc/edit?usp=sharing
- Reference video (23 Sep): https://youtu.be/HJAE5Pk8Nyw
- micro-ROS comm repo: https://github.com/codr-shiv/amr-esp-comm

**Media** (renamed from the chat export; descriptions of each test in [testing-videos/](testing-videos/README.md))

| File | Date | Content |
|---|---|---|
| [2026-09-23_motor-product-page.jpg](testing-videos/media/2026-09-23_motor-product-page.jpg) | 23 Sep | Motor product page (PG36M555 with encoder) |
| [2026-09-23_whiteboard-architecture.jpg](testing-videos/media/2026-09-23_whiteboard-architecture.jpg) | 23 Sep | Whiteboard architecture: controller & HW interface, communication, Nav2, SLAM |
| [2026-09-25_pid-velocity-plots.mp4](testing-videos/media/2026-09-25_pid-velocity-plots.mp4) | 25 Sep | Wheel velocity / PID plots during integration |
| [2026-09-25_motor-test-rig.mp4](testing-videos/media/2026-09-25_motor-test-rig.mp4) | 25 Sep | Motor + wheel test rig |
| [2026-09-26_slam-mapping-rviz.mp4](testing-videos/media/2026-09-26_slam-mapping-rviz.mp4) | 26 Sep | SLAM mapping in RViz |
| [2026-09-28_slam-map-rviz.jpg](testing-videos/media/2026-09-28_slam-map-rviz.jpg), [2026-09-28_map-pose-arrows-1.jpg](testing-videos/media/2026-09-28_map-pose-arrows-1.jpg), [2026-09-28_map-pose-arrows-2.jpg](testing-videos/media/2026-09-28_map-pose-arrows-2.jpg) | 28 Sep | SLAM maps in RViz |
| [2026-09-28_driving-tuned-pid.mp4](testing-videos/media/2026-09-28_driving-tuned-pid.mp4) | 28 Sep | Bot driving with tuned PID |
| [2026-09-28_nav2-map-no-drift.jpg](testing-videos/media/2026-09-28_nav2-map-no-drift.jpg) | 28 Sep | Nav2 map with no odom drift |
| [2026-09-28_nav2-autonomous-navigation.mp4](testing-videos/media/2026-09-28_nav2-autonomous-navigation.mp4) | 28 Sep | Nav2 autonomous navigation in RViz (~6 min) |
| [final-product/docs/images/amr-robot.jpg](../final-product/docs/images/amr-robot.jpg) | 28 Sep | Finished bot |

Not in the export: the team photo (28 Sep) and a second finished-bot photo. Duplicate lower-resolution copies of the
whiteboard photo, the first SLAM map screenshot, the PID plot video and the SLAM mapping video were dropped.

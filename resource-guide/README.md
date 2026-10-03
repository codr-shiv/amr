# Resource guide

Foundational material for anyone starting on a mobile-robot project like this one (ROS 2, differential drive,
wheel control on a microcontroller, SLAM, navigation). The goal is to learn the concepts first; the
[final product](../final-product/README.md) then shows how they fit together on a real robot.

Suggested order: **1 → 2 → 3 → 4 → 5 → 6 → 7**. Items marked ★ are the most important ones.

---

## 1. ROS 2 fundamentals
| Resource | What you learn |
|---|---|
| ★ [ROS 2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html) | Nodes, topics, services, actions, parameters, launch files, colcon workspaces: do the Beginner CLI and Client Libraries sections |
| [ROS 2 Humble documentation](https://docs.ros.org/en/humble/) | Reference for everything above, installation, concepts |
| [Learn ROS 2: Beginner to Advanced Course (YouTube)](https://www.youtube.com/watch?v=HJAE5Pk8Nyw) | A full video course on ROS 2 concepts and code; the reference video shared with the team at the project kickoff |
| [Articulated Robotics](https://articulatedrobotics.xyz/) | Building a real ROS 2 mobile robot step by step (URDF, ros2_control, LiDAR, SLAM, Nav2); a great bridge from tutorials to hardware |

## 2. Coordinate frames and TF
| Resource | What you learn |
|---|---|
| ★ [About tf2](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html) and the [tf2 tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html) | How transforms between frames are published and looked up |
| [REP-103](https://www.ros.org/reps/rep-0103.html) | Units and axis conventions (x forward, y left, z up, counter-clockwise yaw) |
| ★ [REP-105](https://www.ros.org/reps/rep-0105.html) | The `map → odom → base_link` frame chain every mobile robot uses, and who publishes what |

## 3. Differential-drive kinematics and odometry
| Resource | What you learn |
|---|---|
| ★ [Kinematics of a differential-drive robot (Columbia course notes, PDF)](http://www.cs.columbia.edu/~allen/F17/NOTES/icckinematics.pdf) | Instantaneous centre of curvature, forward/inverse kinematics, pose integration |
| [Modern Robotics (Lynch & Park), free book + videos](https://hades.mech.northwestern.edu/index.php/Modern_Robotics) | Chapter 13 "Wheeled Mobile Robots": kinematics and odometry more formally |
| [Nav2: setting up odometry](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom/) | What `/odom` and `odom → base_link` must look like for navigation |

## 4. Motor control on a microcontroller
| Resource | What you learn |
|---|---|
| ★ [Understanding PID Control (MathWorks video series)](https://www.mathworks.com/videos/series/understanding-pid-control.html) | P, I, D intuitively, anti-windup, noise filtering, tuning; everything used in the ESP32 wheel controller |
| [Incremental (quadrature) encoders](https://en.wikipedia.org/wiki/Incremental_encoder) | How A/B channels encode position and direction; 1×/2×/4× decoding |
| [Arduino-ESP32 documentation](https://docs.espressif.com/projects/arduino-esp32/en/latest/) | GPIO, LEDC (PWM), Wi-Fi on the ESP32 with the Arduino core |
| [FreeRTOS on the ESP32 (ESP-IDF)](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/freertos.html) | Tasks, core pinning, timing (`vTaskDelayUntil`), critical sections: how the firmware keeps a fixed 50 Hz control loop |
| [ESP32Encoder library](https://github.com/madhephaestus/ESP32Encoder) | Hardware quadrature counting on the ESP32 |

## 5. Connecting a microcontroller to ROS 2: micro-ROS
| Resource | What you learn |
|---|---|
| ★ [micro-ROS overview](https://micro.vulcanexus.org/docs/overview/) and [concepts](https://micro.vulcanexus.org/docs/concepts/) | Client/agent architecture, transports (serial, UDP), what runs where |
| [First micro-ROS application](https://micro.vulcanexus.org/docs/tutorials/core/first_application_linux/) | Publishers, subscribers and executors on the micro-ROS side |
| [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino) | The Arduino library used by the firmware, with examples (Wi-Fi transport, reconnection) |

## 6. SLAM and localization
| Resource | What you learn |
|---|---|
| ★ [Cyrill Stachniss's lectures (YouTube)](https://www.youtube.com/@CyrillStachniss) | Mobile-robot SLAM, graph-based SLAM, scan matching, particle filters, explained from first principles |
| *Probabilistic Robotics* (Thrun, Burgard, Fox; MIT Press) | The reference book: Bayes filters, particle filters / Monte Carlo localization (AMCL), occupancy grid mapping, SLAM |
| [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) | The SLAM package used here: modes, parameters, map serialization |
| [Nav2: configuring AMCL](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/) | Every AMCL parameter and what it does |

## 7. Autonomous navigation: Nav2
| Resource | What you learn |
|---|---|
| ★ [Nav2 navigation concepts](https://docs.nav2.org/rolling/getting_started/navigation_concepts/) | Behavior trees, planners, controllers, costmaps, lifecycle nodes |
| [Nav2 first-time robot setup guide](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/) | Transforms, odometry, sensors, footprint, plugins: the checklist for a new robot |
| [Nav2 tuning guide](https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/) | Inflation, footprint, controller and planner tuning |
| [Nav2 documentation home](https://docs.nav2.org/) | Everything else (the site defaults to the latest release; the concepts apply to Humble too) |

## 8. Hardware used in this project
| Resource | What you learn |
|---|---|
| [Slamtec rplidar_ros](https://github.com/Slamtec/rplidar_ros) | RPLIDAR driver: launch files per model, parameters, udev rule |

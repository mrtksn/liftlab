# Autonomous drone navigation: a concise review

**Reviewed 13 October 2026.** Focus: sensing the real environment and using it for assisted or autonomous flight. This is a selective overview, not a systematic market or literature survey or a performance ranking. Confidence is stronger in the documented capabilities than in completeness of coverage. Commercial specifications, research demonstrations, and hobbyist integrations have different validation standards. Raspberry Pi hardware limits are outside this review.

A common architecture for navigation using an explicit map is **sensors → position estimate and environmental map → trajectory planner → flight controller**, with continuous feedback. Visual-inertial odometry (VIO) estimates movement from cameras and an IMU; simultaneous localization and mapping (SLAM) also builds and revisits a map. Assisted flight lets the pilot choose motion while the system constrains it; this can use local sensing without a persistent map. Autonomous navigation lets the system choose a route toward a goal.

## 1. Commercial cutting edge

Commercial leaders combine dedicated navigation sensors, onboard computation, and tightly integrated flight software. Their public documentation reveals sensor arrangements and behavior; exact estimation and planning algorithms are usually proprietary.

| Representative system | What it does | How it does it |
| --- | --- | --- |
| **Antigravity A1** | Point-to-fly FreeMotion control, subject following, programmed routes, and obstacle braking/bypass. | Motion-controller pointing selects flight direction while a trigger controls speed; goggles can look independently through 360 imagery. The 2026 spring update combines dual-lens 360 sensing with forward/downward binocular vision for surrounding obstacle awareness and bypass planning. [Controls](https://www.antigravity.tech/mc/hub/getting-started-with-antigravity-a1), [sensing update](https://www.antigravity.tech/be/blog/VR-Drones/a1-new-features-update). |
| **Skydio X10** | Surrounding obstacle awareness and visual navigation when GPS is lost. | Six navigation cameras provide surrounding coverage; VIO operates alongside GPS. Dedicated navigation cameras serve a different role from the inspection payload. [Sensors](https://www.skydio.com/x10/technical-specs), [navigation behavior](https://support.skydio.com/hc/en-us/articles/27947877716635-Understanding-telemetry-on-Skydio-X10). |
| **DJI Matrice 400** | Obstacle sensing, automated inspection routes, and supported power-line following workflows. | Binocular vision, rotating and upward LiDAR, infrared ranging, and six-direction millimeter-wave radar provide complementary observations. DJI specifies conditions for power-line detection and limitations for fast-moving obstacles. [Product and conditions](https://enterprise.dji.com/matrice-400), [sensor specifications](https://enterprise.dji.com/matrice-400/specs). |
| **Flyability Elios 3** | Stable GPS-denied inspection, live mapping, and assisted return; premium features include repeat flights. | LiDAR, visual-inertial sensing, and SLAM support positioning and occupancy mapping. Return logic selects a path through the observed environment. [Autonomy documentation](https://www.flyability.com/software/drone-autonomy). |
| **Exyn autonomous mapping systems** | Explore and scan an operator-defined area without individually programmed waypoints. | LiDAR and inertial data build a 3D map; a voxel representation supports localization and selection of flight corridors. The system chooses coverage waypoints during the mission. [Vendor explanation](https://www.exyn.com/lidar-drone). |

The commercial frontier spans **pilot assistance, repeatable inspection, and autonomous exploration**. A drone that brakes before a wall has substantially less autonomy than one that decides how to explore an unknown tunnel. Mapping quality and navigation capability must also be assessed separately.

**Consumer interaction is a separate frontier.** A1 makes flight accessible by separating where the user looks from where the aircraft flies, while automating stabilization and obstacle handling. FreeMotion points a direction; Deep Track follows a selected subject; Sky Path executes a created route. These behaviors do not establish that A1 reconstructs a persistent 3D world. Its internal map representation and planning algorithms are not disclosed in the sources reviewed. [Modes and current avoidance behavior](https://www.antigravity.tech/gb/drone/antigravity-a1).

## 2. Academic cutting edge

Research advances several competing approaches. Published results establish capability in the authors’ experiments; their speeds and success rates are not directly comparable across different environments.

| Direction | Representative work | What advances |
| --- | --- | --- |
| **Camera-based mapping and navigation** | [Visual-inertial SLAM with dense submaps, IROS 2025](https://arxiv.org/abs/2403.09596) | Couples visual-inertial localization with occupancy maps and planning. Demonstrates onboard outdoor/forest navigation up to 3 m/s without LiDAR, including consistent trajectory correction when SLAM updates the map. Closely matches the proposed “build a world and navigate in it” architecture. |
| **Robust sensor fusion** | [FAST-LIVO2, code released 2025](https://github.com/hku-mars/FAST-LIVO2) | Combines LiDAR, camera, and inertial measurements for localization and mapping under degraded sensing conditions. This supplies perception and state estimation; a complete drone still needs planning and control. |
| **Learned agile navigation** | [RAPID, RSS 2025](https://www.roboticsproceedings.org/rss21/p142.html) | Learns to generate waypoints from visual observations without separate perception, mapping, and planning modules. Reports real-flight average and peak speeds of 7 and 8.8 m/s after simulation training. |
| **Language-directed autonomy** | [AutoFly, 2026](https://arxiv.org/abs/2602.09657) | Combines visual, depth-related, and language representations to generate navigation actions. Evaluates simulated and real environments; generalization and dependable deployment remain questions beyond those demonstrations. |

The frontier includes both **explicit geometric world models** and **learned policies that compress perception and planning into a model**. Neither approach has demonstrated unrestricted competence across arbitrary weather, lighting, obstacles, and missions.

## 3. Open-source hobbyist state of the art

The strongest accessible route is a mature autopilot plus separately integrated perception and planning. Sophisticated components are available; calibration, timing, coordinate frames, recovery behavior, and flight validation remain the builder’s work.

| Level | Available approach | Practical boundary |
| --- | --- | --- |
| **Position hold and obstacle assistance** | ArduPilot or PX4 with supported positioning and proximity sensors. PX4 can constrain pilot motion using [collision prevention](https://docs.px4.io/main/en/computer_vision/collision_prevention). | Does not inherently provide a persistent 3D world or exploration. |
| **Autonomous obstacle detours** | ArduPilot [BendyRuler](https://ardupilot.org/copter/docs/common-oa-bendyruler.html); [Dijkstra plus BendyRuler](https://ardupilot.org/copter/docs/common-oa-dijkstrabendyruler.html) combines routing around known fences with local proximity avoidance. | More accessible than a full perception stack, but dependent on sensor coverage and planner limitations. |
| **GPS-denied 3D navigation** | [OpenVINS](https://docs.openvins.com/) or [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) for visual estimation; LiDAR alternatives include FAST-LIVO2. Add obstacle mapping and a planner such as [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm), which also supports one drone. | These components require integration; installing SLAM alone does not deliver obstacle avoidance. |
| **Complete open research platforms** | [OmniNxt](https://hkust-aerial-robotics.github.io/OmniNxt/) publishes an aerial platform with omnidirectional visual perception. | A useful advanced build reference, with research-platform complexity rather than turnkey consumer operation. |

**Current compatibility matters:** PX4 explicitly withdrew its old Path Planning Interface and associated mission-avoidance/Safe Landing features. Its documentation points toward newer integration approaches such as the ROS 2 Interface Library. Old PX4-Avoidance tutorials should therefore not be treated as current supported recipes. [Withdrawal notice](https://docs.px4.io/main/en/computer_vision/path_planning_interface).

For studying the proposed system, the most useful open reference is **visual-inertial localization + an obstacle/free-space map + continuous trajectory replanning + an autopilot**. Keep the estimated map inspectable, distinguish unknown space from observed free space, and evaluate the whole flight pipeline rather than the appearance of its 3D reconstruction.

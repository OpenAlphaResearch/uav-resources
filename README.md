# uav-resources-a

List of open-source algorithms and resources for autonomous drones. The list is a work in progress, so some information may be wrong and lots of useful resources are still missing.

## Perception

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [visual-slam-roadmap](https://github.com/changh95/visual-slam-roadmap)     |             | Great Roadmap for Visual SLAM                                                                                                  |      |
| [voxblox](https://github.com/ethz-asl/voxblox)     | ETH            | voxel-based mapping                                                                                                  | :heavy_check_mark:     |
| [maplab](https://github.com/ethz-asl/mav_voxblox_planning)      | ETH            | visual inertial mapping                                                                                              | :heavy_check_mark:       |
| [orb-slam2](https://github.com/raulmur/ORB_SLAM2)   |                | sparse 3D reconstruction                                                                                             | :heavy_check_mark:                  |
| [open_vins](https://github.com/rpng/open_vins)   | U. of Delaware | EKF fuses inertial info with sparse visual features                                                                  | :heavy_check_mark: | https://github.com/rpng/open_vins                    |
| [SVO 2.0](http://rpg.ifi.uzh.ch/svo2.html)     | ETH            | semi-direct paradigm to estimate pose from pixel intensities and features                                            | :heavy_check_mark:  | http://rpg.ifi.uzh.ch/svo2.html                      |
| [DSO](https://github.com/JakobEngel/dso/)         | TUM            | direct sparse odometry                                                                                               |               | https://github.com/JakobEngel/dso/                   |
| [XIVO](https://feixh.github.io/projects/xivo/)        | UCLA           | inertial-aided visual odometry                                                                                       |               |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | HKUST          | An optimization-based multi-sensor state estimator                                                                   |               |
| [Kimera-VIO](https://github.com/MIT-SPARK/Kimera)  | MIT            | real-time metric-semantic SLAM and VIO                                                                               | :heavy_check_mark:                  |
| [tagSLAM](https://github.com/berndpfrommer/tagslam)     | UPenn          | tagSLAM with apriltags                                                                                               | :heavy_check_mark:                   |
| [LARVIO](https://github.com/PetWorm/LARVIO)      |                | A lightweight, accurate and robust monocular visual inertial odometry based on Multi-State Constraint Kalman Filter. | :heavy_check_mark:                     |
| [R-VIO](https://github.com/rpng/R-VIO)    |                | based on robocentric sliding-window filtering-based VIO framework                                                    |                                  |
| [nanomap](https://github.com/peteflorence/nanomap_ros)  | MIT            | fast, uncertainty-aware proximity queries with lazy search of local 3D data                                          |                 |
| [MSCKF_VIO](https://github.com/KumarRobotics/msckf_vio)  | UPenn          | package is a stereo version of MSCKF                                                                                 |                   |
| [VINS_mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)   | HKUST          | Robust and Versatile Monocular Visual-Inertial State Estimator                                                       |    |
| [SLAM_toolbox](https://github.com/SteveMacenski/slam_toolbox) | Simbe Robotics / Samsung Research | SLAM for massive maps | :heavy_check_mark:  |

## Navigation
| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation)        | ETH          | creates polynomial path                                                                           | :heavy_check_mark:                 |        
| [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning)        | ETH          | planning tool using voxblox (RRT*, etc.)                                                          | :heavy_check_mark:                                                                 |
| [pulp-dronet](https://github.com/pulp-platform/pulp-dronet)                       | ETH          | deep learning visual navigation                                                                   |                                                                                       |
| [Ewok: real-time traj replanning](https://github.com/VladyslavUsenko/ewok)   | TUM          | replanning of global traj, needs prior map                                                        |                                                                                       |
| [Deep RL with Transfer Learning](https://github.com/aqeelanwar/DRLwithTL_real)    | Georgia Tech | end-to-end navigation trained from simulation                                                     |                                                                                    |
| [NVIDIA redtail project](https://github.com/NVIDIA-AI-IOT/redtail)            |              | Autonomous navigation for drones                                                                  |                                                                                         |
| [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner)  | HKUST        | robust and efficient trajectory planner for quads                                                 | :heavy_check_mark:                                                           |
| [ego-planner swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm) | Zhejiang University |  Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments
| [spatio-temporal semantic corridor](https://github.com/HKUST-Aerial-Robotics/spatiotemporal_semantic_corridor) | HKUST        | Safe Trajectory Generation For Complex Urban Environments Using Spatio-temporal Semantic Corridor |                                                          |
| [EVDodgeNet](https://github.com/prgumd/EVDodgeNet)                        | ETH          | obstacle avoidance with event cameras                                                             |                                                                                             |
| [aeplanner](https://github.com/mseln/aeplanner)                         | KTH          | unknown environment exploration based on octomap                                                  |                                                                                             |
| [nvbplanner](https://github.com/ethz-asl/nbvplanner)                        | ETH          | unknown environment exploration                                                                   |                                                                                        |
| [HKUST Aerial Robotics](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan)                   | HKUST        | a complete and robust system for aggressive flight in complex environment               |                          |
| [PX4 generalized intelligence](https://github.com/generalized-intelligence/GAAS)            |              | low-level autonomy for PX4                                                              |                                      |
| [PX4 avoidance](https://github.com/PX4/avoidance)                           |              |        low-level autonomy for PX4           |
| [sim2real_drone_racing](https://github.com/uzh-rpg/sim2real_drone_racing)                   | ETH          | deep learning Sim2Real Drone racing                                                     | :heavy_check_mark:                  |
| [waypoint_navigator](https://github.com/ethz-asl/waypoint_navigator) | ETH | high-level waypoint-following for micro aerial vehicles | :heavy_check_mark: |
| [autonomousmavs](https://github.com/IntelLabs/autonomousmavs) | | navigation in cluttered environment | :heavy_check_mark: |

## Strategic Decision-Making

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [Apollo Autonomous Driving](https://github.com/ApolloAuto/apollo)               | Apollo       | Full autonomous driving stack                                                           |                                                  |
| [ROS_behavior_tree](https://github.com/miccol/ROS-Behavior-Tree)                       |              | Behavior trees for autonomy                                                             |                                            |
| [planning in ROS](https://github.com/KCL-Planning/ROSPlan)                         |              | generic method for task planning                                                        | :heavy_check_mark:                            |
| [EDUM Planner](https://github.com/HKUST-Aerial-Robotics/eudm_planner)                            | HKUST        | decision-making for automated driving using guided branching                            |                                  |
| [autoware.ai](https://github.com/autowarefoundation/autoware.ai)                             |              | self-driving vehicles                                                                   |                                     |
| [dronet: learning to fly](https://github.com/uzh-rpg/rpg_public_dronet)                 | ETH          | deep learning trained from cars to predict steering angle, collision prob               | :heavy_check_mark: | 
| [Deep RL w Airsim](https://github.com/guillemhub/DRLDBackEnd)                        |              | allows RL with Airsim                                                                   |                                             |
| [Autonomous UAV swarms](https://github.com/AlexJinlei/Autonomous_UAVs_Swarm_Mission)                   |              |                                                                                         |                           |
| [autonomous-drone](https://github.com/szebedy/autonomous-drone)                        |              | enable autonomous drone delivery w Aero RTF and PX4                                     | :heavy_check_mark:           |
| [PEDRA](https://github.com/aqeelanwar/PEDRA)                                   | Georgia Tech | RL for drones with unreal engine                                                        |                                                   |
| [drif](https://github.com/VerifiableRobotics/slugs)                                    | Cornell      | Map Natural Language Instructions to Physical Quadcopter Control using Simulated Flight |                                     |
| [slugs](https://github.com/VerifiableRobotics/slugs)                                   |              | slugs - SmalL bUt Complete GROne Synthesizer                                            |                    |                                                                    |
| [LTL_stack](https://github.com/VerifiableRobotics/LTL_stack)                               |              | ROS Packages for running correct-by-construction controllers with ROS                   | :heavy_check_mark:                    | 
| [multidrone_planning](https://github.com/grvcTeam/multidrone_planning)  | | cooperative planning and mission execution in autonomous cinematography with multiple drones| :heavy_check_mark:|


## Multi-Agent

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [Robofleet](https://github.com/ut-amrl/robofleet) | UT Austin | Web-based multi-robot control and visualization for ROS | :heavy_check_mark:  |
| [Swarm-Formation](https://github.com/ZJU-FAST-Lab/Swarm-Formation) | ZJU FAST-Lab | Distributed Swarm Trajectory Optimization for Formation Flight in Dense Environments | |
| [ROS2Swarm](https://gitlab.iti.uni-luebeck.de/ROS2/ros2swarm) | University of Lübeck | Provides swarm behavior for different hardware platforms. | :heavy_check_mark: |

## Controls

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [lbmpc_ipm](https://github.com/mhkabir/LBMPC)                    |             |  Learning-Based Model Predictive Control (LBMPC) that uses the LBmpcIPM solver                                           |                                |
| [neural_mpc](https://github.com/aravindsrinivas/neural-mpc)                   | Berkeley    | Model Predictive Control with one-step feedforward neural network dynamics model from Model-based Reinforcement Learning |                   |
| [Control Toolbox](https://github.com/ethz-adrl/control-toolbox)              | ETH         | efficient C++ library for control, estimation, optimization and motion planning in robotics                              |                    |
| [PythonLinearNonlinearControl](https://github.com/Shunichi09/PythonLinearNonlinearControl) |             | library implementing the linear and nonlinear control theories in python                                                 |      |
| [rpg_mpc](https://github.com/uzh-rpg/rpg_mpc)                      | ETH         | Model Predictive Control for Quadrotors with extension to Perception-Aware MPC                                           |                                              |
| [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control)        | ETH         | alternative to PX4 that works with RotorS                                                                                |                |
| [gymFC](https://github.com/wil3/gymfc)                        |             | flight control tuning framework with a focus in attitude control                                                         |                                   |
| [ACADO toolkit](http://acado.github.io/index.html)                |             | MPC toolkit that takes care of the implementation                                                                        |                               |
| [MPC ETH](https://github.com/ethz-asl/mav_control_rw)                      | ETH         | also has PX4 implementation (claim badly hacked though)                                                                  |                     |
| [mavros_controller](https://github.com/Jaeyoung-Lim/mavros_controllers)            | PX4         | trajectory tracking based on geometric control                                                                           |              |
| [DDC-MPC](https://github.com/uzh-rpg/data_driven_mpc)                      |      ETH       | Data-Driven MPC for Quadrotors
| [Deep-drone acrobatics](https://github.com/uzh-rpg/deep_drone_acrobatics)             | ETH          | fly complex maneuvers with multi-layer perceptron                                                 |                                                                                   |
| [mav_control_rw](https://github.com/ethz-asl/mav_control_rw)                    | ETH          | trajectory tracking with MPC                                                                      |                                                                                         |
| [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control)             | ETH          | complete framework for flying quadrotors                                                          |                                                                            |
| [flight controller](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/n3ctrl)                 | HKUST        | high level controller compatible with DJI N3 flight controller                                    |                     |
| [mavros_trajectory_tracking](https://github.com/mzahana/mavros_trajectory_tracking) | | combines mav_trajectory_generation and waypoint_navigator with mavros_controller |:heavy_check_mark: | 
| [system identification scripts](https://github.com/ethz-asl/mav_system_identification) |ETH | calculates model parameters for a drone | |
| [MRS UAV framework](https://github.com/ctu-mrs/mrs_uav_system) | CTU | framework for controlling drones with PX4 and different advanced controllers | |



## Useful Tools and Resources

| Link   | Who    | Description  | ROS          |
|:-------------|-------------|-------------|-------------|
| [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)             |             | great overview of robotics   |                               
| [Awesome-robotic-tooling](https://github.com/Ly0n/awesome-robotic-tooling)    |             | important tools for robotic programming |
| [awesome-dronecraft](https://github.com/Zarkopafilis/awesome-dronecraft)         |             | everything about drones                                           |                                 |
| [resilience-engineering](https://github.com/lorin/resilience-engineering)     |             | How to make safe systems?                                         |                                 |
| [Trajectory Prediction](https://github.com/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction)      |             | resources for predicting environment like movement of pedestrians |  |
| [hidden markov model](https://github.com/chauvinSimon/hmm_for_autonomous_driving)        |             | models lane switching, might be interesting                       |                      |
| [modelling agents w prob](https://agentmodels.org/)    |             | MPD, POMPD, etc.                                                  |                                   |
| [hierarchical state machine](http://wiki.ros.org/smach/Tutorials) |          | Develop robotic tasks through graphical user interface            |                                             |
| [Uncertainty estimation in deep learning](https://github.com/mattiasegu/uncertainty_estimation_deep_learning) | ETH          | can quantify uncertainty on existing neural networks                                    |                   |
| [Flightmare simulator](https://github.com/uzh-rpg/flightmare)           |             | Flightmare is composed of two main components: a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation.                                        | |
| [US-manufactured drones](https://dronelife.com/2021/05/13/u-s-based-drone-manufacturers-check-out-our-updated-list/) | | A list of drones manufactured in the US. | |

## Labs/Organizations to follow
| Lab Website | Git | Where       |
|:-------------|-------------|-------------|
| [Robotics & Perception Group](http://rpg.ifi.uzh.ch/) | [Link](https://github.com/uzh-rpg) | Zurich, Switzerland |
| [GRASP Lab](https://www.kumarrobotics.org/) | [Link](https://github.com/KumarRobotics) | Philadelphia, USA
| [ZJU FAST Lab](http://zju-fast.com/) | [Link](https://github.com/ZJU-FAST-Lab) | Hangzhou, China |


# uav-resources-b

[![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)](https://github.com/sindresorhus/awesome)

This is a list of various resources related to drones, UAV's and quadcopters. It's an attempt to gather useful material in one place for everybody who wants to learn more about the field.

## Legend

* :dollar: - Paid product
* :ghost: - Outdated or Inactive

## Table of Contents

- [Courses](#courses)
  - [Udemy](#udemy)
- [Software and Librairies](#software-and-librairies)
  - [Simulators](#simulators)
  - [Firmware for Transmitters](#firmware-for-transmitters)
  - [Firmware for Flight Controllers](#firmware-for-flight-controllers)
  - [Libraries](#libraries)
  - [Ground Control Stations](#ground-Control-Stations)
- [Services](#services)
- [Hardware and Components](#hardware-and-components)
  - [Platforms](#platforms)
  - [Remote Control Transmitters](#remote-control-transmitters)
  - [Drone Frames](#drone-frames)
  - [Headsets](#headsets)
  - [Video Receivers](#video-receivers)
  - [Electronics and Motors](#electronics-and-motors)
  - [Cameras](#cameras)
- [Products and Projects](#products-and-projects)
  - [Unmanned Aerial Vehicles](#unmanned-aerial-vehicles)
    - [Consumer](#consumer)
    - [Millitary](#millitary)
  - [Unmanned Ground Vehicles](#unmanned-ground-vehicles)
    - [Autonomous Ground Vehicles](#autonomous-ground-vehicles)
  - [Unmanned Underwater Vehicles](#unmanned-underwater-vehicles)
- [Visual Localization](#visual-localization)

## Courses

* [Flying Car and Autonomous Flight Engineer](https://eu.udacity.com/course/flying-car-nanodegree--nd787) Udacity - Master autonomous flight software engineering skills as you build your career in flying cars and drone robotics.
* [Robotics: Dynamics and Control](https://www.edx.org/course/robotics-dynamics-control-pennx-robo3x) edX - Learn how to develop dynamic models of robot manipulators, mobile robots, and drones (quadrotors).

### Udemy

* [UAS/Drone Remote Pilot Test Prep for Part 107](https://www.udemy.com/remote-pilot-certificate-test-prep-for-part-107-exam/) - :dollar: - A comprehensive class that encompasses everything needed to know to become a proficient Remote Pilot and to pass the FAA written initial or recurrent exam.
* [Drone Photography | Shoot Professional Photos With Any Drone](https://www.udemy.com/course/dronephotography/) - :dollar: - Your Complete Online Guide to Shooting Incredible Drone Photography Like a Professional
* [Drone Programming with Python - Face Recognition & Tracking](https://www.udemy.com/course/drone-programming-with-python-face-recognition-tracking/) - :dollar: - Operating drone with network programming, face recognition using OpenCV, automatic tracking, implementing web camera

## Software and Libraries

* [ArduPilot Mission Planner](https://github.com/ArduPilot/MissionPlanner) - Mission planner software.
* [Paparazzi](http://wiki.paparazziuav.org/wiki/Main_Page) - Software suite for UAVs, including ground control and autopilot.
* [QGroundControl](http://qgroundcontrol.com/) - Ground Control Station for PX4 and ArduPilot based UAVs.

### Simulators

* [AirSim](https://github.com/Microsoft/AirSim) - Open source simulator based on Unreal Engine for autonomous vehicles.
* [Drone Racing Arcade](https://thedroneracingleague.com/arcade/) - Mobile based FPV racing game
* [DRL Drone Racing Simulator](https://thedroneracingleague.com/drlsim/) - FPV Racing game and simulator with official DRL tracks.
* [FPV Air 2](https://store.steampowered.com/app/889040/FPV_Air_2/) - :dollar: - Basic FPV simulator, runs on slower hardware. Available on Steam.
* [FPV Freerider](https://fpv-freerider.itch.io/fpv-freerider) - :dollar: FPV (first person view) and LOS (line of sight) racing simulator.
* [FPV Freerider Recharged](https://fpv-freerider.itch.io/fpv-freerider-recharged) - :dollar: FPV (first person view) and LOS (line of sight) racing simulator.
* [LiftOff](https://www.immersionrc.com/fpv-products/liftoff-drone-race-simulator/) - :dollar: FPV racing simulator with realistic OSD (on-screen display) experience.
* [Orqa FPV.SkyDive](https://skydive.orqafpv.com/) - Orqa FPV's racing and freestyle simulator.
* [RotorRush](http://rotorrush.com/) - :dollar: Formerly known as FPV Event. Subscription based simulator.
* [VelociDrone](https://www.velocidrone.com/) - :dollar: Multiplayer FPV racing simulator.

### Firmware for Transmitters

* [FreedomTX](https://github.com/tbs-fpv/freedomtx) - Custom firmware for TBS Tango 2 based on OpenTX.
* [OpenTX](http://www.open-tx.org/) - Highly configurable open source firmware for RC radio transmitters.

### Firmware for Flight Controllers

* [Ardupilot](https://github.com/ArduPilot/ardupilot)
* [BaseFlight](https://github.com/multiwii/baseflight) - :ghost:
* [Betaflight](https://github.com/betaflight/betaflight) - Fork of Cleanflight.
* [ButterFlight](https://github.com/ButterFlight/butterflight) - :ghost: Fork of Betaflight. Firmware focusing on Mini Quads.
* [Cleanflight](https://github.com/cleanflight/cleanflight) - :ghost: Fork of BaseFlight. Supports more FCs and has additional PID contollers.
* [dRonin](https://dronin.org) - :ghost: Autopilot/flight controller firmware for controllers in the OpenPilot/Tau Labs family.
* [EmuFlight](https://github.com/emuflight/EmuFlight) - FC Firmware focusing on flight performance, innovative filtering, leading-edge feature additions, and wide target support.
* [FalcoX](https://flightone.com/download.php?version=stable) - Formerly known as Raceflight one, FlightOne.
* [INAV](https://github.com/iNavFlight/inav)
* [Kiss](https://www.flyduino.net/en_US/page/downloads) - Firmware for KISS FCs.
* [LibrePilot](https://github.com/librepilot/LibrePilot) - :ghost: (GitHub fork is Outdated/Inactive)
* [Open Source Rover Control Code](https://github.com/nasa-jpl/osr-rover-code) - Nasa JPL command firmware for the OSR.
* [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) - Rebranded to AutoPilot from Firmware
* [SilverWare](https://github.com/silver13/BoldClash-BWHOOP-B-03) - :ghost: Firmware for BoldClash BWHOOP B-03 mini drone
* [SilverWare(NFE)](https://github.com/NotFastEnuf/NFE_Silverware) - :ghost: Firmware for Alienwhoop ZER0, E011, BWHOOP B-03, H8mini, and BETA FPV LITE flight controllers with NotFastEnuf settings and experimental features

### Libraries

* [DJI Onboard SDK](https://github.com/dji-sdk/Onboard-SDK) - The Onboard SDK allows you to connect to a supported DJI flight controller using a serial port (TTL UART).
* [GoBot](https://github.com/hybridgroup/gobot) - Golang framework for robotics, drones, and the Internet of Things (IoT).
* [Libcyphal](https://github.com/OpenCyphal-Garage/libcyphal) - Portable reference implementation of the Cyphal protocol stack in C++ for embedded systems and Linux. Formerly known as LibUAVCAN.
* [MAVLink](https://github.com/mavlink/mavlink) - Micro Air Vehicle Message Marshalling Library.
* [MAVROS](https://github.com/mavlink/mavros) - MAVLink to ROS gateway with a proxy for Ground Control Station.

### Ground Control Stations

* [QGroundControl](https://github.com/mavlink/qgroundcontrol) - Cross-platform ground control station for drones (Android, iOS, Mac OS, Linux, Windows).
* [Arduleader](https://github.com/geeksville/arduleader) - :ghost: An android ground controller (and other things) for Mavlink/Arduplane.
* [Tower](https://github.com/DroidPlanner/Tower) - :ghost: Ground Control Station for Android Devices.
* [MAVProxy](http://ardupilot.github.io/MAVProxy/) - A UAV ground station software package for MAVLink based systems.
* [Ardupilot Mission Planner](https://ardupilot.org/planner/index.html) - A full-featured ground station application for the ArduPilot open source autopilot project.
* [APM Planner 2](https://ardupilot.org/planner2/) - An open-source ground station application for MAVlink based autopilots including APM and PX4/Pixhawk that can be run on Windows, Mac OSX, and Linux.

## Services

* [AirMap](https://www.airmap.com/) - Aeronautical data & services to unmanned aircraft.
* [DroneDeploy](https://www.dronedeploy.com/) - Drone & UAV Mapping Software.
* [RotorBuilds](https://rotorbuilds.com/) - FPV Part lists and Build Logs.
* [Zeitiew](https://www.zeitview.com/) - Online marketplace for Drone services. Formerly known as DroneBase.

## Hardware and Components

### Platforms

* [OpenUAV](https://openuav.eava.ee) - Open-souce UAV platform for research and development

### Remote Control Transmitters

* [FlySky](http://www.flyskyrc.com/) - Entry level transmitters.
* [FrSky](https://www.frsky-rc.com/) - Taranis and Horus line of transmitters powered by OpenTX firmware.
* [Futaba](https://www.futabarc.com)
* [Spektrum](https://www.spektrumrc.com)
* [Team Blacksheep](https://team-blacksheep.com) - Tango 1 and 2 transmitters.

### Drone Frames

* [Source One by TBS](https://github.com/tbs-trappy/source_one) - Open Source freestyle FPV drone frame.
* [Source Two by TBS](https://github.com/ps915/source_two) - Open Source racing FPV drone frame.
* [Source Micro by TBS](https://github.com/ps915/source_micro) - Open Source mini drone frame.
* [Source PodRacer](https://github.com/ps915/source_podracer) - Open source ultra-light drone frame.
* [Source V by TBS](https://github.com/ps915/source_v) - Open Source ultra-stiff drone frame.
* [Source X by TBS](https://github.com/ps915/source_x) - Open Source giant racing drone frame.

### Headsets

* [DJI Digital FPV System](https://www.dji.com/fpv/) - Low latency digital FPV goggles.
* [FatShark](https://www.fatshark.com) - Headsets praised by racers.
* [ORQA FPV.One](https://orqafpv.com) - Headsets and controllers

### Video Receivers

#### 1.3GHz

* [ClearView XLR 1.3](https://clearview-direct.com/product-category/ground-station-receivers/1-3-receivers/)

#### 2.4GHz

* [ClearView XLR 2.4](https://clearview-direct.com/product-category/ground-station-receivers/2-4-receivers/)
* [TBS Ground station](https://www.team-blacksheep.com/products/prod:tbs_gs_2g4)

#### 5.8GHz

* [ClearView Goggle Receiver](https://clearview-direct.com/shop/clearview-goggle-products/clearview-goggle-module/)
* [ClearView 5.8GHz Ground Station Receivers](https://clearview-direct.com/product-category/ground-station-receivers/5-8-receivers/)
* [rapidFIRE](https://www.immersionrc.com/fpv-products/rapidfire/)
* [TBS Fusion](https://www.team-blacksheep.com/products/prod:tbs_fusion) - Has CRSF integration (for changing channels).

### Electronics and Motors

Terminology:

* FC = Flight Controller
* ESC = Electronic Speed Controller
* PDB = Power Distribution Board
* RX = Receivers
* TX = Transmitters (external)
* VRX = Video Receiver
* VTX = Video Transmitter

List:

* [3BHobby](https://www.3bhobby.com) - Motors
* [Airbot](https://store.myairbot.com/flight-controller.html) - FC, ESC
* [BrotherHobby](https://www.brotherhobbystore.com) - Motors
* [ClearView](https://clearview-direct.com/) - VRX, VTX
* [DalProp](http://dalprop.com) - Props
* [Diatone](https://www.diatone.us/) - FC, ESC, VTX, Motors
* [Fl1ghtOne](https://flightone.com) - FC, ESC
* [Flyduino](https://www.flyduino.net/en_US/) - FC, ESC
* [GemFan](https://www.gfprops.com/) - Props
* [Hobbywing](http://www.hobbywing.com) - FC, ESC, Motors
* [Holybro](http://www.holybro.com) - FC, ESC, PDB
* [HQProp](http://www.hqprop.com) - Props
* [iFlight](https://www.iflight-rc.com) - Motors
* [Lumenier](https://www.lumenier.com) - FC, ESC, PDB, VTX, Motors
* [MatekSys](http://www.mateksys.com) - FC, PDB, VTX
* [RacerStar](https://www.racerstar.com) - FC, ESC, Motors
* [SP Racing](http://seriouslypro.com) - FC
* [T-Motor](http://www.tmotor.com) - FC, Motors
* [Team Blacksheep](https://www.team-blacksheep.com) - ESC, RX, TX, VRX, VTX

### Cameras

* [Caddx.us](https://caddxfpv.com/)
* [DJI O3 Air unit](https://www.dji.com/ee/o3-air-unit) - Camera with digital video transmission
* [Foxeer](http://www.foxeer.com)
* [RunCam](https://runcam.com)

## Products and Projects

### Unmanned Aerial Vehicles

#### Consumer

* [Autel](https://www.autelpilot.eu/) - :cn: - Compact EVO series drones, alternative to DJI Mavic series. Dragonfish series fixed-wing UAVs coming soon.
* [DJI](https://www.dji.com/) - :cn: - DJI is the world's leader in the consumer drone market - Mavic, Phantom, Inspire and Matrice series drones.
* [Eachine](https://www.eachine.com/) - :cn: - Mini and micro drones. FPV beginners sets.
* [Hubsan](https://www.hubsan.com/na/) - :cn: - Micro and Mini lower cost drones.
* [Parrot SA](https://www.parrot.com/global/) - :fr: - Famously Parrot Bebop and Parrot AR series drones.
* [Syma](http://www.symatoys.com/) - :cn: - RC toy quadcopters.
* [Yuneec International](https://www.yuneec.com/) - :cn: - Yuneec camera drones.

#### Military

* [AeroVironment](https://www.avinc.com/) - Small range fixed-wing UAVs.
* [Baykar](https://www.baykartech.com/en/) - Bayraktar series battle- proven long-range fixed-wing UAVs.
* [Eli](http://www.uav.ee/) - Pneumatic launchers for fixed wing UAVs.
* [INSITU](https://www.insitu.com/) - Long-range and extended endurance fixed-wing UAVs.
* [Threod Systems](http://threod.com/) - Fixed-wing and multi-rotor UAVs.

### Unmanned Ground Vehicles

#### Autonomous Ground Vehicles

* [JPL Open Source Rover](https://github.com/nasa-jpl/open-source-rover) - Nasa JPL scaled down version of the curiosity rover, made of COTS.
* [Turtlebot](https://www.turtlebot.com) - Open sourced UGV, [ROS](http://www.ros.org/) standard platform.

### Unmanned Underwater Vehicles

* [Geneinno](https://www.geneinno.com/) - Underwater drones.

## Visual Localization
*   [Drone-Satellite-Ground Three Platiform Localization](https://github.com/layumi/University1652-Baseline) 
*   [ACM MM2023 Workshop: UAV in Multimedia](https://www.zdzheng.xyz/ACMMM2023Workshop/)
*   [Visual Localization Leaderboard](https://github.com/layumi/University1652-Baseline/tree/master/State-of-the-art)

## License

[![CC0][CC0-badge]](LICENSE)

To the extent possible under law, [Jaan Janesmae](https://jaan.janesmae.com) has waived all copyright and related or neighbouring rights to this work.

[CC0-badge]: http://mirrors.creativecommons.org/presskit/buttons/88x31/svg/cc-zero.svg

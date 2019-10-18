# Awesome Robotic Tooling ![Awesome](https://cdn.rawgit.com/sindresorhus/awesome/d7305f38d29fed78fa85652e3a63e154dd8e8829/media/badge.svg)

Just a bunch of powerful robotic development resources and tools for professional robotic development.

**ONLY Packages that are still maintained!**

* [Example Format](#example-format)
* [Development Environment](#development-environment)
  * [Backbone](#backbone)
  * [Docker](#docker)
* [Dataprocessing](#data-processing)
  * [Image Processing](#image-processing)
  * [Point Cloud Processing](#point-cloud-processing)
* [Middleware](#package-managers)
  * [Timeline](#timeline)
  * [Spreadsheet](#spreadsheet)
* [Security](#security)
* [Safety](#safety)
* [Documentation](#documentation)
* [Commandline]
* [Robotic Functions]
  * [Localisation]
  * [Detection]
  * [Planning]
  * [Decision]
----


## Coordination and Communication
* [Taiga](https://github.com/benhutchins/docker-taiga) - Agile Projectmanagment Tool
* [Kanboard](https://github.com/kanboard/kanboard) - Minimalistic Kanban Board
* [Gitlab](https://github.com/sameersbn/docker-gitlab) - Simple Selfhosted Gitlab Server with Docker
* [Gitflow](https://github.com/nvie/gitflow) - Makes parallel development very easy, by isolating new development from finished work
* [Gogs](https://github.com/gogs/gogs) - Aims to build a simple, stable and extensible self-hosted Git service that can be setup in the most painless way
* [Woost](https://woost.space/) - Workflow Automatisation
* [Wekan](https://github.com/wekan/wekan) - Meteor based Kanban Board
* [JIRA API](https://github.com/pycontribs/jira) - Python Library for REST API of Jira
* [Taiga API](https://github.com/nephila/python-taiga) - Python Library for REST API of Taiga
* [Agile Development](https://agilemanifesto.org/) - Manifesto for Agile Software Development
* [Chronos - Timetracker](https://github.com/web-pal/chronos-timetracker)  - Desktop client for JIRA. Track time, upload worklogs without a hassle
* [Grge](https://gitlab.com/ApexAI/grge) - Grge is a daemon and command line utility augmenting GitLab
* [Issue Gitlab Good Practice](https://docs.gitlab.com/ee/development/contributing/issue_workflow.html) - Issue triage policies
* [Git-repo](https://gerrit.googlesource.com/git-repo/) - Repo helps manage many Git repositories, does the uploads to revision control systems, and automates parts of the development workflow

## Documentation and Presentation
* [Typora](https://typora.io/) - A Minimalist Markdown Editor
* [Markor](https://github.com/gsantner/markor) - A Simple Markdown Editor for your Android Device
* [Pandoc](https://github.com/jgm/pandoc) - Universal markup converter
* [Yaspeller](https://github.com/hcodes/yaspeller) - Command line tool for spell checking
* [Doxygen](https://github.com/doxygen/doxygen) - Doxygen is the de facto standard tool for generating documentation from annotated C++ sources
* [Word-to-Markdown](https://github.com/benbalter/word-to-markdown) - A ruby gem to liberate content from Microsoft Word document
* [DeepL](https://deepl.com) - Translate complete documents better than most humans
* [ASCIIMATICS](https://github.com/peterbrittain/asciimatics) - Create a GIF for your command line examples
* [Reveal-Hugo](https://github.com/dzello/reveal-hugo) - A Hugo theme for Reveal.js that makes authoring and customization a breeze. With it, you can turn any properly-formatted Hugo content into a HTML presentation.


## Architecture and Design
* [yed](https://www.yworks.com/products/yed) - yEd is a powerful desktop application that can be used to quickly and effectively generate high-quality diagrams
* [yed_py](https://github.com/true-grue/yed_py) - Generates graphML that can be opened in yEd
* [Plantuml](https://github.com/plantuml/plantuml-server) - Web application to generate UML diagrams on-the-fly in your live documentation	
* [rqt_graph](https://wiki.ros.org/rqt_graph) - rqt_graph provides a GUI plugin for visualizing the ROS computation graph
* [rqt_launchtree](https://github.com/pschillinger/rqt_launchtree) - An RQT plugin for hierarchical launchfile configuration introspection.
* [cpp-dependencies](https://github.com/tomtom-international/cpp-dependencies) - Tool to check C++ #include dependencies (dependency graphs created in .dot format)


## Framework
* [ROS](https://github.com/ros) - ROS (Robot Operating System) provides libraries and tools to help software developers create robot applications
* [ROS2](https://github.com/ros2/ros2) - ROS2 is the next generation robot operating system, and is actively being developed to fully replace ROS1 in the near future
* [OpenPilot](https://github.com/commaai/openpilot) - Open Source Aaptive Cruise Control (ACC) and Lane Keeping Assist System (LKAS) 
* [Apollo](https://github.com/ApolloAuto/apollo) - High performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.
* [Autoware.ai](https://gitlab.com/autowarefoundation/autoware.ai) - Autoware.AI is the world's first "All-in-One" open-source software for autonomous driving technology
* [AutowareAuto](https://autowareauto.gitlab.io/AutowareAuto/) - It is a clean slate rewrite of Autoware. Autoware.Auto applies best-in-class software engineering.
* [Stanford Self Driving Car Code](https://github.com/emmjaykay/stanford_self_driving_car_code) - Stanford Code From Cars That Entered DARPA Grand Challenges


## Development Environment

### Operation System
* [Yocto](https://git.yoctoproject.org/) - Produce tools and processes that enable the creation of Linux distributions for embedded software that are independent of the underlying architecture of the embedded hardware
* [Automotive Graded Linux](https://www.automotivelinux.org/software) - is a collaborative open source project that is bringing together automakers, suppliers and technology companies to build a Linux-based, open software platform for automotive applications that can serve as the de facto industry standard
* [robot_upstart](https://github.com/clearpathrobotics/robot_upstart) - presents a suite of scripts to assist with launching background ROS processes on Ubuntu Linux PCs
* [bitbake](https://github.com/openembedded/bitbake) - is a generic task execution engine that allows shell and Python tasks to be run efficiently and in parallel while working within complex inter-task dependency constraints.
* [Jailhouse](https://github.com/siemens/jailhouse) - Jailhouse is a partitioning Hypervisor based on Linux
* [Xen](https://wiki.debian.org/Xen) - is an open-source (GPL) type-1 or baremetal hypervisor
* [QEMU](https://www.qemu.org/) - is a generic and open source machine emulator and virtualizer

### Template

* [ROS](https://github.com/leggedrobotics/ros_best_practices/tree/master/ros_package_template) - Template for ROS node standardization in C++ 
* [Launch](https://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) Templates on how to create launch files for larger projects
* [Bash](https://github.com/ralish/bash-script-template) - A bash scripting template incorporating best practices & several useful functions
* [URDF](https://wiki.ros.org/urdf/Examples) - Examples on how to create Unified Robot Description Format (URDF) for different kinds of robots
* [Python](http://wiki.ros.org/PyStyleGuide) - Style guide to be followed in writing Python code for ROS

### Code and Run
[Vim](https://github.com/amix/vimrc) - The ultimate Vim configuration: vimrc			
[vccode](https://github.com/Microsoft/vscode) - Code editor with for edit-build-debug cycle.
[atom](https://github.com/atom/atom) - Hackable text editor for the 21st century
[Sublime](https://www.sublimetext.com/) - A sophisticated text editor for code, markup and prose
[ade-cli](https://gitlab.com/ApexAI/ade-cli) - The ADE Development Environment (ADE) uses docker and gitlab to manage environments of per project development tools and optional volume images
[Jupyter ROS](https://github.com/RoboStack/jupyter-ros) - Jupyter widget helpers for ROS, the Robot Operating System
[ros_rqt_plugin](https://github.com/ros-industrial/ros_qtc_plugin) - The ROS Qt Creator Plug-in 	
[ROS IDEs](http://wiki.ros.org/IDEs) - This page collects experience and advice on using integrated development environments (IDEs) with ROS.
[TabNine](https://github.com/zxqfl/TabNine) - The all-language autocompleter
[kite](https://kite.com/) - Use machine learning to give you 
 useful code completions for Python
[jedi](https://github.com/davidhalter/jedi) - Autocompletion and static analysis library for python
[roslibpy](https://github.com/gramaziokohler/roslibpy) - Python ROS Bridge library allows to use Python and IronPython to interact with ROS, the open-source robotic middleware. 

### Build and Deploy
[bloom](https://github.com/ros-infrastructure/bloom) - A release automation tool which makes releasing catkin packages easier
[catkin_tools](https://github.com/catkin/catkin_tools) - Command line tools for working with catkin
[industrial_ci](https://github.com/ros-industrial/industrial_ci) - Easy continuous integration repository for ROS repositories
[gitlab-runner](https://gitlab.com/gitlab-org/gitlab-runner) -  runs tests and sends the results to GitLab
[colcon-core](https://github.com/colcon/colcon-core) - command line tool to improve the workflow of building, testing and using multiple software packages
[gitlab-release](https://gitlab.com/alelec/gitlab-release) - Simple python3 script to upload files (from ci) to the current projects release (tag)

## Hardware
[HRIM](https://github.com/AcutronicRobotics/HRIM) - An information model for robot hardware
[URDF](https://github.com/ros/urdf) - Repository forUnified Robot Description Format (URDF) parsing code
[urdf-viz](https://github.com/OTL/urdf-viz) - Visualize URDF/XACRO file, URDF Viewer works on Windows/MacOS/Linux
[solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter) - SolidWorks to URDF Exporter
[FreeCAD](https://github.com/FreeCAD/FreeCAD) - Your own 3D parametric modeler

### Calibration
[lidar_align](https://github.com/ethz-asl/lidar_align) - A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor
[kalibr](https://github.com/ethz-asl/kalibr) - The Kalibr visual-inertial calibration toolbox
[Calibnet](https://github.com/epiception/CalibNet) - Self-Supervised Extrinsic Calibration using 3D Spatial Transformer Networks
[lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration) - ROS package to find a rigid-body transformation between a LiDAR and a camera
[easy_handeye](https://github.com/IFL-CAMP/easy_handeye) - Simple, straighforward ROS library for hand-eye calibration
[imu_utils](https://github.com/gaowenliang/imu_utils) - A ROS package tool to analyze the IMU performance
[kalibr_allan]https://github.com/rpng/kalibr_allan - IMU Allan standard deviation charts for use with Kalibr and inertial kalman filters

## Simulation
[simulator](https://github.com/lgsvl/simulator) - A ROS/ROS2 Multi-robot Simulator for Autonomous Vehicles
[carla](https://github.com/carla-simulator/carla) - Open-source simulator for autonomous driving research
[deepdive](https://github.com/deepdrive/deepdrive) - End-to-end simulation for self-driving cars 
[uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) - Gazebo/ROS packages for underwater robotics simulation
[AirSim](https://github.com/microsoft/AirSim) - Open source simulator for autonomous vehicles built on Unreal Engine 
[self-driving-car-sim](https://github.com/udacity/self-driving-car-sim) - A self-driving car simulator built with Unity
[ROSIntegration](https://github.com/code-iai/ROSIntegration) - Unreal Engine Plugin to enable ROS Support
[gym-gazebo](https://github.com/erlerobot/gym-gazebo) - An OpenAI gym extension for using Gazebo known as gym-gazebo

## Robotic Functions

### Localization
[evo](https://github.com/MichaelGrupp/evo) - Python package for the evaluation of odometry and SLAM
[robot_localization](https://github.com/cra-ros-pkg/robot_localization) - is a package of nonlinear state estimation nodes
[fuse](https://github.com/locusrobotics/fuse) - General architecture for performing sensor fusion live on a 
robot.
[rep-105](https://www.ros.org/reps/rep-0105.html) - Naming conventions and semantic meaning for
coordinate frames of mobile platforms used with ROS.
[GeographicLib](https://github.com/Sciumo/GeographicLib) - A C++ library for geographic projections.
[ntripbrowser](https://github.com/emlid/ntripbrowser) - A Python API for browsing NTRIP (Networked Transport of RTCM via Internet Protocol).

### Sensor Processing
#### Machine Learning
[DLIB](https://github.com/davisking/dlib) - A toolkit for making real world machine learning and data analysis applications in C++
[fastai](https://github.com/fastai/fastai) - The fastai library simplifies training fast and accurate neural nets using modern best practices.
[tpot](https://github.com/EpistasisLab/tpot) - A Python Automated Machine Learning tool that optimizes machine learning pipelines using genetic programming
[deap](https://github.com/DEAP/deap) - Distributed Evolutionary Algorithms in Python
[gym](https://github.com/openai/gym) - A toolkit for developing and comparing reinforcement learning algorithms.


#### Image Processing
[visp](https://github.com/lagadic/visp) - Open Source Visual Servoing Platform
[deep_object_pose](https://github.com/NVlabs/Deep_Object_Pose) - Deep Object Pose Estimation
[DetectAndTrack](https://github.com/facebookresearch/DetectAndTrack) - Detect-and-Track: Efficient Pose
[SfMLearner](https://github.com/tinghuiz/SfMLearner) - An unsupervised learning framework for depth and ego-motion estimation
[imgaug](https://github.com/aleju/imgaug) - Image augmentation for machine learning experiments

#### Point Cloud Processing
[cilantro](https://github.com/kzampog/cilantro) - A lean C++ library for working with point cloud data
[open3d](https://github.com/intel-isl/Open3D) - Open3D: A Modern Library for 3D Data Processing
[SqueezeSeg](https://github.com/BichenWuUCB/SqueezeSeg) - Implementation of SqueezeSeg, convolutional neural networks for LiDAR point clout segmentation
[point_cloud_io](https://github.com/ANYbotics/point_cloud_io) - ROS nodes to read and write point clouds from and to files (e.g. ply, vtk).
[python-pcl](https://github.com/strawlab/python-pcl) - Python bindings to the pointcloud library
[libpointmatcher](https://github.com/ethz-asl/libpointmatcher) - An "Iterative Closest Point" library for 2-D/3-D mapping in Robotics
[depth_clustering](https://github.com/PRBonn/depth_clustering) - Fast and robust clustering of point clouds generated with a Velodyne sensor. 
[lidar-bonnetal](https://github.com/PRBonn/lidar-bonnetal) - Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving

https://github.com/jianboqi/CSF - LiDAR point cloud ground filtering / segmentation (bare earth extraction) method based on cloth simulation

#### Parallel Processing
https://github.com/dask/dask - Parallel computing with task scheduling for Python
https://github.com/cupy/cupy - NumPy-like API accelerated with CUDA
https://github.com/thrust/thrust - Thrust is a C++ parallel programming library which resembles the C++ Standard Library.
https://github.com/arrayfire/arrayfire - ArrayFire: a general purpose GPU library.


#### SLAM
##### Lidar
https://github.com/laboshinl/loam_velodyne - Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar.
https://github.com/hyye/lio-mapping - Implementation of Tightly Coupled 3D Lidar Inertial Odometry and Mapping (LIO-mapping)
https://github.com/HKUST-Aerial-Robotics/A-LOAM - Advanced implementation of LOAM
https://github.com/googlecartographer/cartographer_ros - Provides ROS integration for Cartographer
https://github.com/EdwardLiuyc/StaticMapping - Use LiDAR to map the static world
https://github.com/hku-mars/loam_livox - A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR

#### Camera
https://github.com/JakobEngel/dso/ - Direct Sparse Odometry
https://github.com/srv/viso2 - A ROS wrapper for libviso2, a library for visual odometry
https://github.com/ucla-vision/xivo - X Inertial-aided Visual Odometry

#### Static Map Creation and Matching
https://github.com/eliemichel/MapsModelsImporter - A Blender add-on to import models from google maps
https://github.com/fzi-forschungszentrum-informatik/Lanelet2 - Map handling framework for automated driving
https://github.com/bmwcarit/barefoot -  Online and Offline map matching that can be used stand-alone and in the cloud

### Map Integration

[iD](https://github.com/openstreetmap/iD) - The easy-to-use OpenStreetMap editor in JavaScript
https://github.com/ethz-asl/segmap - A map representation based on 3D segments 

* [Mapbox](https://github.com/mapbox/mapbox-gl-jsMapbox) is a JavaScript library for interactive, customizable vector maps on the web

[osrm-backend](https://github.com/Project-OSRM/osrm-backend) - Open Source Routing Machine - C++ backend

### Behavior and Decision

https://github.com/BehaviorTree/BehaviorTree.CPP - Behavior Trees Library in C++
https://github.com/DLR-RM/RAFCON - Uses hierarchical state machines, featuring concurrent state execution, to represent robot programs

### Planning and Control
https://github.com/RoboJackets/rrt - C++ RRT (Rapidly-exploring Random Tree) implementation
https://github.com/AtsushiSakai/HybridAStarTrailer
https://github.com/karlkurzer/path_planner
https://github.com/ros-geographic-info/open_street_map
https://github.com/eleurent/highway-env	

#### User Interface
https://wiki.ros.org/dynamic_reconfigure
https://github.com/MirServer/mir
https://github.com/spyder-ide/qtpy
https://github.com/Hjdskes/cage
https://github.com/PySimpleGUI/PySimpleGUI

## Tools
### Command Line
https://github.com/cornerman/dotfiles
https://github.com/cornerman/prompt-hjem
https://github.com/ggreer/the_silver_searcher
https://github.com/junegunn/fzf
https://github.com/orhun/pkgtop
https://github.com/jroimartin/gocui
https://github.com/stefanhaustein/TerminalImageViewer
https://github.com/dheera/rosshow
https://github.com/prompt-toolkit/python-prompt-toolkit
https://github.com/ralish/bash-script-template
https://github.com/Guake/guake
https://github.com/zolrath/wemux
https://github.com/tmux-python/tmuxp
https://github.com/rastapasta/mapscii

### Storage

https://github.com/borgbackup/borg - Deduplicating archiver with compression and authenticated encryption
https://github.com/swri-robotics/bag-database - A server that catalogs bag files and provides a web-based UI for accessing them
https://gitlab.com/ternaris/marv-robotics - MARV Robotics is a powerful and extensible data management platform
https://github.com/nextcloud/server - Nextcloud is a suite of client-server software for creating and using file hosting services.

### High Performance Computing 

https://github.com/kubeflow/kubeflow
https://github.com/AliyunContainerService/log-pilot
https://github.com/containous/traefik
https://github.com/Graylog2/graylog2-server
https://github.com/ansible/ansible
https://github.com/docker/docker-py
https://github.com/novnc/noVNC
https://github.com/SchedMD/slurm
https://github.com/jupyterhub/jupyterhub
https://github.com/portainer/portainer	

### Network 
https://github.com/KimiNewt/pyshark
https://github.com/laixintao/pingtop
https://github.com/gcla/termshark
https://github.com/raboof/nethogs
https://github.com/ebroecker/canmatrix
https://github.com/osxfuse/sshfs
https://github.com/ApexAI/performance_test
https://github.com/appneta/tcpreplay
https://github.com/esnet/iperf
https://github.com/linux-can/can-utils
https://github.com/ros-industrial/ros_canopen
https://github.com/JWhitleyAStuff/decanstructor
https://discourse.ros.org/uploads/short-url/gVQHzWTRCL5R1aMKVmhmw83iZaA.pdf

### Annotation
https://github.com/RMonica/rviz_cloud_annotation
https://github.com/abreheret/PixelAnnotationTool
https://github.com/tzutalin/labelImg
https://github.com/CPFL/3d_labeling_tools

### Visualization
https://github.com/cruise-automation/webviz			
https://github.com/plotly/plotly.py
https://github.com/facontidavide/PlotJuggler
https://github.com/AIS-Bonn/rviz_cinematographer
https://github.com/bokeh/bokeh
https://github.com/voila-dashboards/voila
https://github.com/stevenlovegrove/Pangolin
https://github.com/thebjorn/pydeps
https://github.com/futurice/chilipie-kiosk

#### Point Clouds
https://github.com/CloudCompare/CloudCompare
https://github.com/potree/potree
https://github.com/googlecartographer/point_cloud_viewer
https://github.com/Kitware/ParaView

#### RVIZ 
https://github.com/swri-robotics/mapviz
https://github.com/gareth-cross/rviz_satellite
https://github.com/PickNikRobotics/rviz_visual_tools
https://github.com/AIS-Bonn/rviz_cinematographer
https://github.com/PickNikRobotics/tf_keyboard_cal

### Security
https://github.com/rfjakob/gocryptfs
https://github.com/imthenachoman/How-To-Secure-A-Linux-Server
https://github.com/CISOfy/lynis
https://github.com/WireGuard/WireGuard
https://github.com/ncsa/ssh-auditor
https://github.com/scipag/vulscan
https://github.com/x90skysn3k/brutespray
https://github.com/vulnersCom/nmap-vulners
https://github.com/fail2ban/fail2ban
https://github.com/lirantal/is-website-vulnerable
https://github.com/jeremylong/DependencyCheck

### Safety
https://oas.voyage.auto/
https://github.com/udacity/CarND-Functional-Safety-Project
https://github.com/stanislaw/awesome-safety-critical
https://www.automotivelinux.org/
https://github.com/boostorg/safe_numerics

### Unit and Integration Test

https://wiki.ros.org/Quality/Tutorials/UnitTesting
https://github.com/google/googletest
https://github.com/pytest-dev/pytest/

### Lint and Format
https://github.com/danmar/cppcheck
https://github.com/koalaman/shellcheck
https://github.com/fkie/catkin_lint
https://github.com/PyCQA/pylint/
https://github.com/psf/black
https://github.com/llvm-mirror/clang

### Version Control
https://github.com/sameersbn/docker-gitlab
https://github.com/kaiw/meld
https://github.com/jonas/tig
https://github.com/GNOME/gitg
https://github.com/python-gitlab/python-gitlab
https://github.com/rtyley/bfg-repo-cleaner

## System
https://github.com/hishamhm/htop
https://github.com/Atoptool/atop
https://github.com/giampaolo/psutil
https://github.com/anderskm/gputil
https://github.com/wookayin/gpustat
https://github.com/Syllo/nvtop

### Debugging and Tracing
https://github.com/khamidou/lptrace
https://github.com/facebook/pyre-check
https://github.com/brendangregg/FlameGraph
https://github.com/mikesart/gpuvis
https://github.com/google/sanitizers
https://github.com/andreasfertig/cppinsights

### Support
https://github.com/helpyio/helpy

### Real Time
https://elinux.org/Realtime_Testing_Best_Practices
https://elisa.tech/
https://github.com/eProsima/Fast-RTPS
https://wiki.linuxfoundation.org/realtime/documentation/start

[spdlog](https://github.com/gabime/spdlog) - Very fast, header-only/compiled, C++ logging library



### 


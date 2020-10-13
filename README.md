# Awesome Robotic Tooling [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

Just a bunch of powerful robotic resources and tools for professional development in C++ or Python with a touch of ROS, autonomous driving and aerospace.

> To stop reinventing the wheel you need to know about the wheel.

* [Communication and Coordination](#communication-and-coordination)
* [Documentation and Presentation](#documentation-and-presentation)
* [Requirements and Safety](#requirements-and-safety)
* [Architecture and Design](#architecture-and-design)
* [Frameworks and Stacks](#frameworks-and-stacks)
* [Development Environment](#development-environment)
   * [Code and Run](#code-and-run)
   * [Template](#template)
   * [Build and Deploy](#build-and-deploy)
   * [Unit and Integration Test](#unit-and-integration-test)
   * [Lint and Format](#lint-and-format)
   * [Debugging and Tracing](#debugging-and-tracing)
   * [Version Control](#version-control)
* [Simulation](#simulation)
* [Electronics and Mechanics](#electronics-and-mechanics)
* [Sensor Processing](#sensor-processing)
   * [Calibration and Transformation](#calibration-and-transformation)
   * [Perception Pipeline](#perception-pipeline)
   * [Parallel Processing](#parallel-processing)
   * [Machine Learning](#machine-learning)
   * [Radar Processing](#radar-processing)
   * [Image Processing](#image-processing)
   * [Lidar and Point Cloud Processing](#lidar-and-point-cloud-processing)
   * [Localization and State Estimation](#localization-and-state-estimation)
   * [Simultaneous Localization and Mapping](#simultaneous-localization-and-mapping)
      * [Lidar](#lidar)
      * [Visual](#visual)
   * [Vector Map](#vector-map)
* [Prediction](#prediction)
* [Behavior and Decision](#behavior-and-decision)
* [Planning and Control](#planning-and-control)
* [User Interaction](#user-interaction)
   * [Graphical User Interface](#graphical-user-interface)
   * [Acoustic User Interface](#acoustic-user-interface)
   * [Command Line Interface](#command-line-interface)
   * [Data Visualization and Mission Control](#Data-Visualization-and-Mission-Control)
      * [Annotation](#annotation)
      * [Point Cloud](#point-cloud)
      * [RViz](#rviz)
* [Operation System](#operation-system)
   * [Monitoring](#monitoring)
   * [Database and Record](#database-and-record)
   * [Network Distributed File System](#network-distributed-file-system)
   * [Server Infrastructure and High Performance Computing](#server-infrastructure-and-high-performance-computing)
   * [Embedded Operation System](#embedded-operation-system)
   * [Real-Time Kernel](#real-time-kernel)
   * [Network and Middleware](#network-and-middleware)
      * [Ethernet and Wireless Networking](#ethernet-and-wireless-networking)
      * [Controller Area Network](#controller-area-network)
      * [Sensor and Acuator Interfaces](#sensor-and-acuator-interfaces)
   * [Security](#security)
* [Datasets](#datasets)

## Communication and Coordination
* [Agile Development](https://agilemanifesto.org/) - Manifesto for Agile Software Development
* [Gitflow](https://github.com/nvie/gitflow) - Makes parallel development very easy, by isolating new development from finished work
* [DeepL](https://github.com/uinput/deeplator) - an online translator that outperforms Google, Microsoft and Facebook
* [Taiga](https://github.com/benhutchins/docker-taiga) - Agile Projectmanagment Tool
* [Kanboard](https://github.com/kanboard/kanboard) - Minimalistic Kanban Board
* [kanban](https://gitlab.com/leanlabsio/kanban) - Free, open source, self-hosted, Kanban board for GitLab issues
* [Gitlab](https://github.com/sameersbn/docker-gitlab) - Simple Selfhosted Gitlab Server with Docker
* [Gogs](https://github.com/gogs/gogs) - build a simple, stable and extensible self-hosted Git service that can be setup in the most painless way
* [Wekan](https://github.com/wekan/wekan) - Meteor based Kanban Board
* [JIRA API](https://github.com/pycontribs/jira) - Python Library for REST API of Jira
* [Taiga API](https://github.com/nephila/python-taiga) - Python Library for REST API of Taiga
* [Chronos-Timetracker](https://github.com/web-pal/chronos-timetracker)  - Desktop client for JIRA. Track time, upload worklogs without a hassle
* [Grge](https://gitlab.com/ApexAI/grge) - Grge is a daemon and command line utility augmenting GitLab
* [gitlab-triage](https://gitlab.com/gitlab-org/gitlab-triage) - GitLab's issues and merge requests triage, automated!
* [Helpy](https://github.com/helpyio/helpy) - is a modern, open source helpdesk customer support application
* [ONLYOFFICE](https://github.com/ONLYOFFICE/CommunityServer) -  is a free open source collaborative system developed to manage documents, projects, customer relationship and email correspondence, all in one place.
* [discourse](https://github.com/discourse/discourse) - A platform for community discussion. Free, open, simple
* [Gerrit](https://gerrit.googlesource.com/gerrit/) - is a code review and project management tool for Git based projects
* [jitsi-meet](https://github.com/jitsi/jitsi-meet) - Secure, Simple and Scalable Video Conferences that you use as a standalone app or embed in your web application
* [mattermost](https://github.com/mattermost/mattermost-server) - is an open source, private cloud, Slack-alternative
* [openproject](https://github.com/opf/openproject) - is the leading open source project management software
* [leantime](https://github.com/Leantime/leantime) - Leantime is a lean project management system for innovators
* [gitter](https://gitlab.com/gitlab-org/gitter/webapp) - Gitter is a chat and networking platform that helps to manage, grow and connect communities through messaging, content and discovery.
* [savannah](https://savannah.gnu.org/)  - The software forge for people committed to free software


## Documentation and Presentation
* [Typora](https://typora.io/) - A Minimalist Markdown Editor
* [Markor](https://github.com/gsantner/markor) - A Simple Markdown Editor for your Android Device
* [Pandoc](https://github.com/jgm/pandoc) - Universal markup converter
* [Yaspeller](https://github.com/hcodes/yaspeller) - Command line tool for spell checking
* [ReadtheDocs](https://docs.readthedocs.io/en/stable/development/buildenvironments.html) - Build your local ReadtheDocs Server
* [Doxygen](https://github.com/doxygen/doxygen) - Doxygen is the de facto standard tool for generating documentation from annotated C++ sources
* [Sphinx](https://github.com/sphinx-doc/sphinx/) - is a tool that makes it easy to create intelligent and beautiful documentation for Python projects
* [Word-to-Markdown](https://github.com/benbalter/word-to-markdown) - A ruby gem to liberate content from Microsoft Word document
* [paperless](https://github.com/the-paperless-project/paperless) - Index and archive all of your scanned paper documents
* [carbon](https://github.com/carbon-app/carbon) - Share beautiful images of your source code
* [undraw](https://undraw.co/illustrations) - Free Professional business SVGs easy to customize
* [asciinema](https://github.com/asciinema/asciinema) - lets you easily record terminal sessions and replay them in a terminal as well as in a web browser.
* [inkscape](https://inkscape.org/) - Inkscape is a professional vector graphics editor for Linux, Windows and macOS
* [Reveal-Hugo](https://github.com/dzello/reveal-hugo) - A Hugo theme for Reveal.js that makes authoring and customization a breeze. With it, you can turn any properly-formatted Hugo content into a HTML presentation.
* [Hugo-Webslides](https://github.com/RCJacH/hugo-webslides) - This is a Hugo template to create WebSlides presentation using markdown.
* [jupyter2slides](https://github.com/datitran/jupyter2slides) - Cloud Native Presentation Slides with Jupyter Notebook + Reveal.js
* [patat](https://github.com/jaspervdj/patat) - Terminal-based presentations using Pandoc
* [github-changelog-generator](https://github.com/github-changelog-generator/github-changelog-generator) - Automatically generate change log from your tags, issues, labels and pull requests on GitHub. 
* [GitLab-Release-Note-Generator](https://github.com/jk1z/GitLab-Release-Note-Generator) - A Gitlab release note generator that generates release note on latest tag
* [OCRmyPDF](https://github.com/jbarlow83/OCRmyPDF) - adds an OCR text layer to scanned PDF files, allowing them to be searched
* [papermill](https://github.com/nteract/papermill) - is a tool for parameterizing, executing, and analyzing Jupyter Notebooks.
* [docsy](https://github.com/google/docsy-example) - An example documentation site using the Docsy Hugo theme
* [actions-hugo](https://github.com/peaceiris/) - Deploy website based on Hugo to Github Pages 
* [overleaf](https://github.com/overleaf/overleaf) - An open-source online real-time collaborative LaTeX editor
* [landslide](https://github.com/adamzap/landslide) - Generate HTML5 slideshows from markdown, ReST, or textile 
* [libreoffice-impress-templates](https://github.com/dohliam/libreoffice-impress-templates) - Freely-licensed LibreOffice Impress templates 
* [opensourcedesign](https://opensourcedesign.net/resources/) - Community and Resources for Free Design and Logo Creation 
* [olive](https://www.olivevideoeditor.org/) - is a free non-linear video editor aiming to provide a fully-featured alternative to high-end professional video editing software
* [buku](https://github.com/jarun/buku) - Browser-independent bookmark manager 
* [swiftlatex](https://www.swiftlatex.com/) - a WYSIWYG Browser-based LaTeX Editor
* [ReLaXed](https://github.com/RelaxedJS/ReLaXed) - allows complex PDF layouts to be defined with CSS and JavaScript, while writing the content in a friendly, minimal syntax close to Markdown or LaTeX.
* [foam](https://github.com/foambubble/foam) - Foam is a personal knowledge management and sharing system inspired by Roam Research, built on Visual Studio Code and GitHub.
* [CodiMD](https://github.com/codimd/server) - Open Source Online Real-time collaborate on team documentation in markdown
* [jupyter-book](https://github.com/executablebooks/jupyter-book) - Build interactive, publication-quality documents from Jupyter Notebooks
* [InvoiceNet](https://github.com/naiveHobo/InvoiceNet) - Deep neural network to extract intelligent information from invoice documents
* [tesseract](https://github.com/tesseract-ocr/tesseract) - Open Source OCR Engine


## Requirements and Safety
* [awesome-safety-critical](https://github.com/stanislaw/awesome-safety-critical) - List of resources about programming practices for writing safety-critical software.
* [open-autonomous-safety](https://github.com/voyage/open-autonomous-safety) - OAS is a fully open-source library of Voyage’s safety processes and testing procedures, designed to supplement existing safety programs at self-driving car startups across the world.
* [CarND-Functional-Safety-Project](https://github.com/udacity/CarND-Functional-Safety-Project) - Create functional safety documents in this Udacity project
* [Automated Valet Parking Safety Documents](https://avp-project.uk/publication-of-safety-documents) - created to support the safe testing of the Automated Valet Parking function using the StreetDrone test vehicle in a car park.
* [safe_numerics](https://github.com/boostorg/safe_numerics) - Replacements to standard numeric types which throw exceptions on errors
* [Air Vehicle C++ development coding standards](http://www.stroustrup.com/JSF-AV-rules.pdf) - Provide direction and guidance to C++ programmers that will enable them to employ good programming style and proven programming practices leading to safe, reliable, testable, and maintainable code
* [AUTOSAR Coding Standard](https://www.autosar.org/fileadmin/user_upload/standards/adaptive/17-10/AUTOSAR_RS_CPP14Guidelines.pdf) - Guidelines for the use of the C++14 language in critical and safety-related system
* [The W-Model and Lean Scaled Agility for Engineering](https://assets.vector.com/cms/content/consulting/publications/AgileSystemsEngineering_Vector_Ford.pdf) - Ford applied an agile V-Model method from Vector that can be used in safety related project management
* [doorstop](https://github.com/doorstop-dev/doorstop) - Requirements management using version control
* [capella](https://www.eclipse.org/capella/) - Comprehensive, extensible and field-proven MBSE tool and method
to successfully design systems architecture
* [robmosys](https://robmosys.eu/) - RobMoSys envisions an integrated approach built on top of the current code-centric robotic platforms, by applying model-driven methods and tools
* [Papyrus for Robotics](https://www.eclipse.org/papyrus/components/robotics/) - is graphical editing tool for robotic applications that complies with the RobMoSys approach
* [fossology](https://github.com/fossology/fossology) - a toolkit you can run license, copyright and export control scans from the command line
* [ScenarioArchitect](https://github.com/TUMFTM/ScenarioArchitect) - The Scenario Architect is a basic python tool to generate, import and export short scene snapshots


## Architecture and Design
* [Guidelines](https://github.com/S2-group/icse-seip-2020-replication-package/blob/master/ICSE_SEIP_2020.pdf) - on how to architect ROS-based systems
* [yed](https://www.yworks.com/products/yed) - yEd is a powerful desktop application that can be used to quickly and effectively generate high-quality diagrams
* [yed_py](https://github.com/true-grue/yed_py) - Generates graphML that can be opened in yEd
* [Plantuml](https://github.com/plantuml/plantuml-server) - Web application to generate UML diagrams on-the-fly in your live documentation	
* [rqt_graph](https://wiki.ros.org/rqt_graph) - rqt_graph provides a GUI plugin for visualizing the ROS computation graph
* [rqt_launchtree](https://github.com/pschillinger/rqt_launchtree) - An RQT plugin for hierarchical launchfile configuration introspection.
* [cpp-dependencies](https://github.com/tomtom-international/cpp-dependencies) - Tool to check C++ #include dependencies (dependency graphs created in .dot format)
* [pydeps](https://github.com/thebjorn/pydeps) - Python Module Dependency graphs
* [aztarna](https://github.com/aliasrobotics/aztarna) -  a footprinting tool for robots.
* [draw.io](https://www.draw.io/) - is free online diagram software for making flowcharts, process diagrams, org charts, UML, ER and network diagrams 
* [vscode-drawio](https://github.com/hediet/vscode-drawio) - This extension integrates Draw.io into VS Code

## Frameworks and Stacks
* [ROS](https://github.com/ros) - (Robot Operating System) provides libraries and tools to help software developers create robot applications
* [awesome-ros2](https://github.com/fkromer/awesome-ros2) - A curated list of awesome Robot Operating System Version 2.0 (ROS 2) resources and libraries.
* [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto) - Autoware.Auto applies best-in-class software engineering for autonomous driving
* [Autoware.ai](https://github.com/Autoware-AI) - Autoware.AI is the world's first "All-in-One" open-source software for autonomous driving technology
* [OpenPilot](https://github.com/commaai/openpilot) - Open Source Adaptive Cruise Control (ACC) and Lane Keeping Assist System (LKAS) 
* [Apollo](https://github.com/ApolloAuto/apollo) - High performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.
* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/) - This is a Python code collection of robotics algorithms, especially for autonomous navigation.
* [Stanford Self Driving Car Code](https://github.com/emmjaykay/stanford_self_driving_car_code) - Stanford Code From Cars That Entered DARPA Grand Challenges
* [astrobee](https://github.com/nasa/astrobee) - Astrobee is a free-flying robot designed to operate as a payload inside the International Space Station (ISS).
* [CARMAPlatform](https://github.com/usdot-fhwa-stol/CARMAPlatform) - enables cooperative automated driving plug-in
* [Automotive Grade Linux](https://www.automotivelinux.org/) - Automotive Grade Linux is a collaborative open source project that is bringing together automakers, suppliers and technology companies to accelerate the development and adoption of a fully open software stack for the connected car
* [PX4](https://github.com/PX4/Firmware) - is an open source flight control software for drones and other unmanned vehicles
* [KubOS](https://github.com/kubos/kubos) - is an open-source software stack for satellites.
* [mod_vehicle_dynamics_control](https://github.com/TUMFTM/mod_vehicle_dynamics_control) - TUM Roborace Team Software Stack - Path tracking control, velocity control, curvature control and state estimation.
* [Aslan](https://github.com/project-aslan/Aslan) - Open source self-driving software for low speed environments
* [open-source-rover](https://github.com/nasa-jpl/open-source-rover) - A build-it-yourself, 6-wheel rover based on the rovers on Mars from JPL
* [pybotics](https://github.com/nnadeau/pybotics) -  An open-source and peer-reviewed Python toolbox for robot kinematics and calibration
* [makani](https://github.com/google/makani) - contains the working Makani flight simulator, controller (autopilot), visualizer, and command center flight monitoring tools


## Development Environment
### Code and Run
* [Vim-ros](https://github.com/taketwo/vim-ros) - Vim plugin for ROS development
* [Visual Studio Code](https://github.com/Microsoft/vscode) - Code editor for edit-build-debug cycle.
* [atom](https://github.com/atom/atom) - Hackable text editor for the 21st century
* [Teletype](https://github.com/atom/teletype) - Share your workspace with team members and collaborate on code in real time in Atom
* [Sublime](https://www.sublimetext.com/) - A sophisticated text editor for code, markup and prose
* [ade-cli](https://gitlab.com/ApexAI/ade-cli) - The ADE Development Environment (ADE) uses docker and Gitlab to manage environments of per project development tools and optional volume images
* [recipe-wizard](https://github.com/trn84/recipe-wizard) - A Dockerfile generator for running OpenGL (GLX) applications with nvidia-docker2, CUDA, ROS, and Gazebo on a remote headless server system
* [Jupyter ROS](https://github.com/RoboStack/jupyter-ros) - Jupyter widget helpers for ROS, the Robot Operating System
* [ros_rqt_plugin](https://github.com/ros-industrial/ros_qtc_plugin) - The ROS Qt Creator Plug-in for Python
* [xeus-cling](https://github.com/QuantStack/xeus-cling) - Jupyter kernel for the C++ programming language 
* [ROS IDEs](http://wiki.ros.org/IDEs) - This page collects experience and advice on using integrated development environments (IDEs) with ROS.
* [TabNine](https://github.com/zxqfl/TabNine) - The all-language autocompleter
* [kite](https://kite.com/) - Use machine learning to give you 
 useful code completions for Python
* [jedi](https://github.com/davidhalter/jedi) - Autocompletion and static analysis library for python
* [roslibpy](https://github.com/gramaziokohler/roslibpy) - Python ROS Bridge library allows to use Python and IronPython to interact with ROS, the open-source robotic middleware. 
* [pybind11](https://github.com/pybind/pybind11) - Seamless operability between C++11 and Python
* [Sourcetrail](https://github.com/CoatiSoftware/Sourcetrail) - free and open-source cross-platform source explorer
* [rebound](https://github.com/shobrook/rebound) - Command-line tool that instantly fetches Stack Overflow results when an exception is thrown 
* [mybinder](https://mybinder.org/) - open notebooks in an executable environment, making your code immediately reproducible by anyone, anywhere. 
* [ROSOnWindows](https://ms-iot.github.io/ROSOnWindows/) - an experimental release of ROS1 for Windows
* [live-share](https://github.com/MicrosoftDocs/live-share) - Real-time collaborative development from the comfort of your favorite tools 
* [cocalc](https://github.com/sagemathinc/cocalc) - Collaborative Calculation in the Cloud 
* [EasyClangComplete](https://github.com/niosus/EasyClangComplete) - Robust C/C++ code completion for Sublime Text 3
* [vscode-ros](https://github.com/ms-iot/vscode-ros) - Visual Studio Code extension for Robot Operating System (ROS) development
* [awesome-hpp](https://github.com/p-ranav/awesome-hpp) - A curated list of awesome header-only C++ libraries

### Template
* [ROS](https://github.com/leggedrobotics/ros_best_practices/tree/master/ros_package_template) - Template for ROS node standardization in C++ 
* [Launch](https://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) - Templates on how to create launch files for larger projects
* [Bash](https://github.com/ralish/bash-script-template) - A bash scripting template incorporating best practices & several useful functions
* [URDF](https://wiki.ros.org/urdf/Examples) - Examples on how to create Unified Robot Description Format (URDF) for different kinds of robots
* [Python](http://wiki.ros.org/PyStyleGuide) - Style guide to be followed in writing Python code for ROS
* [Docker](https://ade-cli.readthedocs.io/en/latest/create-custom-base-image.html) - The Dockerfile in the minimal-ade project shows a minimal example of how to create a custom base image
* [VS Code ROS2 Workspace Template](https://github.com/athackst/vscode_ros2_workspace) -  Template for using VSCode as an IDE for ROS2 development.

### Build and Deploy
* [qemu-user-static](https://github.com/multiarch/qemu-user-static) - is to enable an execution of different multi-architecture containers by QEMU and binfmt_misc
* [Cross compile ROS 2 on QNX](https://gitlab.apex.ai/snippets/97) -  introduces how to cross compile ROS 2 on QNX
* [bloom](https://github.com/ros-infrastructure/bloom) - A release automation tool which makes releasing catkin packages easier
* [superflore](https://github.com/ros-infrastructure/superflore) - An extended platform release manager for Robot Operating System    
* [catkin_tools](https://github.com/catkin/catkin_tools) - Command line tools for working with catkin
* [industrial_ci](https://github.com/ros-industrial/industrial_ci) - Easy continuous integration repository for ROS repositories
* [ros_gitlab_ci](https://gitlab.com/VictorLamoine/ros_gitlab_ci) - contains helper scripts and instructions on how to use Continuous Integration (CI) for ROS projects hosted on a GitLab instance.
* [gitlab-runner](https://gitlab.com/gitlab-org/gitlab-runner) -  runs tests and sends the results to GitLab
* [colcon-core](https://github.com/colcon/colcon-core) - command line tool to improve the workflow of building, testing and using multiple software packages
* [gitlab-release](https://gitlab.com/alelec/gitlab-release) - Simple python3 script to upload files (from ci) to the current projects release (tag)
* [clang](https://github.com/llvm-mirror/clang) -  This is a compiler front-end for the C family of languages
(C, C++, Objective-C, and Objective-C++) which is built as part of the LLVM
compiler infrastructure project
* [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv) - Bundle python requirements in a catkin package via virtualenv 
* [pyenv](https://github.com/pyenv/pyenv) - Simple Python version management
* [aptly](https://github.com/aptly-dev/aptly) - Debian repository management tool 
* [cross_compile](https://github.com/ros-tooling/cross_compile) - Assets used for ROS2 cross-compilation 
* [docker_images](https://github.com/osrf/docker_images) - Official Docker images maintained by OSRF on ROS(2) and Gazebo
* [robot_upstart](https://github.com/clearpathrobotics/robot_upstart) - presents a suite of scripts to assist with launching background ROS processes on Ubuntu Linux PCs
* [robot_systemd](http://docs.ros.org/kinetic/api/robot_systemd/html/#) - units for managing startup and shutdown of roscore and roslaunch
* [ryo-iso](https://ryo-iso.readthedocs.io/en/latest/) - is a modern ISO builder that streamlines the process of deploying a complete robot operating system from a yaml config file
* [network_autoconfig](http://docs.ros.org/kinetic/api/network_autoconfig/html/) - automatic configuration of ROS networking for most use cases without impacting usage that require manual configuration.
* [rosbuild](https://roscon.ros.org/2016/presentations/ROSCon2016%20Build%20Farm.pdf) - The ROS build farm
* [cros](https://github.com/ros-industrial/cros) - A single thread pure C implementation of the ROS framework


### Unit and Integration Test
* [setup-ros](https://github.com/ros-tooling/setup-ros) - This action sets up a ROS and ROS 2 environment for use in Github actions
* [UnitTesting](https://wiki.ros.org/Quality/Tutorials/UnitTesting) - This page lays out the rationale, best practices, and policies for writing and running unit tests and integration tests for ROS.
* [googletest](https://github.com/google/googletest) - Google's C++ test framework
* [pytest](https://github.com/pytest-dev/pytest/) - The pytest framework makes it easy to write small tests, yet scales to support complex functional testing 
* [doctest](https://github.com/onqtam/doctest) - The fastest feature-rich C++11/14/17/20 single-header testing framework for unit tests and TDD
* [osrf_testing_tools_cpp](https://github.com/osrf/osrf_testing_tools_cpp) - contains testing tools for C++, and is used in OSRF projects.
* [code_coverage](https://github.com/mikeferguson/code_coverage) - ROS package to run coverage testing

### Lint and Format
* [action-ros-lint](https://github.com/ros-tooling/action-ros-lint) - Github action to run linters on ROS 2 packages
* [cppcheck](https://github.com/danmar/cppcheck) - Static analysis of C/C++ code
* [hadolint](https://github.com/hadolint/hadolint) - Dockerfile linter, validate inline bash, written in Haskell 
* [shellcheck](https://github.com/koalaman/shellcheck) - a static analysis tool for shell scripts 
* [catkin_lint](https://github.com/fkie/catkin_lint) - catkin_lint checks package configurations for the catkin build system of ROS.
* [pylint](https://github.com/PyCQA/pylint/) - Pylint is a Python static code analysis tool which looks for programming errors, helps enforcing a coding standard, sniffs for code smells and offers simple refactoring suggestions.
* [black](https://github.com/psf/black) - The uncompromising Python code formatter
* [pydocstyle](https://github.com/PyCQA/pydocstyle) - pydocstyle is a static analysis tool for checking compliance with Python docstring conventions
* [haros](https://github.com/git-afsantos/haros) - H(igh) A(ssurance) ROS - Static analysis of ROS application code. 
* [pydantic](https://github.com/samuelcolvin/pydantic) - Data parsing and validation using Python type hints


### Debugging and Tracing
* [heaptrack](https://github.com/KDE/heaptrack) - traces all memory allocations and annotates these events with stack traces
* [ros2_tracing](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing) - Tracing tools for ROS 2.
* [Linuxperf](http://www.brendangregg.com/linuxperf.html) - Various Linux performance material
* [lptrace](https://github.com/khamidou/lptrace) - It lets you see in real-time what functions a Python program is running
* [pyre-check](https://github.com/facebook/pyre-check) - Performant type-checking for python
* [FlameGraph](https://github.com/brendangregg/FlameGraph) - Visualize profiled code
* [gpuvis](https://github.com/mikesart/gpuvis) - GPU Trace Visualizer
* [sanitizer](https://github.com/google/sanitizers) - AddressSanitizer, ThreadSanitizer, MemorySanitizer
* [cppinsights](https://github.com/andreasfertig/cppinsights) - C++ Insights - See your source code with the eyes of a compiler
* [inspect](https://pymotw.com/2/inspect/) - The inspect module provides functions for learning about live objects, including modules, classes, instances, functions, and methods
* [Roslaunch Nodes in Valgrind or GDB](https://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB) - When debugging roscpp nodes that you are launching with roslaunch, you may wish to launch the node in a debugging program like gdb or valgrind instead.
* [pyperformance](https://github.com/python/pyperformance) - Python Performance Benchmark Suite
* [qira](https://github.com/geohot/qira) - QIRA is a competitor to strace and gdb
* [gdb-frontend](https://github.com/rohanrhu/gdb-frontend) - GDBFrontend is an easy, flexible and extensionable gui debugger.
* [lttng](https://lttng.org/docs/) - is an open source software toolkit which you can use to simultaneously trace the Linux kernel, user applications, and user libraries.
* [ros2-performance](https://github.com/irobot-ros/ros2-performance) - allows to easily create arbitrary ROS2 systems and then measures their performance
* [bcc](https://github.com/iovisor/bcc) - Tools for BPF-based Linux IO analysis, networking, monitoring, and more
* [tracy](https://github.com/wolfpld/tracy) - A real time, nanosecond resolution, remote telemetry frame profiler for games and other applications.
* [bpftrace](https://github.com/iovisor/bpftrace) - High-level tracing language for Linux eBPF
* [pudb](https://github.com/inducer/pudb) - Full-screen console debugger for Python
* [backward-cpp](https://github.com/bombela/backward-cpp) - A beautiful stack trace pretty printer for C++
* [gdb-dashboard](https://github.com/cyrus-and/gdb-dashboard) - GDB dashboard is a standalone .gdbinit file written using the Python API that enables a modular interface showing relevant information about the program being debugged.
* [hotspot](https://github.com/KDAB/hotspot) - The Linux perf GUI for performance analysis.
* [memory_profiler](https://github.com/pythonprofilers/memory_profiler) - a python module for monitoring memory consumption of a process as well as line-by-line analysis of memory consumption for python programs. 
* [ros1_fuzzer](https://github.com/aliasrobotics/ros1_fuzzer)  - This fuzzer aims to help developers and researchers to find bugs and vulnerabilities in ROS nodes by performing fuzz tests over topics that the target nodes process.
* [vscode-debug-visualizer](https://github.com/hediet/vscode-debug-visualizer) - An extension for VS Code that visualizes data during debugging.
* [action-tmate](https://github.com/mxschmitt/action-tmate) - Debug your GitHub Actions via SSH by using tmate to get access to the runner system itself.


### Version Control
* [git-fuzzy](https://github.com/bigH/git-fuzzy) - A CLI interface to git that relies heavily on fzf
* [meld](https://github.com/GNOME/meld) - Meld is a visual diff and merge tool that helps you compare files, directories, and version controlled projects
* [tig](https://github.com/jonas/tig) - Text-mode interface for git 
* [gitg](https://github.com/GNOME/gitg) - is a graphical user interface for git
* [git-cola](https://github.com/git-cola/git-cola) - The highly caffeinated Git GUI
* [python-gitlab](https://github.com/python-gitlab/python-gitlab) - is a Python package providing access to the GitLab server API.
* [bfg-repo-cleaner](https://github.com/rtyley/bfg-repo-cleaner) - Removes large or troublesome blobs like git-filter-branch does, but faster.
* [nbdime](https://github.com/jupyter/nbdime) - Tools for diffing and merging of Jupyter notebooks. 
* [semantic-release](https://github.com/semantic-release/semantic-release) - Fully automated version management and package publishing
* [go-semrel-gitab](https://gitlab.com/juhani/go-semrel-gitlab) - Automate version management for Gitlab
* [Git-repo](https://gerrit.googlesource.com/git-repo/) - Git-Repo helps manage many Git repositories, does the uploads to revision control systems, and automates parts of the development workflow
* [dive](https://github.com/wagoodman/dive) - A tool for exploring each layer in a docker image
* [dvc](https://github.com/iterative/dvc) - Management and versioning of datasets and machine learning models
* [learnGitBranching](https://github.com/pcottle/learnGitBranching) - is a git repository visualizer, sandbox, and a series of educational tutorials and challenges
* [gitfs](https://github.com/Presslabs/gitfs) - You can mount a remote repository's branch locally, and any subsequent changes made to the files will be automatically committed to the remote.
* [git-secret](https://github.com/sobolevn/git-secret) - encrypts files with permitted users' public keys, allowing users you trust to access encrypted data using pgp and their secret keys
* [git-sweep](https://github.com/arc90/git-sweep) - A command-line tool that helps you clean up Git branches that have been merged into master.
* [lazygit](https://github.com/jesseduffield/lazygit) - A simple terminal UI for git commands, written in Go with the gocui library.
* [glab](https://github.com/profclems/glab) - An open-source GitLab command line tool


## Simulation
* [Drake](https://github.com/RobotLocomotion/drake) - Drake aims to simulate even very complex dynamics of robots
* [Webots](https://github.com/cyberbotics/webots) - Webots is an open source robot simulator compatible (among others) with [ROS](http://wiki.ros.org/webots_ros) and [ROS2](http://wiki.ros.org/webots_ros2).
* [lgsv](https://github.com/lgsvl/simulator) - LG Electronics America R&D Center has developed an HDRP Unity-based multi-robot simulator for autonomous vehicle developers.
* [carla](https://github.com/carla-simulator/carla) - Open-source simulator for autonomous driving research
* [awesome-CARLA](https://github.com/Amin-Tgz/awesome-CARLA) - A curated list of awesome CARLA tutorials, blogs, and related projects.
* [ros-bridge](https://github.com/carla-simulator/ros-bridge) - ROS bridge for CARLA Simulator
* [scenario_runner](https://github.com/carla-simulator/scenario_runner) - Traffic scenario definition and execution engine 
* [deepdive](https://github.com/deepdrive/deepdrive) - End-to-end simulation for self-driving cars 
* [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) - Gazebo/ROS packages for underwater robotics simulation
* [AirSim](https://github.com/microsoft/AirSim) - Open source simulator for autonomous vehicles built on Unreal Engine 
* [self-driving-car-sim](https://github.com/udacity/self-driving-car-sim) - A self-driving car simulator built with Unity
* [ROSIntegration](https://github.com/code-iai/ROSIntegration) - Unreal Engine Plugin to enable ROS Support
* [gym-gazebo](https://github.com/erlerobot/gym-gazebo) - An OpenAI gym extension for using Gazebo known as gym-gazebo
* [highway-env](https://github.com/eleurent/highway-env)	- A collection of environments for autonomous driving and tactical decision-making tasks
* [VREP Interface](http://www.coppeliarobotics.com/helpFiles/en/rosInterf.htm) - ROS Bridge for the VREP simulator 
* [car_demo](https://github.com/osrf/car_demo) - This is a simulation of a Prius in gazebo 9 with sensor data being published using ROS kinetic.
* [sumo](https://github.com/eclipse/sumo) - Eclipse SUMO is an open source, highly portable, microscopic and continuous road traffic simulation package designed to handle large road networks
* [open-simulation-interface](https://github.com/OpenSimulationInterface/open-simulation-interface) - A generic interface for the environmental perception of automated driving functions in virtual scenarios.
* [ESIM](https://github.com/uzh-rpg/rpg_esim/) - an Open Event Camera Simulator
* [Menge](https://github.com/MengeCrowdSim/Menge) - Crowd Simulation Framework 
* [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) - Pedestrian simulator powered by the social force model for Gazebo
* [opencrg](http://www.opencrg.org/download.html) -  open file formats and open source tools for the detailed description, creation and evaluation of road surfaces
* [esmini](https://github.com/esmini/esmini) -  is a basic OpenSCENARIO player
* [OpenSceneGraph](https://github.com/openscenegraph/OpenSceneGraph) - is an open source high performance 3D graphics toolkit, used by application developers in fields such as visual simulation, games, virtual reality, scientific visualization and modelling
* [morse](https://github.com/morse-simulator) - is an academic robotic simulator, based on the Blender Game Engine and the Bullet Physics engine.
* [ROSIntegrationVision](https://github.com/code-iai/ROSIntegrationVision) - Support for ROS-enabled RGBD data acquisition in Unreal Engine Projects
* [fetch_gazebo](https://github.com/fetchrobotics/fetch_gazebo) - contains the Gazebo simulation for Fetch Robotics Fetch and Freight Research Edition Robots.
* [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) - provides some multirotor models
* [flow](https://github.com/flow-project/flow) - is a computational framework for deep RL and control experiments for traffic microsimulation.
* [gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) - GNSS + inertial navigation, sensor fusion simulator. Motion trajectory generator, sensor models, and navigation
* [Ignition Robotics](https://ignitionrobotics.org) -  Test control strategies in safety, and take advantage of simulation in continuous integration tests.
* [simulation assets for the SubT](https://subtchallenge.world/openrobotics/fuel/collections/SubT%20Tech%20Repo) - This collection contains simulation assets for the SubT Challenge Virtual Competition in Gazebo
* [gazebo_ros_motors](https://github.com/nilseuropa/gazebo_ros_motors) - contains currently two motor plugins for Gazebo, one with an ideal speed controller and one without a controller that models a DC motor
* [map2gazebo](https://github.com/shilohc/map2gazebo) - ROS package for creating Gazebo environments from 2D maps
* [sim_vehicle_dynamics](https://github.com/TUMFTM/sim_vehicle_dynamics) - Vehicle Dynamics Simulation Software of TUM Roborace Team
* [gym-carla](https://github.com/cjy1992/gym-carla) - An OpenAI gym wrapper for CARLA simulator
* [simbody](https://github.com/simbody/simbody) - High-performance C++ multibody dynamics/physics library for simulating articulated biomechanical and mechanical systems like vehicles, robots, and the human skeleton.
* [gazebo_models](https://github.com/osrf/gazebo_models) - This repository holds the Gazebo model database.
* [pylot](https://github.com/erdos-project/pylot) - Autonomous driving platform running on the CARLA simulator
* [flightmare](https://github.com/uzh-rpg/flightmare) - Flightmare is composed of two main components: a configurable rendering engine built on Unity and a flexible physics engine for dynamics simulation
* [champ](https://github.com/chvmp/champ) - ROS Packages for CHAMP Quadruped Controller
* [rex-gym](https://github.com/nicrusso7/rex-gym) - OpenAI Gym environments for an open-source quadruped robot (SpotMicro)
* [Trick](https://github.com/nasa/Trick) - developed at the NASA Johnson Space Center, is a powerful simulation development framework that enables users to build applications for all phases of space vehicle development
* [usv_sim_lsa](https://github.com/disaster-robotics-proalertas/usv_sim_lsa) - Unmanned Surface Vehicle simulation on Gazebo with water current and winds
* [42](https://github.com/ericstoneking/42) - Simulation for spacecraft attitude control system analysis and design


## Electronics and Mechanics
* [HRIM](https://github.com/AcutronicRobotics/HRIM) - An information model for robot hardware
* [URDF](https://github.com/ros/urdf) - Repository for Unified Robot Description Format (URDF) parsing code
* [phobos](https://github.com/dfki-ric/phobos) - An add-on for Blender allowing to create URDF, SDF and SMURF robot models in a WYSIWYG environment.
* [urdf-viz](https://github.com/OTL/urdf-viz) - Visualize URDF/XACRO file, URDF Viewer works on Windows/MacOS/Linux
* [solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter) - SolidWorks to URDF Exporter
* [FreeCAD](https://github.com/FreeCAD/FreeCAD) - Your own 3D parametric modeler
* [kicad](http://www.kicad-pcb.org/) - A Cross Platform and Open Source Electronics Design Automation Suite
* [PcbDraw](https://github.com/yaqwsx/PcbDraw) - Convert your KiCAD board into a nice looking 2D drawing suitable for pinout diagrams
* [kicad-3rd-party-tools](https://github.com/xesscorp/kicad-3rd-party-tools) - Tools made by others to augment the KiCad PCB EDA suite.
* [PandaPower](http://www.pandapower.org) - An easy to use open source tool for power system modeling, analysis and optimization with a high degree of automation.
* [LibrePCB](https://github.com/LibrePCB/LibrePCB) - A powerful, innovative and intuitive EDA tool for everyone
* [openscad](https://github.com/openscad/openscad) -  is software for creating solid 3D CAD models
* [ngspice](http://ngspice.sourceforge.net/) - is the open source spice simulator for electric and electronic circuits.
* [GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr) - GNSS-SDR provides interfaces for a wide range of radio frequency front-ends and raw sample file formats, generates processing outputs in standard formats
* [riscv](https://riscv.org) - The Free and Open RISC Instruction Set Architecture
* [urdfpy](https://github.com/mmatl/urdfpy) - is a simple and easy-to-use library for loading, manipulating, saving, and visualizing URDF files.
* [FMPy](https://github.com/CATIA-Systems/FMPy) - Simulate Functional Mockup Units (FMUs) in Python
* [FMIKit-Simulink](https://github.com/CATIA-Systems/FMIKit-Simulink) - Import and export Functional Mock-up Units with Simulink
* [oemof-solph](https://github.com/oemof/oemof-solph) - A modular open source framework to model energy supply systems
* [NASA-3D-Resources](https://github.com/nasa/NASA-3D-Resources) - Here you'll find a growing collection of 3D models, textures, and images from inside NASA.
* [SUAVE](https://github.com/suavecode/SUAVE) - An Aircraft Design Toolbox
* [opem](https://github.com/ECSIM/opem) - The Open-Source PEMFC Simulation Tool (OPEM) is a modeling tool for evaluating the performance of proton exchange membrane fuel cells
* [pvlib-python](https://github.com/pvlib/pvlib-python) - is a community supported tool that provides a set of functions and classes for simulating the performance of photovoltaic energy systems
* [WireViz](https://github.com/formatc1702/WireViz) - is a tool for easily documenting cables, wiring harnesses and connector pinouts.
* [Horizon](https://github.com/horizon-eda/horizon) - EDA is an Electronic Design Automation package supporting an integrated end-to-end workflow for printed circuit board design including parts management and schematic entry.
* [tigl](https://github.com/DLR-SC/tigl) - The TiGL Geometry Library can be used for the computation and processing of aircraft geometries stored inside CPACS files
* [foxBMS](https://github.com/foxBMS/foxbms) - is a free, open and flexible development environment to design battery management systems
* [cadCAD](https://github.com/cadCAD-org/cadCAD) - is a Python package that assists in the processes of designing, testing and validating complex systems through simulation, with support for Monte Carlo methods, A/B testing and parameter sweeping.
* [OpenMDAO](https://github.com/OpenMDAO/OpenMDAO) - An open-source framework for efficient multidisciplinary optimization.
* [SUAVE](https://github.com/suavecode/SUAVE) - purpose is to credibly produce conceptual-level design conclusions for future aircraft incorporating advanced technologies.
* [ODrive](https://github.com/madcowswe/ODrive) - aim is to make it possible to use inexpensive brushless motors in high performance robotics projects

## Sensor Processing
### Calibration and Transformation
* [tf2](http://wiki.ros.org/tf2) - transform library, which lets the user keep track of multiple coordinate frames over time
* [lidar_align](https://github.com/ethz-asl/lidar_align) - A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor
* [kalibr](https://github.com/ethz-asl/kalibr) - The Kalibr visual-inertial calibration toolbox
* [Calibnet](https://github.com/epiception/CalibNet) - Self-Supervised Extrinsic Calibration using 3D Spatial Transformer Networks
* [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration) - ROS package to find a rigid-body transformation between a LiDAR and a camera
* [ILCC](https://github.com/mfxox/ILCC) - Reflectance Intensity Assisted Automatic and Accurate Extrinsic Calibration of 3D LiDAR
* [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) - Simple, straighforward ROS library for hand-eye calibration
* [imu_utils](https://github.com/gaowenliang/imu_utils) - A ROS package tool to analyze the IMU performance
* [kalibr_allan](https://github.com/rpng/kalibr_allan) - IMU Allan standard deviation charts for use with Kalibr and inertial kalman filters
* [pyquaternion](https://github.com/KieranWynn/pyquaternion) - is a full-featured Python module for representing and using quaternions
* [robot_calibration](https://github.com/mikeferguson/robot_calibration/) - This package offers calibration of a number of parameters of a robot, such as: 3D Camera intrinsics, extrinsics Joint angle offsets and robot frame offsets
* [multi_sensor_calibration](https://github.com/tudelft-iv/multi_sensor_calibration/) - contains a calibration tool to calibrate a sensor setup consisting of lidars, radars and cameras
* [LiDARTag](https://github.com/UMich-BipedLab/LiDARTag) - A Real-Time Fiducial Tag using Point Clouds Lidar Data
* [multicam_calibration](https://github.com/KumarRobotics/multicam_calibration) - extrinsic and intrinsic calbration of cameras
* [ikpy](https://github.com/Phylliade/ikpy) - An Inverse Kinematics library aiming performance and modularity
* [livox_camera_lidar_calibration](https://github.com/Livox-SDK/livox_camera_lidar_calibration) - Calibrate the extrinsic parameters between Livox LiDAR and camera
* [lidar_camera_calibration](https://github.com/heethesh/lidar_camera_calibration) - Camera LiDAR Calibration using ROS, OpenCV, and PCL


### Perception Pipeline
* [SARosPerceptionKitti](https://github.com/appinho/SARosPerceptionKitti) - ROS package for the Perception (Sensor Processing, Detection, Tracking and Evaluation) of the KITTI Vision Benchmark Suite
* [multiple-object-tracking-lidar](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar) - C++ implementation to Detect, track and classify multiple objects using LIDAR scans or point cloud
* [cadrl_ros](https://github.com/mfe7/cadrl_ros) - ROS package for dynamic obstacle avoidance for ground robots trained with deep RL
* [AugmentedAutoencoder](https://github.com/DLR-RM/AugmentedAutoencoder) - RGB-based pipeline for object detection and 6D pose estimation
* [jsk_recognition](https://github.com/jsk-ros-pkg/jsk_recognition) - is a stack for the perception packages which are used in JSK lab.
* [GibsonEnv](https://github.com/StanfordVL/GibsonEnv) - Gibson Environments: Real-World Perception for Embodied Agents
* [morefusion](https://github.com/wkentaro/morefusion) - Multi-object Reasoning for 6D Pose Estimation from Volumetric Fusion

### Machine Learning
* [DLIB](https://github.com/davisking/dlib) - A toolkit for making real world machine learning and data analysis applications in C++
* [fastai](https://github.com/fastai/fastai) - The fastai library simplifies training fast and accurate neural nets using modern best practices.
* [tpot](https://github.com/EpistasisLab/tpot) - A Python Automated Machine Learning tool that optimizes machine learning pipelines using genetic programming
* [deap](https://github.com/DEAP/deap) - Distributed Evolutionary Algorithms in Python
* [gym](https://github.com/openai/gym) - A toolkit for developing and comparing reinforcement learning algorithms.
* [tensorflow_ros_cpp](https://github.com/tradr-project/tensorflow_ros_cpp) - A ROS package that allows to do Tensorflow inference in C++ without the need to compile TF yourself.
* [Tensorflow Federated](https://github.com/tensorflow/federated) - TensorFlow Federated (TFF) is an open-source framework for machine learning and other computations on decentralized data
* [finn](https://github.com/Xilinx/finn) - Fast, Scalable Quantized Neural Network Inference on FPGAs
* [neuropod](https://github.com/uber/neuropod) - Neuropod is a library that provides a uniform interface to run deep learning models from multiple frameworks in C++ and Python
* [leela-zero](https://github.com/leela-zero/leela-zero) - This is a fairly faithful reimplementation of the system described in the Alpha Go Zero paper "Mastering the Game of Go without Human Knowledge"
* [trax](https://github.com/google/trax) - Trax is a library for deep learning that focuses on sequence models and reinforcement learning
* [mlflow](https://github.com/mlflow/mlflow) - is a platform to streamline machine learning development, including tracking experiments, packaging code into reproducible runs, and sharing and deploying models
* [Netron](https://github.com/lutzroeder/Netron) - Visualizer for neural network, deep learning and machine learning models
* [MNN](https://github.com/alibaba/MNN) - MNN is a blazing fast, lightweight deep learning framework, battle-tested by business-critical use cases in Alibaba
* [tensorforce](https://github.com/tensorforce/tensorforce) - Tensorforce is an open-source deep reinforcement learning framework, with an emphasis on modularized flexible library design and straightforward usability for applications in research and practice
* [dopamine](https://github.com/google/dopamine) - Dopamine is a research framework for fast prototyping of reinforcement learning algorithms.
* [catalyst](https://github.com/catalyst-team/catalyst) - was developed with a focus on reproducibility, fast experimentation and code/ideas reusing
* [ray](https://github.com/ray-project/ray) - A fast and simple framework for building and running distributed applications
* [tf-agents](https://github.com/tensorflow/agents) - A reliable, scalable and easy to use TensorFlow library for Contextual Bandits and Reinforcement Learning.
* [ReAgent](https://github.com/facebookresearch/ReAgent) - is an open source end-to-end platform for applied reinforcement learning (RL) developed and used at Facebook.
* [Awesome-Mobile-Machine-Learning](https://github.com/fritzlabs/Awesome-Mobile-Machine-Learning - A curated list of awesome mobile machine learning resources for iOS, Android, and edge devices.
* [cnn-explainer](https://github.com/poloclub/cnn-explainer) - Learning Convolutional Neural Networks with Interactive Visualization.


### Parallel Processing
* [dask](https://github.com/dask/dask) - Parallel computing with task scheduling for Python
* [cupy](https://github.com/cupy/cupy) - NumPy-like API accelerated with CUDA
* [thrust](https://github.com/thrust/thrust) - Thrust is a C++ parallel programming library which resembles the C++ Standard Library.
* [ArrayFire](https://github.com/arrayfire/arrayfire) - ArrayFire: a general purpose GPU library.
* [OpenMP](https://www.openmp.org/) - OpenMP is an application programming interface that supports multi-platform shared memory multiprocessing programming in C, C++, and Fortra
* [vexcl](https://github.com/ddemidov/vexcl) - VexCL is a C++ vector expression template library for OpenCL/CUDA/OpenMP
* [PYNQ](https://github.com/Xilinx/PYNQ) - is an open-source project from Xilinx that makes it easy to design embedded systems with Zynq All Programmable Systems on Chips
* [numba](https://github.com/numba/numba) - NumPy aware dynamic Python compiler using LLVM
* [TensorRT](https://github.com/NVIDIA/TensorRT) - TensorRT is a C++ library for high performance inference on NVIDIA GPUs and deep learning accelerators
* [libcudacxx](https://github.com/NVIDIA/libcudacxx) - provides a heterogeneous implementation of the C++ Standard Library that can be used in and between CPU and GPU code.


### Image Processing
* [CV-pretrained-model](https://github.com/balavenkatesh3322/CV-pretrained-model) - A collection of computer vision pre-trained models
* [image_pipeline](https://github.com/ros-perception/image_pipeline) - fills the gap between getting raw images from a camera driver and higher-level vision processing
* [gstreamer](https://gstreamer.freedesktop.org/) - is a pipeline-based multimedia framework that links together a wide variety of media processing systems to complete complex workflows
* [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit) -  provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference
* [vision_visp](https://github.com/lagadic/vision_visp) - Wraps the ViSP moving edge tracker provided by the ViSP visual servoing library into a ROS package
* [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) - A ROS wrapper of the AprilTag 3 visual fiducial detector
* [deep_object_pose](https://github.com/NVlabs/Deep_Object_Pose) - Deep Object Pose Estimation
* [DetectAndTrack](https://github.com/facebookresearch/DetectAndTrack) - Detect-and-Track: Efficient Pose
* [SfMLearner](https://github.com/tinghuiz/SfMLearner) - An unsupervised learning framework for depth and ego-motion estimation
* [imgaug](https://github.com/aleju/imgaug) - Image augmentation for machine learning experiments
* [vision_opencv](https://github.com/ros-perception/vision_opencv) - Packages for interfacing ROS with OpenCV, a library of programming functions for real time computer vision.
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros) - YOLO ROS: Real-Time Object Detection for ROS
* [ros_ncnn](https://github.com/nilseuropa/ros_ncnn) - YOLACT / YOLO *( among other things )* on NCNN inference engine for ROS
* [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation) - Deep Pose Estimation implemented using Tensorflow with Custom Architectures for fast inference.
* [find-object](https://github.com/introlab/find-object) - Simple Qt interface to try OpenCV implementations of SIFT, SURF, FAST, BRIEF and other feature detectors and descriptors
* [yolact](https://github.com/dbolya/yolact) - A simple, fully convolutional model for real-time instance segmentation.
* [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) - Real-Time 3D Semantic Reconstruction from 2D data
* [detectron2](https://github.com/facebookresearch/detectron2) - is a next-generation research platform for object detection and segmentation.
* [OpenVX](https://www.khronos.org/openvx/) -  enables performance and power-optimized computer vision processing, especially important in embedded and real-time use cases
* [3d-vehicle-tracking](https://github.com/ucbdrive/3d-vehicle-tracking) - Official implementation of Joint Monocular 3D Vehicle Detection and Tracking
* [pysot](https://github.com/STVIR/pysot) - The goal of PySOT is to provide a high-quality, high-performance codebase for visual tracking research
* [semantic_slam](https://github.com/floatlazer/semantic_slam) - Real time semantic slam in ROS with a hand held RGB-D camera
* [kitti_scan_unfolding](https://github.com/ltriess/kitti_scan_unfolding) - We propose KITTI scan unfolding in our paper Scan-based Semantic Segmentation of LiDAR Point Clouds: An Experimental Study. 
* [packnet-sfm](https://github.com/TRI-ML/packnet-sfm) - Official PyTorch implementation of self-supervised monocular depth estimation methods invented by the ML Team at Toyota Research Institute (TRI)
* [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT) - This work proposes a simple yet accurate real-time baseline 3D multi-object tracking system
* [lanenet-lane-detection](https://github.com/MaybeShewill-CV/lanenet-lane-detection) - Unofficial implemention of lanenet model for real time lane detection using deep neural network model
* [OpenDroneMap](https://github.com/OpenDroneMap/ODM) - An open source command line toolkit for processing aerial drone imagery
* [monoloco](https://github.com/vita-epfl/monoloco) - Official implementation of "MonoLoco: Monocular 3D Pedestrian Localization and Uncertainty Estimation" in PyTorch 
* [Poly-YOLO](https://gitlab.com/irafm-ai/poly-yolo) - builds on the original ideas of YOLOv3 and removes two of its weaknesses: a large amount of rewritten labels and inefficient distribution of anchors
* [satellite-image-deep-learning](https://github.com/robmarkcole/satellite-image-deep-learning) - Resources for deep learning with satellite & aerial imagery 
* [robosat](https://github.com/mapbox/robosat) - Semantic segmentation on aerial and satellite imagery
* [big_transfer](https://github.com/google-research/big_transfer) - Model for General Visual Representation Learning created by Google Research
* [LEDNet](https://github.com/xiaoyufenfei/LEDNet) - A Lightweight Encoder-Decoder Network for Real-time Semantic Segmentation
* [TorchSeg](https://github.com/ycszen/TorchSeg) - This project aims at providing a fast, modular reference implementation for semantic segmentation models using PyTorch. 
* [simpledet](https://github.com/tusimple/simpledet) - A Simple and Versatile Framework for Object Detection and Instance Recognition
* [meshroom](https://github.com/alicevision/meshroom) - Meshroom is a free, open-source 3D Reconstruction Software based on the AliceVision Photogrammetric Computer Vision framework.
* [EasyOCR](https://github.com/JaidedAI/EasyOCR) - Ready-to-use Optical character recognition (OCR) with 40+ languages supported including Chinese, Japanese, Korean and Thai
* [pytracking](https://github.com/visionml/pytracking) - A general python framework for visual object tracking and video object segmentation, based on PyTorch.
* [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) - Deep learning inference nodes for ROS with support for NVIDIA Jetson TX1/TX2/Xavier and TensorRT
* [hyperpose](https://github.com/tensorlayer/hyperpose) - HyperPose: A Flexible Library for Real-time Human Pose Estimation
* [fawkes](https://github.com/Shawn-Shan/fawkes) - privacy preserving tool against facial recognition systems.
* [anonymizer](https://github.com/understand-ai/anonymizer) - An anonymizer to obfuscate faces and license plates.
* [opendatacam](https://github.com/opendatacam/opendatacam) - only saves surveyed meta-data, in particular the path an object moved or number of counted objects at a certain point.
* [Cam2BEV](https://github.com/ika-rwth-aachen/Cam2BEV) - TensorFlow Implementation for Computing a Semantically Segmented Bird's Eye View (BEV) Image Given the Images of Multiple Vehicle-Mounted Cameras.
* [ODM](https://github.com/OpenDroneMap/ODM) - A command line toolkit to generate maps, point clouds, 3D models and DEMs from drone, balloon or kite images.
* [lanenet-lane-detection](https://github.com/MaybeShewill-CV/lanenet-lane-detection) - Unofficial implemention of lanenet model for real time lane detection using deep neural network model
* [satpy](https://github.com/pytroll/satpy) - Python package for earth-observing satellite data processing
* [flownet2-pytorch](https://github.com/NVIDIA/flownet2-pytorch) - Pytorch implementation of FlowNet 2.0: Evolution of Optical Flow Estimation with Deep Networks
* [Simd](https://github.com/ermig1979/Simd) - C++ image processing and machine learning library with using of SIMD: SSE, SSE2, SSE3, SSSE3, SSE4.1, SSE4.2, AVX, AVX2, AVX-512, VMX(Altivec) and VSX(Power7), NEON for ARM.


### Radar Processing
* [pyroSAR](https://github.com/johntruckenbrodt/pyroSAR) - Framework for large-scale SAR satellite data processing
* [CameraRadarFusionNet](https://github.com/TUMFTM/CameraRadarFusionNet) - TUM Roborace Team Software Stack - Path tracking control, velocity control, curvature control and state estimation.


### Lidar and Point Cloud Processing
* [cilantro](https://github.com/kzampog/cilantro) - A lean C++ library for working with point cloud data
* [open3d](https://github.com/intel-isl/Open3D) - Open3D: A Modern Library for 3D Data Processing
* [SqueezeSeg](https://github.com/BichenWuUCB/SqueezeSeg) - Implementation of SqueezeSeg, convolutional neural networks for LiDAR point clout segmentation
* [point_cloud_io](https://github.com/ANYbotics/point_cloud_io) - ROS nodes to read and write point clouds from and to files (e.g. ply, vtk).
* [python-pcl](https://github.com/strawlab/python-pcl) - Python bindings to the pointcloud library
* [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) - An "Iterative Closest Point" library for 2-D/3-D mapping in Robotics
* [depth_clustering](https://github.com/PRBonn/depth_clustering) - Fast and robust clustering of point clouds generated with a Velodyne sensor. 
* [lidar-bonnetal](https://github.com/PRBonn/lidar-bonnetal) - Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving
* [CSF](https://github.com/jianboqi/CSF) - LiDAR point cloud ground filtering / segmentation (bare earth extraction) method based on cloth simulation
* [robot_body_filter](https://github.com/peci1/robot_body_filter) - A highly configurable LaserScan/PointCloud2 filter that allows to dynamically remove the 3D body of the robot from the measurements.
* [grid_map](https://github.com/ANYbotics/grid_map) - Universal grid map library for mobile robotic mapping
* [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) - Robot-centric elevation mapping for rough terrain navigation
* [rangenet_lib](https://github.com/PRBonn/rangenet_lib) - contains simple usage explanations of how the RangeNet++ inference works with the TensorRT and C++ interface.
* [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) - Converts a 3D Point Cloud into a 2D laser scan.
* [octomap](https://github.com/OctoMap/octomap) - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
* [pptk](https://github.com/heremaps/pptk) - Point Processing Toolkit from HEREMaps
* [gpu-voxels](https://www.gpu-voxels.org/) - GPU-Voxels is a CUDA based library which allows high resolution volumetric collision detection between animated 3D models and live pointclouds from 3D sensors of all kinds.
* [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) - A new voxel layer leveraging modern 3D graphics tools to modernize navigation environmental representations
* [LAStools](https://github.com/LAStools/LAStools) - award-winning software for efficient LiDAR processing
* [PCDet](https://github.com/sshaoshuai/PCDet) - is a general PyTorch-based codebase for 3D object detection from point cloud. 
* [PDAL](https://github.com/PDAL/PDAL) - is a C++ BSD library for translating and manipulating point cloud data
* [PotreeConverter](https://github.com/potree/PotreeConverter) - Builds a potree octree from las, laz, binary ply, xyz or ptx files.
* [fast_gicp](https://github.com/SMRT-AIST/fast_gicp) - A collection of GICP-based fast point cloud registration algorithms
* [ndt_omp](https://github.com/koide3/ndt_omp) - Multi-threaded and SSE friendly NDT algorithm
* [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) - voxel layer leveraging modern 3D graphics tools to modernize navigation environmental representations
* [laser_line_extraction](https://github.com/kam3k/laser_line_extraction) - A ROS packages that extracts line segments from LaserScan messages.
* [Go-ICP](https://github.com/yangjiaolong/Go-ICP) - Implementation of the Go-ICP algorithm for globally optimal 3D pointset registration
* [PointCNN](https://github.com/yangyanli/PointCNN) - is a simple and general framework for feature learning from point clouds
* [segmenters_lib](https://github.com/LidarPerception/segmenters_lib) - The LiDAR segmenters library, for segmentation-based detection
* [MotionNet](https://github.com/pxiangwu/MotionNet) - Joint Perception and Motion Prediction for Autonomous Driving Based on Bird's Eye View Maps
* [PolarSeg](https://github.com/edwardzhou130/PolarSeg) - An Improved Grid Representation for Online LiDAR Point Clouds Semantic Segmentation
* [traversability_mapping](https://github.com/TixiaoShan/traversability_mapping) - takes in point cloud from a Velodyne VLP-16 Lidar and outputs a traversability map for autonomous navigation in real-time
* [lidar_super_resolution](https://github.com/RobustFieldAutonomyLab/lidar_super_resolution) - Simulation-based Lidar Super-resolution for Ground Vehicles
* [cupoch](https://github.com/neka-nat/cupoch) - Cupoch is a library that implements rapid 3D data processing and robotics computation using CUDA
* [linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation) - Implementation of the ground segmentation algorithm
* [draco](https://github.com/google/draco) - Draco is a library for compressing and decompressing 3D geometric meshes and point clouds
* [votenet](https://github.com/facebookresearch/votenet) - Deep Hough Voting for 3D Object Detection in Point Clouds 
* [lidar_undistortion](https://github.com/ethz-asl/lidar_undistortion) - provides lidar motion undistortion based on an external 6DoF pose estimation input.
* [superpoint_graph](https://github.com/loicland/superpoint_graph) - Large-scale Point Cloud Semantic Segmentation with Superpoint Graphs
* [RandLA-Net](https://github.com/QingyongHu/RandLA-Net) - Efficient Semantic Segmentation of Large-Scale Point Clouds
* [Det3D](https://github.com/poodarchu/Det3D) - Det3D is the first 3D Object Detection toolbox which provides off the box implementations of many 3D object detection algorithms such as PointPillars, SECOND, PIXOR
* [OverlapNet](https://github.com/PRBonn/OverlapNet) - OverlapNet is modified Siamese Network that predicts the overlap and relative yaw angle of a pair of range images generated by 3D LiDAR scans
* [mp2p_icp](https://github.com/MOLAorg/mp2p_icp) - A repertory of multi primitive-to-primitive (MP2P) ICP algorithms in C++
* [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) - OpenPCDet Toolbox for LiDAR-based 3D Object Detection
* [torch-points3d](https://github.com/nicolas-chaulet/torch-points3d) - Pytorch framework for doing deep learning on point clouds.
* [PolyFit](https://github.com/LiangliangNan/PolyFit) - Polygonal Surface Reconstruction from Point Clouds
* [mmdetection3d](https://github.com/open-mmlab/mmdetection3d) - OpenMMLab's next-generation platform for general 3D object detection
* [gpd](https://github.com/atenpas/gpd) - takes a point cloud as input and produces pose estimates of viable grasps as output
* [SalsaNext](https://github.com/TiagoCortinhal/SalsaNext) - Uncertainty-aware Semantic Segmentation of LiDAR Point Clouds for Autonomous Driving
* [Super-Fast-Accurate-3D-Object-Detection](https://github.com/maudzung/Super-Fast-Accurate-3D-Object-Detection) - Super Fast and Accurate 3D Object Detection based on 3D LiDAR Point Clouds (The PyTorch implementation)

## Localization and State Estimation
* [evo](https://github.com/MichaelGrupp/evo) - Python package for the evaluation of odometry and SLAM
* [robot_localization](https://github.com/cra-ros-pkg/robot_localization) - is a package of nonlinear state estimation nodes
* [fuse](https://github.com/locusrobotics/fuse) - General architecture for performing sensor fusion live on a 
robot.
* [GeographicLib](https://github.com/Sciumo/GeographicLib) - A C++ library for geographic projections.
* [ntripbrowser](https://github.com/emlid/ntripbrowser) - A Python API for browsing NTRIP (Networked Transport of RTCM via Internet Protocol).
* [imu_tools](https://github.com/ccny-ros-pkg/imu_tools) - IMU-related filters and visualizers.
* [RTKLIB](https://github.com/rtklibexplorer/RTKLIB) - A version of RTKLIB optimized for single and dual frequency low cost GPS receivers, especially u-blox receivers
* [gLAB](https://gage.upc.edu/gLAB/) - performs precise modeling of GNSS observables (pseudorange and carrier phase) at the centimetre level, allowing standalone GPS positioning, PPP, SBAS and DGNSS
* [ai-imu-dr](https://github.com/mbrossar/ai-imu-dr) - contains the code of our novel accurate method for dead reckoning of wheeled vehicles based only on an IMU
* [Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) - Kalman Filter book using Jupyter Notebook
* [mcl_3dl](https://github.com/at-wat/mcl_3dl) - A ROS node to perform a probabilistic 3-D/6-DOF localization system for mobile robots with 3-D LIDAR(s)
* [se2lam](https://github.com/izhengfan/se2lam) - On-SE(2) Localization and Mapping for Ground Vehicles by Fusing Odometry and Vision
* [mmWave-localization-learning](https://github.com/gante/mmWave-localization-learning) - ML-based positioning method from mmWave transmissions - with high accuracy and energy efficiency 
* [python-sgp4](https://github.com/brandon-rhodes/python-sgp4) -  computes the position and velocity of an earth-orbiting satellite, given the satellite’s TLE orbital elements from a source like Celestrak.
* [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization) - is a ROS package that offers 3 DoF and 6 DoF localization using PCL and allows dynamic map update using OctoMap
* [eagleye](https://github.com/MapIV/eagleye) -  is an open-source software for vehicle localization utilizing GNSS and IMU
* [python-sgp4](https://github.com/brandon-rhodes/python-sgp4) - Python version of the SGP4 satellite position library
* [PROJ](https://github.com/OSGeo/PROJ) - Cartographic Projections and Coordinate Transformations Library
* [rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation) -  implements common used trajectory evaluation methods for visual(-inertial) odometry.
* [pymap3d](https://github.com/geospace-code/pymap3d) - pure-Python (Numpy optional) 3D coordinate conversions for geospace ecef enu eci

### Simultaneous Localization and Mapping
#### Lidar
* [loam_velodyne](https://github.com/laboshinl/loam_velodyne) - Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar.
* [lio-mapping](https://github.com/hyye/lio-mapping) - Implementation of Tightly Coupled 3D Lidar Inertial Odometry and Mapping (LIO-mapping)
* [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) - Advanced implementation of LOAM
* [floam](https://github.com/wh200720041/floam) - Fast LOAM: Fast and Optimized Lidar Odometry And Mapping
* [LIO_SAM](https://github.com/TixiaoShan/LIO-) - Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping
* [cartographer_ros](https://github.com/googlecartographer/cartographer_ros) - Provides ROS integration for Cartographer
* [loam_livox](https://github.com/hku-mars/loam_livox) - A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR
* [StaticMapping](https://github.com/EdwardLiuyc/StaticMapping) - Use LiDAR to map the static world
* [semantic_suma](https://github.com/PRBonn/semantic_suma/) - Semantic Mapping using Surfel Mapping and Semantic Segmentation
* [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - Slam Toolbox for lifelong mapping and localization in potentially massive maps with ROS 
* [maplab](https://github.com/ethz-asl/maplab) - An open visual-inertial mapping framework.
* [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) - is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR
* [interactive_slam](https://github.com/SMRT-AIST/interactive_slam) -  In contrast to existing automatic SLAM packages, we with minimal human effort.
* [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
* [pyslam](https://github.com/luigifreda/pyslam) - contains a monocular Visual Odometry (VO) pipeline in Python
* [PointCNN](https://github.com/yangyanli/PointCNN) - is a simple and general framework for feature learning from point cloud, which refreshed five benchmark records in point cloud processing
* [Kitware SLAM](https://gitlab.kitware.com/keu-computervision/slam/) -  LiDAR-only visual SLAM developped by Kitware, as well as ROS and ParaView wrappings for easier use.
* [horizon_highway_slam](https://github.com/Livox-SDK/horizon_highway_slam) - horizon_highway_slam is a robust, low drift, and real time highway SLAM package suitable for Livox Horizon lidar
* [mola](https://github.com/MOLAorg/mola) - is a Modular System for Localization and Mapping
* [DH3D](https://github.com/JuanDuGit/DH3D) - Deep Hierarchical 3D Descriptors for Robust Large-Scale 6DOF Relocalization
* [iris_lama](https://github.com/iris-ua/iris_lama) - LaMa is a C++11 software library for robotic localization and mapping
* [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) - Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping


#### Visual
* [orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros) - A ROS implementation of ORB_SLAM2
* [orbslam-map-saving-extension](https://github.com/TUMFTM/orbslam-map-saving-extension) - In this extensions the map of ORB-features be saved to the disk as a reference for future runs along the same track
* [dso](https://github.com/JakobEngel/dso/) - Direct Sparse Odometry
* [viso2](https://github.com/srv/viso2) - A ROS wrapper for libviso2, a library for visual odometry
* [xivo](https://github.com/ucla-vision/xivo) - X Inertial-aided Visual Odometry
* [rovio](https://github.com/ethz-asl/rovio) - Robust Visual Inertial Odometry Framework
* [MIT Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) - C++ library for real-time metric-semantic visual-inertial Simultaneous Localization And Mapping (SLAM)
* [LSD-SLAM](https://github.com/tum-vision/lsd_slam) - LSD-SLAM: Large-Scale Direct Monocular SLAM is a real-time monocular SLAM 
* [CubeSLAM and ORB SLAM](https://github.com/shichaoy/cube_slam) - Monocular 3D Object Detection and SLAM Package of CubeSLAM and ORB SLAM
* [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) - A Robust and Versatile Multi-Sensor Visual-Inertial State Estimator
* [openvslam](https://github.com/xdspacelab/openvslam) - OpenVSLAM: A Versatile Visual SLAM Framework
* [basalt](https://gitlab.com/VladyslavUsenko/basalt) - Visual-Inertial Mapping with Non-Linear Factor Recovery
* [Kimera](https://github.com/MIT-SPARK/Kimera) - is a C++ library for real-time metric-semantic simultaneous localization and mapping, which uses camera images and inertial data to build a semantically annotated 3D mesh of the environment
* [tagslam](https://github.com/berndpfrommer/tagslam) - is a ROS-based package for Simultaneous Localization and Mapping using AprilTag fiducial markers
* [LARVIO](https://github.com/PetWorm/LARVIO) - A lightweight, accurate and robust monocular visual inertial odometry based on Multi-State Constraint Kalman Filter.
* [fiducials](https://github.com/UbiquityRobotics/fiducials) - Simultaneous localization and mapping using fiducial markers.
* [open_vins](https://github.com/rpng/open_vins) - An open source platform for visual-inertial navigation research.
* [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) - ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM
* [Atlas](https://github.com/magicleap/Atlas) - End-to-End 3D Scene Reconstruction from Posed Images
* [vilib](https://github.com/uzh-rpg/vilib) - This library focuses on the front-end of VIO pipelines with CUDA
* [hloc](https://github.com/cvg/Hierarchical-Localization) - a modular toolbox for state-of-the-art 6-DoF visual localization. It implements Hierarchical Localization, leveraging image retrieval and feature matching, and is fast, accurate, and scalable.
* [ESVO](https://github.com/HKUST-Aerial-Robotics/ESVO) - is a novel pipeline for real-time visual odometry using a stereo event-based camera


### Vector Map
* [OpenDRIVE](http://www.opendrive.org/index.html) - OpenDRIVE® is an open file format for the logical description of road networks
* [MapsModelsImporter](https://github.com/eliemichel/MapsModelsImporter) - A Blender add-on to import models from google maps
* [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Map handling framework for automated driving
* [barefoot](https://github.com/bmwcarit/barefoot) -  Online and Offline map matching that can be used stand-alone and in the cloud
* [iD](https://github.com/openstreetmap/iD) - The easy-to-use OpenStreetMap editor in JavaScript
* [RapiD](https://github.com/facebookincubator/RapiD) - an enhanced version of iD for mapping with AI created by Facebook
* [segmap](https://github.com/ethz-asl/segmap) - A map representation based on 3D segments 
* [Mapbox](https://github.com/mapbox/mapbox-gl-js) - is a JavaScript library for interactive, customizable vector maps on the web
* [osrm-backend](https://github.com/Project-OSRM/osrm-backend) - Open Source Routing Machine - C++ backend
* [robosat](https://github.com/mapbox/robosat) - Semantic segmentation on aerial and satellite imagery. Extracts features such as: buildings, parking lots, roads, water, clouds 
* [assuremapingtools](https://github.com/hatem-darweesh/assuremapingtools) -  Desktop based tool for viewing, editing and saving road network maps for autonomous vehicle platforms such as Autoware. 
* [geopandas](https://github.com/geopandas/geopandas) - is a project to add support for geographic data to pandas objects.
* [MapToolbox](https://github.com/autocore-ai/MapToolbox) - Plugins to make Autoware vector maps in Unity 
* [imagery-index](https://github.com/ideditor/imagery-index) - An index of aerial and satellite imagery useful for mapping
* [mapillary_tools](https://github.com/mapillary/mapillary_tools) - is a library for processing and uploading images to Mapillary.
* [mapnik](https://github.com/mapnik/mapnik) - combines pixel-perfect image output with lightning-fast cartographic algorithms, and exposes interfaces in C++, Python, and Node.
* [gdal](https://github.com/OSGeo/gdal) - GDAL is an open source X/MIT licensed translator library for raster and vector geospatial data formats.
* [grass](https://github.com/OSGeo/grass) - GRASS GIS - free and open source Geographic Information System (GIS)
* [3d-tiles](https://github.com/CesiumGS/3d-tiles) - Specification for streaming massive heterogeneous 3D geospatial datasets 
* [osmnx](https://github.com/gboeing/osmnx) - Python for street networks. Retrieve, model, analyze, and visualize street networks and other spatial data from OpenStreetMap

## Prediction
* [Awesome-Interaction-aware-Trajectory-Prediction](https://github.com/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction) - A selection of state-of-the-art research materials on trajectory prediction
* [sgan](https://github.com/agrimgupta92/sgan) -  Socially Acceptable Trajectories with Generative Adversarial Networks

## Behavior and Decision
* [Groot](https://github.com/BehaviorTree/Groot) - Graphical Editor to create BehaviorTrees. Compliant with BehaviorTree.CPP
* [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) - Behavior Trees Library in C++
* [RAFCON](https://github.com/DLR-RM/RAFCON) - Uses hierarchical state machines, featuring concurrent state execution, to represent robot programs
* [ROSPlan](https://github.com/KCL-Planning/ROSPlan) - Generic framework for task planning in a ROS system
* [ad-rss-lib](https://github.com/intel/ad-rss-lib) - Library implementing the Responsibility Sensitive Safety model (RSS) for Autonomous Vehicles 
* [FlexBE](https://flexbe.github.io/) - Graphical editor for hierarchical state machines, based on ROS's smach.
* [sts_bt_library](https://github.com/Autonomous-Logistics/sts_bt_library) - This library provides the functionality to set up your own behavior tree logic by using the defined tree structures like Fallback, Sequence or Parallel Nodes
* [SMACC](https://github.com/reelrbtx/SMACC) - An Event-Driven, Asynchronous, Behavioral State Machine Library for real-time ROS (Robotic Operating System) applications written in C++ 
* [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) - Behaviours, trees and utilities that extend py_trees for use with ROS.

## Planning and Control
* [pacmod](https://github.com/astuff/pacmod) -  is designed to allow the user to control a vehicle with the PACMod drive-by-wire system.
* [mpcc](https://github.com/alexliniger/MPCC) - Model Predictive Contouring Controller for Autonomous Racing
* [rrt](https://github.com/RoboJackets/rrt) - C++ RRT (Rapidly-exploring Random Tree) implementation
* [HypridAStarTrailer](https://github.com/AtsushiSakai/HybridAStarTrailer) - A path planning algorithm based on Hybrid A* for trailer truck.
* [path_planner](https://github.com/karlkurzer/path_planner) - Hybrid A* Path Planner for the KTH Research Concept Vehicle
* [open_street_map](https://github.com/ros-geographic-info/open_street_map) - ROS packages for working with Open Street Map geographic information.
* [Open Source Car Control](https://github.com/PolySync/oscc) -  is an assemblage of software and hardware designs that enable computer control of modern cars in order to facilitate the development of autonomous vehicle technology
* [fastrack](https://github.com/HJReachability/fastrack) - A ROS implementation of Fast and Safe Tracking (FaSTrack).
* [commonroad](https://commonroad.in.tum.de/) - Composable benchmarks for motion planning on roads
* [traffic-editor](https://github.com/osrf/traffic-editor) - A graphical editor for robot traffic flows.
* [steering_functions](https://github.com/hbanzhaf/steering_functions) - contains a C++ library that implements steering functions for car-like robots with limited turning radius
* [moveit](https://moveit.ros.org/) - Easy-to-use robotics manipulation platform for developing applications, evaluating designs, and building integrated products
* [flexible-collision-library](https://github.com/flexible-collision-library/fcl) - is a library for performing three types of proximity queries on a pair of geometric models composed of triangles.
* [aikido](https://github.com/personalrobotics/aikido) - Artificial Intelligence for Kinematics, Dynamics, and Optimization
* [casADi](https://github.com/casadi/casadi) - is a symbolic framework for numeric optimization implementing automatic differentiation in forward and reverse modes on sparse matrix-valued computational graphs
* [ACADO Toolkit](https://github.com/acado/acado) - is a software environment and algorithm collection for automatic control and dynamic optimization
* [control-toolbox](https://github.com/ethz-adrl/control-toolbox) - an efficient C++ library for control, estimation, optimization and motion planning in robotics
* [CrowdNav](https://github.com/vita-epfl/CrowdNav) -  Crowd-aware Robot Navigation with Attention-based Deep Reinforcement Learning
* [ompl](https://github.com/ompl/ompl) - consists of many state-of-the-art sampling-based motion planning algorithms.
* [openrave](https://github.com/rdiankov/openrave) - Open Robotics Automation Virtual Environment: An environment for testing, developing, and deploying robotics motion planning algorithms.
* [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) - An optimal trajectory planner considering distinctive topologies for mobile robots based on Timed-Elastic-Bands 
* [pinocchio](https://github.com/stack-of-tasks/pinocchio) - A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives
* [control-toolbox](https://github.com/ethz-adrl/control-toolbox) - an efficient C++ library for control, estimation, optimization and motion planning in robotics.  
* [rmf_core](https://github.com/osrf/rmf_core) - The rmf_core packages provide the centralized functions of the Robotics Middleware Framework (RMF)
* [OpEn](https://github.com/alphaville/optimization-engine) - is a solver for Fast & Accurate Embedded Optimization for next-generation Robotics and Autonomous Systems
* [autogenu-jupyter](https://github.com/mayataka/autogenu-jupyter) - This project provides the continuation/GMRES method (C/GMRES method) based solvers for nonlinear model predictive control (NMPC) and an automatic code generator for NMPC
* [global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) - This repository contains multiple approaches for generating global racetrajectories.
* [toppra](https://github.com/hungpham2511/toppra) - is a library for computing the time-optimal path parametrization for robots subject to kinematic and dynamic constraints
* [tinyspline](https://github.com/msteinbeck/tinyspline) - TinySpline is a small, yet powerful library for interpolating, transforming, and querying arbitrary NURBS, B-Splines, and Bézier curves
* [dual quaternions ros](https://github.com/Achllle/dual_quaternions_ros) - ROS python package for dual quaternion SLERP.
* [mb planner](https://github.com/unr-arl/mbplanner_ros) - Aerial vehicle planner for tight spaces. Used in DARPA SubT Challenge. 
* [ilqr](https://github.com/anassinator/ilqr) - Iterative Linear Quadratic Regulator with auto-differentiatiable dynamics models
* [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner) is a lightweight gradient-based local planner without ESDF construction, which significantly reduces computation time compared to some state-of-the-art methods
* [pykep](https://github.com/esa/pykep) - is a scientific library providing basic tools for research in interplanetary trajectory design.
* [am_traj](https://github.com/ZJU-FAST-Lab/am_traj) - Alternating Minimization Based Trajectory Generation for Quadrotor Aggressive Flight
* [GraphBasedLocalTrajectoryPlanner](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner) - was used on a real race vehicle during the Roborace Season Alpha and achieved speeds above 200km/h


## User Interaction
### Graphical User Interface
* [imgui](https://github.com/ocornut/imgui) - is designed to enable fast iterations and to empower programmers to create content creation tools and visualization / debug tools
* [qtpy](https://github.com/spyder-ide/qtpy) - Provides an uniform layer to support PyQt5, PySide2, PyQt4 and PySide with a single codebase
* [mir](https://github.com/MirServer/mir) - Mir is set of libraries for building Wayland based shells
* [rqt](https://wiki.ros.org/rqt) - rqt is a Qt-based framework for GUI development for ROS. It consists of three parts/metapackages
* [cage](https://github.com/Hjdskes/cage) - This is Cage, a Wayland kiosk. A kiosk runs a single, maximized application.
* [chilipie](https://github.com/futurice/chilipie-kiosk) - Easy-to-use Raspberry Pi image for booting directly into full-screen Chrome
* [pencil](https://github.com/evolus/pencil) - A tool for making diagrams and GUI prototyping that everyone can use.
* [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure) - The focus of dynamic_reconfigure is on providing a standard way to expose a subset of a node's parameters to external reconfiguration
* [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) - allows modifying parameters of a ROS node using the dynamic_reconfigure framework without having to write cfg files
* [elements](https://github.com/cycfi/elements) - is a lightweight, fine-grained, resolution independent, modular GUI library
* [NanoGUI](https://github.com/wjakob/nanogui) - is a minimalistic cross-platform widget library for OpenGL 3.x or higher


### Acoustic User Interface
* [pyo](https://github.com/belangeo/pyo) - is a Python module written in C containing classes for a wide variety of audio signal processing types
* [rhasspy](https://github.com/synesthesiam/rhasspy) - Rhasspy (pronounced RAH-SPEE) is an offline, multilingual voice assistant toolkit inspired by Jasper that works well with Home Assistant, Hass.io, and Node-RED
* [mycroft-core](https://github.com/MycroftAI/mycroft-core) - Mycroft is a hackable open source voice assistant
* [DDSP](https://github.com/magenta/ddsp) - is a library of differentiable versions of common DSP functions (such as synthesizers, waveshapers, and filters)
* [NoiseTorch](https://github.com/lawl/NoiseTorch) - creates a virtual microphone that suppresses noise, in any application
* [DeepSpeech](https://github.com/mozilla/DeepSpeech) - is an open source Speech-To-Text engine, using a model trained by machine learning techniques based on Baidu's Deep Speech research paper.
* [waveglow](https://github.com/NVIDIA/waveglow) - A Flow-based Generative Network for Speech Synthesis


### Command Line Interface
* [the-art-of-command-line](https://github.com/jlevy/the-art-of-command-line) - Master the command line, in one page
* [dotfiles of cornerman](https://github.com/cornerman/dotfiles) - Powerful zsh and vim dotfiles
* [dotbot](https://github.com/anishathalye/dotbot) - A tool that bootstraps your dotfiles
* [prompt-hjem](https://github.com/cornerman/prompt-hjem) - A beautiful zsh prompt
* [ag](https://github.com/ggreer/the_silver_searcher) - A code-searching tool similar to ack, but faster.
* [fzf](https://github.com/junegunn/fzf) - A command-line fuzzy finder
* [pkgtop](https://github.com/orhun/pkgtop) - Interactive package manager and resource monitor designed for the GNU/Linux.
* [asciimatics](https://github.com/peterbrittain/asciimatics) - A cross platform package to do curses-like operations, plus higher level APIs and widgets to create text UIs and ASCII art animations
* [gocui](https://github.com/jroimartin/gocui) - Minimalist Go package aimed at creating Console User Interfaces.
* [TerminalImageViewer](https://github.com/stefanhaustein/TerminalImageViewer)- Small C++ program to display images in a (modern) terminal using RGB ANSI codes and unicode block graphics characters
* [rosshow](https://github.com/dheera/rosshow) - Visualize ROS topics inside a terminal with Unicode/ASCII art
* [python-prompt-toolkit](https://github.com/prompt-toolkit/python-prompt-toolkit) - Library for building powerful interactive command line applications in Python
* [bash-script-template](https://github.com/ralish/bash-script-template) - A best practices Bash script template with several useful functions
* [guake](https://github.com/Guake/guake) - Drop-down terminal for GNOME
* [wemux](https://github.com/zolrath/wemux) - Multi-User Tmux Made Easy
* [tmuxp](https://github.com/tmux-python/tmuxp) -  tmux session manager built on libtmux
* [mapscii](https://github.com/rastapasta/mapscii) - World map renderer for your console
* [terminator](https://launchpad.net/terminator) - The goal of this project is to produce a useful tool for arranging terminals
* [bat](https://github.com/sharkdp/bat) - A cat(1) clone with wings.
* [fx](https://github.com/antonmedv/fx) - Command-line tool and terminal JSON viewer
* [tmate](https://github.com/tmate-io/tmate) - Instant terminal sharing

### Data Visualization and Mission Control
* [xdot](https://github.com/jrfonseca/xdot.py) - Interactive viewer for graphs written in Graphviz's dot language
* [guacamole](https://guacamole.apache.org/) - clientless remote desktop gateway. It supports standard protocols like VNC, RDP, and SSH
* [ros3djs](https://github.com/RobotWebTools/ros3djs) - 3D Visualization Library for use with the ROS JavaScript Libraries
* [webviz](https://github.com/cruise-automation/webviz) - web-based visualization libraries like rviz
* [plotly.py](https://github.com/plotly/plotly.py) - An open-source, interactive graphing library for Python
* [PlotJuggler](https://github.com/facontidavide/PlotJuggler) - The timeseries visualization tool that you deserve 
* [bokeh](https://github.com/bokeh/bokeh) - Interactive Data Visualization in the browser, from Python
* [voila](https://github.com/voila-dashboards/voila) - From Jupyter notebooks to standalone web applications and dashboards
* [Pangolin](https://github.com/stevenlovegrove/Pangolin) - Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input.
* [rqt_bag](http://wiki.ros.org/rqt_bag) - provides a GUI plugin for displaying and replaying ROS bag files.
* [kepler.gl](https://github.com/keplergl/kepler.gl) - Kepler.gl is a powerful open source geospatial analysis tool for large-scale data sets. 
* [qgis_ros](https://github.com/locusrobotics/qgis_ros) - Access bagged and live topic data in a highly featured GIS environment
* [openmct](https://github.com/nasa/openmct) - A web based mission control framework
* [web_video_server](https://github.com/RobotWebTools/web_video_server) - HTTP Streaming of ROS Image Topics in Multiple Formats 
* [rvizweb](https://github.com/osrf/rvizweb) - RVizWeb provides a convenient way of building and launching a web application with features similar to RViz
* [marvros](https://github.com/mavlink/mavros) - MAVLink to ROS gateway with proxy for Ground Control Station 
* [octave](https://www.gnu.org/software/octave/) - provides a convenient command line interface for solving linear and nonlinear problems numerically, and for performing other numerical experiments using a language that is mostly compatible with Matlab.
* [streetscape.gl](https://github.com/uber/streetscape.gl) - Streetscape.gl is a toolkit for visualizing autonomous and robotics data in the XVIZ protocol.
* [urdf-loaders](https://github.com/gkjohnson/urdf-loaders) - URDF Loaders for Unity and THREE.js with example ATHLETE URDF File
* [obs-studio](https://github.com/obsproject/obs-studio) - Free and open source software for live streaming and screen recording


#### Annotation
* [labelbox](https://github.com/Labelbox/labelbox) - is the fastest way to annotate data to build and ship artificial intelligence applications
* [PixelAnnotationTool](https://github.com/abreheret/PixelAnnotationTool) - Annotate quickly images
* [LabelImg](https://github.com/tzutalin/labelImg) - LabelImg is a graphical image annotation tool and label object bounding boxes in images
* [cvat](https://github.com/opencv/cvat) - Powerful and efficient Computer Vision Annotation Tool (CVAT)
* [point_labeler](https://github.com/jbehley/point_labeler) - Tool for labeling of a single point clouds or a stream of point clouds
* [label-studio](https://github.com/heartexlabs/label-studio) - Label Studio is a multi-type data labeling and annotation tool with standardized output format
* [napari](https://github.com/napari/napari) -  a fast, interactive, multi-dimensional image viewer for python 
* [semantic-segmentation-editor](https://github.com/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor) - A web based labeling tool for creating AI training data sets (2D and 3D)
* [3d-bat](https://github.com/walzimmer/3d-bat) - 3D Bounding Box Annotation Tool for Point cloud and Image Labeling
* [labelme](https://github.com/wkentaro/labelme) - Image Polygonal Annotation with Python (polygon, rectangle, circle, line, point and image-level flag annotation)
* [universal-data-tool](https://github.com/UniversalDataTool/universal-data-tool) - Collaborate & label any type of data, images, text, or documents, in an easy web interface or desktop app.
* [BMW-Labeltool-Lite](https://github.com/BMW-InnovationLab/BMW-Labeltool-Lite) - provides you with a easy to use labeling tool for State-of-the-art Deep Learning training purposes.


#### Point Cloud 
* [CloudCompare](https://github.com/CloudCompare/CloudCompare) - CloudCompare is a 3D point cloud (and triangular mesh) processing software.
* [Potree](https://github.com/potree/potree) - WebGL point cloud viewer for large datasets 
* [point_cloud_viewer](https://github.com/googlecartographer/point_cloud_viewer) - makes viewing massive point clouds easy and convenient
* [LidarView](https://github.com/Kitware/LidarView) - LidarView performs real-time visualization and easy processing of live captured 3D LiDAR data from Lidar sensors
* [VeloView](https://github.com/Kitware/VeloView) - VeloView performs real-time visualization of live captured 3D LiDAR data from Velodyne's HDL sensors.
* [entwine](https://github.com/connormanning/entwine/) - is a data organization library for massive point clouds, designed to conquer datasets of trillions of points as well as desktop-scale point clouds.
* [polyscope](https://github.com/nmwsharp/polyscope) - A C++ & Python viewer for 3D data like meshes and point clouds 
* [Pcx](https://github.com/keijiro/Pcx) - Point cloud importer & renderer for Unity
* [ImmersivePoints](https://github.com/rmeertens/ImmersivePoints) - is a web-application for virtual reality devices to explore 3D data in the most natural way possible.


#### RViz 
* [mapviz](https://github.com/swri-robotics/mapviz) - Modular ROS visualization tool for 2D data.
* [rviz_cinematographer](https://github.com/AIS-Bonn/rviz_cinematographer) - Easy to use tools to create and edit trajectories for the rviz camera.
* [rviz_satellite](https://github.com/gareth-cross/rviz_satellite) - Display internet satellite imagery in RViz
* [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools) - C++ API wrapper for displaying shapes and meshes in Rviz
* [xpp](https://github.com/leggedrobotics/xpp) - visualization of motion-plans for legged robots
* [rviz stereo](http://wiki.ros.org/rviz/Tutorials/Rviz%20in%20Stereo) - 3D stereo rendering displays a different view to each eye so that the scene appears to have depth
* [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization) - jsk visualization ros packages for rviz and rqt
* [moveit_visual_tools](https://github.com/ros-planning/moveit_visual_tools) - Helper functions for displaying and debugging MoveIt! data in Rviz via published markers


## Operation System 
### Monitoring
* [rosmon](https://github.com/xqms/rosmon) - ROS node launcher & monitoring daemon 
* [multimaster_fkie](https://github.com/fkie/multimaster_fkie) - GUI-based management environment that is very useful to manage ROS-launch configurations and control running nodes
* [collectd](https://github.com/collectd/collectd/) - collectd is a small daemon which collects system information periodically and provides mechanisms to store and monitor the values in a variety of ways### Launch and Monitor
* [lnav](http://lnav.org/) - is an enhanced log file viewer that takes advantage of any semantic information that can be gleaned from the files being viewed, such as timestamps and log levels
* [htop](https://github.com/hishamhm/htop) - htop is an interactive text-mode process viewer for Unix systems. It aims to be a better 'top'.
* [atop](https://github.com/Atoptool/atop) - System and process monitor for Linux with logging and replay function
* [psutil](https://github.com/giampaolo/psutil) - Cross-platform lib for process and system monitoring in Python
* [gputil](https://github.com/anderskm/gputil) - A Python module for getting the GPU status from NVIDA GPUs using nvidia-smi programmically in Python
* [gpustat](https://github.com/wookayin/gpustat) -  A simple command-line utility for querying and monitoring GPU status
* [nvtop](https://github.com/Syllo/nvtop) - NVIDIA GPUs htop like monitoring tool
* [spdlog](https://github.com/gabime/spdlog) - Very fast, header-only/compiled, C++ logging library
* [ctop](https://github.com/bcicen/ctop) -  Top-like interface for container metrics 
* [ntop](https://github.com/ntop/ntopng) - Web-based Traffic and Security Network Traffic Monitoring
* [jupyterlab-nvdashboard](https://github.com/rapidsai/jupyterlab-nvdashboard) - A JupyterLab extension for displaying dashboards of GPU usage

### Database and Record
* [ncdu](https://dev.yorhel.nl/ncdu) - Ncdu is a disk usage analyzer with an ncurses interface
* [borg](https://github.com/borgbackup/borg) - Deduplicating archiver with compression and authenticated encryption
* [bag-database](https://github.com/swri-robotics/bag-database) - A server that catalogs bag files and provides a web-based UI for accessing them
* [marv-robotics](https://gitlab.com/ternaris/marv-robotics) - MARV Robotics is a powerful and extensible data management platform
* [kitti2bag](https://github.com/tomas789/kitti2bag) - Convert KITTI dataset to ROS bag file the easy way!
* [pykitti](https://github.com/utiasSTARS/pykitti) - Python tools for working with KITTI data
* [rosbag_editor](https://github.com/facontidavide/rosbag_editor) - Create a rosbag from a given one, using a simple GUI
* [nextcloud](https://github.com/nextcloud/server) - Nextcloud is a suite of client-server software for creating and using file hosting services.
* [ros_type_introspection](https://github.com/facontidavide/ros_type_introspection) - Deserialize ROS messages that are unknown at compilation time
* [syncthing](https://github.com/syncthing/syncthing) - is a continuous file synchronization program
* [rqt_bag_exporter](https://gitlab.com/InstitutMaupertuis/rqt_bag_exporter) - Qt GUI to export ROS bag topics to files (CSV and/or video)
* [xviz](https://github.com/uber/xviz) - A protocol for real-time transfer and visualization of autonomy data
* [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) - dataset tools for working with the KITTI dataset raw data and converting it to a ROS bag. Also allows a library for direct access to poses, velodyne scans, and images. 
* [ros_numpy](https://github.com/eric-wieser/ros_numpy) - Tools for converting ROS messages to and from numpy arrays
* [kitti_ros](https://github.com/LidarPerception/kitti_ros) - A ROS-based player to replay KiTTI dataset
* [duckdb](https://github.com/cwida/duckdb) - DuckDB is an embeddable SQL OLAP Database Management System

### Network Distributed File System
* [sshfs](https://github.com/osxfuse/sshfs) - File system based on the SSH File Transfer Protocol
* [moosefs](https://github.com/moosefs/moosefs) -  a scalable distributed storage system
* [ceph](https://github.com/ceph/ceph) - is a distributed object, block, and file storage platform 
* [nfs](https://github.com/sahlberg/libnfs) - is a distributed file system protocol originally developed by Sun Microsystems 
* [ansible-role-nfs](https://github.com/geerlingguy/ansible-role-nfs) - Installs NFS utilities on RedHat/CentOS or Debian/Ubuntu.


### Server Infrastructure and High Performance Computing
* [mass](https://github.com/maas/maas) - Self-service, remote installation of Windows, CentOS, ESXi and Ubuntu on real servers turns your data centre into a bare metal cloud.
* [polyaxon](https://github.com/polyaxon/polyaxon) - A platform for reproducing and managing the whole life cycle of machine learning and deep learning applications.
* [localstack](https://github.com/localstack/localstack) - A fully functional local AWS cloud stack. Develop and test your cloud & Serverless apps offline
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) - Build and run Docker containers leveraging NVIDIA GPUs 
* [kubeflow](https://github.com/kubeflow/kubeflow) - Machine Learning Toolkit for Kubernetes
* [log-pilot](https://github.com/AliyunContainerService/log-pilot) - Collect logs for docker containers
* [traefik](https://github.com/containous/traefik) - The Cloud Native Edge Router 
* [graylog2-server](https://github.com/Graylog2/graylog2-server) - Free and open source log management
* [ansible](https://github.com/ansible/ansible) - Ansible is a radically simple IT automation platform that makes your applications and systems easier to deploy
* [pyinfra](https://github.com/Fizzadar/pyinfra) - It can be used for ad-hoc command execution, service deployment, configuration management and more
* [docker-py](https://github.com/docker/docker-py) - A Python library for the Docker Engine API 
* [noVNC](https://github.com/novnc/noVNC) - VNC client using HTML5
* [Slurm](https://github.com/SchedMD/slurm) - Slurm: A Highly Scalable Workload Manager
* [jupyterhub](https://github.com/jupyterhub/jupyterhub) - Multi-user server for Jupyter notebooks
* [Portainer](https://github.com/portainer/portainer)	 - Making Docker management easy
* [enroot](https://github.com/NVIDIA/enroot) - A simple, yet powerful tool to turn traditional container/OS images into unprivileged sandboxes.
* [docker-firefox](https://github.com/jlesage/docker-firefox) - Run a Docker Container with Firefox and noVNC for remote access to headless servers
* [luigi](https://github.com/spotify/luigi) - is a Python module that helps you build complex pipelines of batch jobs. It handles dependency resolution, workflow management, visualization etc. It also comes with Hadoop support built in.
* [triton-inference-server](https://github.com/NVIDIA/triton-inference-server) - NVIDIA Triton Inference Server provides a cloud inferencing solution optimized for NVIDIA GPUs
* [cudf](https://github.com/rapidsai/cudf) - provides a pandas-like API that will be familiar to data engineers & data scientists, so they can use it to easily accelerate their workflows without going into the details of CUDA programming.


### Embedded Operation System
* [vxworks7-ros2-build](https://github.com/Wind-River/vxworks7-ros2-build) - Build system to automate the build of VxWorks 7 and ROS2 
* [Yocto](https://git.yoctoproject.org/) - Produce tools and processes that enable the creation of Linux distributions for embedded software that are independent of the underlying architecture of the embedded hardware
* [Automotive Graded Linux](https://www.automotivelinux.org/software) - is a collaborative open source project that is bringing together automakers, suppliers and technology companies to build a Linux-based, open software platform for automotive applications that can serve as the de facto industry standard
* [bitbake](https://github.com/openembedded/bitbake) - is a generic task execution engine that allows shell and Python tasks to be run efficiently and in parallel while working within complex inter-task dependency constraints.
* [Jailhouse](https://github.com/siemens/jailhouse) - Jailhouse is a partitioning Hypervisor based on Linux
* [Xen](https://wiki.debian.org/Xen) - is an open-source (GPL) type-1 or baremetal hypervisor
* [QEMU](https://www.qemu.org/) - is a generic and open source machine emulator and virtualizer
* [qemu-xilinx](https://github.com/Xilinx/qemu) - A fork of Quick EMUlator (QEMU) with improved support and modelling for the Xilinx platforms
* [rosserial](https://github.com/ros-drivers/rosserial) - A ROS client library for small, embedded devices, such as Arduino
* [meta-ros](https://github.com/ros/meta-ros/tree/thud-draft) - OpenEmbedded Layer for ROS Applications
* [meta-balena](https://github.com/balena-os/meta-balena) - Run Docker containers on embedded devices
* [micro-ros](https://micro-ros.github.io/) - The major changes compared to "regular" ROS 2 is that micro-ROS uses a Real-Time Operating System (RTOS) instead of Linux, and DDS for eXtremely Resource Constrained Environments
* [nvidia-container-runtime](https://github.com/NVIDIA/nvidia-container-runtime/) - NVIDIA Container Runtime is a GPU aware container runtime, compatible with the Open Containers Initiative (OCI) specification used by Docker, CRI-O, and other popular container technologie
* [fusesoc](https://github.com/olofk/fusesoc) - Package manager and build abstraction tool for FPGA/ASIC development
* [jetson_easy](https://github.com/rbonghi/jetson_easy) - Automatically script to setup and configure your NVIDIA Jetson
* [docker-jetpack-sdk](https://github.com/trn84/docker-jetpack-sdk) -  allows for usage of the NVIDIA JetPack SDK within a docker container for download, flashing, and install
* [Pressed](https://wiki.debian.org/DebianInstaller/Preseed) - provides a way to set answers to questions asked during the installation process of debian, without having to manually enter the answers while the installation is running
* [jetson_stats](https://github.com/rbonghi/jetson_stats) - is a package to monitoring and control your NVIDIA Jetson [Xavier NX, Nano, AGX Xavier, TX1, TX2] Works with all NVIDIA Jetson ecosystem.
* [ros_jetson_stats](https://github.com/rbonghi/ros_jetson_stats) - The ROS jetson-stats wrapper. The status of your NVIDIA jetson in diagnostic messages
* [OpenCR](https://github.com/ROBOTIS-GIT/OpenCR) - Open-source Control Module for ROS
* [acrn-hypervisor](https://github.com/projectacrn/acrn-hypervisor) - defines a device hypervisor reference stack and an architecture for running multiple software subsystems, managed securely, on a consolidated system by means of a virtual machine manager
* [jetson-containers](https://github.com/dusty-nv/jetson-containers) - Machine Learning Containers for Jetson and JetPack 4.4

### Real-Time Kernel
* [ELISA](https://elisa.tech/) -  Project is to make it easier for companies to build and certify Linux-based safety-critical applications – systems whose failure could result in loss of human life, significant property damage or environmental damage
* [PREEMPT_RT kernel patch](https://wiki.linuxfoundation.org/realtime/documentation/start) - Aim of the PREEMPT_RT kernel patch is to minimize the amount of kernel code that is non-preemptible

### Network and Middleware
* [performance_test](https://github.com/ApexAI/performance_test) - Tool to test the performance of pub/sub based              communication frameworks.
* [realtime_support](https://github.com/ros2/realtime_support) - Minimal real-time testing utility for measuring jitter and latency.
* [ros1_bridge](https://github.com/ros2/ros1_bridge) -  ROS 2 package that provides bidirectional communication between ROS 1 and ROS 2
* [Fast-RTPS](https://github.com/eProsima/Fast-RTPS) -  protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, as defined and maintained by the Object Management Group (OMG) consortium
* [protobuf](https://github.com/protocolbuffers/protobuf) - Google's data interchange format
* [opensplice](https://github.com/ADLINK-IST/opensplice) - Vortex OpenSplice Community Edition 
* [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds) - Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation
* [iceoryx](https://github.com/eclipse/iceoryx) - an IPC middleware for POSIX-based systems
* [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) - provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more
* [ros2arduino](https://github.com/ROBOTIS-GIT/ros2arduino) - This library helps the Arduino board communicate with the ROS2 using XRCE-DDS.
* [eCAL](https://github.com/continental/) - The enhanced communication abstraction layer (eCAL) is a middleware that enables scalable, high performance interprocess communication on a single computer node or between different nodes in a computer network
* [AUTOSAR-Adaptive](https://github.com/UmlautSoftwareDevelopmentAccount/AUTOSAR-Adaptive) - The implementation of AUTOSAR Adaptive Platform based on the R19-11
* [ocpp](https://github.com/NewMotion/ocpp) - The Open Charge Point Protocol (OCPP) is a network protocol for communication between electric vehicle chargers and a central backoffice system.

#### Ethernet and Wireless Networking
* [SOES](https://github.com/OpenEtherCATsociety/SOES) - SOES is an EtherCAT slave stack written in c
* [netplan](https://netplan.io/) - simply create a YAML description of the required network interfaces and what each should be configured to do
* [airalab](https://github.com/airalab) -  AIRA is reference Robonomics network client for ROS-enabled cyber-physical systems.
* [rdbox](https://github.com/rdbox-intec/rdbox) - RDBOX is a IT infrastructure for ROS robots
* [ros_ethercat](https://github.com/shadow-robot/ros_ethercat) - This is a reimplementation of the main loop of pr2_ethercat without dependencies on PR2 software
* [wavemon](https://github.com/uoaerg/wavemon) - is an ncurses-based monitoring application for wireless network devices
* [wireless](https://github.com/clearpathrobotics/wireless) - Making info about wireless networks available to ROS.
* [ptpd](https://github.com/ptpd/ptpd) - PTP daemon (PTPd) is an implementation the Precision Time Protocol (PTP) version 2 as defined by 'IEEE Std 1588-2008'. PTP provides precise time coordination of Ethernet LAN connected computers
* [iperf](https://github.com/esnet/iperf) - A TCP, UDP, and SCTP network bandwidth measurement tool
* [tcpreplay](https://github.com/appneta/tcpreplay) - Pcap editing and replay tools
* [nethogs](https://github.com/raboof/nethogs) - It groups bandwidth by process
* [pyshark](https://github.com/KimiNewt/pyshark) - Python wrapper for tshark, allowing python packet parsing using wireshark dissectors
* [pingtop](https://github.com/laixintao/pingtop) - Ping multiple servers and show results in a top-like terminal UI
* [termshark](https://github.com/gcla/termshark) - A terminal UI for tshark, inspired by Wireshark
* [udpreplay](https://github.com/rigtorp/udpreplay) - Replay UDP packets from a pcap file 
* [openwifi](https://github.com/open-sdr/openwifi) - Linux mac80211 compatible full-stack IEEE802.11/Wi-Fi design based on Software Defined Radio

#### Controller Area Network
* [AndrOBD](https://github.com/fr3ts0n/AndrOBD) - Android OBD diagnostics with any ELM327 adapter
* [ddt4all](https://github.com/cedricp/ddt4all) - DDT4All is a tool to create your own ECU parameters screens and connect to a CAN network with a cheap ELM327 interface
* [cabana](https://github.com/commaai/cabana) - CAN visualizer and DBC maker
* [opendbc](https://github.com/commaai/opendbc) - The project to democratize access to the decoder ring of your car.
* [libuavcan](https://github.com/UAVCAN/libuavcan) - is an open lightweight protocol designed for reliable communication in aerospace and robotic applications over robust vehicular networks such as CAN bus.
* [python-can](https://github.com/hardbyte/python-can) - The can package provides controller area network support for Python developers
* [CANopenNode](https://github.com/CANopenNode/CANopenNode) - is the internationally standardized (EN 50325-4) (CiA301) CAN-based higher-layer protocol for embedded control system.
* [python-udsoncan](https://github.com/pylessard/python-udsoncan) - Python implementation of UDS (ISO-14229) standard
* [uds-c](https://github.com/openxc/uds-c) - Unified Diagnostics Service (UDS) and OBD-II (On Board Diagnostics for Vehicles) C Library
* [cantools](https://github.com/eerimoq/cantools) - CAN BUS tools in Python 3
* [CANdevStudio](https://github.com/GENIVI/CANdevStudio) -  CANdevStudio aims to be cost-effective replacement for CAN simulation software. It can work with variety of CAN hardware interfaces
* [can-utils](https://github.com/linux-can/can-utils) - Linux-CAN / SocketCAN user space applications
* [ros_canopen](https://github.com/ros-industrial/ros_canopen) - CANopen driver framework for ROS
* [decanstructor](https://github.com/JWhitleyAStuff/decanstructor) - The definitive ROS CAN analysis tool
* [kvaser_interface](https://github.com/astuff/kvaser_interface) - This package was developed as a standardized way to access Kvaser CAN devices from ROS
* [canmatrix](https://github.com/ebroecker/canmatrix) - Converting CAN Database Formats .arxml .dbc .dbf .kcd
* [autosar](https://github.com/cogu/autosar) - A set of python modules for working with AUTOSAR XML files
* [canopen](https://github.com/christiansandberg/canopen) - A Python implementation of the CANopen standard. The aim of the project is to support the most common parts of the CiA 301 standard in a Pythonic interface.
* [SavvyCAN](https://github.com/collin80/SavvyCAN) - A Qt5 based cross platform tool which can be used to load, save, and capture canbus frames


### Sensor and Acuator Interfaces
* [Tesla-API](https://github.com/timdorr/tesla-api) - provides functionality to monitor and control the Model S (and future Tesla vehicles) remotely
* [flirpy](https://github.com/LJMUAstroecology/flirpy) - is a Python library to interact with FLIR thermal imaging cameras and images.
* [nerian_stereo](https://github.com/nerian-vision/nerian_stereo) - ROS node for Nerian's SceneScan and SP1 stereo vision sensors
* [pymmw](https://github.com/m6c7l/pymmw) - This is a toolbox composed of Python scripts to interact with TI's evaluation module (BoosterPack) for the IWR1443 mmWave sensing device
* [ti_mmwave_rospkg](https://github.com/radar-lab/ti_mmwave_rospkg) - TI mmWave radar ROS driver (with sensor fusion and hybrid
* [pacmod3](https://github.com/astuff/pacmod3) - This ROS node is designed to allow the user to control a vehicle with the PACMod drive-by-wire system, board revision 3.
* [ros2_intel_realsense](https://github.com/intel/ros2_intel_realsense) - These are packages for using Intel RealSense cameras (D400 series) with ROS2.
* [sick_scan](https://github.com/SICKAG/sick_scan) - This stack provides a ROS2 driver for the SICK TiM series of laser scanners.
* [ouster_example](https://github.com/ouster-lidar/ouster_example) - Sample code for connecting to and configuring the OS1, reading and visualizing data, and interfacing with ROS.
* [livox_ros_driver](https://github.com/Livox-SDK/livox_ros_driver) - is a new ROS package, specially used to connect LiDAR products produced by Livox
* [velodyne](https://github.com/ros-drivers/velodyne) -  is a collection of ROS packages supporting Velodyne high definition 3D LIDARs
* [ublox](https://github.com/KumarRobotics/ublox) - provides support for u-blox GPS receivers.
* [crazyflie_ros](https://github.com/whoenig/crazyflie_ros) - ROS Driver for Bitcraze Crazyflie
* [pointgrey_camera_driver](https://github.com/ros-drivers/pointgrey_camera_driver) - ROS driver for Pt. Grey cameras, based on the official FlyCapture2 SDK
* [novatel_gps_driver](https://github.com/swri-robotics/novatel_gps_driver) - ROS driver for NovAtel GPS / GNSS receivers
* [pylon-ros-camera](https://github.com/basler/pylon-ros-camera) - The official pylon ROS driver for Basler GigE Vision and USB3 Vision cameras
* [ethz_piksi_ros](https://github.com/ethz-asl/ethz_piksi_ros) -  contains (python) ROS drivers, tools, launch files, and wikis about how to use Piksi Real Time Kinematic (RTK) GPS device in ROS
* [sick_safetyscanners](https://github.com/SICKAG/sick_safetyscanners) - A ROS Driver which reads the raw data from the SICK Safety Scanners and publishes the data as a laser_scan msg.
* [bosch_imu_driver](https://github.com/mdrwiega/bosch_imu_driver) - A driver for the sensor IMU Bosch BNO055. It was implemented only the UART communication interface (correct sensor mode should be selected)
* [oxford_gps_eth](https://bitbucket.org/DataspeedInc/oxford_gps_eth/) - Ethernet interface to OxTS GPS receivers using the NCOM packet structure
* [ifm3d](https://github.com/ifm/ifm3d) - Library and Utilities for working with ifm pmd-based 3D ToF Cameras
* [cepton_sdk_redist](https://github.com/ceptontech/cepton_sdk_redist/) - Provides ROS support for Cepton LiDAR
* [jetson_csi_cam](https://github.com/peter-moran/jetson_csi_cam) - A ROS package making it simple to use CSI cameras on the Nvidia Jetson TK1, TX1, or TX2 with ROS
* [ros_astra_camera](https://github.com/orbbec/ros_astra_camera) - A ROS driver for Orbbec 3D cameras
* [spot_ros](https://github.com/clearpathrobotics/spot_ros) - ROS Driver for Spot


### Security
* [owasp-threat-dragon-desktop](https://github.com/mike-goodwin/owasp-threat-dragon-desktop) - Threat Dragon is a free, open-source, cross-platform threat modeling application including system diagramming and a rule engine to auto-generate threats/mitigations
* [launch_ros_sandbox](https://github.com/ros-tooling/launch_ros_sandbox) - can define launch files running nodes in restrained environments, such as Docker containers or separate user accounts with limited privileges
* [wolfssl](https://github.com/wolfSSL/wolfssl) - is a small, fast, portable implementation of TLS/SSL for embedded devices to the cloud
* [CANalyzat0r](https://github.com/schutzwerk/CANalyzat0r) - Security analysis toolkit for proprietary car protocols
* [RSF](https://github.com/aliasrobotics/RSF) - Robot Security Framework (RSF) is a standardized methodology to perform security assessments in robotics
* [How-to-Secure-A-Linux-Server](https://github.com/imthenachoman/How-To-Secure-A-Linux-Server) - An evolving how-to guide for securing a Linux server.
* [lynis](https://github.com/CISOfy/lynis) - Security auditing tool for Linux, macOS, and UNIX-based systems. Assists with compliance testing (HIPAA/ISO27001/PCI DSS) and system hardening
* [OpenVPN](https://github.com/OpenVPN/openvpn) - is an open source VPN daemon
* [openfortivpn](https://github.com/adrienverge/openfortivpn) - openfortivpn is a client for PPP+SSL VPN tunnel services and compatible with Fortinet VPNs
* [WireGuard](https://github.com/WireGuard/WireGuard) - WireGuard is a novel VPN that runs inside the Linux Kernel and utilizes state-of-the-art cryptography
* [ssh-auditor](https://github.com/ncsa/ssh-auditor) - Scans for weak ssh passwords on your network
* [vulscan](https://github.com/scipag/vulscan) - Advanced vulnerability scanning with Nmap NSE
* [nmap-vulners](https://github.com/vulnersCom/nmap-vulners) - NSE script based on Vulners.com API
* [brutespray](https://github.com/x90skysn3k/brutespray) - Automatically attempts default creds on found services.
* [fail2ban](https://github.com/fail2ban/fail2ban) - Daemon to ban hosts that cause multiple authentication errors 
* [DependencyCheck](https://github.com/jeremylong/DependencyCheck) - is a software composition analysis utility that detects publicly disclosed vulnerabilities in application dependencies
* [firejail](https://github.com/netblue30/firejail) - Firejail is a SUID sandbox program that reduces the risk of security breaches by restricting the running environment of untrusted applications using Linux namespaces, seccomp-bpf and Linux capabilities
* [RVD](https://github.com/aliasrobotics/RVD) - Robot Vulnerability Database. Community-contributed archive of robot vulnerabilities and weaknesses
* [ros2_dds_security](http://design.ros2.org/articles/ros2_dds_security.html) - adding security enhancements by defining a Service Plugin Interface (SPI) architecture, a set of builtin implementations of the SPIs, and the security model enforced by the SPIs
* [Security-Enhanced Linux](https://github.com/SELinuxProject/selinux) - is a Linux kernel security module that provides a mechanism for supporting access control security policies, including mandatory access controls (MAC)
* [OpenTitan](https://github.com/lowRISC/opentitan) - will make the silicon Root of Trust design and implementation more transparent, trustworthy, and secure for enterprises, platform providers, and chip manufacturers. OpenTitan is administered by lowRISC CIC as a collaborative project to produce high quality, open IP for instantiation as a full-featured product
* [bandit](https://github.com/PyCQA/bandit) - is a tool designed to find common security issues in Python code.
* [hardening](https://github.com/konstruktoid/hardening) - A quick way to make a Ubuntu server a bit more secure.
* [passbolt_docker](https://github.com/passbolt/passbolt_docker) - Passbolt is a free and open source password manager that allows team members to store and share credentials securely
* [gopass](https://github.com/gopasspw/gopass) - is a password manager for the command line written in Go
* [pass](https://www.passwordstore.org/) - the standard unix password manager
* [vault](https://github.com/hashicorp/vault) - Vault is a tool for securely accessing secrets. A secret is anything that you want to tightly control access to, such as API keys, passwords, certificates, and more
* [legion](https://github.com/GoVanguard/legion) - is an open source, easy-to-use, super-extensible and semi-automated network penetration testing framework that aids in discovery, reconnaissance and exploitation of information systems
* [openscap](https://github.com/OpenSCAP/openscap) - The oscap program is a command line tool that allows users to load, scan, validate, edit, and export SCAP documents.
* [pyre-check](https://github.com/facebook/pyre-check/) - ships with Pysa, a security focused static analysis tool we've built on top of Pyre that reasons about data flows in Python applications


## Datasets
* [waymo_ros](https://github.com/YonoHub/waymo_ros) - This is a ROS package to connect Waymo open dataset to ROS
* [waymo-open-dataset](https://github.com/waymo-research/waymo-open-dataset) - The Waymo Open Dataset is comprised of high-resolution sensor data collected by Waymo self-driving cars in a wide variety of conditions.
* [Ford Autonomous Vehicle Dataset](https://avdata.ford.com/home/default.aspx) - Ford presents a challenging multi-agent seasonal dataset collected by a fleet of Ford autonomous vehicles at different days and times.
* [awesome-robotics-datasets](https://github.com/sunglok/awesome-robotics-datasets) - A collection of useful datasets for robotics and computer vision 
* [nuscenes-devkit](https://github.com/nutonomy/nuscenes-devkit) - The devkit of the nuScenes dataset
* [dataset-api](https://github.com/ApolloScapeAuto/dataset-api) - This is a repo of toolkit for ApolloScape Dataset, CVPR 2019 Workshop on Autonomous Driving Challenge and ECCV 2018 challenge.
* [utbm_robocar_dataset](https://github.com/epan-utbm/utbm_robocar_dataset) - EU Long-term Dataset with Multiple Sensors for Autonomous Driving
* [DBNet](https://github.com/driving-behavior/DBNet) - DBNet: A Large-Scale Dataset for Driving Behavior Learning
* [argoverse-api](https://github.com/argoai/argoverse-api) - Official GitHub repository for Argoverse dataset
* [DDAD](https://github.com/TRI-ML/DDAD) - is a new autonomous driving benchmark from TRI (Toyota Research Institute) for long range (up to 250m) and dense depth estimation in challenging and diverse urban conditions
* [pandaset-devkit](https://github.com/scaleapi/pandaset-devkit) - Public large-scale dataset for autonomous driving provided by Hesai & Scale
* [a2d2_to_ros](https://gitlab.com/MaplessAI/external/a2d2_to_ros) - Utilities for converting A2D2 data sets to ROS bags.
* [argoverse-api](https://github.com/argoai/argoverse-api) - Official GitHub repository for Argoverse dataset
* [awesome-satellite-imagery-datasets](https://github.com/chrieke/awesome-satellite-imagery-datasets) - List of satellite image training datasets with annotations for computer vision and deep learning 
* [sentinelsat](https://github.com/sentinelsat/sentinelsat) - Search and download Copernicus Sentinel satellite images
* [adas-dataset-form](https://www.flir.com/oem/adas/adas-dataset-form/) - Thermal Dataset for Algorithm Training
* [h3d](https://usa.honda-ri.com/h3d) - The H3D is a large scale full-surround 3D multi-object detection and tracking dataset from Honda
* [Mapillary Vistas Dataset](https://www.mapillary.com/dataset/vistas) - A diverse street-level imagery dataset with pixel‑accurate and instance‑specific human annotations for understanding street scenes around the world. 
* [TensorFlow Datasets](https://www.tensorflow.org/datasets/catalog/overview) - TensorFlow Datasets provides many public datasets as tf.data.Datasets
* [racetrack-database](https://github.com/TUMFTM/racetrack-database) - contains center lines (x- and y-coordinates), track widths and race lines for over 20 race tracks (mainly F1 and DTM) all over the world
* [BlenderProc](https://github.com/DLR-RM/BlenderProc) - A procedural Blender pipeline for photorealistic training image generation
* [Atlatec Sample Map Data](https://www.atlatec.de/getsampledata.html) - 3D map for autonomous driving and simulation created from nothing but two cameras and GPS in downtown San Francisco.
* [Lyft Level 5 Dataset](https://self-driving.lyft.com/level5/data/) - Level 5 is developing a self-driving system for the Lyft network. We’re collecting and processing data from our autonomous fleet and sharing it with you.
* [holicity](https://github.com/zhou13/holicity) - A City-Scale Data Platform for Learning Holistic 3D Structures
* [UTD19](https://utd19.ethz.ch/) - Largest multi-city traffic dataset publically available
* [ASTYX HIRES2019 DATASET](http://www.pinchofintelligence.com/visualising-lidar-and-radar-in-virtual-reality/) - Automotive Radar Dataset for Deep Learning Based 3D Object Detection


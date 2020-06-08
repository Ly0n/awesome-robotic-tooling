# Awesome Robotic Tooling [![Awesome](https://awesome.re/badge.svg)](https://awesome.re)

Just a bunch of powerful robotic resources and tools for professional development with ROS in C++ with a touch of autonomous driving.

> [Unix philosophy](https://en.wikipedia.org/wiki/Unix_philosophy) - about the concept of software tooling

* [Communication and Coordination](#communication-and-coordination)
* [Documentation and Presentation](#documentation-and-presentation)
* [Architecture and Design](#architecture-and-design)
* [Safety](#safety)
* [Framework](#framework)
* [Development Environment](#development-environment)
   * [Code and Run](#code-and-run)
   * [Template](#template)
   * [Build and Deploy](#build-and-deploy)
   * [Unit and Integration Test](#unit-and-integration-test)
   * [Lint and Format](#lint-and-format)
   * [Launch and Monitor](#launch-and-monitor)
   * [Debugging and Tracing](#debugging-and-tracing)
   * [Version Control](#version-control)
   * [Simulation](#simulation)
* [Hardware](#hardware)
   * [Calibration and Transformation](#calibration-and-transformation)
* [Sensor Processing](#sensor-processing)
   * [Perception Pipeline](#perception-pipeline)
   * [Parallel Processing](#parallel-processing)
   * [Machine Learning](#machine-learning)
   * [Radar Processing](#radar-processing)
   * [Image Processing](#image-processing)
   * [Lidar Processing](#point-cloud-processing)
   * [Localization](#localization)
   * [SLAM](#slam)
      * [Lidar](#lidar)
      * [Camera](#camera)
      * [Static Map](#static-map)
* [Prediction](#prediction)
* [Behavior and Decision](#behavior-and-decision)
* [Planning and Control](#planning-and-control)
* [User Interaction](#user-interaction)
   * [Graphical User Interface](#graphical-user-interface)
   * [Command Line Interface](#command-line-interface)
   * [Visualization](#visualization)
      * [Annotation](#annotation)
      * [Point Cloud](#point-cloud)
      * [RViz](#rviz)
* [Operation System](#system)
   * [Monitoring](#monitoring)
   * [Storage and Record](#storage-and-record)
   * [Network Distributed File System](#network-distributed-file-system)
   * [High Performance Computing](#high-performance-computing)
   * [Embedded Operation System](#embedded-operation-system)
   * [Real-Time Kernel](#real-time-kernel)
   * [Network and Middleware](#network-and-middleware)
   * [Security](#security)
* [Datasets](#datasets)

## Communication and Coordination
* [Agile Development](https://agilemanifesto.org/) - Manifesto for Agile Software Development
* [Gitflow](https://github.com/nvie/gitflow) ![gitflow](https://img.shields.io/github/stars/nvie/gitflow.svg?style=flat&label=Star&maxAge=86400)  - Makes parallel development very easy, by isolating new development from finished work
* [DeepL](https://github.com/uinput/deeplator) ![deeplator](https://img.shields.io/github/stars/uinput/deeplator.svg?style=flat&label=Star&maxAge=86400)  - an online translator that outperforms Google, Microsoft and Facebook
* [Taiga](https://github.com/benhutchins/docker-taiga) ![docker-taiga](https://img.shields.io/github/stars/benhutchins/docker-taiga.svg?style=flat&label=Star&maxAge=86400)  - Agile Projectmanagment Tool
* [Kanboard](https://github.com/kanboard/kanboard) ![kanboard](https://img.shields.io/github/stars/kanboard/kanboard.svg?style=flat&label=Star&maxAge=86400)  - Minimalistic Kanban Board
* [kanban](https://gitlab.com/leanlabsio/kanban) - Free, open source, self-hosted, Kanban board for GitLab issues
* [Gitlab](https://github.com/sameersbn/docker-gitlab) ![docker-gitlab](https://img.shields.io/github/stars/sameersbn/docker-gitlab.svg?style=flat&label=Star&maxAge=86400)  - Simple Selfhosted Gitlab Server with Docker
* [Gogs](https://github.com/gogs/gogs) ![gogs](https://img.shields.io/github/stars/gogs/gogs.svg?style=flat&label=Star&maxAge=86400)  - Aims to build a simple, stable and extensible self-hosted Git service that can be setup in the most painless way
* [Woost](https://woost.space/) - Workflow Automation, Collaboration with Externals
* [Wekan](https://github.com/wekan/wekan) ![wekan](https://img.shields.io/github/stars/wekan/wekan.svg?style=flat&label=Star&maxAge=86400)  - Meteor based Kanban Board
* [JIRA API](https://github.com/pycontribs/jira) ![jira](https://img.shields.io/github/stars/pycontribs/jira.svg?style=flat&label=Star&maxAge=86400)  - Python Library for REST API of Jira
* [Taiga API](https://github.com/nephila/python-taiga) ![python-taiga](https://img.shields.io/github/stars/nephila/python-taiga.svg?style=flat&label=Star&maxAge=86400)  - Python Library for REST API of Taiga
* [Chronos-Timetracker](https://github.com/web-pal/chronos-timetracker) ![chronos-timetracker](https://img.shields.io/github/stars/web-pal/chronos-timetracker.svg?style=flat&label=Star&maxAge=86400)   - Desktop client for JIRA. Track time, upload worklogs without a hassle
* [Grge](https://gitlab.com/ApexAI/grge) - Grge is a daemon and command line utility augmenting GitLab
* [gitlab-triage](https://gitlab.com/gitlab-org/gitlab-triage) - GitLab's issues and merge requests triage, automated!
* [Helpy](https://github.com/helpyio/helpy) ![helpy](https://img.shields.io/github/stars/helpyio/helpy.svg?style=flat&label=Star&maxAge=86400)  - is a modern, open source helpdesk customer support application
* [ONLYOFFICE](https://github.com/ONLYOFFICE/CommunityServer) ![CommunityServer](https://img.shields.io/github/stars/ONLYOFFICE/CommunityServer.svg?style=flat&label=Star&maxAge=86400)  -  is a free open source collaborative system developed to manage documents, projects, customer relationship and email correspondence, all in one place.
* [discourse](https://github.com/discourse/discourse) ![discourse](https://img.shields.io/github/stars/discourse/discourse.svg?style=flat&label=Star&maxAge=86400)  - A platform for community discussion. Free, open, simple
* [Gerrit](https://gerrit.googlesource.com/gerrit/) - is a code review and project management tool for Git based projects
* [jitsi-meet](https://github.com/jitsi/jitsi-meet) ![jitsi-meet](https://img.shields.io/github/stars/jitsi/jitsi-meet.svg?style=flat&label=Star&maxAge=86400)  - Secure, Simple and Scalable Video Conferences that you use as a standalone app or embed in your web application
* [mattermost](https://github.com/mattermost/mattermost-server) ![mattermost-server](https://img.shields.io/github/stars/mattermost/mattermost-server.svg?style=flat&label=Star&maxAge=86400)  - is an open source, private cloud, Slack-alternative
* [openproject](https://github.com/opf/openproject) ![openproject](https://img.shields.io/github/stars/opf/openproject.svg?style=flat&label=Star&maxAge=86400)  - is the leading open source project management software
* [hackmd](https://hackmd.io) - Online Real-time collaborate on team documentation in markdown
* [leantime](https://github.com/Leantime/leantime) ![leantime](https://img.shields.io/github/stars/Leantime/leantime.svg?style=flat&label=Star&maxAge=86400)  - Leantime is a lean project management system for innovators
* [buku](https://github.com/jarun/buku) ![buku](https://img.shields.io/github/stars/jarun/buku.svg?style=flat&label=Star&maxAge=86400)  - Browser-independent bookmark manager

## Documentation and Presentation
* [Typora](https://typora.io/) - A Minimalist Markdown Editor
* [Markor](https://github.com/gsantner/markor) ![markor](https://img.shields.io/github/stars/gsantner/markor.svg?style=flat&label=Star&maxAge=86400)  - A Simple Markdown Editor for your Android Device
* [Pandoc](https://github.com/jgm/pandoc) ![pandoc](https://img.shields.io/github/stars/jgm/pandoc.svg?style=flat&label=Star&maxAge=86400)  - Universal markup converter
* [Yaspeller](https://github.com/hcodes/yaspeller) ![yaspeller](https://img.shields.io/github/stars/hcodes/yaspeller.svg?style=flat&label=Star&maxAge=86400)  - Command line tool for spell checking
* [ReadtheDocs](https://docs.readthedocs.io/en/stable/development/buildenvironments.html) - Build your local ReadtheDocs Server
* [Doxygen](https://github.com/doxygen/doxygen) ![doxygen](https://img.shields.io/github/stars/doxygen/doxygen.svg?style=flat&label=Star&maxAge=86400)  - Doxygen is the de facto standard tool for generating documentation from annotated C++ sources
* [Sphinx](https://github.com/sphinx-doc/sphinx/) ![](https://img.shields.io/github/stars/sphinx-doc/sphinx/.svg?style=flat&label=Star&maxAge=86400)  - is a tool that makes it easy to create intelligent and beautiful documentation for Python projects
* [Word-to-Markdown](https://github.com/benbalter/word-to-markdown) ![word-to-markdown](https://img.shields.io/github/stars/benbalter/word-to-markdown.svg?style=flat&label=Star&maxAge=86400)  - A ruby gem to liberate content from Microsoft Word document
* [carbon](https://github.com/carbon-app/carbon) ![carbon](https://img.shields.io/github/stars/carbon-app/carbon.svg?style=flat&label=Star&maxAge=86400)  - Share beautiful images of your source code
* [undraw](https://undraw.co/illustrations) - Free Professional business SVGs easy to customize
* [asciinema](https://github.com/asciinema/asciinema) ![asciinema](https://img.shields.io/github/stars/asciinema/asciinema.svg?style=flat&label=Star&maxAge=86400)  - lets you easily record terminal sessions and replay them in a terminal as well as in a web browser.
* [inkscape](https://inkscape.org/) - Inkscape is a professional vector graphics editor for Linux, Windows and macOS
* [Reveal-Hugo](https://github.com/dzello/reveal-hugo) ![reveal-hugo](https://img.shields.io/github/stars/dzello/reveal-hugo.svg?style=flat&label=Star&maxAge=86400)  - A Hugo theme for Reveal.js that makes authoring and customization a breeze. With it, you can turn any properly-formatted Hugo content into a HTML presentation.
* [Hugo-Webslides](https://github.com/RCJacH/hugo-webslides) ![hugo-webslides](https://img.shields.io/github/stars/RCJacH/hugo-webslides.svg?style=flat&label=Star&maxAge=86400)  - This is a Hugo template to create WebSlides presentation using markdown.
* [jupyter2slides](https://github.com/datitran/jupyter2slides) ![jupyter2slides](https://img.shields.io/github/stars/datitran/jupyter2slides.svg?style=flat&label=Star&maxAge=86400)  - Cloud Native Presentation Slides with Jupyter Notebook + Reveal.js
* [patat](https://github.com/jaspervdj/patat) ![patat](https://img.shields.io/github/stars/jaspervdj/patat.svg?style=flat&label=Star&maxAge=86400)  - Terminal-based presentations using Pandoc
* [github-changelog-generator](https://github.com/github-changelog-generator/github-changelog-generator) ![github-changelog-generator](https://img.shields.io/github/stars/github-changelog-generator/github-changelog-generator.svg?style=flat&label=Star&maxAge=86400)  - Automatically generate change log from your tags, issues, labels and pull requests on GitHub.
* [GitLab-Release-Note-Generator](https://github.com/jk1z/GitLab-Release-Note-Generator) ![GitLab-Release-Note-Generator](https://img.shields.io/github/stars/jk1z/GitLab-Release-Note-Generator.svg?style=flat&label=Star&maxAge=86400)  - A Gitlab release note generator that generates release note on latest tag
* [OCRmyPDF](https://github.com/jbarlow83/OCRmyPDF) ![OCRmyPDF](https://img.shields.io/github/stars/jbarlow83/OCRmyPDF.svg?style=flat&label=Star&maxAge=86400)  - adds an OCR text layer to scanned PDF files, allowing them to be searched
* [papermill](https://github.com/nteract/papermill) ![papermill](https://img.shields.io/github/stars/nteract/papermill.svg?style=flat&label=Star&maxAge=86400)  - is a tool for parameterizing, executing, and analyzing Jupyter Notebooks.
* [docs](https://github.com/google/docsy-example) ![docsy-example](https://img.shields.io/github/stars/google/docsy-example.svg?style=flat&label=Star&maxAge=86400)  - An example documentation site using the Docsy Hugo theme
* [overleaf](https://www.overleaf.com/project) - The easy to use, online, collaborative LaTeX editor
* [landslide](https://github.com/adamzap/landslide) ![landslide](https://img.shields.io/github/stars/adamzap/landslide.svg?style=flat&label=Star&maxAge=86400)  - Generate HTML5 slideshows from markdown, ReST, or textile
* [libreoffice-impress-templates](https://github.com/dohliam/libreoffice-impress-templates) ![libreoffice-impress-templates](https://img.shields.io/github/stars/dohliam/libreoffice-impress-templates.svg?style=flat&label=Star&maxAge=86400)  - Freely-licensed LibreOffice Impress templates

## Architecture and Design
* [Guidelines](https://github.com/S2-group/icse-seip-2020-replication-package/blob/master/ICSE_SEIP_2020.pdf)  - on how to architect ROS-based systems
* [doorstep](https://github.com/doorstop-dev/doorstop) ![doorstop](https://img.shields.io/github/stars/doorstop-dev/doorstop.svg?style=flat&label=Star&maxAge=86400)  - Requirements management using version control
* [yed](https://www.yworks.com/products/yed) - yEd is a powerful desktop application that can be used to quickly and effectively generate high-quality diagrams
* [yed_py](https://github.com/true-grue/yed_py) ![yed_py](https://img.shields.io/github/stars/true-grue/yed_py.svg?style=flat&label=Star&maxAge=86400)  - Generates graphML that can be opened in yEd
* [Plantuml](https://github.com/plantuml/plantuml-server) ![plantuml-server](https://img.shields.io/github/stars/plantuml/plantuml-server.svg?style=flat&label=Star&maxAge=86400)  - Web application to generate UML diagrams on-the-fly in your live documentation
* [rqt_graph](https://wiki.ros.org/rqt_graph) - rqt_graph provides a GUI plugin for visualizing the ROS computation graph
* [rqt_launchtree](https://github.com/pschillinger/rqt_launchtree) ![rqt_launchtree](https://img.shields.io/github/stars/pschillinger/rqt_launchtree.svg?style=flat&label=Star&maxAge=86400)  - An RQT plugin for hierarchical launchfile configuration introspection.
* [cpp-dependencies](https://github.com/tomtom-international/cpp-dependencies) ![cpp-dependencies](https://img.shields.io/github/stars/tomtom-international/cpp-dependencies.svg?style=flat&label=Star&maxAge=86400)  - Tool to check C++ #include dependencies (dependency graphs created in .dot format)
* [pydeps](https://github.com/thebjorn/pydeps) ![pydeps](https://img.shields.io/github/stars/thebjorn/pydeps.svg?style=flat&label=Star&maxAge=86400)  - Python Module Dependency graphs
* [aztarna](https://github.com/aliasrobotics/aztarna) ![aztarna](https://img.shields.io/github/stars/aliasrobotics/aztarna.svg?style=flat&label=Star&maxAge=86400)  -  a footprinting tool for robots.
* [draw.io](https://www.draw.io/) - is free online diagram software for making flowcharts, process diagrams, org charts, UML, ER and network diagrams
* [capella](https://www.eclipse.org/capella/) - Comprehensive, extensible and field-proven MBSE tool and method
to successfully design systems architecture
* [robmosys](https://robmosys.eu/) - RobMoSys envisions an integrated approach built on top of the current code-centric robotic platforms, by applying model-driven methods and tools
* [Papyrus for Robotics](https://www.eclipse.org/papyrus/components/robotics/) - is graphical editing tool for robotic applications that complies with the RobMoSys approach
* [fossology](https://github.com/fossology/fossology) ![fossology](https://img.shields.io/github/stars/fossology/fossology.svg?style=flat&label=Star&maxAge=86400)  - a toolkit you can run license, copyright and export control scans from the command line
* [vscode-drawio](https://github.com/hediet/vscode-drawio) ![vscode-drawio](https://img.shields.io/github/stars/hediet/vscode-drawio.svg?style=flat&label=Star&maxAge=86400)  - This extension integrates Draw.io into VS Code

### Safety
* [awesome-safety-critical](https://github.com/stanislaw/awesome-safety-critical) ![awesome-safety-critical](https://img.shields.io/github/stars/stanislaw/awesome-safety-critical.svg?style=flat&label=Star&maxAge=86400)  - List of resources about programming practices for writing safety-critical software.
* [open-autonomous-safety](https://github.com/voyage/open-autonomous-safety) ![open-autonomous-safety](https://img.shields.io/github/stars/voyage/open-autonomous-safety.svg?style=flat&label=Star&maxAge=86400)  - OAS is a fully open-source library of Voyage’s safety processes and testing procedures, designed to supplement existing safety programs at self-driving car startups across the world.
* [CarND-Functional-Safety-Project](https://github.com/udacity/CarND-Functional-Safety-Project) ![CarND-Functional-Safety-Project](https://img.shields.io/github/stars/udacity/CarND-Functional-Safety-Project.svg?style=flat&label=Star&maxAge=86400)  - Create functional safety documents in this Udacity project
* [Automated Valet Parking Safety Documents](https://avp-project.uk/publication-of-safety-documents) - created to support the safe testing of the Automated Valet Parking function using the StreetDrone test vehicle in a car park.
* [safe_numerics](https://github.com/boostorg/safe_numerics) ![safe_numerics](https://img.shields.io/github/stars/boostorg/safe_numerics.svg?style=flat&label=Star&maxAge=86400)  - Replacements to standard numeric types which throw exceptions on errors
* [Air Vehicle C++ development coding standards](http://www.stroustrup.com/JSF-AV-rules.pdf) - Provide direction and guidance to C++ programmers that will enable them to employ good programming style and proven programming practices leading to safe, reliable, testable, and maintainable code
* [AUTOSAR Coding Standard](https://www.autosar.org/fileadmin/user_upload/standards/adaptive/17-10/AUTOSAR_RS_CPP14Guidelines.pdf) - Guidelines for the use of the C++14 language in critical and safety-related system
* [The W-Model and Lean Scaled Agility for Engineering](https://assets.vector.com/cms/content/consulting/publications/AgileSystemsEngineering_Vector_Ford.pdf) - Ford applied an agile V-Model method from Vector that can be used in safety related project management

## Framework
* [ROS](https://github.com/ros) - (Robot Operating System) provides libraries and tools to help software developers create robot applications
* [awesome-ros2](https://github.com/fkromer/awesome-ros2) ![awesome-ros2](https://img.shields.io/github/stars/fkromer/awesome-ros2.svg?style=flat&label=Star&maxAge=86400)  - A curated list of awesome Robot Operating System Version 2.0 (ROS 2) resources and libraries.
* [OpenPilot](https://github.com/commaai/openpilot) ![openpilot](https://img.shields.io/github/stars/commaai/openpilot.svg?style=flat&label=Star&maxAge=86400)  - Open Source Adaptive Cruise Control (ACC) and Lane Keeping Assist System (LKAS)
* [Apollo](https://github.com/ApolloAuto/apollo) ![apollo](https://img.shields.io/github/stars/ApolloAuto/apollo.svg?style=flat&label=Star&maxAge=86400)  - High performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.
* [Autoware.ai](https://gitlab.com/autowarefoundation/autoware.ai) - Autoware.AI is the world's first "All-in-One" open-source software for autonomous driving technology
* [AutowareAuto](https://autowareauto.gitlab.io/AutowareAuto/) - It is a clean slate rewrite of Autoware. Autoware.Auto applies best-in-class software engineering.
* [PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics/) ![](https://img.shields.io/github/stars/AtsushiSakai/PythonRobotics/.svg?style=flat&label=Star&maxAge=86400)  - This is a Python code collection of robotics algorithms, especially for autonomous navigation.
* [Stanford Self Driving Car Code](https://github.com/emmjaykay/stanford_self_driving_car_code) ![stanford_self_driving_car_code](https://img.shields.io/github/stars/emmjaykay/stanford_self_driving_car_code.svg?style=flat&label=Star&maxAge=86400)  - Stanford Code From Cars That Entered DARPA Grand Challenges
* [astrobee](https://github.com/nasa/astrobee) ![astrobee](https://img.shields.io/github/stars/nasa/astrobee.svg?style=flat&label=Star&maxAge=86400)  - Astrobee is a free-flying robot designed to operate as a payload inside the International Space Station (ISS).
* [CARMAPlatform](https://github.com/usdot-fhwa-stol/CARMAPlatform) ![CARMAPlatform](https://img.shields.io/github/stars/usdot-fhwa-stol/CARMAPlatform.svg?style=flat&label=Star&maxAge=86400)  - enables cooperative automated driving plug-in
* [Automotive Grade Linux](https://www.automotivelinux.org/) - Automotive Grade Linux is a collaborative open source project that is bringing together automakers, suppliers and technology companies to accelerate the development and adoption of a fully open software stack for the connected car
* [PX4](https://github.com/PX4/Firmware) ![Firmware](https://img.shields.io/github/stars/PX4/Firmware.svg?style=flat&label=Star&maxAge=86400)  - is an open source flight control software for drones and other unmanned vehicles

## Development Environment
### Code and Run
* [Vim-ros](https://github.com/taketwo/vim-ros) ![vim-ros](https://img.shields.io/github/stars/taketwo/vim-ros.svg?style=flat&label=Star&maxAge=86400)  - Vim plugin for ROS development
* [Visual Studio Code](https://github.com/Microsoft/vscode) ![vscode](https://img.shields.io/github/stars/Microsoft/vscode.svg?style=flat&label=Star&maxAge=86400)  - Code editor for edit-build-debug cycle.
* [atom](https://github.com/atom/atom) ![atom](https://img.shields.io/github/stars/atom/atom.svg?style=flat&label=Star&maxAge=86400)  - Hackable text editor for the 21st century
* [Teletype](https://github.com/atom/teletype) ![teletype](https://img.shields.io/github/stars/atom/teletype.svg?style=flat&label=Star&maxAge=86400)  - Share your workspace with team members and collaborate on code in real time in Atom
* [Sublime](https://www.sublimetext.com/) - A sophisticated text editor for code, markup and prose
* [ade-cli](https://gitlab.com/ApexAI/ade-cli) - The ADE Development Environment (ADE) uses docker and Gitlab to manage environments of per project development tools and optional volume images
* [recipe-wizard](https://github.com/trn84/recipe-wizard) ![recipe-wizard](https://img.shields.io/github/stars/trn84/recipe-wizard.svg?style=flat&label=Star&maxAge=86400)  - A Dockerfile generator for running OpenGL (GLX) applications with nvidia-docker2, CUDA, ROS, and Gazebo on a remote headless server system
* [Jupyter ROS](https://github.com/RoboStack/jupyter-ros) ![jupyter-ros](https://img.shields.io/github/stars/RoboStack/jupyter-ros.svg?style=flat&label=Star&maxAge=86400)  - Jupyter widget helpers for ROS, the Robot Operating System
* [ros_rqt_plugin](https://github.com/ros-industrial/ros_qtc_plugin) ![ros_qtc_plugin](https://img.shields.io/github/stars/ros-industrial/ros_qtc_plugin.svg?style=flat&label=Star&maxAge=86400)  - The ROS Qt Creator Plug-in for Python
* [xeus-cling](https://github.com/QuantStack/xeus-cling) ![xeus-cling](https://img.shields.io/github/stars/QuantStack/xeus-cling.svg?style=flat&label=Star&maxAge=86400)  - Jupyter kernel for the C++ programming language
* [ROS IDEs](http://wiki.ros.org/IDEs) - This page collects experience and advice on using integrated development environments (IDEs) with ROS.
* [TabNine](https://github.com/zxqfl/TabNine) ![TabNine](https://img.shields.io/github/stars/zxqfl/TabNine.svg?style=flat&label=Star&maxAge=86400)  - The all-language autocompleter
* [kite](https://kite.com/) - Use machine learning to give you
 useful code completions for Python
* [jedi](https://github.com/davidhalter/jedi) ![jedi](https://img.shields.io/github/stars/davidhalter/jedi.svg?style=flat&label=Star&maxAge=86400)  - Autocompletion and static analysis library for python
* [roslibpy](https://github.com/gramaziokohler/roslibpy) ![roslibpy](https://img.shields.io/github/stars/gramaziokohler/roslibpy.svg?style=flat&label=Star&maxAge=86400)  - Python ROS Bridge library allows to use Python and IronPython to interact with ROS, the open-source robotic middleware.
* [pybind11](https://github.com/pybind/pybind11) ![pybind11](https://img.shields.io/github/stars/pybind/pybind11.svg?style=flat&label=Star&maxAge=86400)  - Seamless operability between C++11 and Python
* [Sourcetrail](https://github.com/CoatiSoftware/Sourcetrail) ![Sourcetrail](https://img.shields.io/github/stars/CoatiSoftware/Sourcetrail.svg?style=flat&label=Star&maxAge=86400)  - free and open-source cross-platform source explorer
* [rebound](https://github.com/shobrook/rebound) ![rebound](https://img.shields.io/github/stars/shobrook/rebound.svg?style=flat&label=Star&maxAge=86400)  - Command-line tool that instantly fetches Stack Overflow results when an exception is thrown
* [mybinder](https://mybinder.org/) - open notebooks in an executable environment, making your code immediately reproducible by anyone, anywhere.
* [ROSOnWindows](https://ms-iot.github.io/ROSOnWindows/) - an experimental release of ROS1 for Windows
* [live-share](https://github.com/MicrosoftDocs/live-share) ![live-share](https://img.shields.io/github/stars/MicrosoftDocs/live-share.svg?style=flat&label=Star&maxAge=86400)  - Real-time collaborative development from the comfort of your favorite tools
* [cocalc](https://github.com/sagemathinc/cocalc) ![cocalc](https://img.shields.io/github/stars/sagemathinc/cocalc.svg?style=flat&label=Star&maxAge=86400)  - Collaborative Calculation in the Cloud
* [EasyClangComplete](https://github.com/niosus/EasyClangComplete) ![EasyClangComplete](https://img.shields.io/github/stars/niosus/EasyClangComplete.svg?style=flat&label=Star&maxAge=86400)  - Robust C/C++ code completion for Sublime Text 3

### Template
* [ROS](https://github.com/leggedrobotics/ros_best_practices/tree/master/ros_package_template) - Template for ROS node standardization in C++
* [Launch](https://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20tips%20for%20larger%20projects) - Templates on how to create launch files for larger projects
* [Bash](https://github.com/ralish/bash-script-template) ![bash-script-template](https://img.shields.io/github/stars/ralish/bash-script-template.svg?style=flat&label=Star&maxAge=86400)  - A bash scripting template incorporating best practices & several useful functions
* [URDF](https://wiki.ros.org/urdf/Examples) - Examples on how to create Unified Robot Description Format (URDF) for different kinds of robots
* [Python](http://wiki.ros.org/PyStyleGuide) - Style guide to be followed in writing Python code for ROS
* [Docker](https://ade-cli.readthedocs.io/en/latest/create-custom-base-image.html) - The Dockerfile in the minimal-ade project shows a minimal example of how to create a custom base image

### Build and Deploy
* [qemu-user-static](https://github.com/multiarch/qemu-user-static) ![qemu-user-static](https://img.shields.io/github/stars/multiarch/qemu-user-static.svg?style=flat&label=Star&maxAge=86400)  - is to enable an execution of different multi-architecture containers by QEMU and binfmt_misc
* [Cross compile ROS 2 on QNX](https://gitlab.apex.ai/snippets/97) -  introduces how to cross compile ROS 2 on QNX
* [bloom](https://github.com/ros-infrastructure/bloom) ![bloom](https://img.shields.io/github/stars/ros-infrastructure/bloom.svg?style=flat&label=Star&maxAge=86400)  - A release automation tool which makes releasing catkin packages easier
* [superflore](https://github.com/ros-infrastructure/superflore) ![superflore](https://img.shields.io/github/stars/ros-infrastructure/superflore.svg?style=flat&label=Star&maxAge=86400)  - An extended platform release manager for Robot Operating System
* [catkin_tools](https://github.com/catkin/catkin_tools) ![catkin_tools](https://img.shields.io/github/stars/catkin/catkin_tools.svg?style=flat&label=Star&maxAge=86400)  - Command line tools for working with catkin
* [industrial_ci](https://github.com/ros-industrial/industrial_ci) ![industrial_ci](https://img.shields.io/github/stars/ros-industrial/industrial_ci.svg?style=flat&label=Star&maxAge=86400)  - Easy continuous integration repository for ROS repositories
* [ros_gitlab_ci](https://gitlab.com/VictorLamoine/ros_gitlab_ci) - contains helper scripts and instructions on how to use Continuous Integration (CI) for ROS projects hosted on a GitLab instance.
* [gitlab-runner](https://gitlab.com/gitlab-org/gitlab-runner) -  runs tests and sends the results to GitLab
* [colcon-core](https://github.com/colcon/colcon-core) ![colcon-core](https://img.shields.io/github/stars/colcon/colcon-core.svg?style=flat&label=Star&maxAge=86400)  - command line tool to improve the workflow of building, testing and using multiple software packages
* [gitlab-release](https://gitlab.com/alelec/gitlab-release) - Simple python3 script to upload files (from ci) to the current projects release (tag)
* [clang](https://github.com/llvm-mirror/clang) ![clang](https://img.shields.io/github/stars/llvm-mirror/clang.svg?style=flat&label=Star&maxAge=86400)  -  This is a compiler front-end for the C family of languages
(C, C++, Objective-C, and Objective-C++) which is built as part of the LLVM
compiler infrastructure project
* [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv) ![catkin_virtualenv](https://img.shields.io/github/stars/locusrobotics/catkin_virtualenv.svg?style=flat&label=Star&maxAge=86400)  - Bundle python requirements in a catkin package via virtualenv
* [pyenv](https://github.com/pyenv/pyenv) ![pyenv](https://img.shields.io/github/stars/pyenv/pyenv.svg?style=flat&label=Star&maxAge=86400)  - Simple Python version management
* [aptly](https://github.com/aptly-dev/aptly) ![aptly](https://img.shields.io/github/stars/aptly-dev/aptly.svg?style=flat&label=Star&maxAge=86400)  - Debian repository management tool
* [cross_compile](https://github.com/ros-tooling/cross_compile) ![cross_compile](https://img.shields.io/github/stars/ros-tooling/cross_compile.svg?style=flat&label=Star&maxAge=86400)  - Assets used for ROS2 cross-compilation
* [docker_images](https://github.com/osrf/docker_images) ![docker_images](https://img.shields.io/github/stars/osrf/docker_images.svg?style=flat&label=Star&maxAge=86400)  - Official Docker images maintained by OSRF on ROS(2) and Gazebo
* [robot_upstart](https://github.com/clearpathrobotics/robot_upstart) ![robot_upstart](https://img.shields.io/github/stars/clearpathrobotics/robot_upstart.svg?style=flat&label=Star&maxAge=86400)  - presents a suite of scripts to assist with launching background ROS processes on Ubuntu Linux PCs
* [robot_systemd](http://docs.ros.org/kinetic/api/robot_systemd/html/#) - units for managing startup and shutdown of roscore and roslaunch
* [ryo-iso](https://ryo-iso.readthedocs.io/en/latest/) - is a modern ISO builder that streamlines the process of deploying a complete robot operating system from a yaml config file
* [network_autoconfig](http://docs.ros.org/kinetic/api/network_autoconfig/html/) - automatic configuration of ROS networking for most use cases without impacting usage that require manual configuration.
* [rosbuild](https://roscon.ros.org/2016/presentations/ROSCon2016%20Build%20Farm.pdf) - The ROS build farm

### Unit and Integration Test
* [setup-ros](https://github.com/ros-tooling/setup-ros) ![setup-ros](https://img.shields.io/github/stars/ros-tooling/setup-ros.svg?style=flat&label=Star&maxAge=86400)  - This action sets up a ROS and ROS 2 environment for use in Github actions
* [UnitTesting](https://wiki.ros.org/Quality/Tutorials/UnitTesting) - This page lays out the rationale, best practices, and policies for writing and running unit tests and integration tests for ROS.
* [googletest](https://github.com/google/googletest) ![googletest](https://img.shields.io/github/stars/google/googletest.svg?style=flat&label=Star&maxAge=86400)  - Google's C++ test framework
* [pytest](https://github.com/pytest-dev/pytest/) ![](https://img.shields.io/github/stars/pytest-dev/pytest/.svg?style=flat&label=Star&maxAge=86400)  - The pytest framework makes it easy to write small tests, yet scales to support complex functional testing
* [doctest](https://github.com/onqtam/doctest) ![doctest](https://img.shields.io/github/stars/onqtam/doctest.svg?style=flat&label=Star&maxAge=86400)  - The fastest feature-rich C++11/14/17/20 single-header testing framework for unit tests and TDD
* [osrf_testing_tools_cpp](https://github.com/osrf/osrf_testing_tools_cpp) ![osrf_testing_tools_cpp](https://img.shields.io/github/stars/osrf/osrf_testing_tools_cpp.svg?style=flat&label=Star&maxAge=86400)  - contains testing tools for C++, and is used in OSRF projects.
* [code_coverage](https://github.com/mikeferguson/code_coverage) ![code_coverage](https://img.shields.io/github/stars/mikeferguson/code_coverage.svg?style=flat&label=Star&maxAge=86400)  - ROS package to run coverage testing

### Lint and Format
* [action-ros-lint](https://github.com/ros-tooling/action-ros-lint) ![action-ros-lint](https://img.shields.io/github/stars/ros-tooling/action-ros-lint.svg?style=flat&label=Star&maxAge=86400)  - Github action to run linters on ROS 2 packages
* [cppcheck](https://github.com/danmar/cppcheck) ![cppcheck](https://img.shields.io/github/stars/danmar/cppcheck.svg?style=flat&label=Star&maxAge=86400)  - Static analysis of C/C++ code
* [hadolint](https://github.com/hadolint/hadolint) ![hadolint](https://img.shields.io/github/stars/hadolint/hadolint.svg?style=flat&label=Star&maxAge=86400)  - Dockerfile linter, validate inline bash, written in Haskell
* [shellcheck](https://github.com/koalaman/shellcheck) ![shellcheck](https://img.shields.io/github/stars/koalaman/shellcheck.svg?style=flat&label=Star&maxAge=86400)  - a static analysis tool for shell scripts
* [catkin_lint](https://github.com/fkie/catkin_lint) ![catkin_lint](https://img.shields.io/github/stars/fkie/catkin_lint.svg?style=flat&label=Star&maxAge=86400)  - catkin_lint checks package configurations for the catkin build system of ROS.
* [pylint](https://github.com/PyCQA/pylint/) ![](https://img.shields.io/github/stars/PyCQA/pylint/.svg?style=flat&label=Star&maxAge=86400)  - Pylint is a Python static code analysis tool which looks for programming errors, helps enforcing a coding standard, sniffs for code smells and offers simple refactoring suggestions.
* [black](https://github.com/psf/black) ![black](https://img.shields.io/github/stars/psf/black.svg?style=flat&label=Star&maxAge=86400)  - The uncompromising Python code formatter
* [pydocstyle](https://github.com/PyCQA/pydocstyle) ![pydocstyle](https://img.shields.io/github/stars/PyCQA/pydocstyle.svg?style=flat&label=Star&maxAge=86400)  - pydocstyle is a static analysis tool for checking compliance with Python docstring conventions
* [haros](https://github.com/git-afsantos/haros) ![haros](https://img.shields.io/github/stars/git-afsantos/haros.svg?style=flat&label=Star&maxAge=86400)  - H(igh) A(ssurance) ROS - Static analysis of ROS application code.

### Launch and Monitor
* [rosmon](https://github.com/xqms/rosmon) ![rosmon](https://img.shields.io/github/stars/xqms/rosmon.svg?style=flat&label=Star&maxAge=86400)  - ROS node launcher & monitoring daemon
* [multimaster_fkie](https://github.com/fkie/multimaster_fkie) ![multimaster_fkie](https://img.shields.io/github/stars/fkie/multimaster_fkie.svg?style=flat&label=Star&maxAge=86400)  - GUI-based management environment that is very useful to manage ROS-launch configurations and control running nodes

### Debugging and Tracing
* [heaptrack](https://github.com/KDE/heaptrack) ![heaptrack](https://img.shields.io/github/stars/KDE/heaptrack.svg?style=flat&label=Star&maxAge=86400)  - traces all memory allocations and annotates these events with stack traces
* [ros2_tracing](https://gitlab.com/micro-ROS/ros_tracing/ros2_tracing) - Tracing tools for ROS 2.
* [Linuxperf](http://www.brendangregg.com/linuxperf.html) - Various Linux performance material
* [lptrace](https://github.com/khamidou/lptrace) ![lptrace](https://img.shields.io/github/stars/khamidou/lptrace.svg?style=flat&label=Star&maxAge=86400)  - It lets you see in real-time what functions a Python program is running
* [pyre-check](https://github.com/facebook/pyre-check) ![pyre-check](https://img.shields.io/github/stars/facebook/pyre-check.svg?style=flat&label=Star&maxAge=86400)  - Performant type-checking for python
* [FlameGraph](https://github.com/brendangregg/FlameGraph) ![FlameGraph](https://img.shields.io/github/stars/brendangregg/FlameGraph.svg?style=flat&label=Star&maxAge=86400)  - Visualize profiled code
* [gpuvis](https://github.com/mikesart/gpuvis) ![gpuvis](https://img.shields.io/github/stars/mikesart/gpuvis.svg?style=flat&label=Star&maxAge=86400)  - GPU Trace Visualizer
* [sanitizer](https://github.com/google/sanitizers) ![sanitizers](https://img.shields.io/github/stars/google/sanitizers.svg?style=flat&label=Star&maxAge=86400)  - AddressSanitizer, ThreadSanitizer, MemorySanitizer
* [cppinsights](https://github.com/andreasfertig/cppinsights) ![cppinsights](https://img.shields.io/github/stars/andreasfertig/cppinsights.svg?style=flat&label=Star&maxAge=86400)  - C++ Insights - See your source code with the eyes of a compiler
* [inspect](https://pymotw.com/2/inspect/) - The inspect module provides functions for learning about live objects, including modules, classes, instances, functions, and methods
* [Roslaunch Nodes in Valgrind or GDB](https://wiki.ros.org/roslaunch/Tutorials/Roslaunch%20Nodes%20in%20Valgrind%20or%20GDB) - When debugging roscpp nodes that you are launching with roslaunch, you may wish to launch the node in a debugging program like gdb or valgrind instead.
* [pyperformance](https://github.com/python/pyperformance) ![pyperformance](https://img.shields.io/github/stars/python/pyperformance.svg?style=flat&label=Star&maxAge=86400)  - Python Performance Benchmark Suite
* [qira](https://github.com/geohot/qira) ![qira](https://img.shields.io/github/stars/geohot/qira.svg?style=flat&label=Star&maxAge=86400)  - QIRA is a competitor to strace and gdb
* [gdb-frontend](https://github.com/rohanrhu/gdb-frontend) ![gdb-frontend](https://img.shields.io/github/stars/rohanrhu/gdb-frontend.svg?style=flat&label=Star&maxAge=86400)  - GDBFrontend is an easy, flexible and extensionable gui debugger.
* [lttng](https://lttng.org/docs/) - is an open source software toolkit which you can use to simultaneously trace the Linux kernel, user applications, and user libraries.
* [ros2-performance](https://github.com/irobot-ros/ros2-performance) ![ros2-performance](https://img.shields.io/github/stars/irobot-ros/ros2-performance.svg?style=flat&label=Star&maxAge=86400)  - allows to easily create arbitrary ROS2 systems and then measures their performance
* [bcc](https://github.com/iovisor/bcc) ![bcc](https://img.shields.io/github/stars/iovisor/bcc.svg?style=flat&label=Star&maxAge=86400)  - Tools for BPF-based Linux IO analysis, networking, monitoring, and more
* [tracy](https://github.com/wolfpld/tracy) ![tracy](https://img.shields.io/github/stars/wolfpld/tracy.svg?style=flat&label=Star&maxAge=86400)  - A real time, nanosecond resolution, remote telemetry frame profiler for games and other applications.

### Version Control
* [git-fuzzy](https://github.com/bigH/git-fuzzy) ![git-fuzzy](https://img.shields.io/github/stars/bigH/git-fuzzy.svg?style=flat&label=Star&maxAge=86400)  - A CLI interface to git that relies heavily on fzf
* [meld](https://github.com/kaiw/meld) ![meld](https://img.shields.io/github/stars/kaiw/meld.svg?style=flat&label=Star&maxAge=86400)  - Meld is a visual diff and merge tool that helps you compare files, directories, and version controlled projects
* [tig](https://github.com/jonas/tig) ![tig](https://img.shields.io/github/stars/jonas/tig.svg?style=flat&label=Star&maxAge=86400)  - Text-mode interface for git
* [gitg](https://github.com/GNOME/gitg) ![gitg](https://img.shields.io/github/stars/GNOME/gitg.svg?style=flat&label=Star&maxAge=86400)  - is a graphical user interface for git
* [git-cola](https://github.com/git-cola/git-cola) ![git-cola](https://img.shields.io/github/stars/git-cola/git-cola.svg?style=flat&label=Star&maxAge=86400)  - The highly caffeinated Git GUI
* [python-gitlab](https://github.com/python-gitlab/python-gitlab) ![python-gitlab](https://img.shields.io/github/stars/python-gitlab/python-gitlab.svg?style=flat&label=Star&maxAge=86400)  - is a Python package providing access to the GitLab server API.
* [bfg-repo-cleaner](https://github.com/rtyley/bfg-repo-cleaner) ![bfg-repo-cleaner](https://img.shields.io/github/stars/rtyley/bfg-repo-cleaner.svg?style=flat&label=Star&maxAge=86400)  - Removes large or troublesome blobs like git-filter-branch does, but faster.
* [nbdime](https://github.com/jupyter/nbdime) ![nbdime](https://img.shields.io/github/stars/jupyter/nbdime.svg?style=flat&label=Star&maxAge=86400)  - Tools for diffing and merging of Jupyter notebooks.
* [semantic-release](https://github.com/semantic-release/semantic-release) ![semantic-release](https://img.shields.io/github/stars/semantic-release/semantic-release.svg?style=flat&label=Star&maxAge=86400)  - Fully automated version management and package publishing
* [go-semrel-gitab](https://gitlab.com/juhani/go-semrel-gitlab) - Automate version management for Gitlab
* [Git-repo](https://gerrit.googlesource.com/git-repo/) - Git-Repo helps manage many Git repositories, does the uploads to revision control systems, and automates parts of the development workflow
* [dive](https://github.com/wagoodman/dive) ![dive](https://img.shields.io/github/stars/wagoodman/dive.svg?style=flat&label=Star&maxAge=86400)  - A tool for exploring each layer in a docker image
* [dvc](https://github.com/iterative/dvc) ![dvc](https://img.shields.io/github/stars/iterative/dvc.svg?style=flat&label=Star&maxAge=86400)  - Management and versioning of datasets and machine learning models

### Simulation
* [Drake](https://github.com/RobotLocomotion/drake) ![drake](https://img.shields.io/github/stars/RobotLocomotion/drake.svg?style=flat&label=Star&maxAge=86400)  - Drake aims to simulate even very complex dynamics of robots
* [Webots](https://github.com/cyberbotics/webots) ![webots](https://img.shields.io/github/stars/cyberbotics/webots.svg?style=flat&label=Star&maxAge=86400)  - Webots is an open source robot simulator compatible (among others) with [ROS](http://wiki.ros.org/webots_ros) and [ROS2](http://wiki.ros.org/webots_ros2).
* [lgsv](https://github.com/lgsvl/simulator) ![simulator](https://img.shields.io/github/stars/lgsvl/simulator.svg?style=flat&label=Star&maxAge=86400)  - LG Electronics America R&D Center has developed an HDRP Unity-based multi-robot simulator for autonomous vehicle developers.
* [carla](https://github.com/carla-simulator/carla) ![carla](https://img.shields.io/github/stars/carla-simulator/carla.svg?style=flat&label=Star&maxAge=86400)  - Open-source simulator for autonomous driving research
* [awesome-CARLA](https://github.com/Amin-Tgz/awesome-CARLA) ![awesome-CARLA](https://img.shields.io/github/stars/Amin-Tgz/awesome-CARLA.svg?style=flat&label=Star&maxAge=86400)  - A curated list of awesome CARLA tutorials, blogs, and related projects.
* [scenario_runner](https://github.com/carla-simulator/scenario_runner) ![scenario_runner](https://img.shields.io/github/stars/carla-simulator/scenario_runner.svg?style=flat&label=Star&maxAge=86400)  - Traffic scenario definition and execution engine
* [deepdive](https://github.com/deepdrive/deepdrive) ![deepdrive](https://img.shields.io/github/stars/deepdrive/deepdrive.svg?style=flat&label=Star&maxAge=86400)  - End-to-end simulation for self-driving cars
* [uuv_simulator](https://github.com/uuvsimulator/uuv_simulator) ![uuv_simulator](https://img.shields.io/github/stars/uuvsimulator/uuv_simulator.svg?style=flat&label=Star&maxAge=86400)  - Gazebo/ROS packages for underwater robotics simulation
* [AirSim](https://github.com/microsoft/AirSim) ![AirSim](https://img.shields.io/github/stars/microsoft/AirSim.svg?style=flat&label=Star&maxAge=86400)  - Open source simulator for autonomous vehicles built on Unreal Engine
* [self-driving-car-sim](https://github.com/udacity/self-driving-car-sim) ![self-driving-car-sim](https://img.shields.io/github/stars/udacity/self-driving-car-sim.svg?style=flat&label=Star&maxAge=86400)  - A self-driving car simulator built with Unity
* [ROSIntegration](https://github.com/code-iai/ROSIntegration) ![ROSIntegration](https://img.shields.io/github/stars/code-iai/ROSIntegration.svg?style=flat&label=Star&maxAge=86400)  - Unreal Engine Plugin to enable ROS Support
* [gym-gazebo](https://github.com/erlerobot/gym-gazebo) ![gym-gazebo](https://img.shields.io/github/stars/erlerobot/gym-gazebo.svg?style=flat&label=Star&maxAge=86400)  - An OpenAI gym extension for using Gazebo known as gym-gazebo
* [highway-env](https://github.com/eleurent/highway-env) ![highway-env](https://img.shields.io/github/stars/eleurent/highway-env.svg?style=flat&label=Star&maxAge=86400)   - A collection of environments for autonomous driving and tactical decision-making tasks
* [VREP Interface](http://www.coppeliarobotics.com/helpFiles/en/rosInterf.htm) - ROS Bridge for the VREP simulator
* [car_demo](https://github.com/osrf/car_demo) ![car_demo](https://img.shields.io/github/stars/osrf/car_demo.svg?style=flat&label=Star&maxAge=86400)  - This is a simulation of a Prius in gazebo 9 with sensor data being published using ROS kinetic.
* [sumo](https://github.com/eclipse/sumo) ![sumo](https://img.shields.io/github/stars/eclipse/sumo.svg?style=flat&label=Star&maxAge=86400)  - Eclipse SUMO is an open source, highly portable, microscopic and continuous road traffic simulation package designed to handle large road networks
* [open-simulation-interface](https://github.com/OpenSimulationInterface/open-simulation-interface) ![open-simulation-interface](https://img.shields.io/github/stars/OpenSimulationInterface/open-simulation-interface.svg?style=flat&label=Star&maxAge=86400)  - A generic interface for the environmental perception of automated driving functions in virtual scenarios.
* [ESIM](https://github.com/uzh-rpg/rpg_esim/) ![](https://img.shields.io/github/stars/uzh-rpg/rpg_esim/.svg?style=flat&label=Star&maxAge=86400)  - an Open Event Camera Simulator
* [Menge](https://github.com/MengeCrowdSim/Menge) ![Menge](https://img.shields.io/github/stars/MengeCrowdSim/Menge.svg?style=flat&label=Star&maxAge=86400)  - Crowd Simulation Framework
* [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) ![pedsim_ros](https://img.shields.io/github/stars/srl-freiburg/pedsim_ros.svg?style=flat&label=Star&maxAge=86400)  - Pedestrian simulator powered by the social force model for Gazebo
* [opencrg](http://www.opencrg.org/download.html) -  open file formats and open source tools for the detailed description, creation and evaluation of road surfaces
* [esmini](https://github.com/esmini/esmini) ![esmini](https://img.shields.io/github/stars/esmini/esmini.svg?style=flat&label=Star&maxAge=86400)  -  is a basic OpenSCENARIO player
* [OpenSceneGraph](https://github.com/openscenegraph/OpenSceneGraph) ![OpenSceneGraph](https://img.shields.io/github/stars/openscenegraph/OpenSceneGraph.svg?style=flat&label=Star&maxAge=86400)  - is an open source high performance 3D graphics toolkit, used by application developers in fields such as visual simulation, games, virtual reality, scientific visualization and modelling
* [morse](https://github.com/morse-simulator) ![morse-simulator](https://img.shields.io/github/stars/morse-simulator.svg?style=flat&label=Star&maxAge=86400)  - is an academic robotic simulator, based on the Blender Game Engine and the Bullet Physics engine.
* [ROSIntegrationVision](https://github.com/code-iai/ROSIntegrationVision) ![ROSIntegrationVision](https://img.shields.io/github/stars/code-iai/ROSIntegrationVision.svg?style=flat&label=Star&maxAge=86400)  - Support for ROS-enabled RGBD data acquisition in Unreal Engine Projects
* [fetch_gazebo](https://github.com/fetchrobotics/fetch_gazebo) ![fetch_gazebo](https://img.shields.io/github/stars/fetchrobotics/fetch_gazebo.svg?style=flat&label=Star&maxAge=86400)  - contains the Gazebo simulation for Fetch Robotics Fetch and Freight Research Edition Robots.
* [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) ![rotors_simulator](https://img.shields.io/github/stars/ethz-asl/rotors_simulator.svg?style=flat&label=Star&maxAge=86400)  - provides some multirotor models
* [flow](https://github.com/flow-project/flow) ![flow](https://img.shields.io/github/stars/flow-project/flow.svg?style=flat&label=Star&maxAge=86400)  - is a computational framework for deep RL and control experiments for traffic microsimulation.
* [gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) ![gnss-ins-sim](https://img.shields.io/github/stars/Aceinna/gnss-ins-sim.svg?style=flat&label=Star&maxAge=86400)  - GNSS + inertial navigation, sensor fusion simulator. Motion trajectory generator, sensor models, and navigation
* [Ignition Robotics](https://ignitionrobotics.org) -  Test control strategies in safety, and take advantage of simulation in continuous integration tests.
* [simulation assets for the SubT](https://subtchallenge.world/openrobotics/fuel/collections/SubT%20Tech%20Repo) - This collection contains simulation assets for the SubT Challenge Virtual Competition in Gazebo


## Hardware
* [HRIM](https://github.com/AcutronicRobotics/HRIM) ![HRIM](https://img.shields.io/github/stars/AcutronicRobotics/HRIM.svg?style=flat&label=Star&maxAge=86400)  - An information model for robot hardware
* [URDF](https://github.com/ros/urdf) ![urdf](https://img.shields.io/github/stars/ros/urdf.svg?style=flat&label=Star&maxAge=86400)  - Repository for Unified Robot Description Format (URDF) parsing code
* [phobos](https://github.com/dfki-ric/phobos) ![phobos](https://img.shields.io/github/stars/dfki-ric/phobos.svg?style=flat&label=Star&maxAge=86400)  - An add-on for Blender allowing to create URDF, SDF and SMURF robot models in a WYSIWYG environment.
* [urdf-viz](https://github.com/OTL/urdf-viz) ![urdf-viz](https://img.shields.io/github/stars/OTL/urdf-viz.svg?style=flat&label=Star&maxAge=86400)  - Visualize URDF/XACRO file, URDF Viewer works on Windows/MacOS/Linux
* [solidworks_urdf_exporter](https://github.com/ros/solidworks_urdf_exporter) ![solidworks_urdf_exporter](https://img.shields.io/github/stars/ros/solidworks_urdf_exporter.svg?style=flat&label=Star&maxAge=86400)  - SolidWorks to URDF Exporter
* [FreeCAD](https://github.com/FreeCAD/FreeCAD) ![FreeCAD](https://img.shields.io/github/stars/FreeCAD/FreeCAD.svg?style=flat&label=Star&maxAge=86400)  - Your own 3D parametric modeler
* [kicad](http://www.kicad-pcb.org/) - A Cross Platform and Open Source Electronics Design Automation Suite
* [PcbDraw](https://github.com/yaqwsx/PcbDraw) ![PcbDraw](https://img.shields.io/github/stars/yaqwsx/PcbDraw.svg?style=flat&label=Star&maxAge=86400)  - Convert your KiCAD board into a nice looking 2D drawing suitable for pinout diagrams
* [PandaPower](http://www.pandapower.org) - An easy to use open source tool for power system modeling, analysis and optimization with a high degree of automation.
* [LibrePCB](https://github.com/LibrePCB/LibrePCB) ![LibrePCB](https://img.shields.io/github/stars/LibrePCB/LibrePCB.svg?style=flat&label=Star&maxAge=86400)  - A powerful, innovative and intuitive EDA tool for everyone
* [openscad](https://github.com/openscad/openscad) ![openscad](https://img.shields.io/github/stars/openscad/openscad.svg?style=flat&label=Star&maxAge=86400)  -  is software for creating solid 3D CAD models
* [ngspice](http://ngspice.sourceforge.net/) - is the open source spice simulator for electric and electronic circuits.
* [GNSS-SDR](https://github.com/gnss-sdr/gnss-sdr) ![gnss-sdr](https://img.shields.io/github/stars/gnss-sdr/gnss-sdr.svg?style=flat&label=Star&maxAge=86400)  - GNSS-SDR provides interfaces for a wide range of radio frequency front-ends and raw sample file formats, generates processing outputs in standard formats
* [riscv](https://riscv.org) - The Free and Open RISC Instruction Set Architecture
* [urdfpy](https://github.com/mmatl/urdfpy) ![urdfpy](https://img.shields.io/github/stars/mmatl/urdfpy.svg?style=flat&label=Star&maxAge=86400)  - is a simple and easy-to-use library for loading, manipulating, saving, and visualizing URDF files.
* [FMPy](https://github.com/CATIA-Systems/FMPy) ![FMPy](https://img.shields.io/github/stars/CATIA-Systems/FMPy.svg?style=flat&label=Star&maxAge=86400)  - Simulate Functional Mockup Units (FMUs) in Python
* [FMIKit-Simulink](https://github.com/CATIA-Systems/FMIKit-Simulink) ![FMIKit-Simulink](https://img.shields.io/github/stars/CATIA-Systems/FMIKit-Simulink.svg?style=flat&label=Star&maxAge=86400)  - Import and export Functional Mock-up Units with Simulink
* [oemof-solph](https://github.com/oemof/oemof-solph) ![oemof-solph](https://img.shields.io/github/stars/oemof/oemof-solph.svg?style=flat&label=Star&maxAge=86400)  - A modular open source framework to model energy supply systems


### Calibration and Transformation
* [tf2](http://wiki.ros.org/tf2) - transform library, which lets the user keep track of multiple coordinate frames over time
* [lidar_align](https://github.com/ethz-asl/lidar_align) ![lidar_align](https://img.shields.io/github/stars/ethz-asl/lidar_align.svg?style=flat&label=Star&maxAge=86400)  - A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor
* [kalibr](https://github.com/ethz-asl/kalibr) ![kalibr](https://img.shields.io/github/stars/ethz-asl/kalibr.svg?style=flat&label=Star&maxAge=86400)  - The Kalibr visual-inertial calibration toolbox
* [Calibnet](https://github.com/epiception/CalibNet) ![CalibNet](https://img.shields.io/github/stars/epiception/CalibNet.svg?style=flat&label=Star&maxAge=86400)  - Self-Supervised Extrinsic Calibration using 3D Spatial Transformer Networks
* [lidar_camera_calibration](https://github.com/ankitdhall/lidar_camera_calibration) ![lidar_camera_calibration](https://img.shields.io/github/stars/ankitdhall/lidar_camera_calibration.svg?style=flat&label=Star&maxAge=86400)  - ROS package to find a rigid-body transformation between a LiDAR and a camera
* [ILCC](https://github.com/mfxox/ILCC) ![ILCC](https://img.shields.io/github/stars/mfxox/ILCC.svg?style=flat&label=Star&maxAge=86400)  - Reflectance Intensity Assisted Automatic and Accurate Extrinsic Calibration of 3D LiDAR
* [easy_handeye](https://github.com/IFL-CAMP/easy_handeye) ![easy_handeye](https://img.shields.io/github/stars/IFL-CAMP/easy_handeye.svg?style=flat&label=Star&maxAge=86400)  - Simple, straighforward ROS library for hand-eye calibration
* [imu_utils](https://github.com/gaowenliang/imu_utils) ![imu_utils](https://img.shields.io/github/stars/gaowenliang/imu_utils.svg?style=flat&label=Star&maxAge=86400)  - A ROS package tool to analyze the IMU performance
* [kalibr_allan](https://github.com/rpng/kalibr_allan) ![kalibr_allan](https://img.shields.io/github/stars/rpng/kalibr_allan.svg?style=flat&label=Star&maxAge=86400)  - IMU Allan standard deviation charts for use with Kalibr and inertial kalman filters
* [pyquaternion](https://github.com/KieranWynn/pyquaternion) ![pyquaternion](https://img.shields.io/github/stars/KieranWynn/pyquaternion.svg?style=flat&label=Star&maxAge=86400)  - is a full-featured Python module for representing and using quaternions
* [robot_calibration](https://github.com/mikeferguson/robot_calibration/) ![](https://img.shields.io/github/stars/mikeferguson/robot_calibration/.svg?style=flat&label=Star&maxAge=86400)  - This package offers calibration of a number of parameters of a robot, such as: 3D Camera intrinsics, extrinsics Joint angle offsets and robot frame offsets
* [multi_sensor_calibration](https://github.com/tudelft-iv/multi_sensor_calibration/) ![](https://img.shields.io/github/stars/tudelft-iv/multi_sensor_calibration/.svg?style=flat&label=Star&maxAge=86400)  - contains a calibration tool to calibrate a sensor setup consisting of lidars, radars and cameras
* [LiDARTag](https://github.com/UMich-BipedLab/LiDARTag) ![LiDARTag](https://img.shields.io/github/stars/UMich-BipedLab/LiDARTag.svg?style=flat&label=Star&maxAge=86400)  - A Real-Time Fiducial Tag using Point Clouds Lidar Data
* [multicam_calibration](https://github.com/KumarRobotics/multicam_calibration) ![multicam_calibration](https://img.shields.io/github/stars/KumarRobotics/multicam_calibration.svg?style=flat&label=Star&maxAge=86400)  - extrinsic and intrinsic calbration of cameras


## Sensor Processing
### Perception Pipeline
* [SARosPerceptionKitti](https://github.com/appinho/SARosPerceptionKitti) ![SARosPerceptionKitti](https://img.shields.io/github/stars/appinho/SARosPerceptionKitti.svg?style=flat&label=Star&maxAge=86400)  - ROS package for the Perception (Sensor Processing, Detection, Tracking and Evaluation) of the KITTI Vision Benchmark Suite
* [multiple-object-tracking-lidar](https://github.com/praveen-palanisamy/multiple-object-tracking-lidar) ![multiple-object-tracking-lidar](https://img.shields.io/github/stars/praveen-palanisamy/multiple-object-tracking-lidar.svg?style=flat&label=Star&maxAge=86400)  - C++ implementation to Detect, track and classify multiple objects using LIDAR scans or point cloud
* [cadrl_ros](https://github.com/mfe7/cadrl_ros) ![cadrl_ros](https://img.shields.io/github/stars/mfe7/cadrl_ros.svg?style=flat&label=Star&maxAge=86400)  - ROS package for dynamic obstacle avoidance for ground robots trained with deep RL
* [AugmentedAutoencoder](https://github.com/DLR-RM/AugmentedAutoencoder) ![AugmentedAutoencoder](https://img.shields.io/github/stars/DLR-RM/AugmentedAutoencoder.svg?style=flat&label=Star&maxAge=86400)  - RGB-based pipeline for object detection and 6D pose estimation
* [jsk_recognition](https://github.com/jsk-ros-pkg/jsk_recognition) ![jsk_recognition](https://img.shields.io/github/stars/jsk-ros-pkg/jsk_recognition.svg?style=flat&label=Star&maxAge=86400)  - is a stack for the perception packages which are used in JSK lab.
* [GibsonEnv](https://github.com/StanfordVL/GibsonEnv) ![GibsonEnv](https://img.shields.io/github/stars/StanfordVL/GibsonEnv.svg?style=flat&label=Star&maxAge=86400)  - Gibson Environments: Real-World Perception for Embodied Agents
* [morefusion](https://github.com/wkentaro/morefusion) ![morefusion](https://img.shields.io/github/stars/wkentaro/morefusion.svg?style=flat&label=Star&maxAge=86400)  - Multi-object Reasoning for 6D Pose Estimation from Volumetric Fusion


### Parallel Processing
* [dask](https://github.com/dask/dask) ![dask](https://img.shields.io/github/stars/dask/dask.svg?style=flat&label=Star&maxAge=86400)  - Parallel computing with task scheduling for Python
* [cupy](https://github.com/cupy/cupy) ![cupy](https://img.shields.io/github/stars/cupy/cupy.svg?style=flat&label=Star&maxAge=86400)  - NumPy-like API accelerated with CUDA
* [thrust](https://github.com/thrust/thrust) ![thrust](https://img.shields.io/github/stars/thrust/thrust.svg?style=flat&label=Star&maxAge=86400)  - Thrust is a C++ parallel programming library which resembles the C++ Standard Library.
* [ArrayFire](https://github.com/arrayfire/arrayfire) ![arrayfire](https://img.shields.io/github/stars/arrayfire/arrayfire.svg?style=flat&label=Star&maxAge=86400)  - ArrayFire: a general purpose GPU library.
* [OpenMP](https://www.openmp.org/) - OpenMP is an application programming interface that supports multi-platform shared memory multiprocessing programming in C, C++, and Fortra
* [vexcl](https://github.com/ddemidov/vexcl) ![vexcl](https://img.shields.io/github/stars/ddemidov/vexcl.svg?style=flat&label=Star&maxAge=86400)  - VexCL is a C++ vector expression template library for OpenCL/CUDA/OpenMP
* [PYNQ](https://github.com/Xilinx/PYNQ) ![PYNQ](https://img.shields.io/github/stars/Xilinx/PYNQ.svg?style=flat&label=Star&maxAge=86400)  - is an open-source project from Xilinx that makes it easy to design embedded systems with Zynq All Programmable Systems on Chips

### Machine Learning
* [DLIB](https://github.com/davisking/dlib) ![dlib](https://img.shields.io/github/stars/davisking/dlib.svg?style=flat&label=Star&maxAge=86400)  - A toolkit for making real world machine learning and data analysis applications in C++
* [fastai](https://github.com/fastai/fastai) ![fastai](https://img.shields.io/github/stars/fastai/fastai.svg?style=flat&label=Star&maxAge=86400)  - The fastai library simplifies training fast and accurate neural nets using modern best practices.
* [tpot](https://github.com/EpistasisLab/tpot) ![tpot](https://img.shields.io/github/stars/EpistasisLab/tpot.svg?style=flat&label=Star&maxAge=86400)  - A Python Automated Machine Learning tool that optimizes machine learning pipelines using genetic programming
* [deap](https://github.com/DEAP/deap) ![deap](https://img.shields.io/github/stars/DEAP/deap.svg?style=flat&label=Star&maxAge=86400)  - Distributed Evolutionary Algorithms in Python
* [gym](https://github.com/openai/gym) ![gym](https://img.shields.io/github/stars/openai/gym.svg?style=flat&label=Star&maxAge=86400)  - A toolkit for developing and comparing reinforcement learning algorithms.
* [tensorflow_ros_cpp](https://github.com/tradr-project/tensorflow_ros_cpp) ![tensorflow_ros_cpp](https://img.shields.io/github/stars/tradr-project/tensorflow_ros_cpp.svg?style=flat&label=Star&maxAge=86400)  - A ROS package that allows to do Tensorflow inference in C++ without the need to compile TF yourself.
* [Tensorflow Federated](https://github.com/tensorflow/federated) ![federated](https://img.shields.io/github/stars/tensorflow/federated.svg?style=flat&label=Star&maxAge=86400)  - TensorFlow Federated (TFF) is an open-source framework for machine learning and other computations on decentralized data
* [finn](https://github.com/Xilinx/finn) ![finn](https://img.shields.io/github/stars/Xilinx/finn.svg?style=flat&label=Star&maxAge=86400)  - Fast, Scalable Quantized Neural Network Inference on FPGAs

### Image Processing
* [image_pipeline](https://github.com/ros-perception/image_pipeline) ![image_pipeline](https://img.shields.io/github/stars/ros-perception/image_pipeline.svg?style=flat&label=Star&maxAge=86400)  - fills the gap between getting raw images from a camera driver and higher-level vision processing
* [gstreamer](https://gstreamer.freedesktop.org/) - is a pipeline-based multimedia framework that links together a wide variety of media processing systems to complete complex workflows
* [ros2_openvino_toolkit](https://github.com/intel/ros2_openvino_toolkit) ![ros2_openvino_toolkit](https://img.shields.io/github/stars/intel/ros2_openvino_toolkit.svg?style=flat&label=Star&maxAge=86400)  -  provides a ROS-adaptered runtime framework of neural network which quickly deploys applications and solutions for vision inference
* [vision_visp](https://github.com/lagadic/vision_visp) ![vision_visp](https://img.shields.io/github/stars/lagadic/vision_visp.svg?style=flat&label=Star&maxAge=86400)  - Wraps the ViSP moving edge tracker provided by the ViSP visual servoing library into a ROS package
* [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) ![apriltag_ros](https://img.shields.io/github/stars/AprilRobotics/apriltag_ros.svg?style=flat&label=Star&maxAge=86400)  - A ROS wrapper of the AprilTag 3 visual fiducial detector
* [deep_object_pose](https://github.com/NVlabs/Deep_Object_Pose) ![Deep_Object_Pose](https://img.shields.io/github/stars/NVlabs/Deep_Object_Pose.svg?style=flat&label=Star&maxAge=86400)  - Deep Object Pose Estimation
* [DetectAndTrack](https://github.com/facebookresearch/DetectAndTrack) ![DetectAndTrack](https://img.shields.io/github/stars/facebookresearch/DetectAndTrack.svg?style=flat&label=Star&maxAge=86400)  - Detect-and-Track: Efficient Pose
* [SfMLearner](https://github.com/tinghuiz/SfMLearner) ![SfMLearner](https://img.shields.io/github/stars/tinghuiz/SfMLearner.svg?style=flat&label=Star&maxAge=86400)  - An unsupervised learning framework for depth and ego-motion estimation
* [imgaug](https://github.com/aleju/imgaug) ![imgaug](https://img.shields.io/github/stars/aleju/imgaug.svg?style=flat&label=Star&maxAge=86400)  - Image augmentation for machine learning experiments
* [vision_opencv](https://github.com/ros-perception/vision_opencv) ![vision_opencv](https://img.shields.io/github/stars/ros-perception/vision_opencv.svg?style=flat&label=Star&maxAge=86400)  - Packages for interfacing ROS with OpenCV, a library of programming functions for real time computer vision.
* [darknet_ros](https://github.com/leggedrobotics/darknet_ros) ![darknet_ros](https://img.shields.io/github/stars/leggedrobotics/darknet_ros.svg?style=flat&label=Star&maxAge=86400)  - YOLO ROS: Real-Time Object Detection for ROS
* [tf-pose-estimation](https://github.com/ildoonet/tf-pose-estimation) ![tf-pose-estimation](https://img.shields.io/github/stars/ildoonet/tf-pose-estimation.svg?style=flat&label=Star&maxAge=86400)  - Deep Pose Estimation implemented using Tensorflow with Custom Architectures for fast inference.
* [find-object](https://github.com/introlab/find-object) ![find-object](https://img.shields.io/github/stars/introlab/find-object.svg?style=flat&label=Star&maxAge=86400)  - Simple Qt interface to try OpenCV implementations of SIFT, SURF, FAST, BRIEF and other feature detectors and descriptors
* [yolact](https://github.com/dbolya/yolact) ![yolact](https://img.shields.io/github/stars/dbolya/yolact.svg?style=flat&label=Star&maxAge=86400)  - A simple, fully convolutional model for real-time instance segmentation.
* [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) ![Kimera-Semantics](https://img.shields.io/github/stars/MIT-SPARK/Kimera-Semantics.svg?style=flat&label=Star&maxAge=86400)  - Real-Time 3D Semantic Reconstruction from 2D data
* [detectron2](https://github.com/facebookresearch/detectron2) ![detectron2](https://img.shields.io/github/stars/facebookresearch/detectron2.svg?style=flat&label=Star&maxAge=86400)  - is a next-generation research platform for object detection and segmentation.
* [OpenVX](https://www.khronos.org/openvx/) -  enables performance and power-optimized computer vision processing, especially important in embedded and real-time use cases
* [3d-vehicle-tracking](https://github.com/ucbdrive/3d-vehicle-tracking) ![3d-vehicle-tracking](https://img.shields.io/github/stars/ucbdrive/3d-vehicle-tracking.svg?style=flat&label=Star&maxAge=86400)  - Official implementation of Joint Monocular 3D Vehicle Detection and Tracking
* [pysot](https://github.com/STVIR/pysot) ![pysot](https://img.shields.io/github/stars/STVIR/pysot.svg?style=flat&label=Star&maxAge=86400)  - The goal of PySOT is to provide a high-quality, high-performance codebase for visual tracking research
* [semantic_slam](https://github.com/floatlazer/semantic_slam) ![semantic_slam](https://img.shields.io/github/stars/floatlazer/semantic_slam.svg?style=flat&label=Star&maxAge=86400)  - Real time semantic slam in ROS with a hand held RGB-D camera
* [kitti_scan_unfolding](https://github.com/ltriess/kitti_scan_unfolding) ![kitti_scan_unfolding](https://img.shields.io/github/stars/ltriess/kitti_scan_unfolding.svg?style=flat&label=Star&maxAge=86400)  - We propose KITTI scan unfolding in our paper Scan-based Semantic Segmentation of LiDAR Point Clouds: An Experimental Study.
* [packnet-sfm](https://github.com/TRI-ML/packnet-sfm) ![packnet-sfm](https://img.shields.io/github/stars/TRI-ML/packnet-sfm.svg?style=flat&label=Star&maxAge=86400)  - Official PyTorch implementation of self-supervised monocular depth estimation methods invented by the ML Team at Toyota Research Institute (TRI)
* [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT) ![AB3DMOT](https://img.shields.io/github/stars/xinshuoweng/AB3DMOT.svg?style=flat&label=Star&maxAge=86400)  - This work proposes a simple yet accurate real-time baseline 3D multi-object tracking system
* [lanenet-lane-detection](https://github.com/MaybeShewill-CV/lanenet-lane-detection) ![lanenet-lane-detection](https://img.shields.io/github/stars/MaybeShewill-CV/lanenet-lane-detection.svg?style=flat&label=Star&maxAge=86400)  - Unofficial implemention of lanenet model for real time lane detection using deep neural network model
* [OpenDroneMap](https://github.com/OpenDroneMap/ODM) ![ODM](https://img.shields.io/github/stars/OpenDroneMap/ODM.svg?style=flat&label=Star&maxAge=86400)  - An open source command line toolkit for processing aerial drone imagery
* [monoloco](https://github.com/vita-epfl/monoloco) ![monoloco](https://img.shields.io/github/stars/vita-epfl/monoloco.svg?style=flat&label=Star&maxAge=86400)  - Official implementation of "MonoLoco: Monocular 3D Pedestrian Localization and Uncertainty Estimation" in PyTorch

### Radar Processing
* [pyroSAR](https://github.com/johntruckenbrodt/pyroSAR) ![pyroSAR](https://img.shields.io/github/stars/johntruckenbrodt/pyroSAR.svg?style=flat&label=Star&maxAge=86400)  - Framework for large-scale SAR satellite data processing

### Lidar Processing
* [cilantro](https://github.com/kzampog/cilantro) ![cilantro](https://img.shields.io/github/stars/kzampog/cilantro.svg?style=flat&label=Star&maxAge=86400)  - A lean C++ library for working with point cloud data
* [open3d](https://github.com/intel-isl/Open3D) ![Open3D](https://img.shields.io/github/stars/intel-isl/Open3D.svg?style=flat&label=Star&maxAge=86400)  - Open3D: A Modern Library for 3D Data Processing
* [SqueezeSeg](https://github.com/BichenWuUCB/SqueezeSeg) ![SqueezeSeg](https://img.shields.io/github/stars/BichenWuUCB/SqueezeSeg.svg?style=flat&label=Star&maxAge=86400)  - Implementation of SqueezeSeg, convolutional neural networks for LiDAR point clout segmentation
* [point_cloud_io](https://github.com/ANYbotics/point_cloud_io) ![point_cloud_io](https://img.shields.io/github/stars/ANYbotics/point_cloud_io.svg?style=flat&label=Star&maxAge=86400)  - ROS nodes to read and write point clouds from and to files (e.g. ply, vtk).
* [python-pcl](https://github.com/strawlab/python-pcl) ![python-pcl](https://img.shields.io/github/stars/strawlab/python-pcl.svg?style=flat&label=Star&maxAge=86400)  - Python bindings to the pointcloud library
* [libpointmatcher](https://github.com/ethz-asl/libpointmatcher) ![libpointmatcher](https://img.shields.io/github/stars/ethz-asl/libpointmatcher.svg?style=flat&label=Star&maxAge=86400)  - An "Iterative Closest Point" library for 2-D/3-D mapping in Robotics
* [depth_clustering](https://github.com/PRBonn/depth_clustering) ![depth_clustering](https://img.shields.io/github/stars/PRBonn/depth_clustering.svg?style=flat&label=Star&maxAge=86400)  - Fast and robust clustering of point clouds generated with a Velodyne sensor.
* [lidar-bonnetal](https://github.com/PRBonn/lidar-bonnetal) ![lidar-bonnetal](https://img.shields.io/github/stars/PRBonn/lidar-bonnetal.svg?style=flat&label=Star&maxAge=86400)  - Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving
* [CSF](https://github.com/jianboqi/CSF) ![CSF](https://img.shields.io/github/stars/jianboqi/CSF.svg?style=flat&label=Star&maxAge=86400)  - LiDAR point cloud ground filtering / segmentation (bare earth extraction) method based on cloth simulation
* [robot_body_filter](https://github.com/peci1/robot_body_filter) ![robot_body_filter](https://img.shields.io/github/stars/peci1/robot_body_filter.svg?style=flat&label=Star&maxAge=86400)  - A highly configurable LaserScan/PointCloud2 filter that allows to dynamically remove the 3D body of the robot from the measurements.
* [grid_map](https://github.com/ANYbotics/grid_map) ![grid_map](https://img.shields.io/github/stars/ANYbotics/grid_map.svg?style=flat&label=Star&maxAge=86400)  - Universal grid map library for mobile robotic mapping
* [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) ![elevation_mapping](https://img.shields.io/github/stars/ANYbotics/elevation_mapping.svg?style=flat&label=Star&maxAge=86400)  - Robot-centric elevation mapping for rough terrain navigation
* [rangenet_lib](https://github.com/PRBonn/rangenet_lib) ![rangenet_lib](https://img.shields.io/github/stars/PRBonn/rangenet_lib.svg?style=flat&label=Star&maxAge=86400)  - contains simple usage explanations of how the RangeNet++ inference works with the TensorRT and C++ interface.
* [pointcloud_to_laserscan](https://github.com/ros-perception/pointcloud_to_laserscan) ![pointcloud_to_laserscan](https://img.shields.io/github/stars/ros-perception/pointcloud_to_laserscan.svg?style=flat&label=Star&maxAge=86400)  - Converts a 3D Point Cloud into a 2D laser scan.
* [octomap](https://github.com/OctoMap/octomap) ![octomap](https://img.shields.io/github/stars/OctoMap/octomap.svg?style=flat&label=Star&maxAge=86400)  - An Efficient Probabilistic 3D Mapping Framework Based on Octrees
* [pptk](https://github.com/heremaps/pptk) ![pptk](https://img.shields.io/github/stars/heremaps/pptk.svg?style=flat&label=Star&maxAge=86400)  - Point Processing Toolkit from HEREMaps
* [gpu-voxels](https://www.gpu-voxels.org/) - GPU-Voxels is a CUDA based library which allows high resolution volumetric collision detection between animated 3D models and live pointclouds from 3D sensors of all kinds.
* [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) ![spatio_temporal_voxel_layer](https://img.shields.io/github/stars/SteveMacenski/spatio_temporal_voxel_layer.svg?style=flat&label=Star&maxAge=86400)  - A new voxel layer leveraging modern 3D graphics tools to modernize navigation environmental representations
* [LAStools](https://github.com/LAStools/LAStools) ![LAStools](https://img.shields.io/github/stars/LAStools/LAStools.svg?style=flat&label=Star&maxAge=86400)  - award-winning software for efficient LiDAR processing
* [PCDet](https://github.com/sshaoshuai/PCDet) ![PCDet](https://img.shields.io/github/stars/sshaoshuai/PCDet.svg?style=flat&label=Star&maxAge=86400)  - is a general PyTorch-based codebase for 3D object detection from point cloud.
* [PDAL](https://github.com/PDAL/PDAL) ![PDAL](https://img.shields.io/github/stars/PDAL/PDAL.svg?style=flat&label=Star&maxAge=86400)  - is a C++ BSD library for translating and manipulating point cloud data
* [PotreeConverter](https://github.com/potree/PotreeConverter) ![PotreeConverter](https://img.shields.io/github/stars/potree/PotreeConverter.svg?style=flat&label=Star&maxAge=86400)  - Builds a potree octree from las, laz, binary ply, xyz or ptx files.
* [fast_gicp](https://github.com/SMRT-AIST/fast_gicp) ![fast_gicp](https://img.shields.io/github/stars/SMRT-AIST/fast_gicp.svg?style=flat&label=Star&maxAge=86400)  - A collection of GICP-based fast point cloud registration algorithms
* [ndt_omp](https://github.com/koide3/ndt_omp) ![ndt_omp](https://img.shields.io/github/stars/koide3/ndt_omp.svg?style=flat&label=Star&maxAge=86400)  - Multi-threaded and SSE friendly NDT algorithm
* [spatio_temporal_voxel_layer](https://github.com/SteveMacenski/spatio_temporal_voxel_layer) ![spatio_temporal_voxel_layer](https://img.shields.io/github/stars/SteveMacenski/spatio_temporal_voxel_layer.svg?style=flat&label=Star&maxAge=86400)  - voxel layer leveraging modern 3D graphics tools to modernize navigation environmental representations
* [laser_line_extraction](https://github.com/kam3k/laser_line_extraction) ![laser_line_extraction](https://img.shields.io/github/stars/kam3k/laser_line_extraction.svg?style=flat&label=Star&maxAge=86400)  - A ROS packages that extracts line segments from LaserScan messages.
* [Go-ICP](https://github.com/yangjiaolong/Go-ICP) ![Go-ICP](https://img.shields.io/github/stars/yangjiaolong/Go-ICP.svg?style=flat&label=Star&maxAge=86400)  - Implementation of the Go-ICP algorithm for globally optimal 3D pointset registration
* [PointCNN](https://github.com/yangyanli/PointCNN) ![PointCNN](https://img.shields.io/github/stars/yangyanli/PointCNN.svg?style=flat&label=Star&maxAge=86400)  - is a simple and general framework for feature learning from point clouds
* [segmenters_lib](https://github.com/LidarPerception/segmenters_lib) ![segmenters_lib](https://img.shields.io/github/stars/LidarPerception/segmenters_lib.svg?style=flat&label=Star&maxAge=86400)  - The LiDAR segmenters library, for segmentation-based detection
* [MotionNet](https://github.com/pxiangwu/MotionNet) ![MotionNet](https://img.shields.io/github/stars/pxiangwu/MotionNet.svg?style=flat&label=Star&maxAge=86400)  - Joint Perception and Motion Prediction for Autonomous Driving Based on Bird's Eye View Maps
* [PolarSeg](https://github.com/edwardzhou130/PolarSeg) ![PolarSeg](https://img.shields.io/github/stars/edwardzhou130/PolarSeg.svg?style=flat&label=Star&maxAge=86400)  - An Improved Grid Representation for Online LiDAR Point Clouds Semantic Segmentation
* [traversability_mapping](https://github.com/TixiaoShan/traversability_mapping) ![traversability_mapping](https://img.shields.io/github/stars/TixiaoShan/traversability_mapping.svg?style=flat&label=Star&maxAge=86400)  - takes in point cloud from a Velodyne VLP-16 Lidar and outputs a traversability map for autonomous navigation in real-time
* [lidar_super_resolution](https://github.com/RobustFieldAutonomyLab/lidar_super_resolution) ![lidar_super_resolution](https://img.shields.io/github/stars/RobustFieldAutonomyLab/lidar_super_resolution.svg?style=flat&label=Star&maxAge=86400)  - Simulation-based Lidar Super-resolution for Ground Vehicles
* [cupoch](https://github.com/neka-nat/cupoch) ![cupoch](https://img.shields.io/github/stars/neka-nat/cupoch.svg?style=flat&label=Star&maxAge=86400)  - Cupoch is a library that implements rapid 3D data processing and robotics computation using CUDA
* [linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation) ![linefit_ground_segmentation](https://img.shields.io/github/stars/lorenwel/linefit_ground_segmentation.svg?style=flat&label=Star&maxAge=86400)  - Implementation of the ground segmentation algorithm
* [draco](https://github.com/google/draco) ![draco](https://img.shields.io/github/stars/google/draco.svg?style=flat&label=Star&maxAge=86400)  - Draco is a library for compressing and decompressing 3D geometric meshes and point clouds
* [votenet](https://github.com/facebookresearch/votenet) ![votenet](https://img.shields.io/github/stars/facebookresearch/votenet.svg?style=flat&label=Star&maxAge=86400)  - Deep Hough Voting for 3D Object Detection in Point Clouds
* [Det3D](https://github.com/poodarchu/Det3D) ![Det3D](https://img.shields.io/github/stars/poodarchu/Det3D.svg?style=flat&label=Star&maxAge=86400)  - A general 3D Object Detection codebase in PyTorch
* [lidar_undistortion](https://github.com/ethz-asl/lidar_undistortion) ![lidar_undistortion](https://img.shields.io/github/stars/ethz-asl/lidar_undistortion.svg?style=flat&label=Star&maxAge=86400)  - provides lidar motion undistortion based on an external 6DoF pose estimation input.


## Localization
* [evo](https://github.com/MichaelGrupp/evo) ![evo](https://img.shields.io/github/stars/MichaelGrupp/evo.svg?style=flat&label=Star&maxAge=86400)  - Python package for the evaluation of odometry and SLAM
* [robot_localization](https://github.com/cra-ros-pkg/robot_localization) ![robot_localization](https://img.shields.io/github/stars/cra-ros-pkg/robot_localization.svg?style=flat&label=Star&maxAge=86400)  - is a package of nonlinear state estimation nodes
* [fuse](https://github.com/locusrobotics/fuse) ![fuse](https://img.shields.io/github/stars/locusrobotics/fuse.svg?style=flat&label=Star&maxAge=86400)  - General architecture for performing sensor fusion live on a
robot.
* [rep-105](https://www.ros.org/reps/rep-0105.html) - Naming conventions and semantic meaning for
coordinate frames of mobile platforms used with ROS.
* [GeographicLib](https://github.com/Sciumo/GeographicLib) ![GeographicLib](https://img.shields.io/github/stars/Sciumo/GeographicLib.svg?style=flat&label=Star&maxAge=86400)  - A C++ library for geographic projections.
* [ntripbrowser](https://github.com/emlid/ntripbrowser) ![ntripbrowser](https://img.shields.io/github/stars/emlid/ntripbrowser.svg?style=flat&label=Star&maxAge=86400)  - A Python API for browsing NTRIP (Networked Transport of RTCM via Internet Protocol).
* [imu_tools](https://github.com/ccny-ros-pkg/imu_tools) ![imu_tools](https://img.shields.io/github/stars/ccny-ros-pkg/imu_tools.svg?style=flat&label=Star&maxAge=86400)  - IMU-related filters and visualizers.
* [RTKLIB](https://github.com/rtklibexplorer/RTKLIB) ![RTKLIB](https://img.shields.io/github/stars/rtklibexplorer/RTKLIB.svg?style=flat&label=Star&maxAge=86400)  - A version of RTKLIB optimized for single and dual frequency low cost GPS receivers, especially u-blox receivers
* [gLAB](https://gage.upc.edu/gLAB/) - performs precise modeling of GNSS observables (pseudorange and carrier phase) at the centimetre level, allowing standalone GPS positioning, PPP, SBAS and DGNSS
* [mola](https://github.com/MOLAorg/mola) ![mola](https://img.shields.io/github/stars/MOLAorg/mola.svg?style=flat&label=Star&maxAge=86400)  - is a Modular system for Localization and Mapping
* [ai-imu-dr](https://github.com/mbrossar/ai-imu-dr) ![ai-imu-dr](https://img.shields.io/github/stars/mbrossar/ai-imu-dr.svg?style=flat&label=Star&maxAge=86400)  - contains the code of our novel accurate method for dead reckoning of wheeled vehicles based only on an IMU
* [Kalman-and-Bayesian-Filters-in-Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python) ![Kalman-and-Bayesian-Filters-in-Python](https://img.shields.io/github/stars/rlabbe/Kalman-and-Bayesian-Filters-in-Python.svg?style=flat&label=Star&maxAge=86400)  - Kalman Filter book using Jupyter Notebook
* [mcl_3dl](https://github.com/at-wat/mcl_3dl) ![mcl_3dl](https://img.shields.io/github/stars/at-wat/mcl_3dl.svg?style=flat&label=Star&maxAge=86400)  - A ROS node to perform a probabilistic 3-D/6-DOF localization system for mobile robots with 3-D LIDAR(s)
* [se2lam](https://github.com/izhengfan/se2lam) ![se2lam](https://img.shields.io/github/stars/izhengfan/se2lam.svg?style=flat&label=Star&maxAge=86400)  - On-SE(2) Localization and Mapping for Ground Vehicles by Fusing Odometry and Vision

### SLAM
#### Lidar
* [loam_velodyne](https://github.com/laboshinl/loam_velodyne) ![loam_velodyne](https://img.shields.io/github/stars/laboshinl/loam_velodyne.svg?style=flat&label=Star&maxAge=86400)  - Laser Odometry and Mapping (Loam) is a realtime method for state estimation and mapping using a 3D lidar.
* [lio-mapping](https://github.com/hyye/lio-mapping) ![lio-mapping](https://img.shields.io/github/stars/hyye/lio-mapping.svg?style=flat&label=Star&maxAge=86400)  - Implementation of Tightly Coupled 3D Lidar Inertial Odometry and Mapping (LIO-mapping)
* [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) ![A-LOAM](https://img.shields.io/github/stars/HKUST-Aerial-Robotics/A-LOAM.svg?style=flat&label=Star&maxAge=86400)  - Advanced implementation of LOAM
* [floam](https://github.com/wh200720041/floam) ![floam](https://img.shields.io/github/stars/wh200720041/floam.svg?style=flat&label=Star&maxAge=86400)  - Fast LOAM: Fast and Optimized Lidar Odometry And Mapping
* [cartographer_ros](https://github.com/googlecartographer/cartographer_ros) ![cartographer_ros](https://img.shields.io/github/stars/googlecartographer/cartographer_ros.svg?style=flat&label=Star&maxAge=86400)  - Provides ROS integration for Cartographer
* [loam_livox](https://github.com/hku-mars/loam_livox) ![loam_livox](https://img.shields.io/github/stars/hku-mars/loam_livox.svg?style=flat&label=Star&maxAge=86400)  - A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR
* [StaticMapping](https://github.com/EdwardLiuyc/StaticMapping) ![StaticMapping](https://img.shields.io/github/stars/EdwardLiuyc/StaticMapping.svg?style=flat&label=Star&maxAge=86400)  - Use LiDAR to map the static world
* [semantic_suma](https://github.com/PRBonn/semantic_suma/) ![](https://img.shields.io/github/stars/PRBonn/semantic_suma/.svg?style=flat&label=Star&maxAge=86400)  - Semantic Mapping using Surfel Mapping and Semantic Segmentation
* [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) ![slam_toolbox](https://img.shields.io/github/stars/SteveMacenski/slam_toolbox.svg?style=flat&label=Star&maxAge=86400)  - Slam Toolbox for lifelong mapping and localization in potentially massive maps with ROS
* [maplab](https://github.com/ethz-asl/maplab) ![maplab](https://img.shields.io/github/stars/ethz-asl/maplab.svg?style=flat&label=Star&maxAge=86400)  - An open visual-inertial mapping framework.
* [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) ![hdl_graph_slam](https://img.shields.io/github/stars/koide3/hdl_graph_slam.svg?style=flat&label=Star&maxAge=86400)  - is an open source ROS package for real-time 6DOF SLAM using a 3D LIDAR
* [interactive_slam](https://github.com/SMRT-AIST/interactive_slam) ![interactive_slam](https://img.shields.io/github/stars/SMRT-AIST/interactive_slam.svg?style=flat&label=Star&maxAge=86400)  -  In contrast to existing automatic SLAM packages, we with minimal human effort.
* [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) ![LeGO-LOAM](https://img.shields.io/github/stars/RobustFieldAutonomyLab/LeGO-LOAM.svg?style=flat&label=Star&maxAge=86400)  - Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
* [pyslam](https://github.com/luigifreda/pyslam) ![pyslam](https://img.shields.io/github/stars/luigifreda/pyslam.svg?style=flat&label=Star&maxAge=86400)  - contains a monocular Visual Odometry (VO) pipeline in Python
* [PointCNN](https://github.com/yangyanli/PointCNN) ![PointCNN](https://img.shields.io/github/stars/yangyanli/PointCNN.svg?style=flat&label=Star&maxAge=86400)  - is a simple and general framework for feature learning from point cloud, which refreshed five benchmark records in point cloud processing

#### Camera
* [orb_slam_2_ros](https://github.com/appliedAI-Initiative/orb_slam_2_ros) ![orb_slam_2_ros](https://img.shields.io/github/stars/appliedAI-Initiative/orb_slam_2_ros.svg?style=flat&label=Star&maxAge=86400)  - A ROS implementation of ORB_SLAM2
* [dso](https://github.com/JakobEngel/dso/) ![](https://img.shields.io/github/stars/JakobEngel/dso/.svg?style=flat&label=Star&maxAge=86400)  - Direct Sparse Odometry
* [viso2](https://github.com/srv/viso2) ![viso2](https://img.shields.io/github/stars/srv/viso2.svg?style=flat&label=Star&maxAge=86400)  - A ROS wrapper for libviso2, a library for visual odometry
* [xivo](https://github.com/ucla-vision/xivo) ![xivo](https://img.shields.io/github/stars/ucla-vision/xivo.svg?style=flat&label=Star&maxAge=86400)  - X Inertial-aided Visual Odometry
* [rovio](https://github.com/ethz-asl/rovio) ![rovio](https://img.shields.io/github/stars/ethz-asl/rovio.svg?style=flat&label=Star&maxAge=86400)  - Robust Visual Inertial Odometry Framework
* [MIT Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) ![Kimera-Semantics](https://img.shields.io/github/stars/MIT-SPARK/Kimera-Semantics.svg?style=flat&label=Star&maxAge=86400)  - C++ library for real-time metric-semantic visual-inertial Simultaneous Localization And Mapping (SLAM)
* [LSD-SLAM](https://github.com/tum-vision/lsd_slam) ![lsd_slam](https://img.shields.io/github/stars/tum-vision/lsd_slam.svg?style=flat&label=Star&maxAge=86400)  - LSD-SLAM: Large-Scale Direct Monocular SLAM is a real-time monocular SLAM
* [CubeSLAM and ORB SLAM](https://github.com/shichaoy/cube_slam) ![cube_slam](https://img.shields.io/github/stars/shichaoy/cube_slam.svg?style=flat&label=Star&maxAge=86400)  - Monocular 3D Object Detection and SLAM Package of CubeSLAM and ORB SLAM
* [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) ![VINS-Fusion](https://img.shields.io/github/stars/HKUST-Aerial-Robotics/VINS-Fusion.svg?style=flat&label=Star&maxAge=86400)  - A Robust and Versatile Multi-Sensor Visual-Inertial State Estimator
* [openvslam](https://github.com/xdspacelab/openvslam) ![openvslam](https://img.shields.io/github/stars/xdspacelab/openvslam.svg?style=flat&label=Star&maxAge=86400)  - OpenVSLAM: A Versatile Visual SLAM Framework
* [basalt](https://gitlab.com/VladyslavUsenko/basalt) - Visual-Inertial Mapping with Non-Linear Factor Recovery
* [Kimera](https://github.com/MIT-SPARK/Kimera) ![Kimera](https://img.shields.io/github/stars/MIT-SPARK/Kimera.svg?style=flat&label=Star&maxAge=86400)  - is a C++ library for real-time metric-semantic simultaneous localization and mapping, which uses camera images and inertial data to build a semantically annotated 3D mesh of the environment
* [tagslam](https://github.com/berndpfrommer/tagslam) ![tagslam](https://img.shields.io/github/stars/berndpfrommer/tagslam.svg?style=flat&label=Star&maxAge=86400)  - is a ROS-based package for Simultaneous Localization and Mapping using AprilTag fiducial markers
* [LARVIO](https://github.com/PetWorm/LARVIO) ![LARVIO](https://img.shields.io/github/stars/PetWorm/LARVIO.svg?style=flat&label=Star&maxAge=86400)  - A lightweight, accurate and robust monocular visual inertial odometry based on Multi-State Constraint Kalman Filter.
* [fiducials](https://github.com/UbiquityRobotics/fiducials) ![fiducials](https://img.shields.io/github/stars/UbiquityRobotics/fiducials.svg?style=flat&label=Star&maxAge=86400)  - Simultaneous localization and mapping using fiducial markers.


#### Static Map
* [OpenDRIVE](http://www.opendrive.org/index.html) - OpenDRIVE® is an open file format for the logical description of road networks
* [MapsModelsImporter](https://github.com/eliemichel/MapsModelsImporter) ![MapsModelsImporter](https://img.shields.io/github/stars/eliemichel/MapsModelsImporter.svg?style=flat&label=Star&maxAge=86400)  - A Blender add-on to import models from google maps
* [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) ![Lanelet2](https://img.shields.io/github/stars/fzi-forschungszentrum-informatik/Lanelet2.svg?style=flat&label=Star&maxAge=86400)  - Map handling framework for automated driving
* [barefoot](https://github.com/bmwcarit/barefoot) ![barefoot](https://img.shields.io/github/stars/bmwcarit/barefoot.svg?style=flat&label=Star&maxAge=86400)  -  Online and Offline map matching that can be used stand-alone and in the cloud
* [iD](https://github.com/openstreetmap/iD) ![iD](https://img.shields.io/github/stars/openstreetmap/iD.svg?style=flat&label=Star&maxAge=86400)  - The easy-to-use OpenStreetMap editor in JavaScript
* [segmap](https://github.com/ethz-asl/segmap) ![segmap](https://img.shields.io/github/stars/ethz-asl/segmap.svg?style=flat&label=Star&maxAge=86400)  - A map representation based on 3D segments
* [Mapbox](https://github.com/mapbox/mapbox-gl-js) ![mapbox-gl-jsMapbox](https://img.shields.io/github/stars/mapbox/mapbox-gl-jsMapbox.svg?style=flat&label=Star&maxAge=86400)  - is a JavaScript library for interactive, customizable vector maps on the web
* [osrm-backend](https://github.com/Project-OSRM/osrm-backend) ![osrm-backend](https://img.shields.io/github/stars/Project-OSRM/osrm-backend.svg?style=flat&label=Star&maxAge=86400)  - Open Source Routing Machine - C++ backend
* [robosat](https://github.com/mapbox/robosat) ![robosat](https://img.shields.io/github/stars/mapbox/robosat.svg?style=flat&label=Star&maxAge=86400)  - Semantic segmentation on aerial and satellite imagery. Extracts features such as: buildings, parking lots, roads, water, clouds
* [sentinelsat](https://github.com/sentinelsat/sentinelsat) ![sentinelsat](https://img.shields.io/github/stars/sentinelsat/sentinelsat.svg?style=flat&label=Star&maxAge=86400)  - Search and download Copernicus Sentinel satellite images
* [assuremapingtools](https://github.com/hatem-darweesh/assuremapingtools) ![assuremapingtools](https://img.shields.io/github/stars/hatem-darweesh/assuremapingtools.svg?style=flat&label=Star&maxAge=86400)  -  Desktop based tool for viewing, editing and saving road network maps for autonomous vehicle platforms such as Autoware.
* [geopandas](https://github.com/geopandas/geopandas) ![geopandas](https://img.shields.io/github/stars/geopandas/geopandas.svg?style=flat&label=Star&maxAge=86400)  - is a project to add support for geographic data to pandas objects.
* [MapToolbox](https://github.com/autocore-ai/MapToolbox) ![MapToolbox](https://img.shields.io/github/stars/autocore-ai/MapToolbox.svg?style=flat&label=Star&maxAge=86400)  - Plugins to make Autoware vector maps in Unity
* [imagery-index](https://github.com/ideditor/imagery-index) ![imagery-index](https://img.shields.io/github/stars/ideditor/imagery-index.svg?style=flat&label=Star&maxAge=86400)  - An index of aerial and satellite imagery useful for mapping

## Prediction
* [Awesome-Interaction-aware-Trajectory-Prediction](https://github.com/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction) ![Awesome-Interaction-aware-Trajectory-Prediction](https://img.shields.io/github/stars/jiachenli94/Awesome-Interaction-aware-Trajectory-Prediction.svg?style=flat&label=Star&maxAge=86400)  - A selection of state-of-the-art research materials on trajectory prediction
* [sgan](https://github.com/agrimgupta92/sgan) ![sgan](https://img.shields.io/github/stars/agrimgupta92/sgan.svg?style=flat&label=Star&maxAge=86400)  -  Socially Acceptable Trajectories with Generative Adversarial Networks

## Behavior and Decision
* [Groot](https://github.com/BehaviorTree/Groot) ![Groot](https://img.shields.io/github/stars/BehaviorTree/Groot.svg?style=flat&label=Star&maxAge=86400)  - Graphical Editor to create BehaviorTrees. Compliant with BehaviorTree.CPP
* [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) ![BehaviorTree.CPP](https://img.shields.io/github/stars/BehaviorTree/BehaviorTree.CPP.svg?style=flat&label=Star&maxAge=86400)  - Behavior Trees Library in C++
* [RAFCON](https://github.com/DLR-RM/RAFCON) ![RAFCON](https://img.shields.io/github/stars/DLR-RM/RAFCON.svg?style=flat&label=Star&maxAge=86400)  - Uses hierarchical state machines, featuring concurrent state execution, to represent robot programs
* [ROSPlan](https://github.com/KCL-Planning/ROSPlan) ![ROSPlan](https://img.shields.io/github/stars/KCL-Planning/ROSPlan.svg?style=flat&label=Star&maxAge=86400)  - Generic framework for task planning in a ROS system
* [ad-rss-lib](https://github.com/intel/ad-rss-lib) ![ad-rss-lib](https://img.shields.io/github/stars/intel/ad-rss-lib.svg?style=flat&label=Star&maxAge=86400)  - Library implementing the Responsibility Sensitive Safety model (RSS) for Autonomous Vehicles
* [FlexBE](https://flexbe.github.io/) - Graphical editor for hierarchical state machines, based on ROS's smach.
* [sts_bt_library](https://github.com/Autonomous-Logistics/sts_bt_library) ![sts_bt_library](https://img.shields.io/github/stars/Autonomous-Logistics/sts_bt_library.svg?style=flat&label=Star&maxAge=86400)  - This library provides the functionality to set up your own behavior tree logic by using the defined tree structures like Fallback, Sequence or Parallel Nodes
* [SMACC](https://github.com/reelrbtx/SMACC) ![SMACC](https://img.shields.io/github/stars/reelrbtx/SMACC.svg?style=flat&label=Star&maxAge=86400)  - An Event-Driven, Asynchronous, Behavioral State Machine Library for real-time ROS (Robotic Operating System) applications written in C++
* [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) ![py_trees_ros](https://img.shields.io/github/stars/splintered-reality/py_trees_ros.svg?style=flat&label=Star&maxAge=86400)  - Behaviours, trees and utilities that extend py_trees for use with ROS.

## Planning and Control
* [pacmod](https://github.com/astuff/pacmod) ![pacmod](https://img.shields.io/github/stars/astuff/pacmod.svg?style=flat&label=Star&maxAge=86400)  -  is designed to allow the user to control a vehicle with the PACMod drive-by-wire system.
* [mpcc](https://github.com/alexliniger/MPCC) ![MPCC](https://img.shields.io/github/stars/alexliniger/MPCC.svg?style=flat&label=Star&maxAge=86400)  - Model Predictive Contouring Controller for Autonomous Racing
* [rrt](https://github.com/RoboJackets/rrt) ![rrt](https://img.shields.io/github/stars/RoboJackets/rrt.svg?style=flat&label=Star&maxAge=86400)  - C++ RRT (Rapidly-exploring Random Tree) implementation
* [HypridAStarTrailer](https://github.com/AtsushiSakai/HybridAStarTrailer) ![HybridAStarTrailer](https://img.shields.io/github/stars/AtsushiSakai/HybridAStarTrailer.svg?style=flat&label=Star&maxAge=86400)  - A path planning algorithm based on Hybrid A* for trailer truck.
* [path_planner](https://github.com/karlkurzer/path_planner) ![path_planner](https://img.shields.io/github/stars/karlkurzer/path_planner.svg?style=flat&label=Star&maxAge=86400)  - Hybrid A* Path Planner for the KTH Research Concept Vehicle
* [open_street_map](https://github.com/ros-geographic-info/open_street_map) ![open_street_map](https://img.shields.io/github/stars/ros-geographic-info/open_street_map.svg?style=flat&label=Star&maxAge=86400)  - ROS packages for working with Open Street Map geographic information.
* [Open Source Car Control](https://github.com/PolySync/oscc) ![oscc](https://img.shields.io/github/stars/PolySync/oscc.svg?style=flat&label=Star&maxAge=86400)  -  is an assemblage of software and hardware designs that enable computer control of modern cars in order to facilitate the development of autonomous vehicle technology
* [fastrack](https://github.com/HJReachability/fastrack) ![fastrack](https://img.shields.io/github/stars/HJReachability/fastrack.svg?style=flat&label=Star&maxAge=86400)  - A ROS implementation of Fast and Safe Tracking (FaSTrack).
* [commonroad](https://commonroad.in.tum.de/) - Composable benchmarks for motion planning on roads
* [traffic-editor](https://github.com/osrf/traffic-editor) ![traffic-editor](https://img.shields.io/github/stars/osrf/traffic-editor.svg?style=flat&label=Star&maxAge=86400)  - A graphical editor for robot traffic flows.
* [steering_functions](https://github.com/hbanzhaf/steering_functions) ![steering_functions](https://img.shields.io/github/stars/hbanzhaf/steering_functions.svg?style=flat&label=Star&maxAge=86400)  - contains a C++ library that implements steering functions for car-like robots with limited turning radius
* [moveit](https://moveit.ros.org/) - Easy-to-use robotics manipulation platform for developing applications, evaluating designs, and building integrated products
* [flexible-collision-library](https://github.com/flexible-collision-library/fcl) ![fcl](https://img.shields.io/github/stars/flexible-collision-library/fcl.svg?style=flat&label=Star&maxAge=86400)  - is a library for performing three types of proximity queries on a pair of geometric models composed of triangles.
* [aikido](https://github.com/personalrobotics/aikido) ![aikido](https://img.shields.io/github/stars/personalrobotics/aikido.svg?style=flat&label=Star&maxAge=86400)  - Artificial Intelligence for Kinematics, Dynamics, and Optimization
* [casADi](https://github.com/casadi/casadi) ![casadi](https://img.shields.io/github/stars/casadi/casadi.svg?style=flat&label=Star&maxAge=86400)  - is a symbolic framework for numeric optimization implementing automatic differentiation in forward and reverse modes on sparse matrix-valued computational graphs
* [ACADO Toolkit](https://github.com/acado/acado) ![acado](https://img.shields.io/github/stars/acado/acado.svg?style=flat&label=Star&maxAge=86400)  - is a software environment and algorithm collection for automatic control and dynamic optimization
* [control-toolbox](https://github.com/ethz-adrl/control-toolbox) ![control-toolbox](https://img.shields.io/github/stars/ethz-adrl/control-toolbox.svg?style=flat&label=Star&maxAge=86400)  - an efficient C++ library for control, estimation, optimization and motion planning in robotics
* [CrowdNav](https://github.com/vita-epfl/CrowdNav) ![CrowdNav](https://img.shields.io/github/stars/vita-epfl/CrowdNav.svg?style=flat&label=Star&maxAge=86400)  -  Crowd-aware Robot Navigation with Attention-based Deep Reinforcement Learning
* [ompl](https://github.com/ompl/ompl) ![ompl](https://img.shields.io/github/stars/ompl/ompl.svg?style=flat&label=Star&maxAge=86400)  - consists of many state-of-the-art sampling-based motion planning algorithms.
* [openrave](https://github.com/rdiankov/openrave) ![openrave](https://img.shields.io/github/stars/rdiankov/openrave.svg?style=flat&label=Star&maxAge=86400)  - Open Robotics Automation Virtual Environment: An environment for testing, developing, and deploying robotics motion planning algorithms.
* [teb_local_planner](https://github.com/rst-tu-dortmund/teb_local_planner) ![teb_local_planner](https://img.shields.io/github/stars/rst-tu-dortmund/teb_local_planner.svg?style=flat&label=Star&maxAge=86400)  - An optimal trajectory planner considering distinctive topologies for mobile robots based on Timed-Elastic-Bands
* [pinocchio](https://github.com/stack-of-tasks/pinocchio) ![pinocchio](https://img.shields.io/github/stars/stack-of-tasks/pinocchio.svg?style=flat&label=Star&maxAge=86400)  - A fast and flexible implementation of Rigid Body Dynamics algorithms and their analytical derivatives
* [control-toolbox](https://github.com/ethz-adrl/control-toolbox) ![control-toolbox](https://img.shields.io/github/stars/ethz-adrl/control-toolbox.svg?style=flat&label=Star&maxAge=86400)  - an efficient C++ library for control, estimation, optimization and motion planning in robotics.
* [rmf_core](https://github.com/osrf/rmf_core) ![rmf_core](https://img.shields.io/github/stars/osrf/rmf_core.svg?style=flat&label=Star&maxAge=86400)  - The rmf_core packages provide the centralized functions of the Robotics Middleware Framework (RMF)
* [OpEn](https://github.com/alphaville/optimization-engine) ![optimization-engine](https://img.shields.io/github/stars/alphaville/optimization-engine.svg?style=flat&label=Star&maxAge=86400)  - is a solver for Fast & Accurate Embedded Optimization for next-generation Robotics and Autonomous Systems
* [autogenu-jupyter](https://github.com/mayataka/autogenu-jupyter) ![autogenu-jupyter](https://img.shields.io/github/stars/mayataka/autogenu-jupyter.svg?style=flat&label=Star&maxAge=86400)  - This project provides the continuation/GMRES method (C/GMRES method) based solvers for nonlinear model predictive control (NMPC) and an automatic code generator for NMPC


## User Interaction
### Graphical User Interface
* [imgui](https://github.com/ocornut/imgui) ![imgui](https://img.shields.io/github/stars/ocornut/imgui.svg?style=flat&label=Star&maxAge=86400)  - is designed to enable fast iterations and to empower programmers to create content creation tools and visualization / debug tools
* [qtpy](https://github.com/spyder-ide/qtpy) ![qtpy](https://img.shields.io/github/stars/spyder-ide/qtpy.svg?style=flat&label=Star&maxAge=86400)  - Provides an uniform layer to support PyQt5, PySide2, PyQt4 and PySide with a single codebase
* [mir](https://github.com/MirServer/mir) ![mir](https://img.shields.io/github/stars/MirServer/mir.svg?style=flat&label=Star&maxAge=86400)  - Mir is set of libraries for building Wayland based shells
* [rqt](https://wiki.ros.org/rqt) - rqt is a Qt-based framework for GUI development for ROS. It consists of three parts/metapackages
* [cage](https://github.com/Hjdskes/cage) ![cage](https://img.shields.io/github/stars/Hjdskes/cage.svg?style=flat&label=Star&maxAge=86400)  - This is Cage, a Wayland kiosk. A kiosk runs a single, maximized application.
* [chilipie](https://github.com/futurice/chilipie-kiosk) ![chilipie-kiosk](https://img.shields.io/github/stars/futurice/chilipie-kiosk.svg?style=flat&label=Star&maxAge=86400)  - Easy-to-use Raspberry Pi image for booting directly into full-screen Chrome
* [pencil](https://github.com/evolus/pencil) ![pencil](https://img.shields.io/github/stars/evolus/pencil.svg?style=flat&label=Star&maxAge=86400)  - A tool for making diagrams and GUI prototyping that everyone can use.
* [dynamic_reconfigure](https://wiki.ros.org/dynamic_reconfigure) - The focus of dynamic_reconfigure is on providing a standard way to expose a subset of a node's parameters to external reconfiguration
* [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) ![ddynamic_reconfigure](https://img.shields.io/github/stars/pal-robotics/ddynamic_reconfigure.svg?style=flat&label=Star&maxAge=86400)  - allows modifying parameters of a ROS node using the dynamic_reconfigure framework without having to write cfg files
* [elements](https://github.com/cycfi/elements) ![elements](https://img.shields.io/github/stars/cycfi/elements.svg?style=flat&label=Star&maxAge=86400)  - is a lightweight, fine-grained, resolution independent, modular GUI library


### Acoustic User Interface
* [pyo](https://github.com/belangeo/pyo) ![pyo](https://img.shields.io/github/stars/belangeo/pyo.svg?style=flat&label=Star&maxAge=86400)  - is a Python module written in C containing classes for a wide variety of audio signal processing types
* [rhasspy](https://github.com/synesthesiam/rhasspy) ![rhasspy](https://img.shields.io/github/stars/synesthesiam/rhasspy.svg?style=flat&label=Star&maxAge=86400)  - Rhasspy (pronounced RAH-SPEE) is an offline, multilingual voice assistant toolkit inspired by Jasper that works well with Home Assistant, Hass.io, and Node-RED
* [mycroft-core](https://github.com/MycroftAI/mycroft-core) ![mycroft-core](https://img.shields.io/github/stars/MycroftAI/mycroft-core.svg?style=flat&label=Star&maxAge=86400)  - Mycroft is a hackable open source voice assistant

### Command Line Interface
* [the-art-of-command-line](https://github.com/jlevy/the-art-of-command-line) ![the-art-of-command-line](https://img.shields.io/github/stars/jlevy/the-art-of-command-line.svg?style=flat&label=Star&maxAge=86400)  - Master the command line, in one page
* [dotfiles of cornerman](https://github.com/cornerman/dotfiles) ![dotfiles](https://img.shields.io/github/stars/cornerman/dotfiles.svg?style=flat&label=Star&maxAge=86400)  - Powerful zsh and vim dotfiles
* [dotbot](https://github.com/anishathalye/dotbot) ![dotbot](https://img.shields.io/github/stars/anishathalye/dotbot.svg?style=flat&label=Star&maxAge=86400)  - A tool that bootstraps your dotfiles
* [prompt-hjem](https://github.com/cornerman/prompt-hjem) ![prompt-hjem](https://img.shields.io/github/stars/cornerman/prompt-hjem.svg?style=flat&label=Star&maxAge=86400)  - A beautiful zsh prompt
* [ag](https://github.com/ggreer/the_silver_searcher) ![the_silver_searcher](https://img.shields.io/github/stars/ggreer/the_silver_searcher.svg?style=flat&label=Star&maxAge=86400)  - A code-searching tool similar to ack, but faster.
* [fzf](https://github.com/junegunn/fzf) ![fzf](https://img.shields.io/github/stars/junegunn/fzf.svg?style=flat&label=Star&maxAge=86400)  - A command-line fuzzy finder
* [pkgtop](https://github.com/orhun/pkgtop) ![pkgtop](https://img.shields.io/github/stars/orhun/pkgtop.svg?style=flat&label=Star&maxAge=86400)  - Interactive package manager and resource monitor designed for the GNU/Linux.
* [asciimatics](https://github.com/peterbrittain/asciimatics) ![asciimatics](https://img.shields.io/github/stars/peterbrittain/asciimatics.svg?style=flat&label=Star&maxAge=86400)  - A cross platform package to do curses-like operations, plus higher level APIs and widgets to create text UIs and ASCII art animations
* [gocui](https://github.com/jroimartin/gocui) ![gocui](https://img.shields.io/github/stars/jroimartin/gocui.svg?style=flat&label=Star&maxAge=86400)  - Minimalist Go package aimed at creating Console User Interfaces.
* [TerminalImageViewer](https://github.com/stefanhaustein/TerminalImageViewer) ![TerminalImageViewer](https://img.shields.io/github/stars/stefanhaustein/TerminalImageViewer.svg?style=flat&label=Star&maxAge=86400) - Small C++ program to display images in a (modern) terminal using RGB ANSI codes and unicode block graphics characters
* [rosshow](https://github.com/dheera/rosshow) ![rosshow](https://img.shields.io/github/stars/dheera/rosshow.svg?style=flat&label=Star&maxAge=86400)  - Visualize ROS topics inside a terminal with Unicode/ASCII art
* [python-prompt-toolkit](https://github.com/prompt-toolkit/python-prompt-toolkit) ![python-prompt-toolkit](https://img.shields.io/github/stars/prompt-toolkit/python-prompt-toolkit.svg?style=flat&label=Star&maxAge=86400)  - Library for building powerful interactive command line applications in Python
* [bash-script-template](https://github.com/ralish/bash-script-template) ![bash-script-template](https://img.shields.io/github/stars/ralish/bash-script-template.svg?style=flat&label=Star&maxAge=86400)  - A best practices Bash script template with several useful functions
* [guake](https://github.com/Guake/guake) ![guake](https://img.shields.io/github/stars/Guake/guake.svg?style=flat&label=Star&maxAge=86400)  - Drop-down terminal for GNOME
* [wemux](https://github.com/zolrath/wemux) ![wemux](https://img.shields.io/github/stars/zolrath/wemux.svg?style=flat&label=Star&maxAge=86400)  - Multi-User Tmux Made Easy
* [tmuxp](https://github.com/tmux-python/tmuxp) ![tmuxp](https://img.shields.io/github/stars/tmux-python/tmuxp.svg?style=flat&label=Star&maxAge=86400)  -  tmux session manager built on libtmux
* [mapscii](https://github.com/rastapasta/mapscii) ![mapscii](https://img.shields.io/github/stars/rastapasta/mapscii.svg?style=flat&label=Star&maxAge=86400)  - World map renderer for your console
* [terminator](https://launchpad.net/terminator) - The goal of this project is to produce a useful tool for arranging terminals
* [bat](https://github.com/sharkdp/bat) ![bat](https://img.shields.io/github/stars/sharkdp/bat.svg?style=flat&label=Star&maxAge=86400)  - A cat(1) clone with wings.
* [fx](https://github.com/antonmedv/fx) ![fx](https://img.shields.io/github/stars/antonmedv/fx.svg?style=flat&label=Star&maxAge=86400)  - Command-line tool and terminal JSON viewer

### Data Visualization and Mission Control
* [xdot](https://github.com/jrfonseca/xdot.py) ![xdot.py](https://img.shields.io/github/stars/jrfonseca/xdot.py.svg?style=flat&label=Star&maxAge=86400)  - Interactive viewer for graphs written in Graphviz's dot language
* [guacamole](https://guacamole.apache.org/) - clientless remote desktop gateway. It supports standard protocols like VNC, RDP, and SSH
* [ros3djs](https://github.com/RobotWebTools/ros3djs) ![ros3djs](https://img.shields.io/github/stars/RobotWebTools/ros3djs.svg?style=flat&label=Star&maxAge=86400)  - 3D Visualization Library for use with the ROS JavaScript Libraries
* [webviz](https://github.com/cruise-automation/webviz) ![webviz](https://img.shields.io/github/stars/cruise-automation/webviz.svg?style=flat&label=Star&maxAge=86400)  - web-based visualization libraries like rviz
* [plotly.py](https://github.com/plotly/plotly.py) ![plotly.py](https://img.shields.io/github/stars/plotly/plotly.py.svg?style=flat&label=Star&maxAge=86400)  - An open-source, interactive graphing library for Python
* [PlotJuggler](https://github.com/facontidavide/PlotJuggler) ![PlotJuggler](https://img.shields.io/github/stars/facontidavide/PlotJuggler.svg?style=flat&label=Star&maxAge=86400)  - The timeseries visualization tool that you deserve
* [bokeh](https://github.com/bokeh/bokeh) ![bokeh](https://img.shields.io/github/stars/bokeh/bokeh.svg?style=flat&label=Star&maxAge=86400)  - Interactive Data Visualization in the browser, from Python
* [voila](https://github.com/voila-dashboards/voila) ![voila](https://img.shields.io/github/stars/voila-dashboards/voila.svg?style=flat&label=Star&maxAge=86400)  - From Jupyter notebooks to standalone web applications and dashboards
* [Pangolin](https://github.com/stevenlovegrove/Pangolin) ![Pangolin](https://img.shields.io/github/stars/stevenlovegrove/Pangolin.svg?style=flat&label=Star&maxAge=86400)  - Pangolin is a lightweight portable rapid development library for managing OpenGL display / interaction and abstracting video input.
* [rqt_bag](http://wiki.ros.org/rqt_bag) - provides a GUI plugin for displaying and replaying ROS bag files.
* [kepler.gl](https://github.com/keplergl/kepler.gl) ![kepler.gl](https://img.shields.io/github/stars/keplergl/kepler.gl.svg?style=flat&label=Star&maxAge=86400)  - Kepler.gl is a powerful open source geospatial analysis tool for large-scale data sets.
* [qgis_ros](https://github.com/locusrobotics/qgis_ros) ![qgis_ros](https://img.shields.io/github/stars/locusrobotics/qgis_ros.svg?style=flat&label=Star&maxAge=86400)  - Access bagged and live topic data in a highly featured GIS environment
* [openmct](https://github.com/nasa/openmct) ![openmct](https://img.shields.io/github/stars/nasa/openmct.svg?style=flat&label=Star&maxAge=86400)  - A web based mission control framework
* [web_video_server](https://github.com/RobotWebTools/web_video_server) ![web_video_server](https://img.shields.io/github/stars/RobotWebTools/web_video_server.svg?style=flat&label=Star&maxAge=86400)  - HTTP Streaming of ROS Image Topics in Multiple Formats
* [rvizweb](https://github.com/osrf/rvizweb) ![rvizweb](https://img.shields.io/github/stars/osrf/rvizweb.svg?style=flat&label=Star&maxAge=86400)  - RVizWeb provides a convenient way of building and launching a web application with features similar to RViz
* [marvros](https://github.com/mavlink/mavros) ![mavros](https://img.shields.io/github/stars/mavlink/mavros.svg?style=flat&label=Star&maxAge=86400)  - MAVLink to ROS gateway with proxy for Ground Control Station
* [octave](https://www.gnu.org/software/octave/) - provides a convenient command line interface for solving linear and nonlinear problems numerically, and for performing other numerical experiments using a language that is mostly compatible with Matlab.
* [streetscape.gl](https://github.com/uber/streetscape.gl) ![streetscape.gl](https://img.shields.io/github/stars/uber/streetscape.gl.svg?style=flat&label=Star&maxAge=86400)  - Streetscape.gl is a toolkit for visualizing autonomous and robotics data in the XVIZ protocol.
* [urdf-loaders](https://github.com/gkjohnson/urdf-loaders) ![urdf-loaders](https://img.shields.io/github/stars/gkjohnson/urdf-loaders.svg?style=flat&label=Star&maxAge=86400)  - URDF Loaders for Unity and THREE.js with example ATHLETE URDF File


#### Annotation
* [rviz_cloud_annotation](https://github.com/RMonica/rviz_cloud_annotation) ![rviz_cloud_annotation](https://img.shields.io/github/stars/RMonica/rviz_cloud_annotation.svg?style=flat&label=Star&maxAge=86400)  - Point cloud annotation tool based on RViz
* [PixelAnnotationTool](https://github.com/abreheret/PixelAnnotationTool) ![PixelAnnotationTool](https://img.shields.io/github/stars/abreheret/PixelAnnotationTool.svg?style=flat&label=Star&maxAge=86400)  - Annotate quickly images
* [LabelImg](https://github.com/tzutalin/labelImg) ![labelImg](https://img.shields.io/github/stars/tzutalin/labelImg.svg?style=flat&label=Star&maxAge=86400)  - LabelImg is a graphical image annotation tool and label object bounding boxes in images
* [cvat](https://github.com/opencv/cvat) ![cvat](https://img.shields.io/github/stars/opencv/cvat.svg?style=flat&label=Star&maxAge=86400)  - Powerful and efficient Computer Vision Annotation Tool (CVAT)
* [point_labeler](https://github.com/jbehley/point_labeler) ![point_labeler](https://img.shields.io/github/stars/jbehley/point_labeler.svg?style=flat&label=Star&maxAge=86400)  - Tool for labeling of a single point clouds or a stream of point clouds
* [label-studio](https://github.com/heartexlabs/label-studio) ![label-studio](https://img.shields.io/github/stars/heartexlabs/label-studio.svg?style=flat&label=Star&maxAge=86400)  - Label Studio is a multi-type data labeling and annotation tool with standardized output format
* [napari](https://github.com/napari/napari) ![napari](https://img.shields.io/github/stars/napari/napari.svg?style=flat&label=Star&maxAge=86400)  -  a fast, interactive, multi-dimensional image viewer for python
* [semantic-segmentation-editor](https://github.com/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor) ![semantic-segmentation-editor](https://img.shields.io/github/stars/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor.svg?style=flat&label=Star&maxAge=86400)  - A web based labeling tool for creating AI training data sets (2D and 3D)

#### Point Cloud
* [CloudCompare](https://github.com/CloudCompare/CloudCompare) ![CloudCompare](https://img.shields.io/github/stars/CloudCompare/CloudCompare.svg?style=flat&label=Star&maxAge=86400)  - CloudCompare is a 3D point cloud (and triangular mesh) processing software.
* [Potree](https://github.com/potree/potree) ![potree](https://img.shields.io/github/stars/potree/potree.svg?style=flat&label=Star&maxAge=86400)  - WebGL point cloud viewer for large datasets
* [point_cloud_viewer](https://github.com/googlecartographer/point_cloud_viewer) ![point_cloud_viewer](https://img.shields.io/github/stars/googlecartographer/point_cloud_viewer.svg?style=flat&label=Star&maxAge=86400)  - makes viewing massive point clouds easy and convenient
* [ParaView](https://github.com/Kitware/ParaView) ![ParaView](https://img.shields.io/github/stars/Kitware/ParaView.svg?style=flat&label=Star&maxAge=86400)  - VTK-based Data Analysis and Visualization Application
* [entwine](https://github.com/connormanning/entwine/) ![](https://img.shields.io/github/stars/connormanning/entwine/.svg?style=flat&label=Star&maxAge=86400)  - is a data organization library for massive point clouds, designed to conquer datasets of trillions of points as well as desktop-scale point clouds.
* [polyscope](https://github.com/nmwsharp/polyscope) ![polyscope](https://img.shields.io/github/stars/nmwsharp/polyscope.svg?style=flat&label=Star&maxAge=86400)  - A C++ & Python viewer for 3D data like meshes and point clouds

#### RViz
* [mapviz](https://github.com/swri-robotics/mapviz) ![mapviz](https://img.shields.io/github/stars/swri-robotics/mapviz.svg?style=flat&label=Star&maxAge=86400)  - Modular ROS visualization tool for 2D data.
* [rviz_cinematographer](https://github.com/AIS-Bonn/rviz_cinematographer) ![rviz_cinematographer](https://img.shields.io/github/stars/AIS-Bonn/rviz_cinematographer.svg?style=flat&label=Star&maxAge=86400)  - Easy to use tools to create and edit trajectories for the rviz camera.
* [rviz_satellite](https://github.com/gareth-cross/rviz_satellite) ![rviz_satellite](https://img.shields.io/github/stars/gareth-cross/rviz_satellite.svg?style=flat&label=Star&maxAge=86400)  - Display internet satellite imagery in RViz
* [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools) ![rviz_visual_tools](https://img.shields.io/github/stars/PickNikRobotics/rviz_visual_tools.svg?style=flat&label=Star&maxAge=86400)  - C++ API wrapper for displaying shapes and meshes in Rviz
* [xpp](https://github.com/leggedrobotics/xpp) ![xpp](https://img.shields.io/github/stars/leggedrobotics/xpp.svg?style=flat&label=Star&maxAge=86400)  - visualization of motion-plans for legged robots
* [rviz stereo](http://wiki.ros.org/rviz/Tutorials/Rviz%20in%20Stereo) - 3D stereo rendering displays a different view to each eye so that the scene appears to have depth
* [jsk_visualization](https://github.com/jsk-ros-pkg/jsk_visualization) ![jsk_visualization](https://img.shields.io/github/stars/jsk-ros-pkg/jsk_visualization.svg?style=flat&label=Star&maxAge=86400)  - jsk visualization ros packages for rviz and rqt

## Operation System
### Monitoring
* [lnav](http://lnav.org/) - is an enhanced log file viewer that takes advantage of any semantic information that can be gleaned from the files being viewed, such as timestamps and log levels
* [htop](https://github.com/hishamhm/htop) ![htop](https://img.shields.io/github/stars/hishamhm/htop.svg?style=flat&label=Star&maxAge=86400)  - htop is an interactive text-mode process viewer for Unix systems. It aims to be a better 'top'.
* [atop](https://github.com/Atoptool/atop) ![atop](https://img.shields.io/github/stars/Atoptool/atop.svg?style=flat&label=Star&maxAge=86400)  - System and process monitor for Linux with logging and replay function
* [psutil](https://github.com/giampaolo/psutil) ![psutil](https://img.shields.io/github/stars/giampaolo/psutil.svg?style=flat&label=Star&maxAge=86400)  - Cross-platform lib for process and system monitoring in Python
* [gputil](https://github.com/anderskm/gputil) ![gputil](https://img.shields.io/github/stars/anderskm/gputil.svg?style=flat&label=Star&maxAge=86400)  - A Python module for getting the GPU status from NVIDA GPUs using nvidia-smi programmically in Python
* [gpustat](https://github.com/wookayin/gpustat) ![gpustat](https://img.shields.io/github/stars/wookayin/gpustat.svg?style=flat&label=Star&maxAge=86400)  -  A simple command-line utility for querying and monitoring GPU status
* [nvtop](https://github.com/Syllo/nvtop) ![nvtop](https://img.shields.io/github/stars/Syllo/nvtop.svg?style=flat&label=Star&maxAge=86400)  - NVIDIA GPUs htop like monitoring tool
* [spdlog](https://github.com/gabime/spdlog) ![spdlog](https://img.shields.io/github/stars/gabime/spdlog.svg?style=flat&label=Star&maxAge=86400)  - Very fast, header-only/compiled, C++ logging library
* [ctop](https://github.com/bcicen/ctop) ![ctop](https://img.shields.io/github/stars/bcicen/ctop.svg?style=flat&label=Star&maxAge=86400)  -  Top-like interface for container metrics
* [ntop](https://github.com/ntop/ntopng) ![ntopng](https://img.shields.io/github/stars/ntop/ntopng.svg?style=flat&label=Star&maxAge=86400)  - Web-based Traffic and Security Network Traffic Monitoring
* [jupyterlab-nvdashboard](https://github.com/rapidsai/jupyterlab-nvdashboard) ![jupyterlab-nvdashboard](https://img.shields.io/github/stars/rapidsai/jupyterlab-nvdashboard.svg?style=flat&label=Star&maxAge=86400)  - A JupyterLab extension for displaying dashboards of GPU usage

### Storage and Record
* [ncdu](https://dev.yorhel.nl/ncdu) - Ncdu is a disk usage analyzer with an ncurses interface
* [borg](https://github.com/borgbackup/borg) ![borg](https://img.shields.io/github/stars/borgbackup/borg.svg?style=flat&label=Star&maxAge=86400)  - Deduplicating archiver with compression and authenticated encryption
* [bag-database](https://github.com/swri-robotics/bag-database) ![bag-database](https://img.shields.io/github/stars/swri-robotics/bag-database.svg?style=flat&label=Star&maxAge=86400)  - A server that catalogs bag files and provides a web-based UI for accessing them
* [marv-robotics](https://gitlab.com/ternaris/marv-robotics) - MARV Robotics is a powerful and extensible data management platform
* [kitti2bag](https://github.com/tomas789/kitti2bag) ![kitti2bag](https://img.shields.io/github/stars/tomas789/kitti2bag.svg?style=flat&label=Star&maxAge=86400)  - Convert KITTI dataset to ROS bag file the easy way!
* [pykitti](https://github.com/utiasSTARS/pykitti) ![pykitti](https://img.shields.io/github/stars/utiasSTARS/pykitti.svg?style=flat&label=Star&maxAge=86400)  - Python tools for working with KITTI data
* [rosbag_editor](https://github.com/facontidavide/rosbag_editor) ![rosbag_editor](https://img.shields.io/github/stars/facontidavide/rosbag_editor.svg?style=flat&label=Star&maxAge=86400)  - Create a rosbag from a given one, using a simple GUI
* [nextcloud](https://github.com/nextcloud/server) ![server](https://img.shields.io/github/stars/nextcloud/server.svg?style=flat&label=Star&maxAge=86400)  - Nextcloud is a suite of client-server software for creating and using file hosting services.
* [ros_type_introspection](https://github.com/facontidavide/ros_type_introspection) ![ros_type_introspection](https://img.shields.io/github/stars/facontidavide/ros_type_introspection.svg?style=flat&label=Star&maxAge=86400)  - Deserialize ROS messages that are unknown at compilation time
* [syncthing](https://github.com/syncthing/syncthing) ![syncthing](https://img.shields.io/github/stars/syncthing/syncthing.svg?style=flat&label=Star&maxAge=86400)  - is a continuous file synchronization program
* [rqt_bag_exporter](https://gitlab.com/InstitutMaupertuis/rqt_bag_exporter) - Qt GUI to export ROS bag topics to files (CSV and/or video)
* [xviz](https://github.com/uber/xviz) ![xviz](https://img.shields.io/github/stars/uber/xviz.svg?style=flat&label=Star&maxAge=86400)  - A protocol for real-time transfer and visualization of autonomy data
* [kitti_to_rosbag](https://github.com/ethz-asl/kitti_to_rosbag) ![kitti_to_rosbag](https://img.shields.io/github/stars/ethz-asl/kitti_to_rosbag.svg?style=flat&label=Star&maxAge=86400)  - dataset tools for working with the KITTI dataset raw data and converting it to a ROS bag. Also allows a library for direct access to poses, velodyne scans, and images.
* [ros_numpy](https://github.com/eric-wieser/ros_numpy) ![ros_numpy](https://img.shields.io/github/stars/eric-wieser/ros_numpy.svg?style=flat&label=Star&maxAge=86400)  - Tools for converting ROS messages to and from numpy arrays
* [kitti_ros](https://github.com/LidarPerception/kitti_ros) ![kitti_ros](https://img.shields.io/github/stars/LidarPerception/kitti_ros.svg?style=flat&label=Star&maxAge=86400)  - A ROS-based player to replay KiTTI dataset
* [duckdb](https://github.com/cwida/duckdb) ![duckdb](https://img.shields.io/github/stars/cwida/duckdb.svg?style=flat&label=Star&maxAge=86400)  - DuckDB is an embeddable SQL OLAP Database Management System

### Network Distributed File System
* [sshfs](https://github.com/osxfuse/sshfs) ![sshfs](https://img.shields.io/github/stars/osxfuse/sshfs.svg?style=flat&label=Star&maxAge=86400)  - File system based on the SSH File Transfer Protocol
* [moosefs](https://github.com/moosefs/moosefs) ![moosefs](https://img.shields.io/github/stars/moosefs/moosefs.svg?style=flat&label=Star&maxAge=86400)  -  a scalable distributed storage system
* [ceph](https://github.com/ceph/ceph) ![ceph](https://img.shields.io/github/stars/ceph/ceph.svg?style=flat&label=Star&maxAge=86400)  - is a distributed object, block, and file storage platform
* [nfs](https://github.com/sahlberg/libnfs) ![libnfs](https://img.shields.io/github/stars/sahlberg/libnfs.svg?style=flat&label=Star&maxAge=86400)  - is a distributed file system protocol originally developed by Sun Microsystems

### High Performance Computing
* [polyaxon](https://github.com/polyaxon/polyaxon) ![polyaxon](https://img.shields.io/github/stars/polyaxon/polyaxon.svg?style=flat&label=Star&maxAge=86400)  - A platform for reproducing and managing the whole life cycle of machine learning and deep learning applications.
* [localstack](https://github.com/localstack/localstack) ![localstack](https://img.shields.io/github/stars/localstack/localstack.svg?style=flat&label=Star&maxAge=86400)  - A fully functional local AWS cloud stack. Develop and test your cloud & Serverless apps offline
* [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) ![nvidia-docker](https://img.shields.io/github/stars/NVIDIA/nvidia-docker.svg?style=flat&label=Star&maxAge=86400)  - Build and run Docker containers leveraging NVIDIA GPUs
* [kubernetes](https://github.com/NVIDIA/kubernetes) ![kubernetes](https://img.shields.io/github/stars/NVIDIA/kubernetes.svg?style=flat&label=Star&maxAge=86400)  - Production-Grade Container Scheduling and Management
* [kubeflow](https://github.com/kubeflow/kubeflow) ![kubeflow](https://img.shields.io/github/stars/kubeflow/kubeflow.svg?style=flat&label=Star&maxAge=86400)  - Machine Learning Toolkit for Kubernetes
* [log-pilot](https://github.com/AliyunContainerService/log-pilot) ![log-pilot](https://img.shields.io/github/stars/AliyunContainerService/log-pilot.svg?style=flat&label=Star&maxAge=86400)  - Collect logs for docker containers
* [traefik](https://github.com/containous/traefik) ![traefik](https://img.shields.io/github/stars/containous/traefik.svg?style=flat&label=Star&maxAge=86400)  - The Cloud Native Edge Router
* [graylog2-server](https://github.com/Graylog2/graylog2-server) ![graylog2-server](https://img.shields.io/github/stars/Graylog2/graylog2-server.svg?style=flat&label=Star&maxAge=86400)  - Free and open source log management
* [ansible](https://github.com/ansible/ansible) ![ansible](https://img.shields.io/github/stars/ansible/ansible.svg?style=flat&label=Star&maxAge=86400)  - Ansible is a radically simple IT automation platform that makes your applications and systems easier to deploy
* [docker-py](https://github.com/docker/docker-py) ![docker-py](https://img.shields.io/github/stars/docker/docker-py.svg?style=flat&label=Star&maxAge=86400)  - A Python library for the Docker Engine API
* [noVNC](https://github.com/novnc/noVNC) ![noVNC](https://img.shields.io/github/stars/novnc/noVNC.svg?style=flat&label=Star&maxAge=86400)  - VNC client using HTML5
* [Slurm](https://github.com/SchedMD/slurm) ![slurm](https://img.shields.io/github/stars/SchedMD/slurm.svg?style=flat&label=Star&maxAge=86400)  - Slurm: A Highly Scalable Workload Manager
* [jupyterhub](https://github.com/jupyterhub/jupyterhub) ![jupyterhub](https://img.shields.io/github/stars/jupyterhub/jupyterhub.svg?style=flat&label=Star&maxAge=86400)  - Multi-user server for Jupyter notebooks
* [Portainer](https://github.com/portainer/portainer) ![portainer](https://img.shields.io/github/stars/portainer/portainer.svg?style=flat&label=Star&maxAge=86400)    - Making Docker management easy
* [enroot](https://github.com/NVIDIA/enroot) ![enroot](https://img.shields.io/github/stars/NVIDIA/enroot.svg?style=flat&label=Star&maxAge=86400)  - A simple, yet powerful tool to turn traditional container/OS images into unprivileged sandboxes.

### Embedded Operation System
* [awesome-embedded-linux](https://github.com/fkromer/awesome-embedded-linux) ![awesome-embedded-linux](https://img.shields.io/github/stars/fkromer/awesome-embedded-linux.svg?style=flat&label=Star&maxAge=86400)  - A curated list of awesome Embedded Linux resources
* [vxworks7-ros2-build](https://github.com/Wind-River/vxworks7-ros2-build) ![vxworks7-ros2-build](https://img.shields.io/github/stars/Wind-River/vxworks7-ros2-build.svg?style=flat&label=Star&maxAge=86400)  - Build system to automate the build of VxWorks 7 and ROS2
* [Yocto](https://git.yoctoproject.org/) - Produce tools and processes that enable the creation of Linux distributions for embedded software that are independent of the underlying architecture of the embedded hardware
* [Automotive Graded Linux](https://www.automotivelinux.org/software) - is a collaborative open source project that is bringing together automakers, suppliers and technology companies to build a Linux-based, open software platform for automotive applications that can serve as the de facto industry standard
* [ROSBerryPI](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi) - Installing ROS Kinetic on the Raspberry Pi
* [bitbake](https://github.com/openembedded/bitbake) ![bitbake](https://img.shields.io/github/stars/openembedded/bitbake.svg?style=flat&label=Star&maxAge=86400)  - is a generic task execution engine that allows shell and Python tasks to be run efficiently and in parallel while working within complex inter-task dependency constraints.
* [Jailhouse](https://github.com/siemens/jailhouse) ![jailhouse](https://img.shields.io/github/stars/siemens/jailhouse.svg?style=flat&label=Star&maxAge=86400)  - Jailhouse is a partitioning Hypervisor based on Linux
* [Xen](https://wiki.debian.org/Xen) - is an open-source (GPL) type-1 or baremetal hypervisor
* [QEMU](https://www.qemu.org/) - is a generic and open source machine emulator and virtualizer
* [qemu-xilinx](https://github.com/Xilinx/qemu) ![qemu](https://img.shields.io/github/stars/Xilinx/qemu.svg?style=flat&label=Star&maxAge=86400)  - A fork of Quick EMUlator (QEMU) with improved support and modelling for the Xilinx platforms
* [rosserial](https://github.com/ros-drivers/rosserial) ![rosserial](https://img.shields.io/github/stars/ros-drivers/rosserial.svg?style=flat&label=Star&maxAge=86400)  - A ROS client library for small, embedded devices, such as Arduino
* [meta-ros](https://github.com/ros/meta-ros/tree/thud-draft) ![thud-draft](https://img.shields.io/github/stars/ros/meta-ros/tree/thud-draft.svg?style=flat&label=Star&maxAge=86400)  - OpenEmbedded Layer for ROS Applications
* [meta-balena](https://github.com/balena-os/meta-balena) ![meta-balena](https://img.shields.io/github/stars/balena-os/meta-balena.svg?style=flat&label=Star&maxAge=86400)  - Run Docker containers on embedded devices
* [micro-ros](https://micro-ros.github.io/) - The major changes compared to "regular" ROS 2 is that micro-ROS uses a Real-Time Operating System (RTOS) instead of Linux, and DDS for eXtremely Resource Constrained Environments
* [nvidia-container-runtime](https://github.com/NVIDIA/nvidia-container-runtime/) ![](https://img.shields.io/github/stars/NVIDIA/nvidia-container-runtime/.svg?style=flat&label=Star&maxAge=86400)  - NVIDIA Container Runtime is a GPU aware container runtime, compatible with the Open Containers Initiative (OCI) specification used by Docker, CRI-O, and other popular container technologie
* [fusesoc](https://github.com/olofk/fusesoc) ![fusesoc](https://img.shields.io/github/stars/olofk/fusesoc.svg?style=flat&label=Star&maxAge=86400)  - Package manager and build abstraction tool for FPGA/ASIC development
* [jetson_easy](https://github.com/rbonghi/jetson_easy) ![jetson_easy](https://img.shields.io/github/stars/rbonghi/jetson_easy.svg?style=flat&label=Star&maxAge=86400)  - Automatically script to setup and configure your NVIDIA Jetson
* [docker-jetpack-sdk](https://github.com/trn84/docker-jetpack-sdk) ![docker-jetpack-sdk](https://img.shields.io/github/stars/trn84/docker-jetpack-sdk.svg?style=flat&label=Star&maxAge=86400)  -  allows for usage of the NVIDIA JetPack SDK within a docker container for download, flashing, and install
* [Pressed](https://wiki.debian.org/DebianInstaller/Preseed) - provides a way to set answers to questions asked during the installation process of debian, without having to manually enter the answers while the installation is running
* [jetson_stats](https://github.com/rbonghi/jetson_stats) ![jetson_stats](https://img.shields.io/github/stars/rbonghi/jetson_stats.svg?style=flat&label=Star&maxAge=86400)  - is a package to monitoring and control your NVIDIA Jetson [Xavier NX, Nano, AGX Xavier, TX1, TX2] Works with all NVIDIA Jetson ecosystem.


### Real-Time Kernel
* [ELISA](https://elisa.tech/) -  Project is to make it easier for companies to build and certify Linux-based safety-critical applications – systems whose failure could result in loss of human life, significant property damage or environmental damage
* [PREEMPT_RT kernel patch](https://wiki.linuxfoundation.org/realtime/documentation/start) - Aim of the PREEMPT_RT kernel patch is to minimize the amount of kernel code that is non-preemptible

### Network and Middleware
* [pyshark](https://github.com/KimiNewt/pyshark) ![pyshark](https://img.shields.io/github/stars/KimiNewt/pyshark.svg?style=flat&label=Star&maxAge=86400)  - Python wrapper for tshark, allowing python packet parsing using wireshark dissectors
* [pingtop](https://github.com/laixintao/pingtop) ![pingtop](https://img.shields.io/github/stars/laixintao/pingtop.svg?style=flat&label=Star&maxAge=86400)  - Ping multiple servers and show results in a top-like terminal UI
* [termshark](https://github.com/gcla/termshark) ![termshark](https://img.shields.io/github/stars/gcla/termshark.svg?style=flat&label=Star&maxAge=86400)  - A terminal UI for tshark, inspired by Wireshark
* [nethogs](https://github.com/raboof/nethogs) ![nethogs](https://img.shields.io/github/stars/raboof/nethogs.svg?style=flat&label=Star&maxAge=86400)  - It groups bandwidth by process
* [canmatrix](https://github.com/ebroecker/canmatrix) ![canmatrix](https://img.shields.io/github/stars/ebroecker/canmatrix.svg?style=flat&label=Star&maxAge=86400)  - Converting CAN Database Formats .arxml .dbc .dbf .kcd
* [performance_test](https://github.com/ApexAI/performance_test) ![performance_test](https://img.shields.io/github/stars/ApexAI/performance_test.svg?style=flat&label=Star&maxAge=86400)  - Tool to test the performance of pub/sub based              communication frameworks.
* [realtime_support](https://github.com/ros2/realtime_support) ![realtime_support](https://img.shields.io/github/stars/ros2/realtime_support.svg?style=flat&label=Star&maxAge=86400)  - Minimal real-time testing utility for measuring jitter and latency.
* [tcpreplay](https://github.com/appneta/tcpreplay) ![tcpreplay](https://img.shields.io/github/stars/appneta/tcpreplay.svg?style=flat&label=Star&maxAge=86400)  - Pcap editing and replay tools
* [iperf](https://github.com/esnet/iperf) ![iperf](https://img.shields.io/github/stars/esnet/iperf.svg?style=flat&label=Star&maxAge=86400)  - A TCP, UDP, and SCTP network bandwidth measurement tool
* [can-utils](https://github.com/linux-can/can-utils) ![can-utils](https://img.shields.io/github/stars/linux-can/can-utils.svg?style=flat&label=Star&maxAge=86400)  - Linux-CAN / SocketCAN user space applications
* [ros_canopen](https://github.com/ros-industrial/ros_canopen) ![ros_canopen](https://img.shields.io/github/stars/ros-industrial/ros_canopen.svg?style=flat&label=Star&maxAge=86400)  - CANopen driver framework for ROS
* [decanstructor](https://github.com/JWhitleyAStuff/decanstructor) ![decanstructor](https://img.shields.io/github/stars/JWhitleyAStuff/decanstructor.svg?style=flat&label=Star&maxAge=86400)  - The definitive ROS CAN analysis tool
* [ros1_bridge](https://github.com/ros2/ros1_bridge) ![ros1_bridge](https://img.shields.io/github/stars/ros2/ros1_bridge.svg?style=flat&label=Star&maxAge=86400)  -  ROS 2 package that provides bidirectional communication between ROS 1 and ROS 2
* [Fast-RTPS](https://github.com/eProsima/Fast-RTPS) ![Fast-RTPS](https://img.shields.io/github/stars/eProsima/Fast-RTPS.svg?style=flat&label=Star&maxAge=86400)  -  protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, as defined and maintained by the Object Management Group (OMG) consortium
* [ptpd](https://github.com/ptpd/ptpd) ![ptpd](https://img.shields.io/github/stars/ptpd/ptpd.svg?style=flat&label=Star&maxAge=86400)  - PTP daemon (PTPd) is an implementation the Precision Time Protocol (PTP) version 2 as defined by 'IEEE Std 1588-2008'. PTP provides precise time coordination of Ethernet LAN connected computers
* [wireless](https://github.com/clearpathrobotics/wireless) ![wireless](https://img.shields.io/github/stars/clearpathrobotics/wireless.svg?style=flat&label=Star&maxAge=86400)  - Making info about wireless networks available to ROS.
* [wavemon](https://github.com/uoaerg/wavemon) ![wavemon](https://img.shields.io/github/stars/uoaerg/wavemon.svg?style=flat&label=Star&maxAge=86400)  - is an ncurses-based monitoring application for wireless network devices
* [protobuf](https://github.com/protocolbuffers/protobuf) ![protobuf](https://img.shields.io/github/stars/protocolbuffers/protobuf.svg?style=flat&label=Star&maxAge=86400)  - Google's data interchange format
* [CANdevStudio](https://github.com/GENIVI/CANdevStudio) ![CANdevStudio](https://img.shields.io/github/stars/GENIVI/CANdevStudio.svg?style=flat&label=Star&maxAge=86400)  -  CANdevStudio aims to be cost-effective replacement for CAN simulation software. It can work with variety of CAN hardware interfaces
* [opensplice](https://github.com/ADLINK-IST/opensplice) ![opensplice](https://img.shields.io/github/stars/ADLINK-IST/opensplice.svg?style=flat&label=Star&maxAge=86400)  - Vortex OpenSplice Community Edition
* [ros_ethercat](https://github.com/shadow-robot/ros_ethercat) ![ros_ethercat](https://img.shields.io/github/stars/shadow-robot/ros_ethercat.svg?style=flat&label=Star&maxAge=86400)  - This is a reimplementation of the main loop of pr2_ethercat without dependencies on PR2 software
* [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds) ![cyclonedds](https://img.shields.io/github/stars/eclipse-cyclonedds/cyclonedds.svg?style=flat&label=Star&maxAge=86400)  - Eclipse Cyclone DDS is a very performant and robust open-source DDS implementation
* [udpreplay](https://github.com/rigtorp/udpreplay) ![udpreplay](https://img.shields.io/github/stars/rigtorp/udpreplay.svg?style=flat&label=Star&maxAge=86400)  - Replay UDP packets from a pcap file
* [iceoryx](https://github.com/eclipse/iceoryx) ![iceoryx](https://img.shields.io/github/stars/eclipse/iceoryx.svg?style=flat&label=Star&maxAge=86400)  - an IPC middleware for POSIX-based systems
* [cantools](https://github.com/eerimoq/cantools) ![cantools](https://img.shields.io/github/stars/eerimoq/cantools.svg?style=flat&label=Star&maxAge=86400)  - CAN BUS tools in Python 3
* [libuavcan](https://github.com/UAVCAN/libuavcan) ![libuavcan](https://img.shields.io/github/stars/UAVCAN/libuavcan.svg?style=flat&label=Star&maxAge=86400)  - is an open lightweight protocol designed for reliable communication in aerospace and robotic applications over robust vehicular networks such as CAN bus.
* [opendbc](https://github.com/commaai/opendbc) ![opendbc](https://img.shields.io/github/stars/commaai/opendbc.svg?style=flat&label=Star&maxAge=86400)  - The project to democratize access to the decoder ring of your car.
* [cabana](https://github.com/commaai/cabana) ![cabana](https://img.shields.io/github/stars/commaai/cabana.svg?style=flat&label=Star&maxAge=86400)  - CAN visualizer and DBC maker
* [rdbox](https://github.com/rdbox-intec/rdbox) ![rdbox](https://img.shields.io/github/stars/rdbox-intec/rdbox.svg?style=flat&label=Star&maxAge=86400)  - RDBOX is a IT infrastructure for ROS robots
* [CANopenNode](https://github.com/CANopenNode/CANopenNode) ![CANopenNode](https://img.shields.io/github/stars/CANopenNode/CANopenNode.svg?style=flat&label=Star&maxAge=86400)  - is the internationally standardized (EN 50325-4) (CiA301) CAN-based higher-layer protocol for embedded control system.
* [uds-c](https://github.com/openxc/uds-c) ![uds-c](https://img.shields.io/github/stars/openxc/uds-c.svg?style=flat&label=Star&maxAge=86400)  - Unified Diagnostics Service (UDS) and OBD-II (On Board Diagnostics for Vehicles) C Library
* [python-can](https://github.com/hardbyte/python-can) ![python-can](https://img.shields.io/github/stars/hardbyte/python-can.svg?style=flat&label=Star&maxAge=86400)  - The can package provides controller area network support for Python developers
* [autosar](https://github.com/cogu/autosar) ![autosar](https://img.shields.io/github/stars/cogu/autosar.svg?style=flat&label=Star&maxAge=86400)  - A set of python modules for working with AUTOSAR XML files
* [python-udsoncan](https://github.com/pylessard/python-udsoncan) ![python-udsoncan](https://img.shields.io/github/stars/pylessard/python-udsoncan.svg?style=flat&label=Star&maxAge=86400)  - Python implementation of UDS (ISO-14229) standard
* [airalab](https://github.com/airalab)  -  AIRA is reference Robonomics network client for ROS-enabled cyber-physical systems.
* [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) ![rosbridge_suite](https://img.shields.io/github/stars/RobotWebTools/rosbridge_suite.svg?style=flat&label=Star&maxAge=86400)  - provides a JSON interface to ROS, allowing any client to send JSON to publish or subscribe to ROS topics, call ROS services, and more
* [netplan](https://netplan.io/) - simply create a YAML description of the required network interfaces and what each should be configured to do

### Security
* [launch_ros_sandbox](https://github.com/ros-tooling/launch_ros_sandbox) ![launch_ros_sandbox](https://img.shields.io/github/stars/ros-tooling/launch_ros_sandbox.svg?style=flat&label=Star&maxAge=86400)  - can define launch files running nodes in restrained environments, such as Docker containers or separate user accounts with limited privileges
* [wolfssl](https://github.com/wolfSSL/wolfssl) ![wolfssl](https://img.shields.io/github/stars/wolfSSL/wolfssl.svg?style=flat&label=Star&maxAge=86400)  - is a small, fast, portable implementation of TLS/SSL for embedded devices to the cloud
* [CANalyzat0r](https://github.com/schutzwerk/CANalyzat0r) ![CANalyzat0r](https://img.shields.io/github/stars/schutzwerk/CANalyzat0r.svg?style=flat&label=Star&maxAge=86400)  - Security analysis toolkit for proprietary car protocols
* [RSF](https://github.com/aliasrobotics/RSF) ![RSF](https://img.shields.io/github/stars/aliasrobotics/RSF.svg?style=flat&label=Star&maxAge=86400)  - Robot Security Framework (RSF) is a standardized methodology to perform security assessments in robotics
* [How-to-Secure-A-Linux-Server](https://github.com/imthenachoman/How-To-Secure-A-Linux-Server) ![How-To-Secure-A-Linux-Server](https://img.shields.io/github/stars/imthenachoman/How-To-Secure-A-Linux-Server.svg?style=flat&label=Star&maxAge=86400)  - An evolving how-to guide for securing a Linux server.
* [lynis](https://github.com/CISOfy/lynis) ![lynis](https://img.shields.io/github/stars/CISOfy/lynis.svg?style=flat&label=Star&maxAge=86400)  - Security auditing tool for Linux, macOS, and UNIX-based systems. Assists with compliance testing (HIPAA/ISO27001/PCI DSS) and system hardening
* [OpenVPN](https://github.com/OpenVPN/openvpn) ![openvpn](https://img.shields.io/github/stars/OpenVPN/openvpn.svg?style=flat&label=Star&maxAge=86400)  - is an open source VPN daemon
* [openfortivpn](https://github.com/adrienverge/openfortivpn) ![openfortivpn](https://img.shields.io/github/stars/adrienverge/openfortivpn.svg?style=flat&label=Star&maxAge=86400)  - openfortivpn is a client for PPP+SSL VPN tunnel services and compatible with Fortinet VPNs
* [WireGuard](https://github.com/WireGuard/WireGuard) ![WireGuard](https://img.shields.io/github/stars/WireGuard/WireGuard.svg?style=flat&label=Star&maxAge=86400)  - WireGuard is a novel VPN that runs inside the Linux Kernel and utilizes state-of-the-art cryptography
* [ssh-auditor](https://github.com/ncsa/ssh-auditor) ![ssh-auditor](https://img.shields.io/github/stars/ncsa/ssh-auditor.svg?style=flat&label=Star&maxAge=86400)  - Scans for weak ssh passwords on your network
* [vulscan](https://github.com/scipag/vulscan) ![vulscan](https://img.shields.io/github/stars/scipag/vulscan.svg?style=flat&label=Star&maxAge=86400)  - Advanced vulnerability scanning with Nmap NSE
* [nmap-vulners](https://github.com/vulnersCom/nmap-vulners) ![nmap-vulners](https://img.shields.io/github/stars/vulnersCom/nmap-vulners.svg?style=flat&label=Star&maxAge=86400)  - NSE script based on Vulners.com API
* [brutespray](https://github.com/x90skysn3k/brutespray) ![brutespray](https://img.shields.io/github/stars/x90skysn3k/brutespray.svg?style=flat&label=Star&maxAge=86400)  - Automatically attempts default creds on found services.
* [fail2ban](https://github.com/fail2ban/fail2ban) ![fail2ban](https://img.shields.io/github/stars/fail2ban/fail2ban.svg?style=flat&label=Star&maxAge=86400)  - Daemon to ban hosts that cause multiple authentication errors
* [DependencyCheck](https://github.com/jeremylong/DependencyCheck) ![DependencyCheck](https://img.shields.io/github/stars/jeremylong/DependencyCheck.svg?style=flat&label=Star&maxAge=86400)  - is a software composition analysis utility that detects publicly disclosed vulnerabilities in application dependencies
* [firejail](https://github.com/netblue30/firejail) ![firejail](https://img.shields.io/github/stars/netblue30/firejail.svg?style=flat&label=Star&maxAge=86400)  - Firejail is a SUID sandbox program that reduces the risk of security breaches by restricting the running environment of untrusted applications using Linux namespaces, seccomp-bpf and Linux capabilities
* [RVD](https://github.com/aliasrobotics/RVD) ![RVD](https://img.shields.io/github/stars/aliasrobotics/RVD.svg?style=flat&label=Star&maxAge=86400)  - Robot Vulnerability Database. Community-contributed archive of robot vulnerabilities and weaknesses
* [ros2_dds_security](http://design.ros2.org/articles/ros2_dds_security.html) - adding security enhancements by defining a Service Plugin Interface (SPI) architecture, a set of builtin implementations of the SPIs, and the security model enforced by the SPIs
* [Security-Enhanced Linux](https://github.com/SELinuxProject/selinux) ![selinux](https://img.shields.io/github/stars/SELinuxProject/selinux.svg?style=flat&label=Star&maxAge=86400)  - is a Linux kernel security module that provides a mechanism for supporting access control security policies, including mandatory access controls (MAC)
* [OpenTitan](https://github.com/lowRISC/opentitan) ![opentitan](https://img.shields.io/github/stars/lowRISC/opentitan.svg?style=flat&label=Star&maxAge=86400)  - will make the silicon Root of Trust design and implementation more transparent, trustworthy, and secure for enterprises, platform providers, and chip manufacturers. OpenTitan is administered by lowRISC CIC as a collaborative project to produce high quality, open IP for instantiation as a full-featured product
* [bandit](https://github.com/PyCQA/bandit) ![bandit](https://img.shields.io/github/stars/PyCQA/bandit.svg?style=flat&label=Star&maxAge=86400)  - is a tool designed to find common security issues in Python code.
* [hardening](https://github.com/konstruktoid/hardening) ![hardening](https://img.shields.io/github/stars/konstruktoid/hardening.svg?style=flat&label=Star&maxAge=86400)  - A quick way to make a Ubuntu server a bit more secure.


## Datasets
* [Ford Autonomous Vehicle Dataset](https://avdata.ford.com/home/default.aspx) - Ford presents a challenging multi-agent seasonal dataset collected by a fleet of Ford autonomous vehicles at different days and times.
* [awesome-robotics-datasets](https://github.com/sunglok/awesome-robotics-datasets) ![awesome-robotics-datasets](https://img.shields.io/github/stars/sunglok/awesome-robotics-datasets.svg?style=flat&label=Star&maxAge=86400)  - A collection of useful datasets for robotics and computer vision
* [nuscenes-devkit](https://github.com/nutonomy/nuscenes-devkit) ![nuscenes-devkit](https://img.shields.io/github/stars/nutonomy/nuscenes-devkit.svg?style=flat&label=Star&maxAge=86400)  - The devkit of the nuScenes dataset
* [dataset-api](https://github.com/ApolloScapeAuto/dataset-api) ![dataset-api](https://img.shields.io/github/stars/ApolloScapeAuto/dataset-api.svg?style=flat&label=Star&maxAge=86400)  - This is a repo of toolkit for ApolloScape Dataset, CVPR 2019 Workshop on Autonomous Driving Challenge and ECCV 2018 challenge.
* [utbm_robocar_dataset](https://github.com/epan-utbm/utbm_robocar_dataset) ![utbm_robocar_dataset](https://img.shields.io/github/stars/epan-utbm/utbm_robocar_dataset.svg?style=flat&label=Star&maxAge=86400)  - EU Long-term Dataset with Multiple Sensors for Autonomous Driving
* [DBNet](https://github.com/driving-behavior/DBNet) ![DBNet](https://img.shields.io/github/stars/driving-behavior/DBNet.svg?style=flat&label=Star&maxAge=86400)  - DBNet: A Large-Scale Dataset for Driving Behavior Learning
* [argoverse-api](https://github.com/argoai/argoverse-api) ![argoverse-api](https://img.shields.io/github/stars/argoai/argoverse-api.svg?style=flat&label=Star&maxAge=86400)  - Official GitHub repository for Argoverse dataset
* [DDAD](https://github.com/TRI-ML/DDAD) ![DDAD](https://img.shields.io/github/stars/TRI-ML/DDAD.svg?style=flat&label=Star&maxAge=86400)  - is a new autonomous driving benchmark from TRI (Toyota Research Institute) for long range (up to 250m) and dense depth estimation in challenging and diverse urban conditions
* [pandaset-devkit](https://github.com/scaleapi/pandaset-devkit) ![pandaset-devkit](https://img.shields.io/github/stars/scaleapi/pandaset-devkit.svg?style=flat&label=Star&maxAge=86400)  - Public large-scale dataset for autonomous driving provided by Hesai & Scale
* [a2d2_to_ros](https://gitlab.com/MaplessAI/external/a2d2_to_ros) - Utilities for converting A2D2 data sets to ROS bags.
* [argoverse-api](https://github.com/argoai/argoverse-api) ![argoverse-api](https://img.shields.io/github/stars/argoai/argoverse-api.svg?style=flat&label=Star&maxAge=86400)  - Official GitHub repository for Argoverse dataset


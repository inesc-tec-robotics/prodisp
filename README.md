# prodisp
CARLoS Projection Display System

---
User interface for the CARLoS robot based on a tool-mounted projector and a Wii remote. This is 3'rd generation of the interface.

  - Version 1: OpenCV based. Used far CARLoS user tests carried out at AAU, Aalborg.
  - Version 2: Gazebo based. Used for minor tests at both AAU and Inesc.
  - Version 3: OpenGL based. Faster than both previous versions and with full support for interface to the CARLoS patners.

# GUIDE
Setup the system by copying the config file (config.cfg) and parse the copy as an argument when starting prodisp.
Parameters:
  - proj_matrix and proj_distortions: Intrinsic projector calibration
  - proj_resolution: As the name says...
  - proj2cam: Not used in the CARLoS setup. Instead prodisp will attempt to read projector_link from the tf tree.

# TEST MODE
Start prodisp in test mode by:
  1. roscore
  2. roslaunch prodisp tfs.launch
  3. roscd prodisp/stubs/   ->   rosparam load mission_super_weld.yaml mission
  4. rosrun prodisp prodisp
  5. roscd prodisp/stubs/   ->   bash teach.bash

# New Driver for KUKA YouBot manipulator - under development

Usage on Windows

1. Install winpcap
2. Clone this repository
3. Using cmake-gui try to configure
- set CMAKE\_INSTALL_PREFIX to a writeable folder (from Program Files), like "your binary folder"/install
- choose policy for the 3rd parties (most simply Policy\_ALL\_3RD\_PARTIES to Download all)
- give the path of your installed MATLAB (e.g. "C:\Program Files\MATLAB\R2022b")

4. Configure again (it can take some time because of the build&install of the third parties)

5. Generate, open project, build and have fun!

- The tester project is an executable to show the pure functionality of the manipulator
- The modul tester shows the functionality of the modul handling the robot on a detached thread, and task can be defined and launched on the robot
(later similarly for the gripper and the platform)

- The youbotarmmanager produces a matlab function. By building and installing the project, it will produce a matlab app in the install/MATLAB folder
that is able to perform joint angle control. 

- A few cases are not handled yet e.g. turning off the manipulator during a matlab session

The project is now in proof of concept state!

Later directions:

- driver for the gripper, and the base wheels
- cmake development for real time linux and the corresponding threads
- Jacobian-based Cartesian space methods to move the arm (as ManipulatorTask-s)
- profileing and optimising the project
- getting the firmware 2.00 for TMCM-1610 (now position mode is forbidden because of random errors of the driver), unfortunatels KUKA inc. cannot find its archive. If anyone has it, let me know...
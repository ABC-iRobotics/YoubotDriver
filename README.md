# New Driver for KUKA YouBot manipulator - under development

Usage on Windows

1. Install winpcap
2. Clone this repository
3. Using cmake-gui try to configure
4. To eliminate the errors
- give a writeable CMAKE_INSTALL_PREFIX folder, like "your binary folder"/install
- choose "Download all" for Policy_ALL_3RD_PARTIES
- give the path of your installed MATLAB (e.g. "C:\Program Files\MATLAB\R2022b")

Configure again (it can take some time because of the build&install of the third parties)

5. Generate, open project, build and have fun!

The tester project is an executable to show the functionality.

The youbotarmmanager produces a matlab function. By building and installing the project, it will produce a matlab app in the install/MATLAB folder
that is able to perform joint angle control.

The project is now in proof of concept state!

Later directions:
- driver for the gripper, and the base wheels
- cmake development for real time linux and the corresponding threads
- Jacobian-based Cartesian space methods to move the arm
- profileing and optimising the project
- making commute, calibration, etc. methods more robust

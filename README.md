# New Driver for KUKA YouBot manipulator - under development

Usage on Windows

1. Install winpcap
2. Clone this repository
3. Using cmake-gui try to configure (it can take some time because of the build&install of the third parties)
- set CMAKE\_INSTALL_PREFIX to a writeable folder (from Program Files), like "your binary folder"/install
- set "ONLY\_VIRTUAL\_ROBOT", "BUILD\_MATLAB\_WRAPPER", "BUILD\_PYTHON\_WRAPPER" to "ON"/"OFF" according to your aims
- give the path of your installed MATLAB (e.g. "C:\Program Files\MATLAB\R2022b")
- Set "Policy\_x" for x Third party to download then the cmake will try to clone/config/build and install it
("Policy\_ALL\_3RD\_PARTIES" can simplify the process)

4. Configure again

5. Generate, open project, build and have fun with the tester projects!

- The tester project is an executable to show the pure functionality of the manipulator
- The modul tester shows the functionality of the modul handling the robot on a detached thread, and task can be defined and launched on the robot
(later similarly for the gripper and the platform)

6. The install of the solution will provide a CMake package from the library and matlab/python wrappers if you have chosen them (you will find MATLAB and Python folders in the install folder)

- The matlabwrapper project produces a matlab function. By building and installing the project, it will produce a matlab app in the install/MATLAB folder
that is able to perform joint angle control. 
(A few cases are not handled yet e.g. turning off the manipulator during a matlab session)

- The youbotpython project produces the python interface.

The project is now in proof of concept state!

Later directions:

- driver for the gripper, and the base wheels
- cmake development for real time linux and the corresponding threads
- Jacobian-based Cartesian space methods to move the arm (as ManipulatorTask-s)
- profileing and optimising the project
- getting the firmware 2.00 for TMCM-1610 (now position mode is forbidden because of random errors of the driver), unfortunatels KUKA inc. cannot find its archive. If anyone has it, let me know...

## Acknowledgement
The software has been implemented with the support provided from the National Research, Development and Innovation Fund of Hungary, financed under the 2019-1.3.1-KK funding scheme (contract id: 2019-1.3.1-KK-2019-00007).

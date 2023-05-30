0. The current stateof the project is "proof of concepts", the current functions do work, but speed/accuracy are not refined at all.

Structure: youbot::Manager class is able to initialize the virtual or physical manipulator and manage it on an inner std::thread, you can see its handling in the ModulTester executable project.

1. The configuration of the motor drivers, the commutation task and the calibration task is started automatically, according to the used config file:
"config/youBotArmConfig_fromKeisler.config": autocommutation, autocalibration flags. If the manipulator was not turned off since the latest calibration, calibration is performed automatically only if "forceCalibration" is 1.

(Calibration task can be started manually, too.)

2. What are these "task"-s? Subclasses of MTask (ManipulatorTask) with overridden functions:

 "Command getCommand(manipulatorState)" - based on the provided manipulatorState, the current time, inner variables (maybe coming from other functions IN A THREAD_SAFE WAY) a command must be defined to the BLDC motors. A command is a type: joint velocity [rad/s], motor velocity [RPM], joint torque [Nm], motor current [mA], motor PWM [?], motor stop, encoder set reference [tick], and a value of the given dimension.
 
 "bool _finished()" - to help the motion layer to decide if the task is over

To perform a task, an std::shared_ptr<MTask> must be given to manager. So there can be more pointers saved, more objects can set their inner variables: e.g. a ROS component can modify the inner variables from a ROS component... BUT ONLY IN A THREAD SAFE WAY, mutexes, mutex_guards, atomic variables, etc...

If we send a new task, e.g. MotorStop - the current task will be stopped. For this reason, we have to wait till Calibration task finishes before doing anything as it is in the modultester.

3. There are 2 simple tasks, for joint position control and joint velocity control:
 MTaskRawConstantJointPosition, MTaskRawConstantJointSpeed
 
They control the robot, to the given constant joint velocity/position, can be used as an example for the ROS related task.

4. Structural comments:
- you can modify the manipulatormodul project (especially define one or more new MTask)
- you can add a new executable project to the solution via CMake - similarly to the current executables... 
- but please do inject ROS dependency into the libs, only into your new executable
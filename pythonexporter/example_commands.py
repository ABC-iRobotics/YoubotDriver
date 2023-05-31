# You need to import the package and the numpy package for its usage 
import youbotpython, numpy

# Initialization of the module
config_w_path = "../config/youBotArmConfig_fromKeisler.json"
is_virtual = True # or false for controlling a physical instance
y = youbotpython.youbot(config_w_path,is_virtual)

# Start the thread that will control/simulate the robot
# the commutation and calibration will start according to the "AutoCommutation" and "AutoCalibration"
# if the manipulator is already calibrated the calibration will only be performed if "ForceCalibration" is true
y.start_thread_and_init()

# The status of the manipulator can be checked as
status = y.get_status()

# To wait until the calibration is done
import time
status = y.get_status()
while not "CALIBRATED" in status["manipulator_status"]
    status = y.get_status()
    time.sleep(0.1)

# Joint position control to a given joint vector in degrees
q_target = numpy.array([1, 1, 1, 1, 1])
time_limit = 2000 # sec
y.joint_position(q_target,time_limit)
time.sleep(5)

# To stop the current tasks
y.stop_task()

# To use zero current mode ~free drive
time_limit = 2000 # sec
y.zero_current(time_limit)
time.sleep(5)

# Joint velocity control
dq_target = numpy.array([1, 1, 1, 1, 1])
time_limit = 3 # sec
y.joint_velocity(dq_target,time_limit)
time.sleep(5)

# Fast control without changing tasks, providing a general interface
y.start_bridged_task()
mode = numpy.array([10, 10, 10, 10, 10])# 0: STOP, 10: POSITION_JOINT_DEG,11: VELOCITY_JOINT_DEGPERSEC, 12: TORQUE_JOINT_NM
target = numpy.array([-1, -1, -1, -1,-1])
time_limit = 10 # sec
y.set_bridged_task(mode, target,time_limit)
time.sleep(5)

# To recalibrate the manipulator
# y.calibrate()

# To stop the thread that handles the manipulator
# y.stop_thread()

# delete, calling desctructor
del y
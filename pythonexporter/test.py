import youbotpython, numpy

y = youbotpython.youbot("../config/youBotArmConfig_fromKeisler.json",True)

y.start_thread_and_init()

y.get_status()

y.joint_position(numpy.array([1, 1, 1, 1, 1]),20)

y.start_bridged_task()

# 0: STOP, 10: POSITION_JOINT_RAD,11: VELOCITY_JOINT_RADPERSEC, 12: TORQUE_JOINT_NM
y.set_bridged_task(numpy.array([10, 10, 10, 10, 10]), numpy.array([-1, -1, -1, -1,-1]),10)


del y
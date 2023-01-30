obj = YoubotManager("D:/Desktop/myYoubotDrive/install/config/youBotArmConfig_fromKeisler.json",1);

%%
obj.GetTrueJointAngles
%%
obj.StartThread();
%%
obj.SetJointVelocity([1 1 -1 -1 1]*30,10);
obj.Plot();
%%
[q,dq,tau,mode] = GetStatus(obj);
q'
dq'
tau'
mode

%%
obj.SetJointVelocity([1 1 -1 -1 1]*0.2,10);
obj.StopJoints();
[q,dq,tau,mode] = GetStatus(obj)
obj.StopThread();


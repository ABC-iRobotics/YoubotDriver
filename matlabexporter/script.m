virtual = 1;
obj = YoubotManager("../config/youBotArmConfig_fromKeisler.json",virtual);
obj.StartThread();
obj.StartPlot;

%%
clear all
%%
obj.StartThread();
obj.StartPlot;
%%
obj.FreeDrive(100);
%%
obj.GetTrueJointAngles
%%
obj.SetJointVelocity([1 1 -1 1 -1]*(5),10);
%%
obj.StopJoints();
%%
[q,dq,tau,mode] = GetStatus(obj);
q',dq',tau',mode
%%
obj.SetJointVelocity(-[1 1 -1 -1 1]*0.2,10);
obj.StopJoints();
[q,dq,tau,mode] = GetStatus(obj)
%%
obj.StopPlot;
obj.StopThread();


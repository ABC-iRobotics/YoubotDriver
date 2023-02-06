%% Initialize the manipulator
virtual = 1; % choose if virtual or real robot is needed
obj = YoubotManager("../config/youBotArmConfig_fromKeisler.json",virtual);
obj.StartThread(); % Start the initialization process

% Start realtime plotter
obj.StartPlot; % the window must be closed before calling the destructor of YoubotManager
%%
obj.SetJointVelocity([5 5 -5 5 -5],10); % given joint velocity (deg/s) for given time limit

%% Stop
obj.StopJoints();

%% Zero Current mode as "FreeDrive"
obj.FreeDrive(100);

%% Getter for the current status
[q,dq,tau,mode] = GetStatus(obj);
q,dq,tau,mode

%%
clear all

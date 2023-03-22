%% Initialize the manipulator
virtual = 1; % choose if virtual or real robot is needed
obj = YoubotManager("../config/youBotArmConfig_fromKeisler.json",virtual);
obj.StartThread(); % Start the initialization process

% Start realtime plotter
obj.StartPlot; % the window must be closed before calling the destructor of YoubotManager

% Wait till init and autotasks end
[~,~,~,mode] = GetStatus(obj);
while mode.task ~= "stop"
    pause(0.001);
    [~,~,~,mode] = GetStatus(obj);
end

%%
obj.SetJointVelocity([5 5 -5 5 -5],10); % given joint velocity (deg/s) for given time limit

%%
obj.SetJointPosition([0 0 0 0 0]); % given joint velocity (deg/s) for given time limit
for i=1:10000
    [q,dq,tau,mode] = GetStatus(obj);
    pause(0.001)
    q'
end
%% To show the GetStatus function, especially the mode variable
for (i=1:1000)
    [q,dq,tau,mode] = GetStatus(obj);
    pause(0.001)
    mode
end

%% Start realtime plotter
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

classdef YoubotManager < handle
% Proof of concept state - a lot of things need to be done...

    properties (Access=private)
        ptr;
        plotter;
    end
    
    methods
        function obj = YoubotManager(configfilename,virtual)
            if (nargin==0)
                configfilename = 'D:\Tresors\WORK\PROJECT - KUKA youbot\myYouBotDriver\config/youBotArmConfig_fromKeisler.json';
            end
            if isa(configfilename,'string')
                configfilename = convertStringsToChars(configfilename);
            end
            obj.ptr = youbotarmmanager(0,configfilename,virtual);
        end
        
        function StartThread(obj)
            youbotarmmanager(5,obj.ptr);
        end
        
        function StopThread(obj)
            youbotarmmanager(7,obj.ptr);
        end
        
        function delete(obj)
            disp("YoubotManager delete started");
            obj.StopPlot;
            try
                obj.StopThread();
                youbotarmmanager(1,obj.ptr);
            catch ME
                disp(['Error: ' ME.message])
            end
            disp("YoubotManager deleted");
        end
        
        function StopJoints(obj)
            youbotarmmanager(6,obj.ptr,[],0,0);
        end
        
        function SetJointVelocity(obj, dqDegPsec, tlimit)
            youbotarmmanager(6,obj.ptr,dqDegPsec,1,tlimit);
        end
        
        function FreeDrive(obj,T)
            if nargin<2
                T = 10000;
            end
            youbotarmmanager(6,obj.ptr,0,2,T);
        end
        
        function Commutation(obj,T)
            if nargin<2
                T = 10000;
            end
            youbotarmmanager(6,obj.ptr,0,3,T);
        end
        
        function Calibration(obj,T)
            if nargin<2
                T = 10000;
            end
            youbotarmmanager(6,obj.ptr,0,4,T);
        end
        
        function q = GetTrueJointAngles(obj)
            q = youbotarmmanager(8,obj.ptr);
        end
        
        % YoubotManager.GetStatus: returns the state ofthe manipulator arm
        % q: joint angles in deg
        % dq: joint speed in deg/s
        % tau: joint torque in Nm
        % mode.task: current task
        % mode.*: bool flags about properties as
        % configinprogress/isconfigurated/commutated/calibrated
        function [q,dq,tau,mode] = GetStatus(obj)
            [q,dq,tau,mode_] = youbotarmmanager(9,obj.ptr);
            mode.calibrated = (mode_ >= 10000);
            mode_ = mode_ - (mode_ >= 10000)*10000;
            mode.commutated = (mode_ >= 1000);
            mode_ = mode_ - (mode_ >= 1000)*1000;
            mode.isconfigurated = (mode_ >= 200);
            mode_ = mode_ - (mode_ >= 200)*200;
            mode.configinprogress = (mode_ >= 100);
            mode_ = mode_ - (mode_ >= 100)*100;
            switch (mode_)
                case 10
                    mode.task = "conversion is not defined in c++";
                case 4
                    mode.task = "calibration";
                case 3
                    mode.task = "commutation";
                case 2
                    mode.task = "raw constant joint speed";
                case 1
                    mode.task = "stop";
                case 0
                    mode.task = "initialization";
                otherwise
                    mode.task = "conversion is not defined in MATLAB";
            end
        end

        function StartPlot(obj)

            obj.initGraph;
            obj.updateGraph;
        
            timer_ = timer;
            timer_.TimerFcn = @(~,~)updateGraph(obj);
            timer_.Period = 0.1;
            timer_.ExecutionMode = 'fixedRate';
            timer_.start;
            obj.plotter.timerstop = @timer_.stop;
            
        end

        function StopPlot(obj)
            obj.closeCallback;
        end
    end

    methods (Access=private)
        function closeCallback(obj,f,event)
            try % try to do the new, ideal way
                if ~isempty(obj.plotter.f)
                    obj.plotter.timerstop();
                    delete(obj.plotter.f);
                    obj.plotter.f = [];
                end
                timerstop;
            catch % do the original callback
                if isempty(gcbf)
                    if length(dbstack) == 1
                        warning(message('MATLAB:closereq:ObsoleteUsage'));
                    end
                    close('force');
                else
                    delete(gcbf);
                end
            end
        end

        function initGraph(obj)
            %%%%%%%%%%%%%%%%%%%%%% Init phase
            obj.plotter.f = figure;
            obj.plotter.f.CloseRequestFcn = @(f,event)closeCallback(obj,f,event);
            % Plot the base
            alpha = atan(40/87);
            phi = (-pi+alpha):0.1:(pi-alpha);
            plot3(0.087*cos(phi),0.087*sin(phi),0*phi,'k-','linewidth',3); hold on;grid on
            axis equal;
            box off
            % bar of the base
            x = [0 -0.02 -0.02 0];
            y = [0.04 0.04 -0.04 -0.04];
            plot3(x+0.087*cos(pi-alpha),y,x*0,'k-','linewidth',3)
            % base frame
            plot3([0 0.2],[0,0],[0,0],'r-','linewidth',3)
            plot3([0 0],[0,0.2],[0,0],'g-','linewidth',3)
            plot3([0 0],[0,0],[0,0.2],'b-','linewidth',3)

            zlim([-0.05,0.8]);
            xlim([-0.4,0.4]);
            ylim([-0.4,0.4]);

            obj.plotter.lines = cell(7,1);
            for i=1:7
                obj.plotter.lines{i} = MovingLine();
            end
        end

        function updateGraph(obj)
            if ~isempty(obj.plotter.f)
                q = obj.GetTrueJointAngles() / 180 * pi;
                %%%%%%%%%%%%%%%%%%% Recompute phase
                % Segments
                p0 = [0;0;0;1];
                l{1} = [0 0; 0 0; 0 -0.05; 1 1];
                T = HRotz(-q(1))*Tran([0.033;0;0.147]);
                p1 = T(1:4,4);
    
                a_ = [0 0; 0 0.05; 0 0; 1 1];
                l{2} = T*a_;
                T = T*HRoty(q(2))*Tran([0;0;0.155]);
                p2 = T(1:4,4);
    
                a_ = [0 0; 0 0.05; 0 0; 1 1];
                l{3} = T*a_;
                T = T*HRoty(q(3))*Tran([0;0;0.135]);
                p3 = T(1:4,4);
    
                a_ = [0 0; 0 0.05; 0 0; 1 1];
                l{4} = T*a_;
                T = T*HRoty(q(4))*Tran([0;0;0.1]);
                p4 = T(1:4,4);
    
                T = T*HRotz(q(5));
                T2 = T*Tran([0;0.035;0]);
                p5 = T2(1:4,4);
    
                a_ = [0 0; 0 0; 0 0.05; 1 1];
                l{5} = T*a_;
    
                T2 = T2*Tran([0;0;0.117]);
                p6 = T2(1:4,4);
    
                T3 = T*Tran([0;-0.035;0]);
                p7 = T3(1:4,4);
    
                T3 = T3*Tran([0;0;0.117]);
                p8 = T3(1:4,4);
    
                l{6} = [p0 p1 p2 p3 p4];
                l{7} = [p6 p5 p7 p8];
    
                for i=1:7
                    if i<=5
                        str = 'm-';
                    else
                        str = 'k-';
                    end
                    obj.plotter.lines{i}.Move(l{i},str);
                end
                pause(0.01)
            end
        end
    end
end

function R = Rotx(f)
    R = [1 0 0; 0 cos(f) -sin(f); 0 sin(f) cos(f)];
end
function R = Roty(f)
    R = [cos(f) 0 sin(f); 0 1 0; -sin(f) 0 cos(f)];
end
function R = Rotz(f)
    R = [cos(f) -sin(f) 0; sin(f) cos(f) 0; 0 0 1];
end
function T = HRotx(f)
    T = [1 0 0 0 ; 0 cos(f) -sin(f) 0; 0 sin(f) cos(f) 0; 0 0 0 1];
end
function T = HRoty(f)
    T = [cos(f) 0 sin(f) 0; 0 1 0 0; -sin(f) 0 cos(f) 0; 0 0 0 1];
end
function T = HRotz(f)
    T = [cos(f) -sin(f) 0 0; sin(f) cos(f) 0 0; 0 0 1 0; 0 0 0 1];
end
function T = Tran(z)
    T = [eye(3) z;0 0 0 1];
end
function timerstop()
  %fprintf('Entering StopTimer...\n');
  listOfTimers = timerfindall; % List all timers, just for info.
  % Get handle to the one timer that we should have.
  if isempty(listOfTimers)
    % Exit if there is no timer to turn off.
    %fprintf('There are no timers to turn off. Leaving StopTimer().\n');
    return;
  end

  i=1;
  while i<=length(listOfTimers)
    t = listOfTimers(i);
    if (t.ExecutionMode=="fixedRate")
        % Stop that timer.
        if (t.Running=="on")
            stop(t);
        end
        listOfTimers(i)=[];
        delete(t);
        disp('timer deleted')
    else
        i=i+1;
    end
  end      
  % Delete all timers from memory.
  listOfTimers = timerfindall;
  %fprintf('Left StopTimer and turned off all timers.\n');
end
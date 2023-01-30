classdef YoubotManager < handle
% Proof of concept state - a lot of things need to be done...

    properties (Access=private)
        ptr;
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
            try
                obj.StopThread();
                youbotarmmanager(1,obj.ptr);
            catch ME
                disp(['Error: ' ME.message])
            end
        end
        
        function StopJoints(obj)
            youbotarmmanager(6,obj.ptr,[],0,0);
        end
        
        function SetJointVelocity(obj, dqDegPsec, tlimit)
            youbotarmmanager(6,obj.ptr,dqDegPsec,1,tlimit);
        end
        
        function q = GetTrueJointAngles(obj)
            q = youbotarmmanager(8,obj.ptr);
        end
        
        function [q,dq,tau,mode] = GetStatus(obj)
            [q,dq,tau,mode] = youbotarmmanager(9,obj.ptr);
        end
        
        function Plot(obj)
            %%%%%%%%%%%%%%%%%%%%%% Init phase
            figure,
            % Plot the base
            alpha = atan(40/87);
            phi = (-pi+alpha):0.1:(pi-alpha);
            plot3(0.087*cos(phi),0.087*sin(phi),0*phi,'k-'); hold on;
            axis equal;
            box off
            % bar of the base
            x = [0 -0.02 -0.02 0];
            y = [0.04 0.04 -0.04 -0.04];
            plot3(x+0.087*cos(pi-alpha),y,x*0,'k-')
            % base frame
            plot3([0 0.2],[0,0],[0,0],'r-')
            plot3([0 0],[0,0.2],[0,0],'g-')
            plot3([0 0],[0,0],[0,0.2],'b-')

            zlim([-0.05,0.8]);
            xlim([-0.4,0.4]);
            ylim([-0.4,0.4]);

            lines = cell(7,1);
            for i=1:7
                lines{i} = MovingLine();
            end

            while 1
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

                a_ = [0 0; 0 -0.05; 0 0; 1 1];
                l{4} = T*a_;
                T = T*HRoty(-q(4))*Tran([0;0;0.1]);
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
                    %plotline(l{i},str);
                    lines{i}.Move(l{i},str);
                end
                pause(0.001)
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
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
        
        function FreeDrive(obj,T)
            if nargin<2
                T = 10000;
            end
            youbotarmmanager(6,obj.ptr,0,2,T);
        end
        
        function q = GetTrueJointAngles(obj)
            q = youbotarmmanager(8,obj.ptr);
        end
        
        function [q,dq,tau,mode] = GetStatus(obj)
            [q,dq,tau,mode] = youbotarmmanager(9,obj.ptr);
        end

        function StartPlot(obj)
            obj.plotter = youbotplotter(obj);
            obj.plotter.Start;
        end

        function StopPlot(obj)
            obj.plotter.Stop;
        end
    end
end

classdef YoubotManager < handle
% Proof of concept state - a lot of things need to be done...

    properties (Access=private)
        ptr;
        arm_setup;
    end
    
    methods
        function out = IsSetup(obj)
            out = obj.arm_setup;
        end
        
        function obj = YoubotManager(configfilename)
            if (nargin==0)
                configfilename = 'D:\Tresors\WORK\PROJECT - KUKA youbot\myYouBotDriver\config/youBotArmConfig_fromKeisler.json';
            end
            if isa(configfilename,'string')
                configfilename = convertStringsToChars(configfilename);
            end
            obj.ptr = youbotarmmanager(0,configfilename);
            obj.arm_setup = 0;
        end
        
        function delete(obj)
            try
                youbotarmmanager(1,obj.ptr);
            catch ME
                disp(['Error: ' ME.message])
            end
        end
        
        function setupRobot(obj)
            try 
                youbotarmmanager(2,obj.ptr);
                obj.arm_setup = 1;
            catch ME
                disp(['Error: ' ME.message])
                obj.arm_setup = 0;
            end
        end
        
        function StartProcessCommunication(obj,Ts)
            if obj.arm_setup
                youbotarmmanager(5,obj.ptr,Ts);
            end
        end
        
        function StopProcessCommunication(obj)
            youbotarmmanager(7,obj.ptr);
        end
        
        function SetJointAngles(obj, q)
            if obj.arm_setup
                youbotarmmanager(6,obj.ptr,q);
            else
                error("");
            end
        end
        
        function q = GetJointAngles(obj)
            if obj.arm_setup
                q = youbotarmmanager(8,obj.ptr);
            else
                error("");
            end
        end
    end
end


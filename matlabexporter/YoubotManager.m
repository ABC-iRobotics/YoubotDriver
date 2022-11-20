classdef YoubotManager < handle
% Proof of concept state - a lot of things need to be done...

    properties (Access=private)
        ptr;
        arm_found, arm_setup;
    end
    
    methods
        function obj = YoubotManager(configfilename)
            if (nargin==0)
                configfilename = 'D:\Tresors\WORK\PROJECT - KUKA youbot\myYouBotDriver\config/youBotArmConfig_fromKeisler.json';
            end
            if isa(configfilename,'string')
                configfilename = convertStringsToChars(configfilename);
            end
            obj.ptr = youbotarmmanager(0,configfilename);
            obj.arm_found = 0;
            obj.arm_setup = 0;
        end
        
        function delete(obj)
            try
                youbotarmmanager(1,obj.ptr);
            catch ME
                disp(['Error: ' ME.message])
            end
        end
        
        function findRobot(obj)
            try 
                youbotarmmanager(0.5,obj.ptr);
                obj.arm_found = 1;
            catch ME
                disp(['Error: ' ME.message])
                obj.arm_found = 0;
            end
        end
        
        function setupRobot(obj)
            if obj.arm_found
                try 
                    youbotarmmanager(2,obj.ptr); %config
                    youbotarmmanager(3,obj.ptr); %commute
                    youbotarmmanager(4,obj.ptr); %calibrate
                    obj.arm_setup = 1;
                catch
                    obj.arm_found = 0;
                    obj.arm_setup = 0;
                end
            else
                disp('First find the robot...')
            end
        end
        
        function StartProcessCommunication(obj)
            if obj.arm_setup
                youbotarmmanager(5,obj.ptr);
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


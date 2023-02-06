classdef MovingLine < handle
    %MOVINGLINE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        initialized = 0;
        object;
    end
    
    methods
        function obj = MovingLine()
        end
        
        function Move(obj,l,str)
            if (~obj.initialized)
                obj.object = plot3(l(1,:),l(2,:),l(3,:),str,'linewidth',3);
                obj.initialized = 1;
            else
                obj.object.XData = l(1,:);
                obj.object.YData = l(2,:);
                obj.object.ZData = l(3,:);
            end
        end
    end
end


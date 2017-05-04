%{
	Robot class.

    Explanation goes here...
%}
classdef Robot < Odometry & Sensing
    properties

    end
    
    methods
        function obj = Robot(sEnv, oTheta, oDist, sRes, sAngle)
            obj@Odometry(oTheta, oDist);
            obj@Sensing(sEnv, sRes, sAngle);
        end
    end
    
    % Functions accessible from subclasses
    methods(Access = protected)
        % Returns the laser read to the user algorithm: This class must use
        % the odometry real position to pass it into the sensing read 
        % position. This is necessary to restrict the user from getting
        % access to the real position.
        function read = laserRead(obj)
        
        end
        
        % Returns the position and the direction faced with added noise. 
        function [x, y, dir] = getPosition(obj)
            % Use the super class odometry.
        end
        
        % Move the robot in direction by a distance and return
        % noisy position
        %   dir      angle relative to robot direction in which to move
        %   dist     distance to move
        function [x, y] = move(obj, dir, dist)
            % Call super class odometry.
        end
    end
end


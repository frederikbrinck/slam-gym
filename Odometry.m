%{
	Odometry class.

    Explanation goes here...
%}
classdef Odometry
    properties
        x;              % robot x position
        y;              % robot y position
        dir;            % direction robot is facing
        maxTheta = 0.1; % constraint on rotation
        maxDist = 0.1;  % constraint on movement
    end
    
    methods
        function obj = Odometry(mTheta, mDist)
            obj.maxTheta = mTheta;
            obj.maxDist = mDist;
        end
    end
    
    % Functions accessible from subclasses
    methods(Access = protected)
        % Reurns the real position and direction of the robot.
        function [x, y, dir] = getPosition(obj)
            x = obj.x;
            y = obj.y;
            dir = obj.dir;
        end
        
        % Returns the position and the direction faced with added noise. 
        % Use the Noise.m class to add noise.
        function [x, y, dir] = getPositionNoisy(obj)
            % Use obj.getPosition
        end
        
        % Move the robot in direction by a distance and return
        % noisy position
        %   dir      angle relative to robot direction in which to move
        %   dist     distance to move
        function [x, y] = move(obj, dir, dist)
            % Set constraints on |dir - obj.dir|
            % and |dist| to make sure that we don't move unrealistically
            % using obj.maxTheta and obj.maxDist.
            
            % Allow for edge cases where dir or dist is 0, to move 
            % either straight or rotate, respectively.
        end
    end
end


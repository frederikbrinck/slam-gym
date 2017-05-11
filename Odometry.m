%{
	Created by Mufei Li on 2017/05/07.

	Implementation of Odometry class, a super 
	class for Robot.
%}

classdef Odometry

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	x;        % robot x position
    	y;		  % robot y position 
    	theta;	  % direction robot is facing
    	maxTheta; % constraint on rotation
    	maxDist;  % constraint on movement 
        radius;   % radius of robot
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	%{
    		Constructor for Odometry

    		Theta is the relative angle of the robot 
    		and the x axis and is in terms of degrees.
    	%} 
    	function obj = Odometry(x, y, theta, radius, maxTheta, maxDist)
    		if nargin < 4
    			maxTheta = 30;
    			maxDist = 0.1;
                radius = 0.5;
    		end

    		obj.x = x;
    		obj.y = y;
    		obj.theta = theta;
    		obj.maxTheta = maxTheta;
    		obj.maxDist = maxDist;
            obj.radius = radius;
    	end

    	% return current position of the Odometry.
    	function p = getPosition(obj)
    		p = [obj.x obj.y obj.theta];
    	end

    	% Move in direction dx, dy and dtheta.
    	function move(obj, dx, dy, dtheta)
    		% Make sure our robot moves reasonably.
    		if dx > obj.maxDist
    			dx = obj.maxDist;
    		end
    		if dy > obj.maxDist
    			dy = obj.maxDist;
    		end
    		if dtheta > obj.maxTheta
    			dtheta = obj.maxTheta;
    		end

    		obj.x = obj.x + dx;
    		obj.y = obj.y + dy;
    		obj.theta = obj.theta + dtheta;
    	end
    end
end
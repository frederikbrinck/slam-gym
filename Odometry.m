%{
	Created by Mufei Li on 2017/05/07.

	Implementation of Odometry class, a super 
	class for Robot. Odometry handles all movement of the robot.
%}

classdef Odometry < handle

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	x;        % robot x position
    	y;		  % robot y position 
    	theta;	  % direction of the robot with respect to the x-axis
        radius;   % radius of robot

    	maxTheta; % constraint on rotation
    	maxDist;  % constraint on movement 
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
                radius = 0.5;
            end
            if nargin < 5
                maxTheta = 30;
    			maxDist = 0.1;
            end

            % Set positions.
    		obj.x = x;
    		obj.y = y;
    		obj.theta = theta;
            
            % Set constraints and radius.
            obj.radius = radius;
    		obj.maxTheta = maxTheta;
    		obj.maxDist = maxDist;
    	end

    	% Return current position of the Odometry.
    	function [x, y, theta] = getPosition(obj)
    		x = obj.x;
            y = obj.y;
            theta = obj.theta; 
        end


    	% Move in direction dx, dy and dtheta and update the position
        % correctly. The move function manipulates the ground truth.
    	function pos = move(obj, dx, dy, dtheta)
    		obj.x = obj.x + dx;
    		obj.y = obj.y + dy;
    		obj.theta = obj.theta + dtheta;
            pos = [obj.x, obj.y, obj.theta];
        end
        
        % Move function used by the robot implementation: it updates 
        % the groundtruth with the updated variables without noise,
        % and then updated the estimated position given in controls with
        % the update and noise. Finally, return the noisy position.
        %   controls =  3d vector with x, y and theta
        %   update   =  3d vector with dx, dy, and theta
        function [update, noise] = moveNoisy(obj, state, s, t, noise)            
            % Check for movement constraints before update and
            % returning the noisy position.
            if s > obj.maxDist
    			s = obj.maxDist;
            end
    		if t > obj.maxTheta
    			t = obj.maxTheta;
            end
            if t < -obj.maxTheta
                t = -obj.maxTheta;
            end
            
            % Move the ground truth without noise.
            obj.move(s * cosd(t + obj.theta), s * sind(t + obj.theta), t);

            % Get noise and return position with noise accordingly.
            if s == 0 && t == 0
                % No movement, so no noise.
                update = [0 0];
                noise = [0 0];
            elseif s == 0
                % Only rotation, so add rotation noise.
                update = [0 t];
                noise = [0 ; noise(2) .* Noise.gaussian(1,1)];
            else
                % Movement so add noise.
                update = [s ; t];
                noise = noise .* Noise.gaussian(2,1);
            end
        end
    end
end
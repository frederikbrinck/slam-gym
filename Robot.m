%{
	Created by Mufei Li on 2017/05/04.

	This file is an abstract robot class that serves
	as the superclass of Algorithm.
%}

classdef Robot < Odometry
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	% Set the threshold for the laser scanner. If the laser scanner 
    	% cannot tell the exact length for a specific angle or if the 
    	% length is bigger than the threshold, the laser scanner will 
    	% simply return the threshold as the range for the angle.
    	degreesPerScan = 5; % This is in terms of degrees, not radians.
        sensing;
        dir;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	% The Robot must be instantiated with certain parameters:
        %   x =         x position
        %   y =         y position
        %   theta =     start theta with respect to the x-axis
        %   radius =    robot radius
        %   limit
    	function obj = Robot(x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorEnv, sensorAngle, sensorThreshold)
            if nargin < 7
    			maxTheta = 180;
    			maxDist = 0.1;
            end
            
    		% Call the constructor for odometry.
    		obj = obj@Odometry(x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed);
            % Create a sensing object for this robot
    		obj.sensing = Sensing(sensorEnv, obj.degreesPerScan, sensorAngle, sensorThreshold, obj.radius);
            obj.dir = [cos(deg2rad(obj.theta));sin(deg2rad(obj.theta))];
    	end

    	function scanOutput = laserRead(obj)
    		scanOutput = obj.sensing.laserRead(obj.x,obj.y,obj.dir);
        end
        
        function read = laserReadPoints(obj)
            read = obj.sensing.laserReadPoints(obj.x, obj.y, obj.theta);
        end
        
        function [range, bearing] = computeBearing(obj, position)
            [x, y, theta] = obj.getPosition();
            u = [cosd(theta) sind(theta) 0];
            v = [(position - [x y]) 0];
            range = norm(v);
            bearing = atan2d(norm(cross(v, u)),dot(v, u));
            if ~Geom2d.leftOf([x y], [(x + u(1)) (y+u(2))], position)
                bearing = -bearing;                
            end
        end
        
        function [] = plotPoints(obj, points)
            pos = obj.getPosition();
            obj.sensing.plotPoints(pos(1), pos(2), pos(3), points);
        end
        
        function out = drawRobot(obj)
            % Robot
            hold on
            a = Draw.disc([obj.x,obj.y],obj.radius,360,90,[0,0,0]);
            arr = Draw.arrow([obj.x,obj.y],obj.radius,obj.theta);
            hold off
            out = [arr(1),arr(2),arr(3),a];
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function r = test()
            filename = 'environments/env2.txt';
            env = Environment;
            env = env.readFile(filename);
            env.showEnv();
            x = 2;
            y = 2;
            theta = 90;
            rad = 0.5;
            limit = 3;
            r = Robot(x,y,theta,rad,limit,env);
            r.drawRobot();
            read = r.laserReadPoints();
            r.plotPoints(read);
        end
    end 
end
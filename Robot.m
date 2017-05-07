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
    	laserThreshold = 10;
    	degreesPerScan = 5; % This is in terms of degrees, not radians.
        sensing;
        dir;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    	% Constructor for Robot
    	function obj = Robot(x, y, theta, maxTheta, maxDist, env)
    		if nargin < 4
    			maxTheta = 30;
    			maxDist = 0.1;
    		end

    		% Call the constructor for odometry.
    		obj = obj@Odometry(x, y, theta, maxTheta, maxDist);
            % Create a sensing object for this robot
    		obj.sensing = Sensing(env, obj.degreesPerScan,...
                maxTheta, obj.laserThreshold);
            obj.dir = [cos(deg2rad(obj.theta));sin(deg2rad(obj.theta))];
    	end

    	function scanOutput = laserRead(obj)
    		scanOutput = obj.sensing.laserRead(obj.x,obj.y,obj.dir);
    	end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function r = test()
            filename = 'environments/env.txt';
            env = Environment;
            env = env.readFile(filename);
            env.showEnv();
            r = Robot(8,2,90,180,10,env);
            read = r.laserRead();
            r.sensing.plotScan(r.x,r.y,r.dir,read);
        end
    end 
end
%{
	Created by Mufei Li on 2017/05/04.

	This file is an abstract robot class that serves
	as the superclass of Algorithm.
%}

classdef Robot < Odometry & Sensing

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	% Set the threshold for the laser scanner. If the laser scanner 
    	% cannot tell the exact length for a specific angle or if the 
    	% length is bigger than the threshold, the laser scanner will 
    	% simply return the threshold as the range for the angle.
    	laserThreshold = 8.1;
    	degreesPerScan = 0.5; % This is in terms of degrees, not radians.
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    	% Constructor for Robot
    	function obj = Robot(x, y, theta, maxTheta, maxDist)
    		if nargin < 4
    			maxTheta = 30;
    			maxDist = 0.1;
    		end

    		% Call the constructor for odometry.
    		obj = obj@Odometry(x, y, theta, maxTheta, maxDist);
    	end

    	function scanOutput = laserRead(obj)
    		scanOutput = Sensing.laserRead(obj.laserThreshold, ... 
    			obj.degreesPerScan);
    	end
    end
end

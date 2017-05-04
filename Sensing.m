%{
	Sensing class.

    Explanation goes here...
%}
classdef Sensing
    properties
        env;                % must be initialised with environment
        resolution = 20;    % resolution of readings
        angle = pi;        % angle of readings
    end
    
    methods(Access = public)
        % Our constructor function setting the needed parameters
        %   env         the environment
        %   res         the resolution of the laser
        %   angle       the angle for the readings
        function obj = Sensing(env, res, angle)
            obj.env = env;
        end
        
        % Returns a reading from the current environment over direction
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        function read = laserRead(obj, x, y, dir)
            % Use Geom2.d to do math.
        end
        
        % Returns a reading from the current environment and adds
        % noise. The reading is a long array of distance measures 
        % scanning from left to right. There should be resolution/angle
        % (in degree) number of readings. Use Noise.m to add noise.
        function read = laserReadNoisy(obj, x, y, dir)
           % Use obj.laserRead and add noise.
        end
    end
    
end


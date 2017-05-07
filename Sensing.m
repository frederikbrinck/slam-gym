%{
	Sensing class.

    Explanation goes here...
%}
classdef Sensing
    properties
        env;                % must be initialised with environment
        resolution = 1;     % resolution of readings
        angle = pi;         % angle of readings
        limit = 10;         % maximum range of sensor
    end
    
    methods(Access = public)
        % Our constructor function setting the needed parameters
        %   env         the environment
        %   res         the resolution of the laser
        %   angle       the angle for the readings in degrees
        %   limit       maximum range of sensor
        function obj = Sensing(env, res, angle, lim)
            obj.env = env;
            obj.resolution = res;
            obj.angle = angle;
            obj.limit = lim;
        end
        
        % Returns a reading from the current environment over direction
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        % It returns an array of distances to the closest obstacle for
        % various angles (determined by angle and resolution)
        function read = laserRead(obj, x, y, dir)
            % Use Geom2.d to do math.
            % We read obj.angle radians from startVec to endVec
            % We read angles in degrees based on the resolution
            read = ones(round(obj.angle/obj.resolution)+1,1);
            startAng = -obj.angle/2;
            endAng = obj.angle/2;
            count = 1;            
            for i = endAng:-obj.resolution:startAng
                a = deg2rad(i);
                R = [cos(a),-sin(a);sin(a),cos(a)];
                % Calculating the vector
                vec = R*dir;
                sep = Geom2d.closestPoint(x,y,vec,obj.env,obj.limit);
                read(count) = sep;
                count = count+1;
                plot([x,x+vec(1)*sep],[y,y+vec(2)*sep])
            end
        end
        
        % Plots the lines of the laser scan
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        %   read    the array of separations from the scan
        function plotScan(obj,x,y,dir,read)
            obj.env.showEnv();
            count = 1;            
            for i = obj.angle/2:-obj.resolution:-obj.angle/2
                a = deg2rad(i);
                R = [cos(a),-sin(a);sin(a),cos(a)];
                % Calculating the vector
                hold on;
                vec = R*dir;            
                plot([x,x+vec(1)*read(count)],[y,y+vec(2)*read(count)])
                count = count+1;
            end
            hold off;
        end
        
        % Returns a reading from the current environment and adds
        % noise. The reading is a long array of distance measures 
        % scanning from left to right. There should be resolution/angle
        % (in degree) number of readings. Use Noise.m to add noise.
        function read = laserReadNoisy(obj, x, y, dir)
           % Use obj.laserRead and add noise.
           cleanRead = obj.laserRead(x,y,dir);
           read = cleanRead + Noise.addNoise();
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function s = test(filename)
            if nargin < 1
                filename = 'environments/env.txt';
            end
            a = Environment;
            a = a.readFile(filename);
            res = 5;
            limit = 10;
            angle = 180;
            s = Sensing(a,res,angle,limit);
            x = 8;
            y = 2;
            dir = randi([-5 5], 2,1);
            disp('Sensing');
            dir = dir/norm(dir);
            disp('Displaying the scan as an array');
            read = s.laserRead(x,y,dir);
            disp(read);
            disp('Plotting the lines');
            s.plotScan(x,y,dir,read);
        end
    end 
end
%{
	% Sensing class.

    % This is a property of the Robot class
    % This class helps with laser scanning and returns an array of 
    % separation from the nearest obstacle given the robot position. It
    % scans from left to right and scans for a number of degrees specified
    % by angle. 
    
    % This output can be plotted using the plotScan() function.
%}
classdef Sensing < handle
    properties
        env;                % must be initialised with environment
        resolution = 1;     % resolution of readings
        angle = pi;         % angle of readings
        limit = 10;         % maximum range of sensor
        radius = 1;         % radius of robot
    end
    
    methods(Access = public)
        % Our constructor function setting the needed parameters
        %   env         the environment
        %   res         the resolution of the laser
        %   angle       the angle for the readings in degrees
        %   limit       maximum range of sensor
        function obj = Sensing(env, res, angle, lim, radius)
            if nargin < 2
                res = 10;
                angle = 180;
                lim = 5;
                radius = 0.5;
            end
            
            obj.env = env;
            obj.resolution = res;
            obj.angle = angle;
            obj.limit = lim;
            obj.radius = radius;
        end
        
        % Returns a reading from the current environment over direction
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        % It returns an array of distances to the closest obstacle for
        % various angles (determined by angle and resolution).
        % Note that this is currently not in use, since we are only dealing
        % with points.
        function read = laserRead(obj, x, y, dir)
            % We read obj.angle radians from startAng to endAng
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
            end
        end
        
        % Returns a reading from the current environment over direction
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        % It returns an array of points if we are assuming that we deal 
        % with point obstacles in the environment. These points are within
        % the sector of the circle defined by the maximum distance that the
        % robot can scan and the points in the environment and the angle of
        % scan (which we assume is 180 degrees)
        function read = laserReadPoints(obj,x,y,dir)
            % Use Geom2.d to do math.
            r = obj.limit;
            circleRead = Geom2d.pointsInsideCircle(x,y,obj.env,r);
            read = [];
            c = cosd(dir);
            s = sind(dir);
            % Ends of the semi-circle
            p1 = [x-r*s, y+r*c];
            p2 = [x+r*s, y-r*c];
            for i = 1:size(circleRead,1)
                % Obstacle point
                p3 = [circleRead(i,1) circleRead(i,2)];
                if Geom2d.leftOf(p1,p2,p3)
                    read = [read; circleRead(i,:)];
                end
            end
        end
        
        % Plots the points of the laser scan
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        %   read    the array of separations from the scan
        function scan = plotPoints(obj,x,y,dir,read)
            r = obj.limit;
            scan = [];
            hold on
            % Semi circle
            scan(1) = Draw.disc([x y],r,obj.angle,dir, [0.5, 0.5, 0]);   
            for i = 1:size(read, 1)
                p3 = [read(i,1) read(i,2)];
                scan(end+1) = Draw.disc(p3,0.1,360,90,[0,0,0]);
            end
            hold off
        end
        
        % Plots the lines of the laser scan
        % where
        %   x       the real x position of the robot
        %   y       the real y position of the robot
        %   dir     the direction of the robot
        %   read    the array of separations from the scan
        % This assumes that read is an array of distances and so should not
        % be used when we scan point polygons and return an array of points
        % in the environment.
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
        % Note that this function is not in use; rather we add noise
        % manually in the Algorithm class.
        function read = laserReadNoisy(obj, x, y, dir)
           % Use obj.laserRead and add noise.
           cleanRead = obj.laserRead(x,y,dir);
           read = cleanRead + Noise.addNoise();
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Show a test of the laser scan.
        function s = test(filename)
            if nargin < 1
                filename = 'environments/env1.txt';
            end
            a = Environment;
            a = a.readFile(filename);
            res = 5;
            limit = 3;
            angle = 180;
            s = Sensing(a,res,angle,limit,1);
            x = 4;
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
        
        % Show a test of the points scan.
        function s = testPoints(filename)
            if nargin < 1
                filename = 'environments/env1.txt';
            end
            a = Environment;
            a = a.readFile(filename);
            s = Sensing(a);
            a.showEnv();
            x = 7;
            y = 6;
            dir = 140;
            disp('Sensing');
            read = s.laserReadPoints(x,y,dir);
            s.plotPoints(x,y,dir,read);
        end
        
    end
end
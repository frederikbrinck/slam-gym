classdef Algorithm < Robot
    properties
        landmarks;
        landmarkId = 1;
        maxAssociateError = 0.1
        maxLandmarkError = 0.2;
    end
    
    methods
        function obj = Algorithm(x, y, theta, radius, limit, env,...
                maxTheta, maxDist)
            obj = obj@Robot(x, y, theta, radius, limit, env,...
                maxTheta, maxDist);
            obj.landmarkId = 4;

        end
        
        function [] = changeVar(obj)
            disp('Called...');
            obj.landmarkId = 3;
            disp(obj.landmarkId);
            disp(obj);
        end
        
        function lm = associateLandmark(obj, x, y)
           % Associate landmark
           lm = false;
           for i = 1:length(obj.landmarks)
               cLm = obj.landmarks(i);
               if norm([x y] - cLm.position) < obj.maxAssociateError
                   lm = cLm;
                   break;
               end
           end
           
           % Add landmark to database if it isn't to close to another
           % landmark
           if ~isa(lm, 'Landmark')
               tooClose = false;
               for i = 1:length(obj.landmarks)
                   cLm = obj.landmarks(i);
                   if norm([x y] - cLm.position) < obj.maxLandmarkError
                       tooClose = true;
                       break;
                   end
               end
               
               if ~tooClose
                   lm = Landmark();
                   disp('Creating landmarks');
                   disp(obj.landmarkId);
                   lm.id = obj.landmarkId;
                   obj.landmarks = [obj.landmarks lm];
                   obj.landmarkId = 6;
                   disp('Hi Jovan');
                   disp(obj.landmarkId);
               end
           end
        end
        
        function read = getFeatures(obj)
            conf = obj.getPosition();
            x = conf(1);
            y = conf(2);
            theta = conf(3);
            points = obj.laserReadPoints();
            read = {};
            for i = 1:length(points)
                p = points{i};
                disp(obj.landmarks);
                lm = obj.associateLandmark(p(1), p(2));
                if isa(lm, 'Landmark')
                    hold on
                    robToPoint = p - [x y];
                    dist = norm(robToPoint);
                    Draw.segment(p, [x y], [1, 0, 0]);

                    dir = [cosd(theta) sind(theta) 0];
                    p3 = [robToPoint 0];
                    bearing = atan2d(norm(cross(p3,dir)),dot(p3,dir));
                    %bearing = rad2deg(acos(dot(robToPoint, [x y]) / (norm(robToPoint) * norm([x y]))));

                    Draw.arrow([x,y],dist,bearing + theta);
                    hold off
                    % See if this point already exists and set
                    % i accordingly
                    read{i} = [dist bearing, lm.id];
                    lm.range = dist;
                    lm.bearing = bearing;
                end
            end
        end
    end
    
    methods(Static)
        function s = test(filename)
            if nargin < 1
                filename = 'environments/env2.txt';
            end
            env = Environment;
            env = env.readFile(filename);
            env.showEnv();
            
            alg = Algorithm(1, 0, 30, 0.9, 6, env,...
                180, 0.5);
            features = alg.getFeatures();
            disp(alg.landmarkId);
            alg.changeVar();
            disp(alg);
            alg.drawRobot();
            alg.plotPoints({});
        end
    end
end
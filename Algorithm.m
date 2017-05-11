classdef Algorithm < Robot
    
    properties
    end
    
    methods
        function obj = Algorithm(x, y, theta, radius, limit, env,...
                maxTheta, maxDist)
            obj = obj@Robot(x, y, theta, radius, limit, env,...
                maxTheta, maxDist);
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
                robToPoint = p - [x y];
                dist = norm(robToPoint);
                bearing = acos(dot(robToPoint, [x y]) / (norm(robToPoint) * norm([x y])));
                % See if this point already exists and set
                % i accordingly
                read{i} = [dist bearing, i];
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
            
            alg = Algorithm(1, 1, 20, 0.9, 4, env,...
                180, 0.5);
            features = alg.getFeatures();
            alg.plotPoints(features);
        end
    end
end
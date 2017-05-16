%{
	Map class.

    Explanation goes here...
%}

classdef Map < handle
    properties
        pointSet        % given as [X Y]
        estimatedSet    % given as {i} = [X[] Y[]]
    end
    
    methods
        function obj = Map
            
        end
        
        % Adds an obstacle with X and Y being the coordinates,
        % where length(X)=1 is a point, length(X)=2 is a line and 
        % length(X)>2 is a polygon in counter clockwise order.
        function [] = addObstacle(obj, X, Y)
            % Add the obstacle to the estimated set
            obj.estimatedSet{end + 1} = [X; Y];
        end
        
        % Adds a point to the point set for later use of 
        function [] = addPoint(obj, x, y)
            obj.pointSet(1, end + 1) = x;
            obj.pointSet(2, end + 1) = y;
        end
        
        % Run the RANSAC algorithm on the point set to obtain
        % an estimated map.
        function eset = computeRansac(obj)
            % Implement RANSAC in different file.
        end
        
        % Run the Spikes algorothm on the point set to obtain an 
        % estimated map.
        function eset = computeSpikes(obj)
            % Implement Spikes in different file.
        end
       
        % Show the estimated set and the robot.
        function [] = showMap(obj, robot, bool)
            if nargin < 3
                bool = true;
            end
            
            % Set map if needed.
            if bool
                figure(1);
                clf(1);
                axis square tight;
            end
            
            % Display obstacles in brown.
            brown = [0.8, 0.2, 0.5];
            Draw.polygons(obj.estimatedSet, brown);

            % Display robot
            blue = [0, 0.3, 0.8];
            Draw.disc([robot.x robot.y], robot.r, blue);
            % Transparency (to show overlaps).
            alpha(0.3);
        end
        
        % Perform any map resetting needed to be able to run a new
        % algorithm.
        function resetMap(obj)
            
        end
    end
    
    methods(Static)
        % Scores the map based on the time, the bumps, and the f-measure
        % for the oset and est.
        %   time    the running time of the algorithm
        %   bumps   the number of times the algorithm hit the walls
        %   oset    the obstacle set [X{1..m} Y{1..m}]
        %   eset    the estimated set [X{1..n} Y{1..n}]
        function [precision, recall, score] = score(time, bumps, oset, eset)
            % Define alpha and beta
            alpha = 0.99;
            beta = 0.05;
            
            % Compute intersection.
            overlap = Geom2d.polygonSetIntersect(oset, eset);
            % Compute precision.
            esetA = Geom2d.polygonSetArea(eset);
            precision = Geom2d.polygonSetArea(overlap) / esetA;
            % Compute recall.
            osetA = Geom2d.polygonSetArea(oset);
            recall = Geom2d.polygonSetArea(overlap) / osetA;
            % Compute harmonic mean.
            F = 2 * (precision * recall) / (precision + recall);
            
            % Return the thresholded score.
            accuracy = alpha^time * F - beta * bumps;
            score = max(0, accuracy);
        end
        
        % Simple function that tests functionality of the map class.
        function [] = test(filename)
            if nargin < 1
                filename = 'environments/env.txt';
            end
            
            % Show environment.
            disp('Showing environment');
            a = Environment;
            a = a.readFile(filename);
            a.showEnv(); % Show the entire environment
            
            % Create robot
            robot.x = 1;
            robot.y = 1;
            robot.r = 0.7;
            
            % Create polygon
            disp('Showing estimated map.');
            m = Map;
            m.addObstacle([2, 8, 8, 2], [2, 2, 8, 8]);
            m.showMap(robot, false);
                        
            [precision, recall, score] = Map.score(20, 2, a.polygons, m.estimatedSet);
            fprintf('Computed scores for 20 seconds and 2 bumps: \n\tScore: %f\n\tPrecision: %f\n\tRecall: %f\n', score, precision, recall);
            
        end
    end
end


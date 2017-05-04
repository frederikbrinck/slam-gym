%{
	Map class.

    Explanation goes here...
%}

classdef Map
    properties
        pointSet        % given as [X Y]
        estimatedSet    % given as [X{1..n} Y{1..n}]
    end
    
    methods
        % Adds an obstacle with X and Y being the coordinates,
        % where length(X)=1 is a point, length(X)=2 is a line and 
        % length(X)>2 is a polygon in counter clockwise order.
        function [] = addObstacle(obj, X, Y)
            % Add the obstacle to the estimated set
        end
        
        % Adds a point to the point set for later use of 
        function [] = addPoint(obj, x, y)
            
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
       
        % Show the estimated set and the robot
        function [] = showMap(obj, robot)
            
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
        function score = score(time, bumps, oset, eset)
            % If eset is empty, use the class' estimatedSet
        end
    end
    
end


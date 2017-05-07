%{
	Geom2d class.

    Explanation goes here...
%}

classdef Geom2d
    properties
    end
   
	methods (Static = true)
        function bool = isPoint(poly)
            bool = (length(poly(1,:)) == 1); 
        end
        
        % Intersects two polygons by using the polygon clip algorithm library,
        % which hooks into the C implementation of the gcp algorithm that
        % runs in O((n + k)log n) where n is the number of lines, and k
        % is the number of crossings. Note that this wrapper does not
        % allow for polygons with holes, but it can be extended to do so.
        %
        % The input is given as polygonal sets, i.e. a{i} = [X[] Y[]].
        function intersect = polygonSetIntersect(a, b)
            % Recast input.
            for poly = 1:length(a)
                p1(poly) = Geom2d.castPolygonToClip(a{poly});
            end
            
            for poly = 1:length(b)
                p2(poly) = Geom2d.castPolygonToClip(b{poly});
            end
            
            % Do intersection.
            p = PolygonClip(p1, p2, 1);
            for i = 1:length(p)
               intersect{i} = [p(i).x p(i).y]'; 
            end
        end
        
        % Recasts the polygon to the format used for the clipping
        % algorithm.
        function polygon = castPolygonToClip(a)
            polygon.x = a(1, :);
            polygon.y = a(2, :);
            polygon.hole = 0;
        end
        
        % Given a set of polygons in the form a{i} = [X[] Y[]], 
        % computes the total area.
        function area = polygonSetArea(a)
            area = 0;
            for polygon = a
                polygon = polygon{1};
                area = area + Geom2d.polygonArea(polygon);
            end
        end
        
        % Wrapper of the polyarea function that returns the
        % area of one polygon a = [X[] Y[]]
        function area = polygonArea(a)
            area = polyarea(a(1, :), a(2, :));
        end
        
        function [] = test(filename)
            if nargin < 1
                filename = 'environments/env.txt';
            end
            
            disp('Showing environment.');
            a = Environment;
            a = a.readFile(filename);
            a.showEnv(); % Show the entire environment.
            
            disp('Showing estimated environment.');
            polygonSet = {};
            polygonSet{1} = [[2, 4, 4]; [2, 2, 7]];
            polygonSet{2} = [[6, 9, 9, 7.5]; [8, 8, 9, 9]];
            Draw.polygons(polygonSet, [0, 0.3, 0.6]);
            alpha(0.3);
            
            disp('Showing intersection set.');
            set = Geom2d.polygonSetIntersect(polygonSet, a.polygons);
            Draw.polygons(set, [0, 0.8, 0]);
            alpha(0.3);
            
            disp('Calculates intersected area.');
            disp(['Area:', num2str(Geom2d.polygonSetArea(set))])
        end
    end
end


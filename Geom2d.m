%{
	Geom2d class.

    Explanation goes here...
%}

classdef Geom2d
  
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
        
           
        % Function called by laser read that converts the polygons to edges
        % and calls sepPointEdgeDir for all the edges in the polygons. It
        % returns the separation from the robot point (x,y) to the closest
        % point in the environment. 
        function minSep = closestPoint(x,y,vec,env,lim)
            minSep = lim;
            edge = [0,10;10,10];
            for i = 1:1
                sep = Geom2d.sepPointEdgeDir(x,y,vec,edge);
                if (sep < minSep)
                    minSep = sep;
                end
            end
        end
        
        % Function used to find the separation from a point (x,y)
        % to an edge in a particular direction, given by vec
        % Useful for laser read
        function sep = sepPointEdgeDir(x,y,vec,edge)
            % Help obtained from here
            % https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
            query = [x,y;x+vec(1)*10,y+vec(2)*10];
            sep = 100;
            x1 = x;
            y1 = y;
            x2 = query(2,1);
            y2 = query(2,2);
            x3 = edge(1,1);
            y3 = edge(1,2);
            x4 = edge(2,1);
            y4 = edge(2,2);
            
            % let pX,pY be the intersection point
            pxNum = (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4);
            pxDen = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
            pyNum = (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4);
            pyDen = (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4);
            pX = pxNum/pxDen;
            pY = pyNum/pyDen;
            if Geom2d.onEdge([pX,pY],edge)
                sep = norm([pX-x,pY-y]);
            end
        end
        
        % Returns true if a point is contained within an edge. Useful for 
        % figuring out whether an intersection point is valid or not.
        function bool = onEdge(point,edge)
            if point(1) <= max(edge(1,1),edge(2,1)) ...
               && point(1) >= min(edge(1,1),edge(2,1)) ...
               && point(2) <= max(edge(1,2),edge(2,2)) ...
               && point(2) >= min(edge(1,2),edge(2,2))
               bool = 1;
            else
                bool = 0;
            end
        end
        
        function test2()
            filename = 'environments/env.txt';
            a = Environment;
            a = a.readFile(filename);
            Geom2d.closestPoint(0,1,[0,1],a,5);
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

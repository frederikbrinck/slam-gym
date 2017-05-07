%{
	Geom2d class.

    Explanation goes here...
%}

classdef Geom2d
  
	methods (Static = true)
        function bool = isPoint(poly)
           bool = (length(poly) == 1); 
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
        
        function test()
            filename = 'environments/env.txt';
            a = Environment;
            a = a.readFile(filename);
            Geom2d.closestPoint(0,1,[0,1],a,5);
        end
    end
end
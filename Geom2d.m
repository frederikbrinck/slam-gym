%{
	Geom2d class, used for various things including calculating separation,
    point circle overlap and more. Adapted from the original form of Geom2d     
    used for the robotics class.
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
        % This function was used for map metrics, but are currently not
        % used.
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
        function minSep = closestPoint(x,y,vec,a,lim)
            minSep = lim;
            pol = a.polygons;
            pol{end+1} = [a.boundX';a.boundY'];
            for i = 1:length(pol)
                len = size(pol{i},2);
                % Check if we are dealing with a line or polygon
                if len > 1
                    % Loop over the edges and compute 
                    % separation
                    for j = 1:len
                        edge(1,1) = pol{i}(1,j);
                        edge(1,2) = pol{i}(2,j);
                        if j == len
                            edge(2,1) = pol{i}(1,1);
                            edge(2,2) = pol{i}(2,1);
                        else
                            edge(2,1) = pol{i}(1,j+1);
                            edge(2,2) = pol{i}(2,j+1);
                        end     
                        sep = Geom2d.sepPointEdgeDir(x,y,vec,edge);
                        % Check if separation should be updated.
                        if (sep < minSep)
                            minSep = sep;
                        end
                    end
                else
                    % Checking separation with point polygons
                    sep = Geom2d.checkPoint(x,y,vec,pol{i});
                    if (sep < minSep)
                        minSep = sep;
                    end
                end
            end
        end
        
        % Function used to find the separation from a point (x,y)
        % to an edge in a particular direction, given by vec.
        % Useful for laser read
        function sep = sepPointEdgeDir(x,y,vec,edge)
            % Help obtained from here
            % https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
            query = [x,y;x+vec(1)*10,y+vec(2)*10];
            sep = 10;
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
            
            if Geom2d.onEdge([pX,pY],edge) && Geom2d.onEdge([pX,pY],query)
                sep = norm([pX-x,pY-y]);
            end
        end
        
        % Separation between a point (P) and a line defined by Q2 and Q1
        % https://www.mathworks.com/matlabcentral/newsreader/view_thread/164048
        function sep = sepPointLine(P,Q1,Q2)
            sep = abs(det([Q2-Q1;P-Q1]))/norm(Q2-Q1); % for row vectors.
        end
        
        % Checks if p3 is to the left of the edge defined by points p1 and
        % p2 where we assume that p1 and p2 are 2-dimensional vectors
        function bool = leftOf(p1,p2,p3)
            mat = [p1,1;p2,1;p3,1];
            if det(mat) >= 0
                bool = 1;
            else 
                bool = 0;
            end
        end
        
        % This function is useful when we have points as our obstacles in
        % our environment. It returns points inside a circle defined by the
        % robot position x,y and the maximum limit that the sensor can reach
        % which is passed here as the parameter radiusScan as well as the 
        % point polygons which are present in the environment
        function points = pointsInsideCircle(x, y, env, radiusScan)
            points = [];
            pol = env.polygons;
            for i = 1:length(pol)
                if norm([x y]-[pol{i}(1) pol{i}(2)]) < radiusScan
                    points = [points; [pol{i}(1) pol{i}(2)]];
                end
            end
        end
        
        % Checks the separation between our robot and a point polygon,
        % given a particular direction and the position of the robot. If
        % the point polygon and the vector from the robot don't intersect,
        % this returns a large value (like 1000).
        function sep = checkPoint(x,y,vec,point)
            sep = 1000;
            query = [x,y;x+vec(1)*10,y+vec(2)*10];
            b = cross([query(1,:)-query(2,:),0],[x-point(1)...
                y-point(2),0]);
            if Geom2d.onEdge(point,query) && abs(b(3)) < 0.0001
                sep = norm([point(1)-x,point(2)-y]);
            end
        end
        
        % Returns true if a point is contained within an edge. Useful for 
        % figuring out whether an intersection point is valid or not.
        function bool = onEdge(point,edge)
            e = 0.0001;
            if point(1) <= max(edge(1,1),edge(2,1)) +e ...
               && point(1) >= min(edge(1,1),edge(2,1)) - e ...
               && point(2) <= max(edge(1,2),edge(2,2)) + e  ...
               && point(2) >= min(edge(1,2),edge(2,2)) -e
               bool = 1;
            else
                bool = 0;
            end
        end
        
        function test()
            disp('Simply test for points being inside a circle (red)');
            filename = 'environments/env1.txt';
            a = Environment;
            a = a.readFile(filename);
            x = 4;
            y = 4;
            rad = 2;
            points = Geom2d.pointsInsideCircle(x,y,a,rad);
            a.showEnv();
            Draw.disc([x y],rad);
            disp(points);
            hold on
            plot(points(:,1), points(:,2), 'ro', 'LineWidth', 4);
            hold off
        end
    end
end

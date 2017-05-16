%{
	Draw gives utensils for drawing polygons on the scene using
    the patch functions.
%}

classdef Draw
    properties
    end
   
	methods (Static = true)
        % Draw polygons from the polygon set a{i}=[X[] Y[]] onto the
        % current figure with colour c.
        function [] = polygons(a, c)
            if nargin < 2
               c = [0.8, 0.5, 0]; % Default colour is brown.
            end
            
            for poly = a
                poly = poly{1};
                if Geom2d.isPoint(poly) % Check if we are displaying a point.
                    Draw.disc(poly, 0.04, 360, 90, c);
                else
                    patch(poly(1,:), poly(2,:), c);
                end
            end 
        end
        
        % Draw an arrow disc with p1=[x y] onto the robot of radius r and
        % direction dir given in degrees.
        function arr = arrow(p1,r,dir)
            % Get direction in rad
            a = deg2rad(dir);
            % Sz is the angle of the arrow tip and p2
            % a point in the direction given.
            sz = deg2rad(20);
            p2 = [p1(1)+r*cos(a);p1(2)+r*sin(a)];
            % Compute arrow tails
            tailL = [p1(1)+r*0.7*cos(a+sz);p1(2)+r*0.7*sin(a+sz)];
            tailR = [p1(1)+r*0.7*cos(a-sz);p1(2)+r*0.7*sin(a-sz)];
            % Draw the arrow.
            arr(1) = plot([p1(1),p2(1)],[p1(2),p2(2)], 'black');
            arr(2) = plot([p2(1),tailR(1)],[p2(2),tailR(2)], 'black');
            arr(3) = plot([p2(1),tailL(1)],[p2(2),tailL(2)], 'black');
        end
        
        % Draw a line segment from a to b with color c.
        function segment(a, b, c)
            if nargin < 3
                c = [1, 1, 1];
            end
            plot([a(1), b(1)], [a(2), b(2)], 'color', c);
        end
        
        % Draw a disc with pos=[x y] of colour c and radius r onto the c
        % current figure. Allow for having a disc of smaller angle, say
        % a half disk (angle=180°) and give it a rotation as dir.
        function d = disc(pos, r, angle, dir, c)
            if nargin == 4
                c = [0,1,0];
            elseif nargin == 3
                c = [0,1,0];
                dir = 90;
            elseif nargin == 2
                c = [0,1,0];
                angle = 360;
                dir = 90;
            end
            
            % Create disc.
            theta = deg2rad(dir-angle/2):deg2rad(3):deg2rad(dir+angle/2);
            x = r * cos(theta) + pos(1);
            y = r * sin(theta) + pos(2);
            
            % Fill disc.
            d = patch(x, y, c);
            alpha(0.3);
        end
    end
end


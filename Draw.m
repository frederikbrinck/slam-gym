%{
	Draw class.

    Explanation goes here...
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
                    Draw.disc(poly, 0.04, c);
                else
                    patch(poly(1,:), poly(2,:), c);
                end
            end 
        end
        
        % Draw a disc with pos=[x y] of colour c and radius r onto the c
        % current figure.
        function d = disc(pos, r, c)
            % Create disc.
            theta = 0:pi/50:2*pi;
            x = r * cos(theta) + pos(1);
            y = r * sin(theta) + pos(2);
            
            % Fill disc.
            d = patch(x, y, c);
        end
    end
end


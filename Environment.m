%{
	Environment class.

    Explanation goes here...
%}

classdef Environment < handle
     properties
        boundX = [];
        boundY = [];
        polyX = {};
        polyY = {};
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	% readFile( obj, fname )
	%	-- Reads an "env.txt" file with lines in this order:
	%	     radius, eps, BBX, BBY, start, goal, {PX,PY}*
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         function obj = readFile(obj, filename) %fills all the properties
            itr = 1; % Logic for populating fields.
            pol = 1; % Logic for populating polygons fields.
            % Iterate through file.
            fileDesc = fopen(filename, 'r');
            tline = fgetl(fileDesc);
            while ischar(tline)
                if size(tline, 1) > 0 && (tline(1) ~= '%')
                    str = regexprep(tline, '[^\d.]*', ' '); % Remove all non digits.
                    str = regexprep(str, '[^\d]*(\.)+[^\d]+', ' '); % Remove individual dots.
                    c = sscanf(str, '%f', size(str,2)); % Look for object.
                    switch itr
                        case 1
                            obj.boundX = c;
                            itr = itr + 1;
                        case 2
                            obj.boundY = c;
                            itr = itr + 1;
                        case 3
                            obj.polyX{pol} = c;
                            itr = itr + 1;
                        case 4
                            obj.polyY{pol} = c;
                            % Loop back to collect next elements
                            itr = itr - 1;
                            pol = pol + 1;
                    end
                end
                tline = fgetl(fileDesc);
            end
            
            % Add bound box to polygons
            % obj.Polygons{size(obj.Polygons,2) + 1} = mapshape(obj.bound.x, obj.bound.y, 'Geometry', 'polygon', 'bound',true);    
         end


	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 % Display Robot at conf(a,b) with color c 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         function d = showDisc(obj, a, b, color, radius)
            if nargin < 5
                radius = 1;
            end
            
            % Create disc.
            theta = 0:pi/50:2*pi;
            x = radius * cos(theta) + a;
            y = radius * sin(theta) + b;
            % Fill disc.
            d = patch(x, y, color);
         end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 % Display Bounding Box 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         function showBound(obj)
            % Show bounding box.
            patch(obj.boundX, obj.boundY, [1 1 1], 'EdgeColor', [0 0 0], 'LineWidth', 2);
         end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 % Output Image to file 
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function outputFile(obj, fname)
            % Save with gcf being the current figure.
            saveas(gcf, fname);
        end

	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 % Display Environment:
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       function obj = showEnv(obj)
	     figure(1);
	     clf(1);  % Clear fig 1.
	     axis square tight;
	     % Show Bounding Box.
         obj.showBound();
         
	     % Display obstacles in brown.
	     brown = [0.8, 0.5, 0];
	     for i = 1:length(obj.polyX)
             if Geom2d.isPoint(obj.polyX{i}) % Check if we are displaying a point.
                    obj.showDisc(obj.polyX{i}, obj.polyY{i}, brown, 0.04);
                else
                    patch(obj.polyX{i}, obj.polyY{i}, brown);
             end
         end
         
         % Transparency (to show overlaps).
    	 alpha(0.3);	
         
	     % Output an image file.
	     obj.outputFile('image.jpg');
       end
     end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function a = test(filename)
            if nargin < 1
                filename = 'env.txt';
            end
            
            a = Environment;
            a = a.readFile(filename);
            a.showEnv(); % Show the entire environment
        end
    end
end

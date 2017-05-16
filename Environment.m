%{
	Environment class adapted from the SSS project. This class
    can read points, lines and polygons. Currently, however, our
    implementation only works with points - and the EKF slam doesn't work.

%}

classdef Environment < handle
     properties
         start = [];
         goal = [];
         boundX = [];
         boundY = [];
         polygons = {}
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
            % Iterate through file.
            fileDesc = fopen(filename, 'r');
            tline = fgetl(fileDesc);
            polyX = [];
            while ischar(tline)
                if size(tline, 1) > 0 && (tline(1) ~= '%')
                    str = regexprep(tline, '[^\d.]*', ' '); % Remove all non digits.
                    str = regexprep(str, '[^\d]*(\.)+[^\d]+', ' '); % Remove individual dots.
                    c = sscanf(str, '%f', size(str,2)); % Look for object.
                    switch itr
                        case 1
                            obj.start(1) = c;
                            itr = itr + 1;
                        case 2
                            obj.start(2) = c;
                            itr = itr + 1;
                        case 3
                            obj.goal(1) = c;
                            itr = itr + 1;
                        case 4
                            obj.goal(2) = c;
                            itr = itr + 1;
                        case 5
                            obj.boundX = c;
                            itr = itr + 1;
                        case 6
                            obj.boundY = c;
                            itr = itr + 1;
                        case 7
                            polyX = c;
                            itr = itr + 1;
                        case 8
                            obj.polygons{end + 1} = [polyX c]';
                            % Loop back to collect next elements
                            itr = itr - 1;
                    end
                end
                tline = fgetl(fileDesc);
            end
            
            % Uncomment this if bounding box should be part of the
            % polygons.
            % obj.Polygons{size(obj.Polygons,2) + 1} = mapshape(obj.bound.x, obj.bound.y, 'Geometry', 'polygon', 'bound',true);    
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
        function obj = showEnv(obj, bool)
            if nargin < 2
                bool = true;
            end
            
            % Create a new figure.
            if bool
                % figure(1);
                % clf(1);
                axis square tight;
            end
            
	        % Show Bounding Box.
            obj.showBound();
                     
            % Display obstacles in brown.
            brown = [0.8, 0.5, 0];
            Draw.polygons(obj.polygons, brown);
            Draw.disc(obj.goal, 0.5, 360, 90, [0,0,1]);
            alpha(0.3);
            xlim([0 10]);
            ylim([0 10]);
        end
     end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function a = test(filename)
            if nargin < 1
                filename = 'environments/env1.txt';
            end
            
            a = Environment;
            a = a.readFile(filename);
            a.showEnv(); % Show the entire environment
        end
    end
end

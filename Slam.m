%{
	% Slam class.
    % Is a singleton class
    % Acts as a controller in the model-view-controller framework
    % Explanation goes here
%}
classdef (Sealed) Slam < handle
   properties
       env;
       robot;
       algorithm;
   end
   methods
       function show(obj)
           obj.env.showEnv();
           obj.robot.drawRobot();
       end
       
       function change(obj)
           obj.robot.x = obj.robot.x+1;
       end
   end
   methods (Access = private)
      function obj = Slam(filename,x, y, theta, radius, limit)
          if nargin < 1
              filename = 'environments/env2.txt';
              x = 2;
              y = 2;
              theta = 90;
              radius = 0.5;
              limit = 4;
          end
          obj.env = Environment;
          obj.env = obj.env.readFile(filename);
          obj.robot = Robot(x, y, theta, radius, limit, obj.env);
      end
   end
   methods (Static)
      function singleObj = getInstance
         persistent localObj
         if isempty(localObj) || ~isvalid(localObj)
             disp('new');
             localObj = Slam();
         end
         singleObj = localObj;
      end
   end
end
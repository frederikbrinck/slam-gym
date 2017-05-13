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
       drawnRobot;
       scan;
       scanShown = false;
       setScan = false;
       points = {};
       startFlag = false;
   end
   methods
       function show(obj)
           obj.env.showEnv();
       end
       
       function deleteOldRobot(obj)
           for i = 1:length(obj.drawnRobot)
               delete(obj.drawnRobot(i));
           end
       end
       
       function startSimulation(obj)
           if obj.startFlag == true
               obj.show();
           end
       end
       
       function stopSimulation(obj)
           if obj.startFlag == false
               obj.points = {};
               obj.setScan = false;
               obj.scanShown = false;
               obj.robot.x = 2;
               obj.robot.y = 2;
           end
       end
       
       function runSimulation(obj)
           while obj.startFlag == true
               pause(0.1);
               obj.deleteOldRobot();
               
               % BASIC MOTION PLAN. ODOMETRY AND MOTION WILL GO BELOW
               obj.change(0.1,0.1,10);
               
               obj.showRobot();
           end
           obj.deleteOldRobot();
       end
       
       function deleteScan(obj)
           if obj.scanShown == true
               for i = 1:length(obj.scan)
                   delete(obj.scan(i));
               end
               obj.scanShown = false;
           end
       end
       
       function showScan(obj)
           obj.deleteScan();
           x = obj.robot.x;
           y = obj.robot.y;
           dir = obj.robot.theta;
           read = obj.robot.sensing.laserReadPoints(x,y,dir);
           obj.scan = obj.robot.sensing.plotPoints(x,y,dir,read);
           obj.scanShown = true;
       end
       
       function showRobot(obj)
           hold on
           obj.points{end+1} = [obj.robot.x, obj.robot.y];
           obj.drawnRobot = obj.robot.drawRobot();
           if obj.setScan == true
               obj.showScan();
           end
           hold off
       end
       
       function change(obj, dx, dy, dtheta)
           if obj.startFlag == true
               newPos = obj.robot.move(dx, dy, dtheta);
               obj.robot.x = newPos(1);
               obj.robot.y = newPos(2);
               obj.robot.theta = newPos(3);
           end
       end
   end
   methods (Access = private)
      function obj = Slam(filename,x, y, theta, radius, limit)
          if nargin < 1
              filename = 'environments/env1.txt';
              x = 2;
              y = 2;
              theta = 90;
              radius = 0.5;
              limit = 4;
          end
          obj.env = Environment;
          obj.env = obj.env.readFile(filename);
          obj.points = {};
          obj.points{1} = [x,y];
          obj.robot = Robot(x, y, theta, radius, limit, obj.env);
      end
   end
   methods (Static)
      function singleObj = getInstance()
         persistent localObj
         if isempty(localObj) || ~isvalid(localObj)
             disp('new');
             localObj = Slam();
         end
         singleObj = localObj;
      end
      
      function test
          Gui;
      end
   end
end
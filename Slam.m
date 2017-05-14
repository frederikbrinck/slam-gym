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
       drawnLandmarks;
       scan;
       scanShown = false;
       setScan = false;
       points = {};
       startFlag = false;
       estHandle;
       up = false;
       down = false;
       left = false;
       right = false;
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
               obj.robot = Algorithm(obj.env);
           end
       end
       
       function stopSimulation(obj)
           if obj.startFlag == false
               obj.points = {};
               obj.setScan = false;
               obj.scanShown = false;
           end
       end
       
       function [s, theta] = getSpeedAndRotation(obj) 
           s = 0;
           theta = 0;
           if obj.up
               s = s + 0.1;
           end
           if obj.down 
               s = s - 0.1;
           end
           if obj.left
               theta = theta + 5;
           end
           if obj.right
               theta = theta - 5;
           end
       end
       
       function [s, theta] = plannedMotion(obj) 
           s = 0.1;          
           [x, y, t] = obj.robot.getPosition();
           position = obj.env.goal - [x y]; 
           position = position/norm(position);
           u = [1 0 0]; 
           v = [position 0];
           
           bearing = atan2d(norm(cross(v, u)),dot(v, u));
           a = cross(v,u);
           if a(3) > 0;
               bearing = 360 - bearing;
           end
           theta = bearing - t;
       end
       
       function bool = checkCondition(obj)
           [x, y, ~] = obj.robot.getPosition();
           if norm([x y] - obj.env.goal) < 1
               bool = true;
           else
               bool = false;
           end
       end
       
       function runSimulation(obj)
           while obj.startFlag == true
               obj.deleteOldRobot();
               % [s, t] = obj.getSpeedAndRotation();
               [s, t] = obj.plannedMotion();
               obj.robot.simulate(s, t);
               
               if obj.checkCondition() == true
                   break;
               end
               
               % BASIC MOTION PLAN. ODOMETRY AND MOTION WILL GO BELOW
               % obj.change(0.1,0.1,10);
               obj.showRobot();
               landmarks = obj.robot.getLandmarkPositions();
               hold on
               d = obj.showEstRobot();
               a = obj.showEstArrow();
               for i = 1:size(landmarks, 1)
                   l = landmarks(i, :);
                   d = Draw.disc([l(1), l(2)], 0.2, 360, 0, [0,0,1]);
                   obj.drawnLandmarks = [obj.drawnLandmarks d];
               end
               hold off
               
               pause(0.05);
               delete(d);
               delete(a);
               for i = 1:length(obj.drawnLandmarks)
                   delete(obj.drawnLandmarks(i));
                   obj.drawnLandmarks(i) = [];
               end
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
       
       function d = showEstRobot(obj)
           state = obj.robot.ekf.state();
           d = Draw.disc([state(1), state(2)], 0.5, 360, 0, [0, 0.5, 0]);
       end
       
       function d = showEstArrow(obj)
           state = obj.robot.ekf.state();
           d = Draw.arrow([state(1), state(2)], 0.5, state(3));
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
      function obj = Slam(filename, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorAngle, sensorThreshold)
          if nargin < 1
              filename = 'environments/env1.txt';
              theta = 45;
              radius = 0.5;
              odometryMaxTheta = 4;
              odometryMaxSpeed = 4;
              sensorAngle = 180;
              sensorThreshold = 2;
              sensorEnv = Environment;
          end
          sensorEnv = sensorEnv.readFile(filename);
          obj.env = sensorEnv;
          x = obj.env.start(1);
          y = obj.env.start(2);
          obj.points = {};
          obj.env = sensorEnv;
          obj.points{1} = [x,y];
          obj.robot = Algorithm(obj.env, x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorAngle, sensorThreshold);
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
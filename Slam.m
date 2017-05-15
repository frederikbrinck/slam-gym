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
           inc = 0.1;
           s = inc;
           lms = obj.robot.sensing.laserReadPoints(obj.robot.x...
                       ,obj.robot.y,obj.robot.theta);
           
           [x, y, t] = obj.robot.getPosition();
           toX = obj.env.goal(1);
           toY = obj.env.goal(2);
           bearing = obj.computeBearing(x,y,toX,toY);
           
           r = obj.robot.radius;
           ang = obj.robot.theta;
           closestPoints = {};
           for i = 1:size(lms, 1)
                % bear = obj.computeBearing(x,y,lms(i,1),lms(i,2));                
                sep = Geom2d.sepPointLine([lms(i,1),lms(i,2)],[x,y],[toX,toY]);
                if sep < r*2.5
                    closestPoints{end+1} = [lms(i,1),lms(i,2)];
                    if sep < r*1.5
                        inc = inc/5;
                    end
                end
           end
           
           theta = bearing - t;
           
           for i = 1:length(closestPoints)
               bear = obj.computeBearing(x,y,closestPoints{1}(1),closestPoints{1}(2)); 
               deg = t-bear;
               if abs(deg) < 90
                  hold on
                  if deg < 0
                      theta = theta - 50;
                  else
                      theta = theta + 50;
                  end
                  plot([x,closestPoints{1}(1)],[y,closestPoints{1}(2)]);
               end
           end
           hold off
           
%            if minSep < r*2
%                if Geom2d.leftOf(closestPoint,[toX,toY],[x,y])                 
%                    theta = theta + 90;
%                else
%                    theta = theta - 90;
%                end
%            end
           
           if x - r*cos(ang) < r || x + r*cos(ang) > 10-r ||...
              y - r*sin(ang) < r || y + r*sin(ang) > 10-r
               s = 0;
               theta = theta + 180;
           end
       end
       
       function bearing = computeBearing(~,x,y,toX,toY)
           position = [toX toY] - [x y]; 
           position = position/norm(position);
           u = [1 0 0]; 
           v = [position 0];
           bearing = atan2d(norm(cross(v, u)),dot(v, u));
           a = cross(v,u);
           if a(3) > 0;
               bearing = 360 - bearing;
           end
       end
       
       function bool = checkCondition(obj)
           [x, y, ~] = obj.robot.getPosition();
           if norm([x y] - obj.env.goal) < 0.5
               bool = true;
           else
               bool = false;
           end
       end
       
       function runSimulation(obj)
           while obj.startFlag == true
               obj.deleteOldRobot();
               [s, t] = obj.getSpeedAndRotation();
               %[s, t] = obj.plannedMotion();
               obj.robot.simulate(s, t);
               %obj.robot.simulateFake(s, t);
               if obj.checkCondition() == true
                   break;
               end
               
               obj.showRobot();
               landmarks = obj.robot.getLandmarkPositions();
               hold on
               l = plot([obj.robot.x,obj.env.goal(1)],[obj.robot.y,obj.env.goal(2)]);
               d = obj.showEstRobot();
               a = obj.showEstArrow();
               drawnLandmarks = [];
               for i = 1:size(landmarks, 1)
                   l = landmarks(i, :);
                   lm = Draw.disc([l(1), l(2)], 0.2, 360, 0, [0,0,1]);
                   disp(obj.robot.ekf.x);
                   drawnLandmarks = [drawnLandmarks lm];                   
               end
               hold off
               
               pause(0.05);
               delete(d);
               delete(a);
               for i = 1:size(drawnLandmarks,2)
                   delete(drawnLandmarks(i));
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
           [x, y, ~] = obj.robot.ekf.state();
           d = Draw.disc([x y], 0.5, 360, 0, [0, 0.5, 0]);
       end
       
       function d = showEstArrow(obj)
           [x, y, t] = obj.robot.ekf.state();
           d = Draw.arrow([x y], 0.5, t);
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
              x = 6;
              y = 2;
              theta = 90;
              radius = 0.5;
              odometryMaxTheta = 4;
              odometryMaxSpeed = 4;
              sensorAngle = 180;
              sensorThreshold = 8;
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
%{
	% The slam class is a singleton class that connects the GUI to the
    % algorithm. It is the main entry point of the implementation and
    % owns the main loop of the algorithm which is run during 
    % simulation.
%}
classdef (Sealed) Slam < handle
   properties
       % Key handles.
       env;
       robot;
       algorithm;
       % Booleans for GUI logic.
       usingEkf = true;
       drawnRobot;
       scan;
       scanShown = false;
       setScan = false;
       points = {};
       startFlag = false;
       estHandle;
       % Keys for movement.
       up = false;
       down = false;
       left = false;
       right = false;
   end
   methods
       % Wrapper for environment show.
       function show(obj)
           obj.env.showEnv();
       end
       
       % Delete the drawn robot.
       function deleteOldRobot(obj)
           for i = 1:length(obj.drawnRobot)
               delete(obj.drawnRobot(i));
           end
       end
       
       % Start a simulation.
       function startSimulation(obj)
           if obj.startFlag == true
               obj.show();
               % Initiate algorithm.
               obj.robot = Algorithm(obj.env);
           end
       end
       

       % Stop a simulation
       function stopSimulation(obj)
           if obj.startFlag == false
               obj.points = {};
               obj.setScan = false;
               obj.scanShown = false;
           end
       end
       
       % Return speed and rotation for key movement.
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
       
       % Planned motion algorithn. This algorithm simply moves from 
       % the start position towards the goal position with the purpose
       % of avoiding collision with any obstacles by "circling" around
       % them. It is not robust and works only for simple environments.
       function [s, theta] = plannedMotion(obj) 
           inc = 0.1;
           s = inc;
           lms = obj.robot.sensing.laserReadPoints(obj.robot.x...
                       ,obj.robot.y,obj.robot.theta);
           
           [x, y, t] = obj.robot.getPosition();
           toX = obj.env.goal(1);
           toY = obj.env.goal(2);
           % Get bearing to goal.
           bearing = obj.computeBearing(x,y,toX,toY);
           
           r = obj.robot.radius;
           ang = obj.robot.theta;
           closestPoints = {};
           % Find points that we are about to collide with naively.
           for i = 1:size(lms, 1)
                dist = norm([lms(i,1),lms(i,2)]-[x,y]);
                sep = Geom2d.sepPointLine([lms(i,1),lms(i,2)],[x,y],[toX,toY]);
                if sep < r*2.5 && dist < r*4
                    closestPoints{end+1} = [lms(i,1),lms(i,2)];
                    if sep < r*1.5
                        inc = inc/5;
                    end
                end
           end
           
           % Based on the bearing and the collision points, avoid
           % upcoming obstacles by "nudging" the angle.
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
           
           % Handle the case of the boundary.
           if x - r*cos(ang) < r || x + r*cos(ang) > 10-r ||...
              y - r*sin(ang) < r || y + r*sin(ang) > 10-r
               s = 0;
               theta = theta + 180;
           end
       end
       
       % Computes the bearing given x and y position with respect
       % to the x-axis. 
       function bearing = computeBearing(~,x,y,toX,toY)
           position = [toX toY] - [x y]; 
           position = position/norm(position);
           u = [1 0 0]; 
           v = [position 0];
           % Get bearing
           bearing = atan2d(norm(cross(v, u)),dot(v, u));
           a = cross(v,u);
           % Handle edge case.
           if a(3) > 0
               bearing = 360 - bearing;
           end
       end
       
       % Function that returns if we have reached goal or not.
       function bool = checkCondition(obj)
           [x, y, ~] = obj.robot.getPosition();
           if norm([x y] - obj.env.goal) < 0.5
               bool = true;
           else
               bool = false;
           end
       end
       
       % Run the simulation and handle drawing and display of the proper
       % objects.
       function runSimulation(obj)
           while obj.startFlag == true
               obj.deleteOldRobot();
               % If we are using EKF allow for keyboard controls.
               if obj.usingEkf == true
                   [s, t] = obj.getSpeedAndRotation();
                   obj.robot.simulate(s, t);
               else
               % Otherwise do planned motion.
                   [s, t] = obj.plannedMotion();
                   obj.robot.simulateFake(s, t);
               end
               
               % Check for ending condition.
               if obj.checkCondition() == true
                   break;
               end
               
               % Draw all landmarks as observed by the robot, if we are
               % using ekf.
               obj.showRobot();
               landmarks = obj.robot.getLandmarkPositions();
               hold on
               l = plot([obj.robot.x,obj.env.goal(1)],[obj.robot.y,obj.env.goal(2)]);
               d = obj.showEstRobot();
               a = obj.showEstArrow();
               drawnLandmarks = [];
               drawnUncertainties = [];
               for i = 1:size(landmarks, 1)
                   l = landmarks(i, :);
                   lm = Draw.disc([l(1), l(2)], l(3), 360, 0, [0,0,1]);
                   drawnLandmarks = [drawnLandmarks lm];
               end
               hold off
               
               % Delete all landmarks, if we are using ekf.
               pause(0.05);
               delete(d);
               delete(a);
               if obj.usingEkf == false
                   delete(l);
               end
               for i = 1:size(drawnLandmarks,2)
                   delete(drawnLandmarks(i));
               end
           end
           obj.deleteOldRobot();
       end
       
       % Function used for hiding the scan.
       function deleteScan(obj)
           if obj.scanShown == true
               for i = 1:length(obj.scan)
                   delete(obj.scan(i));
               end
               obj.scanShown = false;
           end
       end
       
       % Function used for showing the scan.
       function showScan(obj)
           obj.deleteScan();
           x = obj.robot.x;
           y = obj.robot.y;
           dir = obj.robot.theta;
           read = obj.robot.sensing.laserReadPoints(x,y,dir);
           obj.scan = obj.robot.sensing.plotPoints(x,y,dir,read);
           obj.scanShown = true;
       end
       
       % Show the estimated position of the robot for ekf.
       function d = showEstRobot(obj)
           [x, y, ~] = obj.robot.ekf.state();
           d = Draw.disc([x y], 0.5, 360, 0, [0, 0.5, 0]);
       end
       
       % Show the estimated arrow of the robot for ekf.
       function d = showEstArrow(obj)
           [x, y, t] = obj.robot.ekf.state();
           d = Draw.arrow([x y], 0.5, t);
       end 
       
       % Show the robot.
       function showRobot(obj)
           hold on
           obj.points{end+1} = [obj.robot.x, obj.robot.y];
           obj.drawnRobot = obj.robot.drawRobot();
           if obj.setScan == true
               obj.showScan();
           end
           hold off
       end
      
   end
   
   % Constructor file which takes in all needed parameters, and,
   % followingly, initiates the algorithm.
   methods (Access = private)
      function obj = Slam(filename, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorAngle, sensorThreshold)
          if nargin < 1
              filename = 'environments/env1.txt';
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
   
   % Handle the GUI initialisation by making the slam.m a singleton object.
   methods (Static)
      function singleObj = getInstance()
         persistent localObj
         if isempty(localObj) || ~isvalid(localObj)
             localObj = Slam();
         end
         singleObj = localObj;
      end
      
      % Run the gui upon Slam.test()
      function test
          Slam.initialize();
          Gui;
      end
      
      % Initialise the slam environment.
      function initialize
        s = Slam.getInstance();
        s.usingEkf = true;
        file = 'environments/env1.txt';
        a = Environment;
        a = a.readFile(file);
        s.env = a;
        s.robot.sensing.env = a;
        s.robot.x = a.start(1);
        s.robot.y = a.start(2); 
      end      
   end
end
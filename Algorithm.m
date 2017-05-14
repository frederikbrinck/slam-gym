%{
  Created by Mufei Li on 2017/05/04.
  Modified by Frederik Jensen 2017/05/12

  
%}

classdef Algorithm < Robot
    properties
        landmarks;
        landmarkId = 1;
        maxAssociateError = 0.5
        maxLandmarkError = 0.2;
        ekf;
        db;
    end
    
    methods
        % Constructor of the algorithm class which instantiates the
        % super class robot, and runs the EKF filter.
        function obj = Algorithm(x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorEnv, sensorAngle, sensorThreshold)
            obj = obj@Robot(x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorEnv, sensorAngle, sensorThreshold);

            obj.ekf = EKF([x y theta], 0.1, 0.1);
            obj.db = LandmarkDatabase();
        end
        
        % The main loop run by the Slam.m algorithm.
        function bool = simulate(obj, s, t)
            if nargin < 3
                s = 0.1;
                t = 0;
            end
            
            bool = true;
            % Get robot position and move it
            state = obj.ekf.state();
            controls = obj.moveNoisy(state, s, t);
            % Do the prediction
            
            obj.ekf.prediction(controls);
            % Perform laser scan and loop over all observed landmarks
            observations = obj.laserReadPoints();
            
            % Get estimated position from landmark
            state = obj.ekf.state();
            for i = 1:size(observations,1)
                [range, bearing] = obj.computeBearing(observations(i,:));
                observations(i,:) =  [state(1) state(2)] + range * [cosd(bearing + state(3)) sind(bearing + state(3))]; 
            end
            
            lms = obj.db.extractLandmarks(observations);
            for i = 1:size(lms, 2)
                 % Correct the estimates based on the current landmark.
                 lm = lms(i);
                 [range, bearing] = obj.computeBearing(lm.position);
                 lm.range = range;
                 lm.bearing = bearing;
                 obj.ekf.correction([lms(i)]); 
            end            
            
            % Extract all new landmarks
            nlms = obj.db.addNewLandmarks();
            for i = 1:size(nlms, 2)
               % Get landmark and add it to ekf. 
               lm = nlms(i);
               [range, bearing] = obj.computeBearing(lm.position);
               lm.range = range;
               lm.bearing = bearing;
               obj.ekf.addLandmark(lm);
            end
        end
        
        function lms = getLandmarkPositions(obj)
            lms = obj.ekf.getLandmarks();
        end
    end
   
    
    
    methods(Static)
        function s = test(filename)
            if nargin < 1
                filename = 'environments/env2.txt';
            end
            env = Environment;
            env = env.readFile(filename);
            env.showEnv();
            alg = Algorithm(1, 0, 30, 0.9, 6, 5, env, 180, 5);
            while alg.simulate()
                alg.drawRobot();
            end
        end
    end
end
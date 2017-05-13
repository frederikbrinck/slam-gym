%{
  Created by Mufei Li on 2017/05/04.
  Modified by Frederik Jensen 2017/05/12

  
%}

classdef Algorithm < Robot
    properties
        landmarks;
        landmarkId = 1;
        maxAssociateError = 0.1
        maxLandmarkError = 0.2;
        ekf;
    end
    
    methods
        % Constructor of the algorithm class which instantiates the
        % super class robot, and runs the EKF filter.
        function obj = Algorithm(x, y, theta, radius, limit, env, maxTheta, maxDist)
            obj = obj@Robot(x, y, theta, radius, limit, env, maxTheta, maxDist);

            obj.ekf = EKF([x y theta]', 0.1, 0.1);
        end
        
        % The main loop run by the Slam.m algorithm.
        function bool = simulate(obj)         
            bool = false;
            % Get robot position and move it
            state = obj.ekf.state();
            state = obj.moveNoisy(state, [dx dy theta]);
            
            % Do the prediction
            obj.ekf.prediction(state);
            
            % Perform laser scan and loop over all observed landmarks
            observations = obj.laserReadPoints();
            lms = LandmarkDatabase.extractLandmarks(observations);
            for i = 1:length(lms)
                % Correct the estimates based on the current landmark.
                lm = lms(i);
                [range, bearing] = obj.computeBearing(lm.position);
                lm.range = range;
                lm.bearing = bearing;
                obj.ekf.correction(lms(i)); 
            end
            
            % Extract all new landmarks
            nlms = LandmarkDatabase.addNewLandmarks();
            for i = 1:length(nlms)
               % Get landmark and add it to ekf. 
               lm = nlms(i);
               obj.ekf.addLandmark(lm);
            end
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
            
            alg = Algorithm(1, 0, 30, 0.9, 6, env,...
                180, 0.5);
            alg.simulate();
        end
    end
end
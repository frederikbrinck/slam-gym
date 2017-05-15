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
        function obj = Algorithm(sensorEnv, x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorAngle, sensorThreshold)
            if nargin < 2
                theta = 45;
                radius = 0.5;
                odometryMaxTheta = 4;
                odometryMaxSpeed = 4;
                sensorAngle = 180;
                sensorThreshold = 8;
                x = sensorEnv.start(1);
                y = sensorEnv.start(2);
            end
            
            obj = obj@Robot(x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorEnv, sensorAngle, sensorThreshold);

            % Get signal and measurement noise for EKF
            s = [0.1;0.05];
            S = diag(s.^2);
            m = [1.5; 1*pi/180];
            M = diag(m.^2);
            
            obj.ekf = EKF2([x y theta], s, S, m, M);
            obj.db = LandmarkDatabase();
        end
        
        function bool = simulateFake(obj,s,t)
            if nargin < 3
                s = 0.1;
                t = 0;
            end
            bool = false;
            % Get robot position and move it
            state = obj.ekf.state();
            controls = obj.moveNoisy(state, s, t);
            % Do the prediction
            obj.ekf.prediction(controls);
        end
        
        % The main loop run by the Slam.m algorithm.
        function bool = simulate(obj, s, t)
            if nargin < 3
                s = 0.1;
                t = 0;
            end
                        
            bool = true;
            % Get robot position and move the ground truth. In turn
            % use the noised signal for ekf state prediction.
            [x, y, theta] = obj.ekf.state();
            [signal, noise]= obj.moveNoisy([x y theta], s, t, [0.1 ; 0.05]);
            % Do the prediction
            obj.ekf.predict(signal, noise);
            
            % Perform laser scan and loop over all observed landmarks
            observations = obj.laserReadPoints();
            rbT = zeros(size(observations));
            for i = 1:size(observations,1)
                rbT(i,:) = obj.ekf.observe([obj.x ; obj.y; obj.theta], observations(i,:)');
            end
            
            for i = 1:size(rbT, 2):2
                 % Correct the estimates based on the current landmark.
                 j = 2 + 2*i;
                 if j < size(obj.ekf.x, 1)
                    obj.ekf.correct(i, rbT', observations(i,:)'); 
                 end
            end            
            
            % Extract all new landmarks
            for i = 1:size(rbT, 2):2
                j = 2 + 2*i;
                if j >= size(obj.ekf.x, 1)
                    % Get landmark and add it to ekf. 
                    
                    obj.ekf.add(rbT(i,:)');
                end
            end
        end
        
        function lms = getLandmarkPositions(obj)
           lms = obj.ekf.lms();
            %observations = obj.laserReadPoints();
            
            % Get estimated position from landmark
            %state = obj.ekf.state();
            %for i = 1:size(observations,1)
            %    [range, bearing] = obj.computeBearing(observations(i,:));
            %    observations(i,:) =  [state(1) state(2)] + range * [cosd(bearing + state(3)) sind(bearing + state(3))]; 
            %end
        end
    end
      
    methods(Static)
        function test(filename)
            if nargin < 1
                filename = 'environments/env2.txt';
            end
            env = Environment;
            env = env.readFile(filename);
            env.showEnv();
            alg = Algorithm(1, 0, 30, 0.9, 6, 5, env, 180, 5);
            while true
                alg.simulate([0.1 0]);
            end
        end
    end
end
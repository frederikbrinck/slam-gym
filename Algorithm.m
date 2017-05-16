%{
  Created by Mufei Li on 2017/05/04.
  Modified by Frederik Jensen 2017/05/12  

  The main algorithm run by Slam.m. This algorithm should be responsible
  for estimating the robots position and mapping out the terrain. 
  The implementation is based off of EKF and is using the file EKF2. 
  Unfortunately, it is currently not working.
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
                sensorThreshold = 2;
                x = sensorEnv.start(1);
                y = sensorEnv.start(2);
            end
            
            % Call the super class
            obj = obj@Robot(x, y, theta, radius, odometryMaxTheta, odometryMaxSpeed, sensorEnv, sensorAngle, sensorThreshold);

            % Get signal and measurement noise for EKF
            s = [0.1;0.05];
            S = diag(s.^2);
            m = [1.5; 1*pi/180];
            M = diag(m.^2);
            
            % Initialise EKF and the landmark database.
            obj.ekf = EKF2([x y theta], s, S, m, M);
            obj.db = LandmarkDatabase();
        end
        
        % The main loop run by Slam.m algorithm WITHOUT using EKF
        % landmark correction.
        function bool = simulateFake(obj,s,t)
            if nargin < 3
                s = 0.1;
                t = 0;
            end
            bool = false;
            % Get robot position and move it            
            [x, y, theta] = obj.ekf.state();
            [signal, noise]= obj.moveNoisy([x y theta], s, t, [0.1 ; 0.05]);
            % Do the prediction
            obj.ekf.predict(signal, noise);
            
        end
        
        % The main loop run by the Slam.m algorithm using the EKF
        % with landmark correction (note this is NOT working properly.)
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
            disp(obj.ekf.x(1:3))
            obj.ekf.predict(signal', noise');
            disp(obj.ekf.x(1:3))
            
            % Perform laser scan and loop over all observed landmarks
            observations = obj.laserReadPoints();
            rbT = zeros(size(observations));
            for i = 1:size(observations,1)
                % Return the true range and bearing with noise added.
                m = [1.5; 1*pi/180];
                M = diag(m.^2);
                rbT(i,:) = obj.ekf.observe([obj.x ; obj.y; obj.theta], observations(i,:)' + (M*randn(2,1)));
            end
        
            % Run through all landmarks.
            for i = 1:size(rbT, 1)
                 % Correct the estimates based on the current landmark.
                 j = 2 + 2*i;
                 if j < size(obj.ekf.x, 1)
                    obj.ekf.correct(i, rbT(i,:)', observations(i,:)'); 
                 end
            end            

            % Extract all new landmarks
            for i = 1:size(rbT, 1)
                j = 2 + 2*i;
                if j >= size(obj.ekf.x, 1)
                    % Get landmark and add it to ekf.                     
                    obj.ekf.add(rbT(i,:)');
                end
            end
        end
        
        % Return the landmark positions and uncertainties.
        function lms = getLandmarkPositions(obj)
           lms = obj.ekf.lms();
        end
    end
      
    methods(Static)
        function test(filename)
            if nargin < 1
                filename = 'environments/env1.txt';
            end
            env = Environment;
            env = env.readFile(filename);
            env.showEnv();
            alg = Algorithm(env, 1, 0, 30, 0.9, 6, 5, 180, 5);
            while true
                alg.simulate([0.1 0]);
            end
        end
    end
end
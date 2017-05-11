classdef EKF < handle
    properties
        robotConfig; % 3-tuple (x,y,theta)
        landmarks;
        stateEstimate;
        P;

    end
    methods
        function obj = EKF(initialUncertainty)
            % state init.
            obj.stateEstimate = obj.initialConfig';
            
            % covariance matrix setup
            if nargin < 1
                initialUncertainty = .1;
            end
            v = initialUncertainty*ones(1, 3);
            obj.P = diag(v);
            
        end
        function Jxr = landmarkPredictionJacobian(deltaX, deltaY)
            Jxr = [[1 0 -deltaY]; [0 1 -deltaX]];
        end
        
        function A = predictionJacobian(deltaX, deltaY)
            A = [[1 0 -deltaY];[0 1 deltaX]; [0 0 1]];
        end
        
        function H = measurementJacobian(landX, landY, x, y)
            r = sqrt((landX-x)^2 + (landY-y)^2);
            H = [[(x-landX)/r (y-landY)/r 0];...
                 [(landY-y)/r^2 (landX-x)/r^2 -1]];
        end
        % RB = w.r.t. [range, bearing]
        function Jz = predictionJacobianRB(theta, deltaTheta, deltaX)
            deltaT = -deltaX/cos(theta);
            Jz = [[cosd(theta + deltaTheta) -deltaT*sind(theta+deltaTheta)];...
                [sind(theta + deltaTheta) deltaT*cosd(theta + deltaTheta)]];
        end
        function systemState = updateEstimate (obj, q)
            if nargin < 2:
                q = Noise.measurementNoise();
            end
            lNumber = length(obj.landmarks(1));
            obj.stateEstimate = zeros(3 + 2*lNumber, 1);
            % update robot's position
            obj.stateEstimate(1:3) = ...
                obj.stateEstimate (1:3) + controls*diag(q);
            
            % get the relevant landmarks
            for l = 1:lNumber
                obj.stateEstimate(3+lNumber) = obj.landmarks(1,lNumber);
                obj.stateEstimate(3+lNumber+1) = obj.landmarks(2,lNumber);
            end
        end
        
        function [x_hat, P_hat] = timeUpdate(x, p, controls)
            x_hat = x(1:3) + controls';
            % update the cov. for the robot position
            P_hat(1:3, 1:3) = ...
                predictionJacobian(controls(1), controls(2))...
                    * p(1:3, 1:3) * predictionJacobian(controls(1), controls(2))'...
                    + Noise.processNoise(controls(1), controls(2), controls(3));
        end
        
        function measurementUpdate(H, R, x_hat, P_hat)
            K = kalmanGain(P_hat, H, R);
            x_final = x_hat + K*(z_k - H*x_hat);
            P_final = (eye() - K*H)
        end
        
        function update ()
            timeUpdate();
            measurementUpdate();
        end
        function P = covarianceUpdate()
            
            for i = 1:length(obj.landmarks(1))
                % update/set cov(R, l_i)
                P(2 + i*2 : 3 + i*2, 1:3) = predictionJacobian(deltaX, deltaY)*...
                    P(2 + i*2 : 3 + i*2, 1:3);
            end
        end
        
        function K = kalmanGain(P, H, R)
            K = (P * H') \ (H * P * H' + R);
        end
        
        

    end
end
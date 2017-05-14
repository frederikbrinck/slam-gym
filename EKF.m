% This implementation is based primarily on the EKF-SLAM tutorial 
% http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf
% Which in turn is based on Thurn et al. "Probabilistic robotics", ch.10.

classdef EKF < handle
    % Initialize with initial robot configuration, uncertainty in range 
    % and bearing (from the sensor) and (optionally) the uncertainty in 
    % initial robot position.
    % 
    properties
        x_hat;
        p_hat;
        Q;
    end
    methods
        % deltaR, deltaB = uncertainties in range 
        % and bearing measurement,respectively.
        function obj = EKF(initConfig, deltaR, deltaB, initialUncertainty)
            % state init.
            obj.x_hat = initConfig';
            % covariance matrix setup
            if nargin < 4
                initialUncertainty = .1;
            end
            v = initialUncertainty*ones(1, 3);
            obj.p_hat = diag(v);
            obj.Q = [[deltaR 0];[0 deltaB]]; 
        end
        
        % returns current robot config estimate
        function config = state(obj)
            config = obj.x_hat(1:3)';
        end
        
        function landmark_j = getLandmark(obj, j)
            landmark_j = obj.x_hat((2+2*j):(2+2*j+1));
        end        
        
        function prediction(obj, controls)
            twoN = size(obj.x_hat, 1) - 3;
            F_x = [eye(3) zeros(3,twoN)];
            obj.x_hat = obj.x_hat + F_x'*controls';
            predictionJacobian = eye(twoN+3) + F_x'*[[0 0 controls(1)];[0 0 -controls(2)];[0 0 0]]*F_x;
            
            % Update the cov for the robot position
            obj.p_hat = predictionJacobian * obj.p_hat * predictionJacobian' + F_x'*Noise.processNoise(controls(1), controls(2), controls(3))*F_x;
        end
        
        % landmarks are passed in as an array [x_l y_l idx],
        % where idx is the global index (when the landmark was first
        % observed)
        function correction(obj, observations)
            x_t = obj.x_hat(1); y_t = obj.x_hat(2); th_t = obj.x_hat(3);
            
            for i = 1:length(observations)
                z_i = observations(i);
                j = z_i.id;
                lJ = 2 + 2*j;
                nLandmarks = (size(obj.x_hat, 1) - 3)/2;
                if j <= nLandmarks
                    delta = [(obj.x_hat(lJ) - x_t); ...
                             (obj.x_hat(lJ + 1) - y_t)]; 
                    q = delta'*delta;
                    z_est = [sqrt(q); atan2d(delta(2), delta(1)) - th_t];
                    F_xj = [eye(3); zeros(2,3)];
                    F_xj = [F_xj zeros(5, 2*nLandmarks)];
                    F_xj (4:5, (2*j + 2):(2*j+3)) = eye(2);

                    temp = [[-sqrt(q)*delta(1) -sqrt(q)*delta(2) 0 sqrt(q)*delta(1) sqrt(q)*delta(2)];...
                        [delta(2) -delta(1) -q -delta(2) delta(1)]];
                    H = 1/q * temp * F_xj;
                    K = (obj.p_hat * H') / (H * obj.p_hat * H' + obj.Q);
                    obj.x_hat = obj.x_hat + K*([z_i.range z_i.bearing]' - z_est);
                    KH = K*H;
                    obj.p_hat = (eye(size(KH,1)) - KH)*obj.p_hat;
                end
            end
        end

        function addLandmark(obj, lm)
            x_t = obj.x_hat(1); y_t = obj.x_hat(2); th_t = obj.x_hat(3);
            lJ = 2 + 2*lm.id;
            nLandmarks = (size(obj.x_hat, 1) - 3)/2;
            % if the feature is not in state space, we need to set its
            % x_hat, y_hat and add it to state space.
            if  nLandmarks < lm.id
               lm.position = [x_t y_t] + lm.range * [cosd(lm.bearing + th_t) sind(lm.bearing + th_t)];
               obj.x_hat = [obj.x_hat; lm.position'];
               P_rr = obj.p_hat(1:3,1:3);
               P_rm = obj.p_hat(1:3, 4:end);
               G_r = [[1 0 -lm.range*sind(lm.bearing)];[0 1 lm.range*cosd(lm.bearing)]];
               G_y = [[cosd(lm.bearing) -lm.range*sind(lm.bearing)];[sind(lm.bearing) lm.range*sind(lm.bearing)]];
               P_ll = G_r*P_rr*G_r' + G_y*Noise.measurementNoise(lm.range)*G_y';
               P_lx = G_r*[P_rr P_rm];
               obj.p_hat = [obj.p_hat; P_lx];
               obj.p_hat = [obj.p_hat [P_lx'; P_ll]];
               %obj.p_hat = [obj.p_hat; [eye(2,3) zeros(2, 2*nLandmarks)]];
               %obj.p_hat = [obj.p_hat [eye(2,3) zeros(2, 2*nLandmarks) eye(2,2)]'];
            end
        end
        
        function lms = getLandmarks(obj)
           landmarks = obj.x_hat(4:end);
           lms = [];
           for i = 1:(length(obj.x_hat) - 3):2
               lms = [lms; [landmarks(i) landmarks(i + 1)]];
           end
        end
    end
end
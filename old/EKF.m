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
            obj.p_hat = zeros(3,3);
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
                    y_i = obj.computeBearing([obj.x_hat(lJ) obj.x_hat(lJ + 1)])';
                    h_i = [z_i.range z_i.bearing]';
                    z = y_i-h_i;
                    H_r = [(z_i.position(1) - x_t)/z_i.range (z_i.position(2) - y_t)/z_i.range 0;...
                           (z_i.position(2) - y_t)/(z_i.range^2) -(z_i.position(1) - x_t)/(z_i.range^2) -1];
                    H_l = [(x_t - z_i.position(1))/z_i.range (y_t - z_i.position(2))/z_i.range;...
                           (y_t - z_i.position(2))/(z_i.range^2) -(x_t - z_i.position(1))/(z_i.range^2)];
                    P_rr = obj.p_hat(1:3,1:3);
                    P_rl = obj.p_hat(1:3,lJ:(lJ+1));
                    P_ll = obj.p_hat(lJ:(lJ+1), lJ:(lJ+1));
                    P_rm = obj.p_hat(1:3, 4:end);
                    P_ml = obj.p_hat(4:end, lJ:(lJ+1));
                    Z = [H_r H_l] * [P_rr P_rl; P_rl' P_ll] * [H_r H_l]' + Noise.measurementNoise(z_i.range);
                    K = [P_rr P_rl; P_rm' P_ml] * [H_r'; H_l'] / Z; 
                    obj.x_hat = obj.x_hat + K*z;
                    obj.p_hat = obj.p_hat - K*Z*K';
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
            end
        end
        
        function lms = getLandmarks(obj)
           landmarks = obj.x_hat(4:end);
           lms = [];
           for i = 1:(length(obj.x_hat) - 3):2
               lms = [lms; [landmarks(i) landmarks(i + 1)]];
           end
        end
        
        function rb = computeBearing(obj, position)
            x = obj.x_hat(1);
            y = obj.x_hat(2);
            theta = obj.x_hat(3);
            u = [cosd(theta) sind(theta) 0];
            v = [(position - [x y]) 0];
            range = norm(v);
            bearing = atan2d(norm(cross(v, u)),dot(v, u));
            if ~Geom2d.leftOf([x y], [(x + u(1)) (y+u(2))], position)
                bearing = -bearing;                
            end
            rb = [range bearing];
        end
    end
end
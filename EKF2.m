% This implementation is based primarily on the EKF-SLAM tutorial 
% http://ais.informatik.uni-freiburg.de/teaching/ws12/mapping/pdf/slam04-ekf-slam.pdf
% Which in turn is based on Thurn et al. "Probabilistic robotics", ch.10.

classdef EKF2 < handle
    % Initialize with initial robot configuration, uncertainty in range 
    % and bearing (from the sensor) and (optionally) the uncertainty in 
    % initial robot position.
    % 
    properties
        x; % Robot state
        l; % Landmarks
        P; % Covariance matrix
        sn; % Signal noise and variation
        sv;
        mn; % Measurement noise and variation
        mv;
    end
    methods
        function obj = EKF2(x, snoise, svar, mnoise, mvar)
            obj.sn = snoise;
            obj.sv = svar;
            obj.mn = mnoise;
            obj.mv = mvar;
            obj.x = x';
            obj.P = zeros(3,3);
        end
        
        % Frame transformation that returns the a point p
        % from the robot frame to the global coordinate frame.
        function [p, Jr, Jpr] = ff(obj, robot, pr) 
            
            % Get robot parameters d = [x y], t = theta
            d = robot(1:2);
            t = robot(3);
            
            % Build rotation matrix.
            R = [cosd(t) -sind(t) ; sind(t) cosd(t)];

            % Compute transformation.
            p = R*pr + d;
            
            if nargout > 1
                prx = pr(1);
                pry = pr(2);

                % Compute jacobian with respect to the robot state.
                Jr = [[ 1, 0, - pry*cosd(t) - prx*sind(t)];  ...
                      [ 0, 1,   prx*cosd(t) - pry*sind(t)]];

                % Compute jacobian with respect to the point in the 
                % robot frame.
                Jpr = R;
            end
        end
        
        % Frame transformation that returns a point pr
        % in the robot frame from the robot frame.
        function [pr, Jr, Jp] = tf(obj, robot, p)
            % Get robot parameters d = [x y], t = theta
            d = robot(1:2);
            t = robot(3);
            
            % Build rotation matrix.
            R = [cosd(t) -sind(t) ; sind(t) cosd(t)];

            % Compute transformation.
            pr = R'*(p - d);
            
            if nargin > 1
                % Get coordinates needed.
                px = p(1);
                py = p(2);
                rx = d(1);
                ry = d(2);
                
                % Compute jacobian with respect to the robot state.
                Jr = [[-cosd(t), -sind(t),  cosd(t)*(py - ry) - sind(t)*(px - rx)]; ...
                      [ sind(t), -cosd(t), -cosd(t)*(px - rx) - sind(t)*(py - ry)]];
                
                % Compute jacobian with respect to the point in the global
                % frame.
                Jp = R';
            end
        end
        
        function [r, Jrm, Jnm] = move(obj, signal, noise) 
            % Load in state angle and add noise
            % to speed and theta
            t = obj.x(3);
            s = signal(1) + noise(1);
            rt = signal(2) + t + noise(2);
            % Coordinate in robot frame
            tp = [s;0];
%             if ao > pi
%                 ao = ao - 2*pi;
%             end
%             if ao < -pi
%                 ao = ao + 2*pi;
%             end

            if nargout > 1      
                % Return moved point from robot frame
                [p, Jr, Jp] = obj.ff(obj.x(1:3), tp);
                
                % Compute jacobian based on the from frame with added
                % angle row
                Jrm = [Jr ; 0 0 1];
                
                % Compute jacobian based on the from frame with respect
                % to noise
                Jnm = [Jp(:,1) zeros(2,1) ; 0 1];
            else 
                % Return moved point from robot frame.
                p = obj.ff(obj.x(1:3), tp);
            end
            r = [p; rt]; 
        end
        
        function [o, Jor, Jop] = observe(obj, robot, p) 
            % Get point in robot frame before
            % performing the observation.
            [pr, Jr, Jp] = obj.tf(robot, p);
            prx = pr(1);
            pry = pr(2);
            
            % Compute range and angle 
            range = sqrt(prx^2 + pry^2);
            t = atan2d(pry, prx);
            
            % Set observation
            o = [range ; t];

            if nargout > 1
                % Compute jacobian of the observation with respect to
                % the observed point.
                Jo =[[prx/(prx^2 + pry^2)^(1/2), pry/(prx^2 + pry^2)^(1/2)]; ...
                     [-pry/(prx^2*(pry^2/prx^2 + 1)), 1/(prx*(pry^2/prx^2 + 1))]];
                  
                % Chain together the two jacobians before returning
                Jor = Jo * Jr;
                Jop = Jo * Jp;
            end
        end
        
        function [p, Jr, Jopr] = iobserve(obj, robot, o)
            % Get range and bearing
            r = o(1);
            b = o(2);
            
            % Compute observed point in robot frame, and
            % changed frame to global frame.
            ox = r * cosd(b);
            oy = r * sind(b);
            pr = [ox ; oy];
            [p, Jr, Jpr] = obj.ff(robot, pr);

            if nargout > 1
                % Compute jacobians using the 
                % chain rule.
                Jo = [[cosd(b) -r*sind(b)]; ...
                      [sind(b)  r*cosd(b)]]; 
                Jopr = Jpr * Jo;
            end
        end
        
        function predict(obj, signal, noise) 
            [obj.x(1:3), Jr, Jn] = obj.move(signal, noise);
            % Get robot covariance matrix.
            Prr = obj.P(1:3, 1:3);
            
            % Update robot covariance.
            obj.P(1:3,1:3) = Jr*Prr*Jr' + Jn*obj.sv*Jn';
            
            % Update robot landmark cross covariances.
            obj.P(1:3,:) = Jr*obj.P(1:3,:);
            obj.P(:,1:3) = obj.P(1:3,:)';
        end
        
        function correct(obj, id, rbT, pos) 
            idx = 2 + 2*id;
            lm = obj.x(idx:(idx+1));
            % Get expected landmark range and bearing.
            [rb, Jr, Jp] = obj.observe(obj.x(1:3), pos);
            Jrp = [Jr Jp];
            
            % Compute expected matrix.
            index = [1:3 idx:(idx+1)];
            E = Jrp * obj.P(index, index) * Jrp';
            
            % Let's find the innovation
            disp([rbT rb]);
            z = rbT - rb;
            disp(z);
            
%             if z(2) > pi
%                 z(2) = z(2) - 2*pi;
%             end
%             if z(2) < -pi
%                 z(2) = z(2) + 2*pi;
%             end

            % Compute the Kalman gain based on the expected matrix
            Z = E + obj.mv;
            K = obj.P(:, index) * Jrp' * Z^(-1);

            % Update the state.
            zzz = K*z;
            zzzz = K*Z*K';
            obj.x = obj.x + K * z;
            obj.P = obj.P - K * Z * K';
        end
        
        function add(obj, pos)
            % Add landmark to state vector by computing its position.
            [lm, Jr, Jo] = obj.iobserve(obj.x(1:3), pos);
            obj.x = [obj.x; lm];

            % Update the covariance matrix properly.
            Prm = Jr * obj.P(1:3,:);
            Prr = Jr * obj.P(1:3,1:3) * Jr' + Jo * obj.mv * Jo';
            obj.P = [obj.P ; Prm];
            obj.P = [obj.P [Prm' ; Prr]];
        end
        
        function [x, y, t] = state(obj)
            x = obj.x(1);
            y = obj.x(2);
            t = obj.x(3);
        end
        
        function [x, y] = lm(obj, id)
            idx = 2 + 2*id;
            x = obj.x(idx);
            y = obj.x(idx+1);
        end
        
        function lms = lms(obj)
            lms = [];
            for i = 1:(size(obj.x, 1) - 3):2
                id = 2 + 2*i;
                lms = [lms ; obj.x(id:(id+1))'];
            end
        end
    end
end
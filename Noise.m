classdef Noise
    %   Here, we generate process noise (Q) and measurement noise(R)
    %   Process noise is inherently part of the process, and is present 
    %   even if the measurements were perfect. Typically defined as 
    %   a Gaussian, with var. proportional to the controls.
    %   Measurement noise is defined by the precision of our (simulated)
    %   sensor, a gaussian with var. proportional to the control range and
    %   constant w.r.t. control bearings.
  
    properties
    end
    
    methods(Static=true)
        function Q = processNoise(deltaX, deltaY, deltaTheta, exactness)
            W = [ -deltaX; deltaY; deltaTheta ]';
            if nargin<4 
                exactness = 0.01;
            end
            C = normrnd(0, exactness, [3 3]);
            Q = W*C*W';
        end
        
        % assumes the angle measurement has a constant error in degrees.
        function measNoise = measurementNoise(range, rangeVar, bearingError)
            V = eye(3);
            if nargin < 3
                bearingError = 1;
            end
            if nargin < 2
                rangeVar = 0.01; % in meters
            end
            if nargin < 1
                range = 5;
            end
            C = normrnd(0, sqrt(rangeVar), [1 2]);
            
            R = [[range*C 0]; [0 bearingError]];
            measNoise = V*R*V';
        end
    end
    
end


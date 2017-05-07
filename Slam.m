%{
	% Slam class.

    % Explanation goes here
%}
classdef Slam
    properties 
        env;                % must be initialised with environment
    end
    
    methods
        % Our constructor function setting the needed parameters
        %   env         the environment
        function obj = Slam(env)
            obj.env = env;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods(Static)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function s = test(filename)
            if nargin < 1
                filename = 'environments/env.txt';
            end
            a = Environment;
            a = a.readFile(filename);
            s = Slam(a);
        end
    end 
end
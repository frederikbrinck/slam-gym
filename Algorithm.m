%{
	Algorithm class.

    Explanation goes here...
%}
classdef Algorithm < Robot.m
    properties

    end
    
    methods(Access = public)
        % The mainloop for the algorithm. It is called at every iteration
        % during the slam simulation.
        function [] = simulate(obj)
            % Use Robot.move() and Robot.read() to do magic.
        end
    end
    
end


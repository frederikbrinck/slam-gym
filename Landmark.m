%{
	Created by Mufei Li on 2017/05/05.

	A class for landmark.
%}

classdef Landmark

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	position;               % (x,y) position relative to the map
    	id = -1;                % the unique ID of landmark

    	% If a landmark hasn not been observed for obj.life times while 
    	% it has a high probability to be reobserved. Discard it.
    	life;                  

    	totalTimesObserved = 0; % the number of times we have seen it
    	range;                  % last observed range 
    	bearing;                % last observed bearing

    	% If the landmark is from a line y = a * x + b.
    	a;                      % slope of the line
    	b;						% intercept of the line

    	% distance from the robot position to the wall we are using as
    	% a landmark (to calculate error).
    	rangeError;

    	% bearing from the robot position to the wall we are using as 
    	% a landmark (to calculate error).
    	bearingError;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    	% Constructor for Landmark
    	function obj = Landmark(lifeSpan)
    		if nargin < 1
    			lifeSpan = 40;
    		end
    		obj.life = 40;
    	end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Static = true)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    	% Given two landmarks, calculate the distance between them.
    	function d = distance(landmark1, landmark2)
    		d = sqrt((landmark1.position(1)-landmark2.position(1))^2+ ...
    			(landmark1.position(2)-landmark2.position(2))^2);
    	end
    end
end

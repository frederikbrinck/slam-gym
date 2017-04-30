%{
	Geom2d class.

    Explanation goes here...
%}

classdef Geom2d
    properties
    end
   
	methods (Static = true)
        function bool = isPoint(poly)
           bool = (length(poly) == 1); 
        end
    end
    
end


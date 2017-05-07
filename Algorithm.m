%{
	Created by Mufei Li on 2017/05/04.

	This file extends the Robot class and it 
	implements two landmark extraction algorithms:
		1. spike landmark extraction
		2. random sampling consensus(RANSAC for short)

	Both of them assume a simulation of a laser scan.

	Besides, this class also does the work of data 
	association.
%}

classdef Algorithm < Robot

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    properties
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	landmarks;

    	% We allow at most obj.maxLandmarks landmarks.
    	maxLandmarks = 3000;

    	% If a landmark is within 20 cms of another landmark, it is the 
    	% same landmark.
    	maxError = 0.2;

    	% Number of times a landmark must be observed to be recognized as 
    	% a landmark.
    	minObservations = 15;

    	% Max times to run RANSAC algorithm.
    	maxTrials = 1000;

    	% Max number of samples points to be randomly selected in RANSAC.
    	maxSample = 10;

    	% For RANSAC, if less than minLinepoints points left, no need to 
    	% find consensus (stop the algorithm).
    	minLinepoints = 30;

    	% For RANSAC, if a point is within ransacTolerance distance of 
    	% a line, it is considered as part of the line.
    	ransacTolerance = 0.05;

    	% For RANSAC, if there are more than ransacConsensus points are 
    	% determined to lie on a line, we say there is a line.
    	ransacConsensus = 30;

    	% The initial life value for a new landmark.
    	LIFE = 40;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    	% Constructor for Algorithm
    	function obj = Algorithm()
    		% Call the constructor for the robot class.
    		obj = obj@Robot();
    	end

    	% Implement RANSAC for extracting line landmarks.
    	function foundLandmarks = extractLineLandmarks(obj)
    		% Two arrays corresponding to found lines.
    		% We assume all lines take the form y = ax + b.
    		% la stores the corresponding a and lb stores the 
    		% corresponding b.
    		la = [];
    		lb = [];

    		% Record the number of lines.
    		totalLines = 0; 

    		% An array of the laser scan output points corresponding to
    		% the seen lines.
    		linepoints = [];

    		% Get the output of a simulated laser scan. This is an array 
    		% of scalars representing ranges(distance between robot and 
    		% closest obstacle) stroed from left to right. 
    		scanOutput = obj.laserRead();

    		% Array to keep track of found landmarks.
    		tempLandmarks = [];

    		% Store empty landmarks at all positions of tempLandmarks.
    		for i = 1:length(scanOutput)
    			tempLandmarks = [tempLandmarks Landmark(obj.LIFE)];
    		end

    		for i = 1:length(scanOutput)
    			linepoints = [linepoints i];
    		end

    		% Record the number of such scan output points.
    		totalLinepoints = length(linepoints);

    		% Retrieve the position of the robot as we will widely use it.
    		robotPosition = obj.getPosition();

    		% Now we will touch RANSAC.

    		% Record the number of trials that it already takes to find 
    		% a line. It will be set to be 0 again once we find a line.
    		noTrials = 0;

    		%{
    			The number of readings is still larger than the consensus 
    			(obj.minLinepoints), and we have done less than 
    			obj.maxTrials trials.
    		%}
    		while (noTrials < obj.maxTrials) && ...
    			(totalLinepoints > obj.minLinepoints)

    			% Initialize array for storing the indexes of the randomly
    			% chosen samples later.
    			randomSelectedPoints = [];

    			%{
    				Randomly select a subset of n data points and estimate
    				the line. Choose one point randomly and then sample
    				neighbours within some defined radius. 
    			%}

    			% Choose a random laser data reading.
    			centerPoint = randi([obj.maxSample+1, totalLinepoints]);
    			randomSelectedPoints(1) = centerPoint;

    			% The loop implies that we will sample obj.maxSample-1 data readings.
    			for i=2:obj.maxSample
    				newPoint = false;
    				while ~newPoint
    					%{
    						Sample a reading that lies within obj.maxSample * 
    						obj.degreesPerScan degrees of the randomly selected 
    						laser data reading.
    					%}
    					temp = centerPoint + (randi(2)-1)*randi([0, obj.maxSample]);
    					for j = 1:i-1
    						if randomSelectedPoints(j) == temp 
    							% if the point has already been selected 
    							break
    						end
    						if j == i-1
    							% if the point has not already been selected
    							newPoint = true;
    						end
    					end
    				end
    				% We have found a new point so we want to store it.
    				randomSelectedPoints(i) = temp;
    			end

    			% Now estimate the line y = ax + b.
    			a = 0;
    			b = 0;

    			%{
    				Use a total number of obj.maxSample readings to calculate
    				a least squares best fit line.

					We pass by value in MATLAB instead of passing by reference
					in the orignal C# code. 
    			%} 
    			lineParameters = obj.leastSquaresLineEstimate(scanOutput, ...
    				randomSelectedPoints, obj.maxSample, a, b);

    			a = lineParameters(1);
    			b = lineParameters(2);

    			% Now we will determine how many laser data readings that lie
    			% within X centimers of this best fit line.
    			consensusPoints = [];

    			newLinePoints = [];
    			totalNewLinePoints = 0;

    			x = 0;
    			y = 0;
    			d = 0;
    			for i = 1:totalLinepoints
    				%{ 
    					Convert ranges and bearing to coordinates. relative to 
    					the map. Note that cos and sin functions take radians 
    					instead of degrees.
    				%} 
    				x = cos(((linepoints(i)-1)*obj.degreesPerScan + ...
    					robotPosition(3))*pi/180) * scanOutput(linepoints(i)) + ...
    					robotPosition(1);
    				y = sin(((linepoints(i)-1)*obj.degreesPerScan + ...
    					robotPosition(3))*pi/180) * scanOutput(linepoints(i)) + ...
    					robotPosition(2);

    				% Calculate the distance between (x,y) and y = ax + b
    				d = Algorithm.distanceToLine(x, y, a, b);

    				if d < obj.ransacTolerance
    					% The point is close enough to the line, consider it as 
    					% part of the line.
    					consensusPoints = [consensusPoints linepoints(i)];
    				else
    					% The point is not close enough to the line. Consider it
    					% an element of newLinePoints.
    					newLinePoints = [newLinePoints linepoints(i)];
    					totalNewLinePoints = totalNewLinePoints + 1;
    				end
    			end

    			%{
    				If the number of laser data readings on the line is above
    				some consensus:
    				1. Calculate new least squares best fit line based on all the
    				   laser readings determined to lie on the old best fit line.
    				2. Add this best fit line to the lines we have extracted.
    				3. Remove the number of readings lying on the line from the 
    				   total set of unassociated readings.
    			%}

    			% Calculate the number of consensus points.
    			totalConsensusPoints = length(consensusPoints);

    			if totalConsensusPoints > obj.ransacConsensus
    				% Calculate the updated line based on consensus points.
    				updatedLineParameters = obj.leastSquaresLineEstimate(... 
    					scanOutput, consensusPoints, totalConsensusPoints, a, b);
    				a = updatedLineParameters(1);
    				b = updatedLineParameters(2);

    				% Remove points that have now been associated to this line.
    				for i = 1:totalConsensusPoints
    					linepoints = newLinePoints;
    					totalLinepoints = totalNewLinePoints;
    				end

    				% Add the line to found lines.
    				la = [la a];
    				lb = [lb b];
    				totalLines = totalLines + 1;

    				% Since we found a line, restart the search.
    				noTrials = 0;
    			else 
    				% We have taken one more trial for finding a line.
    				noTrials = noTrials + 1;
    			end
    		end

    		% End of RANSAC.

    		%{
    			Our EKF is assumed to be dealing with points only. 
    			For each line we found, calculate the point on the line
    			that is closest to the origin (0,0). Add this point as
    			a landmark.
    		%} 

    		for i = 1:totalLines
    			tempLandmarks(i) = obj.getLineLandmark(la(i), lb(i));
    		end

    		% Return landmarks found in an array.
    		foundLandmarks = [];
    		for i = 1:totalLines
    			foundLandmarks = [foundLandmarks tempLandmarks(i)];
    		end
    	end

    	% Extract spike landmarks and return an array of them. The code 
    	% is based on the one from SLAM for Dummies.
    	function foundLandmarks = spikeLandmarkExtraction(obj)
    		% Initialize an array to keep track of found landmarks for 
    		% the temporary.
    		tempLandmarks = [];

    		% Get the output of a simulated laser scan. This is an array 
    		% of scalars representing ranges(distance between robot and 
    		% closest obstacle) stroed from left to right. 
    		scanOutput = obj.laserRead();

    		% Store empty landmarks at all positions of tempLandmarks.
    		for i = 1:length(scanOutput)
    			tempLandmarks = [tempLandmarks Landmark(obj.LIFE)];
    		end

    		for i = 2:length(scanOutput)-1
    			% The original code in slam for dummies store results of 
    			% the laser scan from right to left while our simulation 
    			% does it from left to right. j is then used to simulate
    			% the laser scan output from right to left.
    			j = length(scanOutput)-i+1;
    			if scanOutput(j-1) < obj.laserThreshold
    				if scanOutput(j+1) < obj.laserThreshold
    					if ((scanOutput(j-1)-scanOutput(j))+ ...
    						(scanOutput(j+1)-scanOutput(j))) > 0.5
    						tempLandmarks(i) = obj.getLandmark(scanOutput(j), j); 
    					else
    						if (scanOutput(j-1) - scanOutput(j)) > 0.3
    							tempLandmarks(i) = obj.getLandmark(scanOutput(j), j);
    						end
    					end
    				end
    			elseif scanOutput(j+1) < obj.laserThreshold
    				if scanOutput(j+1) - scanOutput(j) > 0.3
    					tempLandmarks(i) = obj.getLandmark(scanOutput(j), j);
    				end	
    			end
    		end

    		% Return found landmarks in the following array.
    		foundLandmarks = [];
    		for i = 1:length(tempLandmarks)
    			if tempLandmarks(i).id ~= -1
    				foundLandmarks = [foundLandmarks tempLandmarks(i)];
    			end
    		end
    	end

    	% Construct a landmark object with particular property values.
    	function landmark = getLandmark(obj, range, i)
    		landmark = Landmark(obj.LIFE);
    		robotPosition = obj.getPosition();

    		% First, obtain the landmark position relative to the robot. 

    		% Convert landmark to map coordinate
    		% Note that cos takes radians so we need to convert 
    		% everything into radians.
    		landmarkPosition1 = cos(((i-1) * obj.degreesPerScan + ... 
    			robotPosition(3))*pi/180) * range;
    		landmarkPosition2 = sin(((i-1) * obj.degreesPerScan + ... 
    			robotPosition(3))*pi/180) * range;

    		landmark.position = [landmarkPosition1 landmarkPosition2];

    		% Add robot position to get the position relative to the map.
    		landmark.position(1) = landmark.position(1) + robotPosition(1);
    		landmark.position(2) = landmark.position(2) + robotPosition(2);

    		% Update the range and bearing properties of the landmark.
    		landmark.range = range;
    		landmark.bearing = i;

    		% Associate the landmark to its closest landmark.
    		id = -1;
    		totalTimesObserved = 0;
    		obj.getClosestAssociation(landmark, id, totalTimesObserved);
    	end

    	% Given a landmark, we find the closest landmark in the database.
    	function getClosestAssociation(obj, landmark, id, totalTimesObserved)
    		closestLandmark = 1;

    		% Initialize the leastDistance to be the maximum real number.
    		leastDistance = realmax; 

    		for i = 1:length(obj.landmarks)

    			% Only associate to landmarks we have seen more than 
    			% minObservations times.
    			if obj.landmarks(i).totalTimesObserved > obj.minObservations
    				temp = Landmark.distance(landmark, obj.landmarks(i));
    				if temp < leastDistance
    					leastDistance = temp;
    					closestLandmark = obj.landmarks(i).id;
    				end
    			end
    		end

    		if leastDistance == realmax
    			id = -1;
    		else
    			id = obj.landmarks(closestLandmark).id;
    			totalTimesObserved = ... 
    			obj.landmarks(closestLandmark).totalTimesObserved;
    		end

    		% Update the id of the landmark.
    		landmark.id = id;

    		% Update the totalTimesObserved of the landmark.
    		landmark.totalTimesObserved = totalTimesObserved
    	end

    	% Find the best fit line for the points measured by squared distance.
    	function lineFit = leastSquaresLineEstimate(obj, scanOutput, ...
    		selectedPoints, arraySize, a, b)
    		robotPosition = obj.getPosition();

    		sumY = 0;  % sum of y coordinates
    		sumYY = 0; % sum of y^{2} for each coordinate 
    		sumX = 0;  % sum of x coordinates 
    		sumXX = 0; % sum of x^{2} for each coordinate
    		sumYX = 0; % sum of y*x for each point

    		for i = 1:arraySize
    			% Convert the range and bearing into coordinates relative to 
    			% the map.
    			range = scanOutput(selectedPoints(i));
    			x = cos(((selectedPoints(i)-1) * obj.degreesPerScan + ... 
    				robotPosition(3))*pi/180) * range + robotPosition(1);
    			y = sin(((selectedPoints(i)-1) * obj.degreesPerScan + ... 
    				robotPosition(3))*pi/180) * range + robotPosition(2);

    			sumY = sumY + y;
    			sumYY = sumYY + y^2;
    			sumX = sumX + x;
    			sumXX = sumXX + x^2;
    			sumYX = sumYX + y * x;
    		end

    		% intercept
    		b = (sumY*sumXX-sumX*sumYX)/(arraySize*sumXX-sumX^2);
    		% slope
    		a = (arraySize*sumYX-sumX*sumY)/(arraySize*sumXX-sumX^2);
    		lineFit = [a b];
    	end

    	% Calculate a point on the line closest to origin (0,0) so that
    	% it can be used as a point for EKF.
    	function landmark = getLineLandmark(obj, a, b)
    		% First calculate the line perpendicular to the input line.
    		% Assume the line perpendicular to y = ax + b is y = aox + bo.
    		% Then bo = y - aox. Note here we have (px, py) = (0,0).
    		ao = -1.0/a;

    		% Same as what we have in Algorithm.distanceToLine, 
    		% px = (b - bo) / (ao - a) and py = ao * px + bo. 
    		x = b / (ao - a);
    		y = (ao * b) / (ao - a);

    		% Retrieve the position of the robot.
    		robotPosition = obj.getPosition();

    		range = Algorithm.distance(x, y, robotPosition(1), ... 
    			robotPosition(2));
    		% Note the bearing is in radians.
    		bearing = atan((y - robotPosition(2))/(x - ... 
    			robotPosition(1))) - robotPosition(3);

    		% Now get the point on the wall that is closest to the 
    		% robot instead.

    		% Recall y = ao * x + bo.
    		bo = robotPosition(2) - ao * robotPosition(1);

    		% Get the intersection of y = a * x + b and y = ao * x + bo.
    		px = (b - bo) / (ao - a);
    		py = ((ao * (b - bo))/(ao - a)) + bo;

    		rangeError = Algorithm.distance(robotPosition(1), ... 
    			robotPosition(2), px, py);
    		bearingError = atan((py - robotPosition(2))/(px - ... 
    			robotPosition(1))) - robotPosition(3);

    		landmark = Landmark(obj.LIFE);
    		landmark.position = [x y];

    		landmark.range = range;
    		landmark.bearing = bearing;

    		landmark.a = a;
    		landmark.b = b;
    		landmark.rangeError = rangeError;
    		landmark.bearingError = bearingError;

    		% Associate the landmark to the closest landmark.
    		obj.getClosestAssociation(landmark, 0, 0);
    	end

    	% If we have seen a landmark within a box based on the robot 
    	% with some radius for more than landmark.life times, we discard
    	% it. 
    	function removeBadLandmarks(obj, scanOutput, robotPosition)
    		if nargin < 2
    			scanOutput = obj.laserRead();
    			robotPosition = obj.getPosition();
    		elseif nargin < 3
    			robotPosition = obj.getPosition();
    		end

    		% Initialize the radius for the bounding box.
    		maxRange = 0; 

    		% Loop over the scanOutput and set the radius of the box to 
    		% be the maximum value among them.
    		for i = 2:(length(scanOutput)-1)
    			% Distance further away than obj.laserThreshold are 
    			% failed returns.
    			if scanOutput(i-1) < obj.laserThreshold
    				if scanOutput(i+1) < obj.laserThreshold
    					if scanOutput(i) > maxRange
    						maxRange = scanOutput(i);
    					end
    				end
    			end
    		end

    		Xbounds = [];
    		Ybounds = [];

    		Xbounds(1) = cos((obj.degreesPerScan + ... 
    			robotPosition(3))*pi/180) * maxRange + robotPosition(1);
    		Ybounds(1) = sin((obj.degreesPerScan + ... 
    			robotPosition(3))*pi/180) * maxRange + robotPosition(2);
    		Xbounds(2) = Xbounds(1) + cos((180 * obj.degreesPerScan + ... 
    			robotPosition(3))*pi/180) * maxRange;
    		Ybounds(2) = Ybounds(1) + sin((180 * obj.degreesPerScan + ...
    			robotPosition(3))*pi/180) * maxRange;
    		Xbounds(3) = cos((359 * obj.degreesPerScan + ... 
    			robotPosition(3))*pi/180) * maxRange + robotPosition(1);
    		Ybounds(3) = sin((359 * obj.degreesPerScan + ...
    			robotPosition(3))*pi/180) * maxRange + robotPosition(2);
    		Xbounds(4) = Xbounds(3) + cos((180 * obj.degreesPerScan + ...
    			robotPosition(3))*pi/180) * maxRange;
    		Ybounds(4) = Ybounds(3) + sin((180 * obj.degreesPerScan + ...
    			robotPosition(3))*pi/180) * maxRange;

    		for k = 1:length(obj.landmarks)
    			pntx = obj.landmarks(k).position(1);
    			pnty = obj.landmarks(k).position(2);
    			% Deterime if the robot is in the box.
    			if (robotPosition(1) < 0) | (robotPosition(2) < 0)
    				inRectangle = false;
    			else
    				inRectangle = true;
    			end

    			j = 0;

    			for i = 1:4
    				% Check if the landmark lies in the box.
    				if (((((Ybounds(i) <= pnty) & (pnty < Ybounds(j))) | ...
    					((Ybounds(j) <= pnty) & (pnty < Ybounds(i)))) & ... 
    					(pntx < (Xbounds(j) - Xbounds(i)) * ... 
    						(pnty - Ybounds(i)) / (Ybounds(j) - ... 
    							Ybounds(i)) + Xbounds(i))))
    					if inRectangle == false
    						inRectangle = true;
    					else 
    						inRectangle = false;
    					end
    				end
    				j = i;
    				i = i + 1;
    			end

    			if inRectangle
    				% It has taken one more time to reobserve the landmark.
    			 	% Decrement the life of the landmark. 
    				obj.landmarks(k).life = obj.landmarks(k).life - 1;

    				if obj.landmarks(k).life <= 0
    					for kk = k:length(obj.landmarks)
    						if kk == (length(obj.landmarks) - 1)
    							obj.landmarks(kk) = obj.landmarks(kk+1);
    						else
    							obj.landmarks(kk+1).id = obj.landmarks(kk ... 
    								+1).id - 1;
    							obj.landmarks(kk) = obj.landmarks(kk + 1);
    						end
    					end
    					obj.landmarks(length(obj.landmarks)) = [];
    				end
    			end
    		end
    	end

    	function landmark = updateLandmark(obj, lm)
    		% Do data association for landmarks. 
    		id = obj.getAssociation(lm);

    		% If we fail to associate the landmark to some existing 
    		% landmarks. Add it to obj.landmarks.
    		if id == -1
    			id = obj.addToLandmarks(lm);
    		end

    		lm.id = id;

    		landmark = lm;
    	end

    	function identity = getAssociation(obj, landmark, innovation)
    		% Innovation is the difference between the estimated robot 
    		% position and the actual robot position.
    		if nargin < 3
    			innovation = obj.maxError;
    		end

    		identity = -1;

    		for i = 1:length(obj.landmarks)
    			if (Landmark.distance(landmark, obj.landmarks(i)) < ... 
    				innovation) & (obj.landmarks(i).id ~= -1)
    				% We succesfully reobserve the landmark, reset its 
    				% life counter.
    				obj.landmarks(i).life = obj.LIFE;
    				obj.landmarks(i).totalTimesObserved = ... 
    				obj.landmarks(i).totalTimesObserved + 1;

    				obj.landmarks(i).bearing = landmark.bearing;
    				obj.landmarks(i).range = landmark.range;

    				identity = obj.landmarks(i).id;
    				return 
    			end
    		end
    	end

    	% Add a new landmark to our collection of landmarks.
    	function identity = addToLandmarks(obj, lm)

    		identity = -1;

    		% Check if we have reached the capacity for storing landmarks.
    		if (length(obj.landmarks)+1) < obj.maxLandmarks
    			newLandmark = Landmark(obj.LIFE);
    			newLandmark.position = lm.position;
    			newLandmark.id = length(obj.landmarks) + 1;
    			newLandmark.totalTimesObserved = 1;
    			newLandmark.bearing = lm.bearing;
    			newLandmark.range = lm.range;
    			newLandmark.a = lm.a;
    			newLandmark.b = lm.b;
    			obj.landmarks = [obj.landmarks newLandmark];

    			identity = newLandmark.id;
    		end
    	end

    	% Update and add line landmarks.
    	function landmarks = updateAndAddLineLandmarks(obj, extractedLandmarks)
    		landmarks = [];
    		for i = 1:length(extractedLandmarks)
    			landmarks(i) = obj.updateLandmark(extractedLandmarks(i));
    		end
    	end

    	% Update and add line landmarks using EKF results.
    	function landmarks = updateAndAddLineLandmarksUsingEKF(obj, ... 
    		matched, id, ranges, bearings)
    		landmarks = [];
    		for i = 1:length(matched)
    			landmarks(i) = obj.updateLandmarkUsingEKF(obj, matched(i), ... 
    				id(i), ranges(i), bearings(i))
    		end
    	end

    	% Update landmark using EKF results.
    	function landmark = updateLandmarkUsingEKF(obj, matched, id, ... 
    		distance, readingNo)
    		landmark = Landmark(obj.LIFE);
    		robotPosition = obj.getPosition();
    		if matched
    			% EKF matched the landmark. Increase times the landmark has 
    			% been observed. 
    			obj.landmarks(id).totalTimesObserved = obj.landmarks(id).totalTimesObserved ... 
    			+ 1;
    			landmark = obj.landmarks(id);
    		else 
    			% EKF failed to match the landmark. Add it to the collection as a 
    			% new landmark. 
    			landmark.position(1) = cos((readingNo * obj.degreesPerScan + ... 
    				robotPosition(3))*pi/180) * distance;
    			landmark.position(2) = sin((readingNo * obj.degreesPerScan + ...
    				robotPosition(3))*pi/180) * distance;

    			% Add robot position.
    			landmark.position(1) = landmark.position(1) + robotPosition(1);
    			landmark.position(2) = landmark.position(2) + robotPosition(2);

    			landmark.bearing = readingNo;
    			landmark.range = distance;

    			id = obj.addToLandmarks(landmark);
    			landmark.id = id;
    		end 
    	end
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    methods (Static = true)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	function distance = distanceToLine(x, y, a, b)
    		%{
    			Find a point on the line y = ax + b that is closest to 
    			(x,y). Then calculate the distance between them.
    		%}
    		% The slope of the line perpendicular to y = ax + b is -1/a.
    		ao = -1.0/a;
    		% Suppose the line perpendicular to y = ax + b is y = aox + bo.
    		% Then bo = y - aox.
    		bo = y - ao * x;

    		% Get intersection of the two lines. aox + bo = ax + b. Then 
    		% x = (b - bo)/(ao-a) and y = ao * x + bo.
    		px = (b - bo) / (ao - a);
    		py = (ao * (b - bo) / (ao - a)) + bo;

    		% Calculate the distance between the two points.
    		distance = Algorithm.distance(x, y, px, py);
    	end

    	function d = distance(x, y, px, py)
    		% Calculate the Euclidean distance between (x,y) and (px, py).
    		d = sqrt((x-px)^2+(y-py)^2);
    	end
    end
end

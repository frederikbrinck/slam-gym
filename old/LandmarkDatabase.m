classdef LandmarkDatabase < handle
    properties
        landmarks = [];
        maxAssociateError = 10;
        maxLandmarkError = 0.8;

        % New landmarks do not need to get updated through EKF. Keep
        % them separate and add them to our landmark database obj.landmarks
        % after updating through EKF.
        newLandmarks = [];
    end
    
    methods
        function obj = LandmarkDatabase(associateError, landmarkError)
           if nargin == 2
               obj.maxAssociateError = associateError;
               obj.maxLandmarkError = landmarkError;
           end
        end

        function lms = extractLandmarks(obj, observations)
           % Apply obj.associateLandmark to all observations.
           lms = [];
           for i = 1:size(observations,1)
               lm = obj.associateLandmark(observations(i,1),observations(i,2));
               if isa(lm, 'Landmark')
                   lms = [lms lm];
               end
           end
        end
        
        function lms = extractNewLandmarks(obj)
           lms = obj.newLandmarks;
        end
        
        function lm = associateLandmark(obj, x, y)
           % There are three cases.
           % 1. If the point is associated to an existing landmark, 
           %    return the landmark associated.
           % 2. If the point is very close to an existing landmark, 
           %    but not close enough to be associated. Discard both 
           %    the point and the landmark as it is not distinguishable.
           % 3. Otherwise, consider it as a new landmark which will be
           %    added to the landmark database after EKF update.

           lm = false;
           %disp('Cheking landmark for pos')
           %disp([x y])
           %disp('---')
           for i = 1:length(obj.landmarks)
               cLm = obj.landmarks(i);
               %disp(norm([x y] - cLm.position))
               if norm([x y] - cLm.position) < obj.maxAssociateError
                   lm = cLm;
                   break;
               end
           end
           
           % If the point is not close enough to an existing landmark,
           % store it in obj.newLandmarks.
           if ~isa(lm, 'Landmark')
               tooClose = false;  
               for i = 1:length(obj.landmarks)
                   cLm = obj.landmarks(i);
                   if norm([x y] - cLm.position) < obj.maxLandmarkError
                       tooClose = true;
                       break;
                   end
               end

               if tooClose 
                   % Don't update landmark ids, due to EKF
                   % filter
                   %for j = i+1:length(obj.landmarks)
                   %    obj.landmarks(j).id = obj.landmarks(j).id - 1;
                   %end 
                   obj.landmarks(i) = [];
               end
               
               if ~tooClose
                   lm = Landmark();
                   lm.position = [x y];

                   % Don't give dynamic ids due to EKF
                   %if isempty(obj.newLandmarks)
                       lm.id = length(obj.landmarks) + 1;
                   %else
                   %    lm.id = obj.newLandmarks(length(obj.newLandmarks)).id + 1;
                   %end
                   obj.newLandmarks = [obj.newLandmarks lm];
                   
                   % Make return object empty
                   lm = [];
               end
           end
        end

        function lms = addNewLandmarks(obj)
           % Add new landmarks to our database and set it to be empty.
           lms = obj.newLandmarks;
           obj.landmarks = [obj.landmarks obj.newLandmarks];
           obj.newLandmarks = [];
        end
    end
    
    methods(Static)
        function s = test()
           DB = LandmarkDatabase();
           DB.extractLandmarks([1 2; 3 4]);
           DB.addNewLandmarks();

           disp(['You should see two landmarks in the database.']);
           DB
           DB.landmarks(1)
           DB.landmarks(2)

           disp(['New observations are [1.001 2.001] and [3.1 4.1].']);
           disp(['[1.001 2.001] would be associated with [1 2].']);
           disp(['[3.1 4.1] is very close to [3 4] and both ', ... 
            'of them will not be good landmarks.'])
           disp(['You should see one landmark in the database.']);
           DB.extractLandmarks([1.001 2.001; 3.1 4.1]);
           DB
           DB.landmarks(1)

           disp(['You should see no changes.']);
           DB.addNewLandmarks();
           DB
           DB.landmarks(1)
        end
    end
end
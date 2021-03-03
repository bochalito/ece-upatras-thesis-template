function [ xT,yT ] = findPosition2D( Xnew,Ynew,Xlast,Ylast,Radius )
% Uses a simple localization logic. When a new scan is ready
% it uses the scan matching algorithm to estimate the position and
% then compares the result with the last known valid position.
%
%
%   Ordinary Usage:      
%   
%   [ xT,yT ] = findPosition2D(Xnew, Ynew, Xlast, Ylast, Radius)
%   
%   Returns the position estimation based on the last known position.
%   If the difference between the last known position and the result from
%   scan matching is larger than Radius there is no update of the current
%   position.
%
%   INPUT:
%   
%   [Xnew,Ynew]   - position estimation from scan matching
%   [Xlast,Ylast] - current position
%   Radius        - new position boundary
%

	if ( abs(abs(Xnew)-abs(Xlast)) > Radius ) 
		xT = Xlast;
	else
		xT = Xnew;
		Xlast = xT;    
	end
		
	if ( abs(abs(Ynew)-abs(Ylast)) > Radius ) 
		yT = Ylast;
	else
		yT = Ynew;
		Ylast = yT;    
	end

end


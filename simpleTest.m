clc,close all,clear all
load -ascii data1.txt

data = data1;

ref_dist = data(1:360);

ref_dist = data(1:360);
j = 132; 
cur_dist = data((360*j+1):(360*(j+1)));

close all
    
lastPos = refArray(j-1,:);
lastPos(1) = -lastPos(1);
lastPos(2) = -lastPos(2);
    
scanMatch2D(cur_dist,ref_dist,'step',5,'filter','yes','plot','setupscan',...
	[2500 1100],'lastpos',lastPos);
     
%scanMatch2D(cur_dist,ref_dist,'step',5,'filter','yes','plot',...
%	'setupscan',[0 0]);


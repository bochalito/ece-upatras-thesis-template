function [Xt,Yt,Rotation] = scanMatch2D(cur_dist,ref_dist,varargin)
% Uses corellation of Range-Finder scans as presented 
% in [Weib,Wetzler,Puttkamer 2009].This version does not use
% the modulo transformation. Instead it calculates the histograms
% based on the actual measurements.
%
%   Ordinary Usage:      
%   
%   [Xt,Yt,Rot] = scanMatch2D(cur_dist,ref_dist)
%   
%   Returns the translation and the rotation between 2 Laser Scans
%   with 360 data measurements each(from Neato Lidar).
%
%   INPUT:
%   
%   cur_dist  - 360x1 distance measurements
%   ref_dist  - 360x1 distance measurements
%   plot      - 'yes' for plotting the procedure
%   step      - the step for the EDGES vector (see histc) *Key factor*
%   filter    - 'yes' for filtering data(ex. outliers)
%   setupScan - set up global coordinate system
%
%   Special usage:
%
%   [Xt,Yt,Rot] = scanMatch2D(cur_dist,ref_dist,'plot','yes')
%   [Xt,Yt,Rot] = scanMatch2D(cur_dist,ref_dist,'step',50)
%
%   version 1.0

%Default parameters value
showPlot=false;
filterData=false;
step=20;
X_Ref = 0 ;
Y_Ref = 0;
lastPos = false;
X_last = 0;
Y_last = 0;
Rot_last = 0;

%Check if parameters exist else use default values.
while ~isempty(varargin)
    switch upper(varargin{1})      
        case 'PLOT'
            showPlot = true;
            varargin(1) = []; 
        case 'SETUPSCAN'
            setup = varargin{2};
            X_Ref = setup(1);
            Y_Ref = setup(2);
            varargin(1:2) = [];  
        case 'LASTPOS'
            lastPos = true;
            setup = varargin{2};
            X_last = setup(1);
            Y_last = setup(2);
            Rot_last = setup(3);         
            varargin(1:2) = [];  
        case 'STEP'
            step = varargin{2};
            varargin(1:2) = [];  
        case 'FILTER'
            filterData = strcmp( varargin{2},'yes' );
            varargin(1:2) = []; 
        otherwise
            error(['Invalid option: ' varargin{1}])
    end
end

%Finds rotation and returns the angle histograms and their corellation
[Rotation,histC,histR,angleCor] = findRotationNoPlot(cur_dist,ref_dist);     

%Dead-Reckoning for better rotation estimation. We check the largest peaks
%of the correlation function and estimate rotation based on the previous
%value.
    if (lastPos)
        %we find the peaks of the angle-histograms corellation and sort
        %them from the largest to the smallest.
        [~,lsor] = findpeaks(angleCor,'SortStr','descend');
        
        findRot = true;
        indexR = 1;

        while ( findRot )
            
            rotation = lsor(indexR);
            
            %rotation is between [-180,180]
            if (rotation > 180)
                rotation = 360-rotation;
            else
                rotation = - rotation;
            end
            
            %compare with the previous estimated value. If absolute
            %difference is big then we check the next peak.
            if ( abs(rotation - Rot_last) > 10 )
                indexR = indexR + 1;   
            else
            %if not rotation is estimated correctly.
                findRot = false;
                Rotation = rotation;                            
            end
            
            %we check up to 7 peaks before we decide that a rotation
            %estimation from the current scan is impossible.
            if ( indexR > 7 )
                Rotation = Rot_last;
                break;
            end

        end
    end
        
%***BUG***
%Subscript indices must either be real positive integers or logicals.
%When rotation becomes -1 for example
%***BUG***

%Rotate scans to their most common direction
%rotationR = find(histR == max(histR(:)));
%rotationC = find(histC == max(histC(:)));

%with the current code we have our refernce scan already rotated and
%parallel to the main axes.
rotationR=0;
ref_distF = [ref_dist;ref_dist];
ref_distF = ref_distF(rotationR+1:360+rotationR);
rotationC = Rotation;
cur_distF = rotateScan(cur_dist,rotationC);

%check the filter radius
if (filterData)
    cur_distF = myFilter(cur_distF,150);
    ref_distF = myFilter(ref_distF,150);
end

%get the cartesian coordinates from the two(2) scans.
[X1,Y1] = pol2cart( (0:2*pi/360:2*pi*(1-1/360))' , cur_distF );
[X2,Y2] = pol2cart( (0:2*pi/360:2*pi*(1-1/360))' , ref_distF ) ;

%Translate the reference scan with the reference values.That means that the
%position will be computed according to the reference values. To be
%checked!
X2 = X2+X_Ref;
Y2 = Y2+Y_Ref;

%Compute the edges vector for the x-y histograms
Xedges= round(min(min(X1),min(X2)))-step:step:round(max(max(X1),max(X2)))+step;
Yedges= round(min(min(Y1),min(Y2)))-step:step:round(max(max(Y1),max(Y2)))+step;

%Compute X-Y histograms
histX1 = histc(X1,Xedges);
histX2 = histc(X2,Xedges);
histY1 = histc(Y1,Yedges);
histY2 = histc(Y2,Yedges);

%If filter data option is selected then we make zero the largest value on
%each computed histogram. To be checked!
if (filterData)
    histX1(histX1 == max(histX1(:))) = 0;
    histX2(histX2 == max(histX2(:))) = 0;
    histY1(histY1 == max(histY1(:))) = 0;
    histY2(histY2 == max(histY2(:))) = 0;
end

%Then we find the max value of each histogram.
maxY1 = find(histY1 == max(histY1(:)));
maxY2 = find(histY2 == max(histY2(:)));
maxX1 = find(histX1 == max(histX1(:)));
maxX2 = find(histX2 == max(histX2(:)));

%fixes a bug when maxX is larger than 1 element.
maxX1 = max(maxX1);
maxX2 = max(maxX2);

%Here we perform dead-reckoning for [x,y] values.
if ( lastPos ) 
     
     yCor = findTranslation(histY1,histY2);
     [~,lsor] = findpeaks(yCor,'SortStr','descend');
     point  = min(lsor(1),lsor(2));
     Yt = -step*point;
     
     %radius is 350. Too big ? ?
     if ( abs(abs(Yt)-abs(Y_last)) > 250 ) 
          yCor = findTranslation(histY2,histY1);
          [~,lsor] = findpeaks(yCor,'SortStr','descend');
          point  = min(lsor(1),lsor(2));
          Yt = step*point;
          
          if (Yt > 0 )
              Yt = - Yt;
          end
		  
          if ( abs(abs(Yt)-abs(Y_last)) > 250 ) 
             Yt = Y_last;            
             if (Yt > 0 )
				Yt = - Yt;
             end           
         end
     end
          
        if (maxX1 < maxX2)
            xCor = findTranslation(histX1,histX2);
            Xt = -step*max(find(xCor == max(xCor(:))));
        else
            xCor = findTranslation(histX2,histX1);
            Xt = step*max(find(xCor == max(xCor(:))));
        end

        if (maxX1 ==  maxX2)
            Xt = 0;
        end    
    %Xt computed...
    %***************
	
%simple computation without dead-reckoning
else
    if (maxY1 <  maxY2)
        yCor = findTranslation(histY1,histY2);
        Yt = -step*max(find(yCor == max(yCor(:))));       
    else
        yCor = findTranslation(histY2,histY1);
        Yt = step*max(find(yCor == max(yCor(:))));
    end

    if (maxY1 ==  maxY2)
        Yt = 0;
    end
    %Yt computed...   
    if (maxX1 < maxX2)
        xCor = findTranslation(histX1,histX2);
        Xt = -step*max(find(xCor == max(xCor(:))));
    else
        xCor = findTranslation(histX2,histX1);
        Xt = step*max(find(xCor == max(xCor(:))));
    end

    if (maxX1 ==  maxX2)
        Xt = 0;
    end
    %Xt computed...   
end

if (showPlot)
        
        subplot(2,2,1)
        polar((0:2*pi/360:2*pi*(1-1/360))',cur_dist,'b.');
        hold on
        polar((0:2*pi/360:2*pi*(1-1/360))',ref_dist,'g.'); 
        title('Lidar Polar Plot');
        legend1 = legend('Current scan','Reference scan');
        set(legend1,'Position',[0.15 0.49 0.28 0.08],'FontSize',11,'Orientation','Horizontal');
        
        subplot(2,2,2)
        plot( (1:360),histC,'b' ,'linewidth',2);
        hold on
        plot( (1:360),histR,'g' ,'linewidth',2);
        title('Angle Histograms');
        
        subplot(2,2,4)
        plot((1:360),angleCor,'r','linewidth',2);     
        title('Cross corellation');

        subplot(2,2,3)
        polar((0:2*pi/360:2*pi*(1-1/360))',rotateScan(cur_dist,Rotation),'b.');
        hold on
        polar((0:2*pi/360:2*pi*(1-1/360))',double(ref_dist),'g.'); 
        title(['Scan rotated by ',num2str(Rotation),' deg.']);
        
        figure
        subplot(3,2,1)
        plot(X1,Y1,'b.'); hold on
        plot(X2,Y2,'g.');
        title('Before translation');
        subplot(3,2,2);
        plot(X1-Xt,Y1-Yt,'b.');
        hold on
        plot(X2,Y2,'g.');
        title('After translation');
        
        subplot(3,2,3)
        plot( histX1,'b-','linewidth',2);
        hold on
        plot(  histX2,'g-','linewidth',2);

        c = max(normCrossCorr(histX1,histX2)); 
        title(['X hist with quality=',num2str(uint16(100*c)),'%']);

        subplot(3,2,4)
        plot(  histY1,'b-','linewidth',2);
        hold on
        plot(  histY2,'g-','linewidth',2);

        c = max(normCrossCorr(histY1,histY2));
        title(['Y hist with quality=',num2str(uint16(100*c)),'%']);

        subplot(3,2,5);
        plot(xCor,'r','linewidth',2);
        title(['xT = ',num2str(Xt),' mm']);

        subplot(3,2,6);
        plot(yCor,'r','linewidth',2);
        title(['yT = ',num2str(Yt),' mm']);    
end


end

function [rotation,hist1,hist2,corellation] = findRotationNoPlot( current_scan ,ref_scan )
% Corellates two angle histograms and returns the rotation
% between them.

        %Calculate angle Histogram
        hist1 = angleHistogram(current_scan);
        hist2 = angleHistogram(ref_scan);
       
        %Compute cross corellation of the 2 angle histograms
        [rotation,corellation] = crossCorelation(hist1,hist2);
       
        if (rotation > 180)
            rotation = 360-rotation;
        else
            rotation = -rotation;
        end
end

function [hist] = angleHistogram( distance )
% Returns the angle histogram of a Laser Scan as presented in the paper.

    [X,Y] = pol2cart( (0:2*pi/360:2*pi*(1-1/360))' , distance );
    angle = [];
    
    for i = 1 : 1 : 360
       
        %this is the basic line
        p2 = [X(i)+2,Y(i)];
        p1 = [X(i)  ,Y(i)];
        
        step = 2;
        
        if ( (i+step)>360 )
            j  = (step+i) - 360;
            p4 = [X(j),Y(j)];
        else
            p4 = [X(i+step),Y(i+step)]; 
        end
               
        p3 = [X(i),Y(i)];
        
        v1 = p2 - p1;
        v2 = p4 - p3;
        
        angle = [ angle; 180/pi*mod( atan2( det([v1',v2']) , dot(v1,v2) ) , 2*pi )];

    end
    
    angle = int16(angle);
    hist  = zeros(1,360);

    for i = 1:1:360
        if (angle(i)==0) 
            hist(1) = hist(1)+1;
        else
            hist(angle(i)) = hist(angle(i)) + 1;
        end    
    end
end

function [ scan ] = myFilter(scan,flag)
% Returns a filtered scan. Right now the filtering justs
% makes all distances < flag to 0.
    for i=1:1:numel(scan)
        
        if (scan(i) < flag)
            scan(i)=0;
        end
    end  
end

function [corellation] = normCrossCorr(hist1 , hist2)
% Returns a value from -1 to 1. 
% Returns:
%      1: data are perfectly corellated
%      0: data are not corellated
%
% This value is used to determine if a corellation is done with
% sufficient quality but it can only be used for data that have 
% certain significances.

    hist1mean = mean(hist1);
    hist2mean = mean(hist2);
    
    corellation = [];

    hist2 = hist2' ;
    hist1 = hist1' ;

    hist2 = [hist2;hist2];

    for j = 1:1:numel(hist1)  
        sum1 = 0;
        sum2 = 0;
        sum3 = 0;
        
        for i = 1:1:numel(hist1)      
            sum1 = sum1 + ( (hist1(i)-hist1mean) * (hist2(i+j)-hist2mean) );
            sum2 = sum2 + ( (hist1(i)-hist1mean)^2 );
            sum3 = sum3 + ( (hist2(i+j)-hist2mean)^2 );
        end

        corellation = [corellation ; ( sum1 / sqrt(sum2*sum3) ) ];
    end
end

function [rotation,corellation] = crossCorelation(hist1 , hist2)

    corellation = [];
    hist2 = hist2' ;
    hist1 = hist1' ;
    hist2 = [hist2;hist2];

    for j = 1:1:360  
        sum = 0;

        for i = 1:1:360      
            sum = sum + (hist1(i)*hist2(i+j)) ;  
        end
        corellation = [corellation ; sum];
    end
    [xcor, rotation]=max(corellation);
end

function [ rotatedScan ] = rotateScan( scan , angle )
    
    scan = [scan;scan;];   
    if (angle>0)
       rotatedScan = scan(angle+1:360+angle);
    else
       angle = 360+angle;
       rotatedScan = scan(angle+1:360+angle);
    end   
end

function [ corellation ] = findTranslation( histK, histM )

    corellation = [];
    histM = [histM;histM];

    for j = 1:1:numel(histK)
        sum = 0;

        for i = 1:1:numel(histK) 
            sum = sum + (histK(i)*histM(i+j)) ;  
        end

        corellation = [corellation ; sum];
    end
end

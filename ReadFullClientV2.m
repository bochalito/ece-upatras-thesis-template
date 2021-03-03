%--------------------------------------------------------------------------
%           Ship Control Panel Interface
% 
%                                                       created:14/06/2015
%--------------------------------------------------------------------------


%check if bluetooth connection exists and terminate it.
if ( evalin('base','exist(''fid'',''var'')') )
    fclose(fid);
end
%check if server connection exists and terminate it.
if ( evalin('base','exist(''myServer'',''var'')') )
    fclose(myServer);
end

clc, clear all, close all


%Arduino has a static IP this time
myServer = tcpip('192.168.1.50',2390,'NetworkRole','Server');
fopen(myServer);

pause(2)
%Make sure that the arduino client can connect to the server
while ( myServer.BytesAvailable == 0 )
    fclose(myServer);
    pause(0.5)
    fopen(myServer);
    pause(2)
end

%Read the connection acknowledgement from the Arduino
fread(myServer,myServer.BytesAvailable);
disp('Server is on');

%set up bluetooth connection
    %fid = serial('COM4');
    %set(fid,'BaudRate',115200);
fid = Bluetooth('Lidar',1);
set(fid,'InputBufferSize',400000);   
%set(fid,'Timeout',1);                
fopen(fid);
disp('Comm is on');

%load the controller
FIS  = readfis('Controller/control_2_FR_R');

%set up initial values
first_scan = true ;             %true until we have reference scan available
data = [];                      %holds some data
controlData = [];               %holds some more data
scanNum = 0;                    %holds the current scan number
control_StartTime = 0;          %holds the time we pressed autopilot
control_Successful = false;     %true if inside acceptable limits
angleSpeed = [];

%set up values for dead-reckoning
X_last = 0;
Y_last = 0;
Rot_last = 0;

%controller input values
DISTANCE = 0;
MAIN_ORIEN = 0;
MINOR_ORIEN = 0;
SPEED = 0;

%Target point and orientation
targetX = 0;
targetY = 0;
targetRot = 90;

%*change this implentation please!
btnString = 'Lidar OFF';

global lidarON;
global programRun;
global setupRun;

setupRun   = 1;
programRun = 1;
lidarON    = 1;

IMU_ON = false;
%--------------------------------------------------------------------------
%myGUI
%--------------------------------------------------------------------------
S = controlPanelGui();

%set callBacks and handles
set(S.btnExit,'callback',{@exitBtn,fid,myServer} ); 
set(S.serverButton,'callback',{@serverConnect,myServer} ); 
set(S.btnLidar,'callback',{@lidarBtn,fid,btnString} ); 
set(S.pickTargetPointbtn,'callback',...
	{@pickPointBtn,S.target_X_txt,S.target_Y_txt} );
set(S.pickTargetRotbtn,'callback',{@pickRotBtn,S.target_Rot_txt} );
set(S.stopMotorsBtn,'callback',{@stopMotorBtn,myServer} );
set(S.setMotorsBtn,'callback',{@setMotorBtn,myServer,S.Propeller_txt,...
    S.Rudder_txt,S.Bow_Txt,S.Rear_Txt} );
%--------------------------------------------------------------------------
%myGUI - end
%--------------------------------------------------------------------------

%Tell Lidar to send 1 packet.
fwrite(fid,0);

while (programRun) 
    
    if (fid.BytesAvailable >= 720)
        %set up reference scan
        if (first_scan) 
            disp('First Scan')
            %fid.BytesAvailable
            ref_dist = uint8(fread(fid,720));
            %************************************
            if ( IMU_ON )
                heading = str2num(char(fread(fid,3))');
            end
            %************************************
            ref_dist = typecast(ref_dist,'uint16');
            
            %--------------------------------------------------------------
            %Setup GUI
            %--------------------------------------------------------------
            
            %Setup reference scan and global axis system
            set(S.myFigure,'Visible','off');
            
            setupRun = 1;      
            setupFigure = figure('Position', [100, 100, 1600, 500],...
				'Resize','off');
            
            subplot(1,2,1)
            polarPlotSetup = polar((0:2*pi/360:2*pi*(1-1/360))',...
				ones(360,1)+5000,'b.');
            set(polarPlotSetup,'MarkerSize',15);
            title('[Setup] Reference Scan','FontWeight','Bold'); 
            [Xref,Yref] = pol2cart((0:2*pi/360:2*pi*(1-1/360))',...
				double(ref_dist));
            
            set(polarPlotSetup,'YData',Yref); 
            set(polarPlotSetup,'XData',Xref);
            
            XLabelSetup = uicontrol('Style','text','FontSize',12,...
                'Position',[20 340 80 20],...
                'String','X (mm)','FontWeight','Bold');
            XTxtSetup = uicontrol('Style','edit','FontSize',12,...
                'Position',[20 320 80 20],...
                'String',num2str(ref_dist(180)),'FontWeight','Bold',...
                'BackgroundColor','White','ForegroundColor',[0,0.5,0]);
            YLabelSetup = uicontrol('Style','text','FontSize',12,...
                'Position',[20 290 80 20],...
                'String','Y (mm)','FontWeight','Bold');  
            rotationLabelSetup = uicontrol('Style','text','FontSize',12,...
                'Position',[20 390 80 20],...
                'String','Rot (deg)','FontWeight','Bold'); 
            rotationTxtSetup = uicontrol('Style','edit','FontSize',12,...
                'Position',[20 370 80 20],...
                'String','0','FontWeight','Bold',...
                'BackgroundColor','White','ForegroundColor',[0,0.5,0]);
            fileSetup = uicontrol('Style','edit','FontSize',12,...
                'Position',[20 130 150 20],...
                'String',datestr(now,'mm_dd_yyyy_HH_MM'),...
                'BackgroundColor','White','Visible','off','Enable','off');
            fileLabelSetup = uicontrol('Style','text','FontSize',12,...
                'Position',[20 150 150 20],...
                'String','Filename (.txt)','FontWeight','Bold',...
                'Visible','off','Enable','off');
            fileCheckBoxSetup = uicontrol('Style','checkbox','FontSize',12,...
                'Position',[20 180 150 20],...
                'String','Create a File',...
                'Callback',{@createFile,fileSetup,fileLabelSetup}); 
            YTxtSetup = uicontrol('Style','edit','FontSize',12,...
                'Position',[20 270 80 20],...
                'String',num2str(ref_dist(270)),'FontWeight','Bold',...
                'BackgroundColor','White','ForegroundColor',[0,0.5,0]); 
            btnExitSetup = uicontrol('Style', 'pushbutton', 'String',...
                'Complete Setup' ,'FontSize',12,...
                'Position', [20 20 150 30],...
                'Callback', {@completeSetup} );
             
            subplot(1,2,2)
            cartRefSetup = plot(1,1,'b.');
            hold on
            
            plot([0 0],[-5000 5000],'LineWidth',2,'Color','r');
            plot([-5000 5000],[0 0],'LineWidth',2,'Color','r');
            set(cartRefSetup,'YData',Yref); 
            set(cartRefSetup,'XData',Xref);
            title('[Setup] Map','FontWeight','Bold'); 
            
            btnY = uicontrol('Style', 'pushbutton', 'String', 'Set',...
				'FontSize',10,'Position', [110 270 90 25],...
				'FontWeight','Bold','Callback',...
				{@rotationSlider,ref_dist,polarPlotSetup,...
                rotationTxtSetup,cartRefSetup,XTxtSetup,YTxtSetup} );
            %--------------------------------------------------------------
            %Setup GUI - end
            %--------------------------------------------------------------
            
            
            while (setupRun)
                drawnow
            end
            
            %check if user wants to create a txt file
            write2File = get(fileCheckBoxSetup,'Value');
            
            if ( write2File )
                myFileId = fopen([get(fileSetup,'String'),'.txt'],'w');
                disp('File created');
            end
            
            %get the reference scan optimization details
            ROT_REF = str2num(get(rotationTxtSetup,'String'));
            X_REF   = str2num(get(XTxtSetup,'String'));
            Y_REF   = str2num(get(YTxtSetup,'String'));
            
            %update the values for dead-reckoning
            X_last = X_REF;
            Y_last = Y_REF;
            
            %setup now completed
            close(setupFigure);
            
            %main window is visible again
            set(S.myFigure,'Visible','on');
            
            %Reference scan Optimization
            %rotate the reference scan to align with x-y axis
            ref_distF = [ref_dist;ref_dist];
            if (ROT_REF >= 0)
                ref_dist  = ref_distF(ROT_REF+1:360+ROT_REF);
            else
                ROT_REF = 360+ROT_REF;
                ref_dist  = ref_distF(ROT_REF+1:360+ROT_REF);
            end
    
            load('refDIST.mat');
            [Xref,Yref] = pol2cart((0:2*pi/360:2*pi*(1-1/360))',...
					double(c_ref_dist));
            xref2 = X_REF;
            yref2 = Y_REF;
            
            X_REF = 2400;
            Y_REF = 2140;
            
            load('goodScan.mat');
            
            ref_dist = c_ref_dist;
            %update polar and cartesian data at the main window
            set(S.polarRef,'YData',Yref); 
            set(S.polarRef,'XData',Xref); 
            set(S.cartRef,'YData',goodScan(:,2)); 
            set(S.cartRef,'XData',goodScan(:,1));
            

            %Get target point - user input
            [targetX,targetY] = ginput(1);
            
            drawnow
            
            %check if reference scan is valid
            if ( max(ref_dist) < 7000 )
                first_scan=false;
                if ( write2File)
                    fprintf(myFileId,'%d\n',ref_dist);
                end
                %total scan number
                scanNum = scanNum + 1;
                %tStart = tic ;      
            end
            
            
            %Tell Lidar to send two(2) packets.
            fwrite(fid,0);
            fwrite(fid,0);
            pause(1)
            
            %count actual scans that arrive per second
            scanPerSec=0;
            %set the main timer - toc everytime position is updated
            tStart = tic ;
            
            %update textboxes
            set(S.target_X_txt,'string',num2str(targetX));
            set(S.target_Y_txt,'string',num2str(targetY));
            set(S.target_Rot_txt,'string',num2str(targetRot));
                
        else
            %run as long as lidar is ON
            if (lidarON)
                %scans per second calculation -start time
                if ( scanPerSec==0 )
                    tLocal = tic;
                end
                %get current scan
                %fid.BytesAvailable
                a = uint8(fread(fid,720));
                cur_scan = typecast(a,'uint16');
                %************************************
                if ( IMU_ON )
                    heading = str2num(char(fread(fid,3))');
                    
                end
                
                %************************************
                %Tell Lidar to send one(1) more packet.
                
                fwrite(fid,0); 
                %write if selected data points to a file
                if (write2File)
                    fprintf(myFileId,'%d\n',cur_scan);
                end
                
                %----------------------------------------------------------
                % position/rotation estimation - HSM algorithm
                %----------------------------------------------------------
                
                %[Xref,Yref]: we can match the 2 scans and then apply this 
                %information, or we can translate one scan with this 
                %information and then run the algorithm. To be checked!
                %
                %check angle histogram step : it seems that it has a BIG
                %impact on successful estimation!
                [x,y,ROT] = scanMatch2D(double(cur_scan),double(ref_dist),...
                    'step',5,'setupscan',[X_REF,Y_REF],'filter','yes',...
                    'lastpos',[X_last Y_last Rot_last]);
                
                %dead-reckoning: radius is set to 350mm. To be checked!
                [X_last,Y_last] = findPosition2D(x,y,X_last,Y_last,350);
                Rot_last = ROT;
                
                if (X_last > 0)
                    X_last = - X_last;
                end
                %here we hold the algorithm results so far + time passed
                data = [ data ; [X_last Y_last Rot_last toc(tStart) ] ];
                
                
               set(S.imuRotationTxt,'String',num2str(data(numel(data(:,4)),4)));
                %----------------------------------------------------------
                % HSM algorithm - end
                %----------------------------------------------------------
                
                %----------------------------------------------------------
                % Fuzzy control logic - compute parameters
                %----------------------------------------------------------
                
                %ship's bow orientation.To be checked!
                %ROT should be between [0,360] degrees
                %***************
                ROT = 180 - ROT;
                
                
                %calculate current distance from target point
                DISTANCE = sqrt( (-X_last-targetX)^2 + (-Y_last-targetY)^2);
                
                %calculate the orientation of the line that connects
                %target point and current position
                minorTheta = atan2d( (targetY + Y_last) , (targetX + X_last) );

                %Main/Minor Orientation Calculation
                
                %ORIEN > 0: steer to the right
                %ORIEN < 0: steer to the left To be checked!
                MINOR_ORIEN = ROT - minorTheta;
                MAIN_ORIEN  = ROT - targetRot;
                
                if (MAIN_ORIEN > 180)
                    MAIN_ORIEN = MAIN_ORIEN - 360;
                end
                
                if (MINOR_ORIEN > 180)
                    MINOR_ORIEN = MINOR_ORIEN - 360;
                end
                
                %Speed Calculation: We use consecutive position estimations
                %SPEED is a positive value!!! To change the controller.
                %To be checked!
                speedStep = 1;
                index = numel(data(:,1));
                if (scanNum < 15)
						%Dx  = -X_last + data(index-speedStep,1);
						%Dy  = -Y_last + data(index-speedStep,2);
						%Dt  = data(index,4) - data(index-speedStep,4);
						%speedX = Dx / Dt ;
						%speedY = Dy / Dt ;
						%SPEED = sqrt( speedX^2 + speedY^2 ); 
                      SPEED = 1;                
                else                      
                    if ( scanNum < 100)
                        U = speedFilter(data(:,1),data(:,2),data(:,4),2);
                        SPEED = U(numel(U));
                    else
                        numE = numel(data(:,1));
                        numS = numE - 50;
                        U = speedFilter(data(numS:numE,1),data(numS:numE,2),data(numS:numE,4),2);
                        SPEED = U(numel(U));
                    end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
					%Backwards movement calculation - needs better implementation
                    %speedStep = 5;
                    %Dx  = -X_last + data(index-speedStep,1);
                    %Dy  = -Y_last + data(index-speedStep,2);
                    %Dt  = data(index,4) - data(index-speedStep,4);
                    
                    %angleS = atan2d( (Dy / Dt) , (Dx / Dt) );
                    
                    %angleSpeed = [ angleSpeed ; angleS ];
                    
                    %if ( numel(angleSpeed) < 100)
                    %    angleSpeedP = smooth(angleSpeed,0.5,'rloess');
                    %    angleS = angleSpeedP(numel(angleSpeedP));
                    %else
                    %    numE = numel(angleSpeed);
                    %    numS = numE - 50;
                     
                    %   angleSpeedP = smooth(angleSpeed(numS:numE),0.5,'rloess');
                    %   angleS = angleSpeedP(numel(angleSpeedP));
                    %end
                    %
                    %if ( angleS < 0 )
                    %    SPEED = - SPEED;
                    %end
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                end
                
                %minus(-) sign has to do with the fuzzy control rules
                CONTROL_IN = [DISTANCE, MINOR_ORIEN, SPEED, MAIN_ORIEN];
                
                %Check if inside acceptable control Limits
                if ( (DISTANCE < 250) && ( abs(MAIN_ORIEN) < 10 ) )
                    set(S.SuccessLed,'BackgroundColor','green');
                else
                    set(S.SuccessLed,'BackgroundColor',[0.5 0.5 0.5]);
                end
                
                %evaluate the controller
				CONTROL_OUT = evalfis( CONTROL_IN ,FIS);
                CONTROL_OUT = int16(CONTROL_OUT);
				
                if ( get(S.controlCheckBox,'Value') )
                    %AutoPilot is ON (hold the time when button is pressed
                    %for the first time
                    if ( control_StartTime == 0 )
                        control_StartTime = data(numel(data(:,4)),4);
                    end
                    %send via wifi the control variables to arduino client
                    fwrite(myServer, ['PR',num2str(CONTROL_OUT(1))]);
                    fwrite(myServer, ['BT',num2str(CONTROL_OUT(2))]);
                    fwrite(myServer, ['RT',num2str(CONTROL_OUT(3))]);
                    fwrite(myServer, ['RU',num2str(CONTROL_OUT(4))]);
                end
                
                
                
                CONTROL_IN = int16(CONTROL_IN);
                
                %store control results
                controlData = [controlData ; CONTROL_IN CONTROL_OUT];
                %----------------------------------------------------------
                % Fuzzy control logic - end
                %----------------------------------------------------------
                
                %The only drawback is that that target gets updated all the
                %time and can get any value even outside pool limits!
                
                 targetX=double(int16(str2num( get(S.target_X_txt,'string'))));
                 targetY=double(int16(str2num( get(S.target_Y_txt,'string'))));
                 targetRot=str2num( get(S.target_Rot_txt,'string'));
                
                %scans per second calculation - update gui textbox
                timePassed = toc(tLocal);
                if (timePassed > 1)
                    set(S.scanPerSecTxt,'String',num2str(scanPerSec));
                    scanPerSec = 0;  
                else
                    scanPerSec = scanPerSec + 1;
                end
                
                %----------------------------------------------------------
                % update GUI elements
                %----------------------------------------------------------
				
				%update live polar plot
                [Xref,Yref] = pol2cart((0:2*pi/360:2*pi*(1-1/360))',double(cur_scan));
                set(S.polarCur,'YData',Yref); 
                set(S.polarCur,'XData',Xref);
                %plot the current ship orientation arrow
				%the lidar unit is turned by 180deg in reference to bow
                [U,V] = pol2cart(deg2rad(ROT),3500 );
                delete(S.hl)
                S.hl = compass(S.ha,U,V);               
                delete(findall(S.hl,'type','line','linestyle',':'));
                set(S.hl,'linewidth',2) 
				%update ship position marker
                set(S.positionMarker,'YData',-Y_last ); 
                set(S.positionMarker,'XData',-X_last );
                %update the line connecting target with current position
				set(S.linePlot,'XData',[-X_last targetX]);
                set(S.linePlot,'YData',[-Y_last targetY]);
				%update the target marker
                set(S.targetMarker,'XData',targetX);
                set(S.targetMarker,'YData',targetY);
				%update various textboxes
                set(S.rotationTxt,'String',num2str(ROT));
                set(S.positionTxt_X,'String',num2str(-X_last));
                set(S.positionTxt_Y,'String',num2str(-Y_last));
                set(S.mainOrienTxt,'String',num2str(CONTROL_IN(4)));
                set(S.minorOrienTxt,'String',num2str(CONTROL_IN(2)));
                set(S.distanceTxt,'String',num2str(CONTROL_IN(1)));
                set(S.speedTxt,'String',num2str(CONTROL_IN(3)));
				              
                shipPoints = drawShip(-X_last,-Y_last,ROT);
                set(S.ship,'XData', shipPoints(1,:) );
                set(S.ship,'YData', shipPoints(2,:) );                
				%update ship current orientation arrow
                delete(S.hRot)
                S.hRot = quiver(-X_last,-Y_last,1000*cos(deg2rad(ROT)),1000*sin(deg2rad(ROT)),...
                            'Color','r','LineWidth',2);
                %update target orientation arrow
                delete(S.hTargetRot)
                S.hTargetRot = quiver(targetX,targetY,1000*cos(deg2rad(targetRot)),1000*sin(deg2rad(targetRot)),...
                            'Color','g','LineWidth',2);     
                %update minor orientation arrow     
                delete(S.hLineRot)
                S.hLineRot = quiver(targetX,targetY,1000*cosd(minorTheta),1000*sind(minorTheta),...
                            'Color','m','LineWidth',2);
                        
                if ( get(S.controlCheckBox,'Value') )
                    set(S.Propeller_txt,'String',num2str(CONTROL_OUT(1)));
                    set(S.Rudder_txt,'String',num2str(CONTROL_OUT(4)));
                    set(S.Bow_Txt,'String',num2str(CONTROL_OUT(2)));
                    set(S.Rear_Txt,'String',num2str(CONTROL_OUT(3)));
                end
                %----------------------------------------------------------
                % update GUI - end
                %---------------------------------------------------------- 
                %if (myServer.BytesAvailable)
                %    fread(myServer,myServer.BytesAvailable)
                %end
                
				%update total scan number
                scanNum = scanNum + 1; 
            end
        end
    end
    drawnow 
end

%save created variables with various information
save(['data_', datestr(now,'HH_MM')],'data','controlData','X_REF','Y_REF',...
	'ref_dist','targetX','targetY','targetRot','control_StartTime','angleSpeed')

%close created file
if (write2File)
    fclose(myFileId);
end


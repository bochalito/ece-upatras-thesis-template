function completeSetup( source,callbackdata) 
    global setupRun;
    setupRun=0;
end

function createFile( source,callbackdata ,fileSetup,fileLabelSetup)   
   if (get(source,'Value'))
       set(fileSetup,'Enable','on');
       set(fileLabelSetup,'Enable','on');
       set(fileSetup,'Visible','on')
       set(fileLabelSetup,'Visible','on');       
   else
       set(fileSetup,'Enable','off'); 
       set(fileLabelSetup,'Enable','off');
       set(fileSetup,'Visible','off')
       set(fileLabelSetup,'Visible','off');
   end 
end

function [ x_t ] = drawShip( XT, YT, ROT , varargin )
    
while ~isempty(varargin)
    switch upper(varargin{1})      
        case 'SIMPLE'
            x = [-572 500 572 500 -572 -572 -472 -622 -572  -572 ];
            y = [-70  -70  0   70   70   0     0     0    0  -70 ];
            varargin(1) = []; 
        case 'BOW1400'
            x = [-572 500 572 500 300 300  300 250 300 350 300 ...
			300 -300 -300 -300 -250 -300 -350 -300 -300  -572 -572 ...
			-472 -672  -572 -572 ];
            y = [-70  -70  0   70  70  20  120 85  120  85 120 70 70 20   ...
                120   85   120  85  120 70 70 0  0  0  0  -70 ];
            varargin(1) = []; 
       
        case 'INVERSE1'
            x = [-572 -400  -400  -400 -350  -400 -450 -400 -400 500 ...
			572 500 300 300   300 250 300  350 300 300 -572 -572 -472 ...
			-672  -572 -572 ];
            y = [-70  -70    80  -170  -85  -170 -85  -170  -70 -70  ...
			0   70  70  -80  180  85  180  85 180 70  70  0  0 0  0 -70 ];
         varargin(1) = []; 
         
        otherwise
            error(['Invalid option: ' varargin{1}])
    end
end

    trasl = @(dx,dy) [dx; dy];                                        
    rot   = @(theta)  [cos(theta) -sin(theta); sin(theta) cos(theta)];
    
    x_t = bsxfun(@plus,rot(deg2rad(ROT))*([x;y]), trasl(XT ,YT ) );    
end

function  exitBtn(source,callbackdata,fid,myServer)
    
    global programRun;
    
    %pause lidar
    fwrite(fid,1);
    
    if (fid.BytesAvailable)
        fread(fid,fid.BytesAvailable);
    end
    
    fclose(fid);
    clear fid
    
    fclose(myServer);
    clear myServer
    
    programRun = 0;
    disp('End of Program')
    
end


function lidarBtn(source,callbackdata,fid,btnString)
        
        disp('button pressed')
        global lidarON;
        val = get(source,'String');
        tf = strcmp(val,'Lidar OFF');
        
        if (tf)
            %Turn Lidar OFF
            set(source,'String','Lidar ON')
            btnString = 'Lidar ON';
            fwrite(fid,1)
            if (fid.BytesAvailable)
                fread(fid,fid.BytesAvailable);
            end
            lidarON = 0;
        else
            %Turn Lidar ON
            set(source,'String','wait..')
            
            btnString = 'Lidar OFF';
            
            if (fid.BytesAvailable)
                fread(fid,fid.BytesAvailable);
            end
            
            fwrite(fid,0)
            fwrite(fid,0)
            pause(2)
            set(source,'String','Lidar OFF')
            lidarON = 1;
        end
   
end

function  pickPointBtn( source,callbackdata,target_X_txt,target_Y_txt)
    
   [xP yP] = ginput(1);
    
   set(target_X_txt,'string',num2str(xP));
   set(target_Y_txt,'string',num2str(yP));
end

function  pickRotBtn( source,callbackdata,target_Rot)

   val = get(source,'String');
   tf = strcmp(val,'Edit');
   
   if ( tf )       
      set(target_Rot,'enable','on');
      set(source,'string','Done');
      
   else
       set(target_Rot,'enable','inactive');
       set(source,'string','Edit');
   end
   
end
    
function  rotationSlider( source,callbackdata,ref_dist,polarPlotSetup,rotationTxtSetup,cartRefSetup,XTxtSetup,YTxtSetup)
    

    x_ref = str2num (get(XTxtSetup,'String'));
    y_ref = str2num (get(YTxtSetup,'String'));
    
    ref_distF = [ref_dist;ref_dist];
    rotationR = str2num (get(rotationTxtSetup,'String'));
    
    if (rotationR >= 0)
       ref_dist  = ref_distF(rotationR+1:360+rotationR);
    else
       rotationR = 360+rotationR;
       ref_dist  = ref_distF(rotationR+1:360+rotationR);
    end
               
    [Xref,Yref] = pol2cart((0:2*pi/360:2*pi*(1-1/360))',double(ref_dist));
                
    set(polarPlotSetup,'YData',Yref); 
    set(polarPlotSetup,'XData',Xref);
     
    set(cartRefSetup,'YData',Yref+y_ref); 
    set(cartRefSetup,'XData',Xref+x_ref);  
end

function serverConnect( source,callbackdata,myServer)

    set(source,'String','wait..')
    
    if ( strcmp(myServer.Status,'closed') )
        fopen(myServer);
        pause(2)
        
        while ( myServer.BytesAvailable == 0 )
            fclose(myServer);
            pause(0.5)
            fopen(myServer);
            pause(2)
        end
       
       set(source,'String','Stop Server')
    else
       fclose(myServer);
       set(source,'String','Start Server')
    end
    
    
end

function setMotorBtn( source,callbackdata,myServer,Propeller_txt,Rudder_txt,Bow_Txt,Rear_Txt)
        %send via wifi the control variables to arduino client
        fwrite(myServer, ['PR',get(Propeller_txt,'String')]);
        fwrite(myServer, ['BT',get(Bow_Txt,'String')]);
        fwrite(myServer, ['RT',get(Rear_Txt,'String')]);
        fwrite(myServer, ['RU',get(Rudder_txt,'String')]);
end

function stopMotorBtn( source,callbackdata,myServer )
    fwrite(myServer, 'PR1500');
    fwrite(myServer, 'BT1500');
    fwrite(myServer, 'RT1500');
    fwrite(myServer, 'RU1500');
end




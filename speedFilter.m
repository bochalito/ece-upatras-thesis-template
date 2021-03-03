
function [ U ] = speedFilter(X,Y,time,RC)
    %Measurements (X,Y) Filtering using a simple first order lowpass filter
    X_f = X;
    Y_f = Y;
    
   
    for i=2:length(X)
        dt = time(i)-time(i-1);
        a = dt/(RC+dt);
        X_f(i) = a*X(i)+(1-a)*X_f(i-1);
        Y_f(i) = a*Y(i)+(1-a)*Y_f(i-1);
    end

    %Time Derivatives of X Y computation using sliding window
    Xdot = zeros(1,length(X_f));
    Ydot = zeros(1,length(Y_f));

    twin = 2;
    
    %
    for i=1:length(X_f)
        
        if time(i)-time(1) > twin
                init = i;
                break;
        end
    end
    
   
    for i=init+1:length(X_f)
        
        for j=i:-1:1
            if time(i) - time(j) > twin
                istart = j;
                break;
            end
        end
        
        X_window = X_f(istart:i);
        Y_window = Y_f(istart:i);
        Time_window = time(istart:i);
        
        %2nd Order Polynomial coefficients of X and Y in time window
        P_X=polyfit(Time_window,X_window,2);
        P_Y=polyfit(Time_window,Y_window,2);
        
        %Derivative of the polynomial coefficients
        XdotPoly=polyder(P_X);
        YdotPoly=polyder(P_Y);
        
        %Evaluate the polynomial
        Xdot(round((i+istart)/2)) = polyval(XdotPoly,Time_window(round((i-istart)/2)));
        Ydot(round((i+istart)/2)) = polyval(YdotPoly,Time_window(round((i-istart)/2)));
    end
    
    %Zero order hold
    for i=1:length(Xdot)
        if Xdot(i)~=0
            a=i;
%             ccc = length(Xdot) ; 
            break;
        end
    end
    
 % no value given to a from previous for!!!!!!!!!
    for i=a:length(Xdot)
        if Xdot(i) == 0
            Xdot(i) = Xdot(i-1);
        end

        if Ydot(i) == 0
            Ydot(i) = Ydot(i-1);
        end
    end
    
    
    %Velocity computation
    U= Xdot.*Xdot + Ydot.*Ydot ;
    U= U.^(1/2);
    
end
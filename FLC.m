%Define Fuzzy Logic Controller 

control_1 = newfis('Matlab_controller');

%Define inputs and outputs of Fuzzy Controller

%------------------------------------------------
%                     Distance
%------------------------------------------------
control_1 = addvar(control_1,'input','Distance',[0 4000]);
control_1 = addmf(control_1,'input',1,'Zero','gaussmf', [34.5 -811.910052910053 84 122.089947089947]);
control_1 = addmf(control_1,'input',1,'Small','gaussmf', [207 704]);
control_1 = addmf(control_1,'input',1,'Medium','gaussmf', [454 1862.32804232804]);
control_1 = addmf(control_1,'input',1,'Big','gauss2mf', [342 3561.40438643045 25.7 4353.96825396825]);

%------------------------------------------------
%                   MinorOrienDiff
%------------------------------------------------
control_1 = addvar(control_1,'input','MinorOrienDiff',[-180 180]);
control_1 = addmf(control_1,'input',2,'BigN','gaussmf',  [26 -95]);
control_1 = addmf(control_1,'input',2,'MedN','gaussmf',  [6 -30]);
control_1 = addmf(control_1,'input',2,'SmallN','gaussmf', [6 -15]);
control_1 = addmf(control_1,'input',2,'Zero','gaussmf', [5 0]);
control_1 = addmf(control_1,'input',2,'SmallP','gaussmf',[6 15.61]);
control_1 = addmf(control_1,'input',2,'MedP','gaussmf', [6 30]);
control_1 = addmf(control_1,'input',2,'BigP','gaussmf',  [26 95]);
control_1 = addmf(control_1,'input',2,'VBigN','gauss2mf', [122 -371.3 5.35 -171.3]);
control_1 = addmf(control_1,'input',2,'VBigP','gauss2mf', [6.98 170.8 122 376.7]);


%------------------------------------------------
%                        Speed
%------------------------------------------------
control_1 = addvar(control_1,'input','Speed',[0 200]);
control_1 = addmf(control_1,'input',3,'Zero','gauss2mf',[9 0]);
control_1 = addmf(control_1,'input',3,'Medium','gaussmf',[16 100]);
control_1 = addmf(control_1,'input',3,'High','gauss2mf',[16 147.292524377031 0.75 420.292524377031]);
control_1 = addmf(control_1,'input',3,'Small','gaussmf',[11 41]);
control_1 = addmf(control_1,'input',3,'NegSmall','gaussmf',[11 -39.075406283857]);

%------------------------------------------------
%                       MainOrienDiff
%------------------------------------------------
control_1 = addvar(control_1,'input','MainOrienDiff',[-180 180]);
control_1 = addmf(control_1,'input',4,'Negative','gauss2mf',[4.58 -199 4.436 -10.29]);
control_1 = addmf(control_1,'input',4,'Zero','gaussmf',[5 0]);
control_1 = addmf(control_1,'input',4,'Positive','gauss2mf',[6.3 12.88 10.6 183]);

%------------------------------------------------
%                       Propeler
%------------------------------------------------
control_1 = addvar(control_1,'output','Propeler',[1000 2000]);
control_1 = addmf(control_1,'output',1,'Reverse','gaussmf',[26.6 1178.93650793651]);
control_1 = addmf(control_1,'output',1,'Zero','gaussmf',[40 1502]);
control_1 = addmf(control_1,'output',1,'Forward','gaussmf',[28.09 1630]);
control_1 = addmf(control_1,'output',1,'FullReverse','gaussmf',[11.6 1091.08342361863]);
control_1 = addmf(control_1,'output',1,'Full','gaussmf',[20 1700]);

%------------------------------------------------
%                   Bow-thruster
%------------------------------------------------
control_1 = addvar(control_1,'output','Bow-Thruster',[1000 2000]);
control_1 = addmf(control_1,'output',2,'Right','gaussmf',[45 1370]);
control_1 = addmf(control_1,'output',2,'Zero','gaussmf',[40 1500]);
control_1 = addmf(control_1,'output',2,'Left','gaussmf',[40 1620]);

%------------------------------------------------
%                   Rear-thruster
%------------------------------------------------
control_1 = addvar(control_1,'output',' Rear-Thruster',[1000 2000]);
control_1 = addmf(control_1,'output',3,'Left','gaussmf',[45 1360]);
control_1 = addmf(control_1,'output',3,'Zero','gaussmf',[30 1500]);
control_1 = addmf(control_1,'output',3,'Right','gaussmf',[35 1620]);

%------------------------------------------------
%                       Rudder
%------------------------------------------------
control_1 = addvar(control_1,'output','  Rudder',[1000 2000]);
control_1 = addmf(control_1,'output',4,'Right','gauss2mf',[30.5 922.1 51.8 1277]);
control_1 = addmf(control_1,'output',4,'Med-Right','gaussmf',[40 1400]);
control_1 = addmf(control_1,'output',4,'Straight','gaussmf',[40 1500]);
control_1 = addmf(control_1,'output',4,'Med-Left','gaussmf',[40 1600]);
control_1 = addmf(control_1,'output',4,'Left','gauss2mf',[59.4 1740 30.5 2000]);

%------------------------------------------------
%                   RULES
%------------------------------------------------
Distance = [1,2,3,4];
MinorOrienDiff = [1,2,3,4,5,6,7,8,9];
Speed = [1,2,3,4,5];
MainOrienDiff = [1,2,3];
Rules = [3 1 0 0 5 3 1 3 1 1;3 2 0 0 5 2 2 3 1 1;3 6 0 0 5 2 2 4 1 1;
	     3 3 0 0 5 2 2 2 1 1;3 7 0 0 5 2 2 5 1 1;3 4 0 0 5 2 2 1 1 1;
		 3 5 0 0 5 1 3 1 1 1;2 1 3 0 2 3 1 3 1 1;2 2 3 0 2 2 2 3 1 1;
		 2 6 3 0 2 2 2 4 1 1;2 3 3 0 2 2 2 2 1 1;2 7 3 0 2 2 2 5 1 1;
         2 4 3 0 2 2 2 1 1 1;2 5 3 0 2 1 3 1 1 1;1 1 3 0 1 3 1 3 1 1;
		 1 1 2 0 1 3 1 3 1 1;1 1 1 0 3 3 1 3 1 1;1 2 3 0 1 3 1 3 1 1;
		 1 2 2 0 1 3 1 3 1 1;1 2 1 0 3 3 1 3 1 1;1 6 3 0 1 3 2 3 1 1;
		 1 6 2 0 1 3 2 3 1 1;1 6 1 0 3 3 2 4 1 1;1 3 3 0 1 2 2 2 1 1;
         1 3 2 0 1 2 2 2 1 1;1 3 1 0 3 2 2 2 1 1;1 7 3 0 1 1 2 1 1 1;
		 1 7 2 0 1 1 2 1 1 1;1 7 1 0 3 1 2 5 1 1;1 4 3 0 1 1 3 1 1 1;
		 1 4 2 0 1 1 2 1 1 1;1 4 1 0 3 1 2 1 1 1;1 5 3 0 1 1 3 1 1 1;
		 1 5 2 0 1 1 3 1 1 1;1 5 1 0 3 1 3 1 1 1;4 0 3 1 4 3 3 2 1 1;
         4 0 2 1 1 3 1 2 1 1;4 0 1 1 2 3 1 2 1 1;4 0 3 3 4 1 3 2 1 1;
		 4 0 2 3 1 1 3 2 1 1;3 8 0 0 2 3 1 3 1 1;3 9 0 0 2 1 3 1 1 1;
		 2 8 0 0 2 3 1 3 1 1;2 9 0 0 2 1 3 1 1 1;1 8 4 0 1 2 2 2 1 1;
		 1 9 4 0 1 2 2 2 1 1;4 0 1 3 2 1 3 2 1 1;4 0 3 2 1 2 2 2 1 1;
         4 0 2 2 1 2 2 2 1 1;4 0 1 2 2 2 2 2 1 1;2 1 2 0 2 3 1 3 1 1;
		 2 1 1 0 3 3 1 3 1 1;2 2 2 0 2 2 2 3 1 1;2 2 1 0 3 2 2 3 1 1;
		 2 6 2 0 2 2 2 4 1 1;2 6 1 0 3 2 2 4 1 1;2 3 2 0 2 2 2 2 1 1;
		 2 3 1 0 3 2 2 2 1 1;2 7 2 0 2 2 2 5 1 1;2 7 1 0 3 2 2 5 1 1;
         2 4 2 0 2 2 2 1 1 1;2 4 1 0 3 2 2 1 1 1;2 5 2 0 2 1 3 1 1 1;
		 2 5 1 0 3 1 3 1 1 1;2 1 4 0 3 3 1 3 1 1;2 2 4 0 3 3 1 3 1 1;
		 2 6 4 0 3 2 2 4 1 1;2 3 4 0 3 2 2 2 1 1;2 7 4 0 3 2 2 5 1 1;
		 2 4 4 0 3 1 3 1 1 1;2 5 4 0 3 1 3 1 1 1;4 0 4 3 2 1 3 2 1 1;
         4 0 4 1 2 3 1 2 1 1;4 0 4 2 2 2 2 2 1 1;4 0 5 3 3 1 3 2 1 1;
		 4 0 5 2 3 2 2 2 1 1;4 0 5 1 3 3 1 2 1 1;1 8 2 0 1 2 2 2 1 1;
		 1 8 3 0 4 2 2 2 1 1;1 8 1 0 1 2 2 2 1 1;1 8 5 0 2 2 2 2 1 1;
		 1 9 3 0 4 2 2 2 1 1;1 9 2 0 1 2 2 2 1 1;1 9 1 0 1 2 2 2 1 1;
		 1 9 5 0 2 2 2 2 1 1];

   control_1 = addrule(control_1,Rules);
   writefis(control_1,'FLC');



         









































































%% Setup our world
clc;
clf;
clear
myInterface = InterfaceClass();

%% Testing book get and return
myInterface.ReturnBook(1);  % Example on returning a book number to the shelf
myInterface.GetBook(1);     % Example on getting a book from the shelf
myInterface.EStop();        % Example of toggling EStop
myInterface.HansCute.StartRobot();  % Example of starting the robot 
myInterface.DirectQControl(3,1);    % Example of driving joint 3 in the positive direction
myInterface.DirectQControl(7,-1);   % Example of driving joint 3 in the negitive direction
myInterface.RMRC([0 0 1]);          % Example of moving in RMRC along the positive z axis
myInterface.RMRC([-1 -1 0]);        % Example of moving in RMRC along the negative x-y diagonal

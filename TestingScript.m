%% Sections for testing various components of the robot
%  Simply run different sections to test different things
%% Testing general appearance and movement of robot
%  Also checking that my calculated joint positions are the same as the
%  real ones
clf;
clear;
clc;

myHC = HansCute(transl(0.2,0.2,0.2));
EFTransform = myHC.model.fkine(myHC.qCurrent); 
temp = myHC.GetArmVerticies(myHC.qCurrent)
display(EFTransform);
myHC.model.teach();
pause();
EFTransform = myHC.model.fkine(myHC.model.getpos); 
temp = myHC.GetArmVerticies(myHC.model.getpos)
display(EFTransform);

%% Testing Estop
myHC = HansCute();
pause();
myHC.EStop();
pause();
myHC.StartRobot();
pause();
myHC.EStop();
pause();
myHC.StartRobot();

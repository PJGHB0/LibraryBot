%% Section for testing 
clf;
clear;
clc;

% myRobot = UR5(false);
% myRobot.model.teach();
% myRobot.model.base
% myRobot.model.

myHC = HansCute();
EFTransform = myHC.model.fkine(myHC.qCurrent); 
myHC.stopVariable
myHC.model.base()

testTransform = transl(1.3,2.4,3.1);
testTransform(2,4)
testBase = myHC.model.base();
testBase(3,4)
testBase(1,4)
myHC.qCurrent = [0.2 0.2 0.2 0.2 0.2 0.2];
temp = myHC.GetArmVerticies(myHC.qCurrent)

%% Skeleton code
%dh parameter
%GUI
%create a robot class for hans_cute

%%code main 

%q1=getpos(current arm)

%(thickness,placement) = read_book

%q2 for arm = book_chute
%q3 for arm = book_placement 

%grip_open = thickness_book+offset

%arm_action = (q1,q2)
%grip_close

%arm_action = (q2,q3)
%grip_open = thickness_book+offset






%function = arm_action (q_start,q_des)
    %trajectory calculating = trapezoidal(q_start,q_des)
    %collision_check
    
    %until trajectory is collision free
    
    %start looping + animate
        %animate step
        %emergency_check
        %collision_check
    
    %end loop
%end


%function = collision_check(qin)
    %fkine.(qin)
%end


%function = emergency_check(button)
    %if (button = true)
%end

%function = read_book(code)
    %book_thickness
    %book_placement
%end

%function = grip_action

%% Examples on class structure
% Examples below-----------------------
% function obj = untitled(inputArg1,inputArg2)
%     %UNTITLED Construct an instance of this class
%     %   Detailed explanation goes here
%     obj.Property1 = inputArg1 + inputArg2;
% end
% function outputArg = method1(obj,inputArg)
%     %METHOD1 Summary of this method goes here
%     %   Detailed explanation goes here
%     outputArg = obj.Property1 + inputArg;
% end
% -------------------------------------

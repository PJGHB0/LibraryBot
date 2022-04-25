%% Section for testing 
clf;
clear;
clc;

% myRobot = UR5(false);
% myRobot.model.teach();
% myRobot.model.base
% myRobot.model.

myHC = HansCute();
myHC.model.base %show default base
myHC.model.base = transl(0,0,1);
myHC.model.base %show updated base (should be 0 0 1)

scale = 0.4;
workspace = [-2 2 -2 2 0 4]; %default workspace, %[xmin xmax ymin ymax zmin zmax]  
qZeroes = [0,0,0,0,0,0]; 
myHC.model.plot(qZeroes,'workspace',workspace,'scale',scale);

EFTransform = myHC.model.fkine(qZeroes)


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

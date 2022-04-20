%% Robot class including the robot model, and all 
classdef HansCute < handle %HansCuteRobot class
    properties %all of the variables
        model; %Robot model 
        workspace = [-2 2 -2 2 -0.3 2]; %default workspace  
    end
    methods
        function self = HansCute() %Things to do on class creation
            self.GetHCRobot();
            self.PlotAndColourRobot();
        end
        function GetHCRobot (self) %Creates the robot model
            pause(0.001); %Idk why we use this, but it was in UR5 code...
            L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
            L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
            L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            self.model = SerialLink([L1 L2 L3 L4 L5 L6],'name',"HansCute");
        end
        function PlotAndColourRobot (self)
            %Currently does nothing (need to implement)
        end
        function SetBaseLocation (
    end
end


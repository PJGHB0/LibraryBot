%% Robot class including the robot model, and all 
classdef HansCute < handle %HansCuteRobot class
    properties %all of the variables
        model; %Robot model 
        gripperBool = false; %False for gripper open (not on)
        qCurrent = [0 0 0 0 0 0 0]; %ALWAYS plot using qCurrent. when plotting, change this value and sent it to the model
        workspace = [-2 2 -2 2 0 4];
        scale = 0.4;
        stopVariable = [false false] %First part is whether the machine is stopped. Second is true for collision, false for estop
    
        %DH Params
        DH_d = [0.12 0 0.1408 0 0 0 0.1296];
        DH_a = [0 0 0 0.0718 0.0718 0 0];
        DH_alpha = [-pi/2 pi/2 pi/2 pi/2 -pi/2 pi/2 0];
        DH_offset = [-pi/2 0 0 pi/2 0 pi/2 0];
        DH_qlim = deg2rad([-150,150;-105,105;-150,150;-105,105;-105,105;-105,105;-150,150]);
    end
    methods
        function self = HansCute() %Things to do on class creation
            self.GetHCRobot();
            self.PlotAndColourRobot();
        end
        function GetHCRobot (self) %Creates the robot model (self.model)
            pause(0.01); %Idk why we use this, but it was in UR5 code...
            %CORRECT DH PARAMETERS NEED TO BE ADDED
            for n = 1:7
                L(n) = Link('d',self.DH_d(n),'a',self.DH_a(n),'alpha',self.DH_alpha(n),'offset',self.DH_offset(n),'qlim',self.DH_qlim(1,:));
            end
            self.model = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7)],'name',"HansCute");
        end
        function PlotAndColourRobot (self) %Part of class initialisation
            %Currently does not Colour robot (need to implement)
            self.model.base = transl(0,0,1);
            self.model.plot(self.qCurrent,'workspace',self.workspace,'scale',self.scale); %Now the robot is plotted, and we do NOT have to plot it ever again (we simply animate it)
            view(45,25); %Set an appropriate view angle
        end
        function [transformMatrix] = GetArmVerticies(self,qInput) %Used for collision detection
            transformMatrix = zeros(4,4,7);
            transformMatrix(:,:,1) = self.model.base();
            L = self.model.links;
            for i = 1:7
                transformMatrix(:,:,i+1) = transformMatrix(:,:,i) * trotz(qInput(i) + self.DH_offset(i)) * transl(0,0,self.DH_d(i)) * transl(self.DH_a(i),0,0) * trotx(self.DH_alpha(i));
            end
        end
        function CheckInterception (self)
            interception = false;
            %Need to implement function - find out if there is already
            %another function to do this.  
            %We need to collision detect with both all obsticles, and the
            %other robot (each link)
            if  interception == true %What happens upon interception
                self.stopVariable = [true true];
            end
        end
        function linearRMRC (self, A, B)
            %Move from transform A to transform B, in a straight line. Must
            %check for collisions along the way, and plot the transform
        end
        
    end
end


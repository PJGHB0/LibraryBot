%% Robot class including the robot model, and all 
classdef HansCute < handle %HansCuteRobot class
    properties %all of the variables
        model; %Robot model 
        gripperBool = false; %False for gripper open (not on)
        qCurrent = [0 0 0 0 0 0 0]; %ALWAYS plot using qCurrent. when plotting, change this value and sent it to the model. Note, that the current configuration is actually self.getpos
        workspace = [-0.6 0.6 -0.6 0.6 -0.6 1];
        scale = 0.5;
        stopVariable = [false false] %First part is whether the machine is stopped. Second is true for collision, false for estop
        running = false
        baseLocation = transl(0,0,0);
        
        %DH Params
        DH_d = [0.12 0 0.1408 0 0 0 0.1296];
        DH_a = [0 0 0 0.0718 0.0718 0 0];
        DH_alpha = [-pi/2 pi/2 pi/2 pi/2 -pi/2 pi/2 0];
        DH_offset = [-pi/2 0 0 pi/2 0 pi/2 0];
        DH_qlim = deg2rad([-150,150;-105,105;-150,150;-105,105;-105,105;-105,105;-150,150]);
    end
    methods
        function self = HansCute(baseLocationIn) %Things to do on class creation
            if nargin == 0
                baseLocationIn = transl(0,0,0);
            end
            self.baseLocation = baseLocationIn;
            self.GetHCRobot();
            self.PlotAndColourRobot();
            self.StartRobot();
        end
        function GetHCRobot (self) %Creates the robot model (self.model)
            pause(0.01); %Idk why we use this, but it was in UR5 code...
            for n = 1:7
                L(n) = Link('d',self.DH_d(n),'a',self.DH_a(n),'alpha',self.DH_alpha(n),'offset',self.DH_offset(n),'qlim',self.DH_qlim(n,:));
            end
            self.model = SerialLink([L(1) L(2) L(3) L(4) L(5) L(6) L(7)],'name',"HansCute");
        end
        function PlotAndColourRobot (self) %Part of class initialisation
            %Currently does not Colour robot (need to implement)
            self.model.base = self.baseLocation;
            self.model.plot(self.qCurrent,'workspace',self.workspace,'scale',self.scale); %Now the robot is plotted, and we do NOT have to plot it ever again (we simply animate it)
            view(45,25); %Set an appropriate view angle
        end
        function [transformMatrix] = GetArmVerticies(self,qInput) %Used for collision detection
            transformMatrix(:,:,1) = self.model.base();
            L = self.model.links;
            for i = 1:7
                transformMatrix(:,:,i+1) = transformMatrix(:,:,i) * trotz(qInput(i) + self.DH_offset(i)) * transl(0,0,self.DH_d(i)) * transl(self.DH_a(i),0,0) * trotx(self.DH_alpha(i));
            end
        end
        function [collisionCheck] = CheckInterception (self,tr,vertex,faces,faceNormals)
            %tr - transform matrix of all 8 robot links (4,4,8)
            %[vertex,faces,faceNormals] - only for 1 obsticle - therfore
            %function must be called as many times as there are obsticles,
            %for each qMatrix
            for i = 1 : size(tr,3)-1
                for faceIndex = 1:size(faces,1)
                    vertOnPlane = vertex(faces(faceIndex,1)',:);
                    [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)'); 
                    if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                        collisionCheck = true;
                        return
                    end
                end
            end
                collisionCheck = false;
        end
        function LinearRMRC (self, A, B)
            %Move from transform A to transform B, in a straight line. Must
            %check for collisions along the way, and plot the transform
            
            %Get x, y, and z locations from both matricies
            A_x = A(1,4);
            A_y = A(2,4);
            A_z = A(3,4);
            B_x = B(1,4);
            B_y = B(2,4);
            B_z = B(3,4);
        end
        function EStop(self) % calling this activates and deactivates estop
            if  self.stopVariable(1) == false %If we are not in an emergency stop mode
                self.stopVariable = [true false];
                self.running = false;
                disp("E-Stop activated");
            else %If stop is on, turn off
                self.stopVariable(1) = false;  
                disp("E-Stop de-activated");
                %Note, we do NOT turn on running variable here. StartRobot
                %must be called instead
            end
        end
        function StartRobot(self)
            if self.stopVariable(1) == false %%If we are not in an emergency stop mode
                self.running = true;
                disp("Robot now running");
            else
                if self.stopVariable(2) == false
                    disp("Cannot activate robot: E-stop active");
                else
                    disp("Cannot activate robot: collision detected");
                end
            end
        end
        
    end
end


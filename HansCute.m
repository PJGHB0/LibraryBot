%% Robot class including the robot model, and all 
classdef HansCute < handle %HansCuteRobot class
    properties %all of the variables
        model; %Robot model 
        gripperBool = false; %False for gripper open (not on)
        qCurrent = [0 0 0 0 0 0 0]; %ALWAYS plot using qCurrent. when plotting, change this value and sent it to the model. Note, that the current configuration is actually self.getpos
        workspace = [-1 1 -1 1 0 1];
        scale = 0.5;
        stopVariable = [false false] %First part is whether the machine is stopped. Second is true for collision, false for estop
        running = false
        baseLocation = transl(0,0,0);
        speed = 0.1; % Speed for RMRC
        %DH Params
        DH_d = [0.12 0 0.1408 0 0 0 0.1296];
        DH_a = [0 0 0 0.0718 0.0718 0 0];
        DH_alpha = [-pi/2 pi/2 pi/2 pi/2 -pi/2 pi/2 0];
        DH_offset = [-pi/2 0 0 pi/2 0 pi/2 0];
        DH_qlim = deg2rad([-360,360;-135,135;-150,150;-150,150;-105,105;-105,105;-150,150]);
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
            view(280,20); %Set an appropriate view angle
            hold on;
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
        function [qMatrix] = LinearRMRC (self, qInitial, B)
            % Move from qInitial to transform B, in a straight line.
            % Returns a qMatrix
            
            % We determine an arm velocity. Then, depending on the distance
            % to the desired location, we determine a total time, and
            % number of steps. 
            stepsPerSecond = 20; % Arbitrary - play around with this and see how it changes
            epsilon = 0.1;      % Also arbitrary
            W = diag([1 1 1 0.1 0.1 0.1]);          % Weighting vector places more emphasis on the linear motion, than the rotational
            
            A = self.model.fkine(qInitial);
            changeInPose = B - A;
            distanceTotal = (changeInPose(1,4)^2 + changeInPose(2,4)^2 + changeInPose(3,4)^2)^0.5;
            timeTotal = distanceTotal / self.speed;
            stepsTotal = stepsPerSecond*timeTotal;  % Arbitrarily set 20 steps per second
            deltaT = timeTotal/stepsTotal;          % Time for each step
            deltaAngle = 2*pi/stepsTotal;           % Discrete angle change
            
            % Now we setup the arrays
            % Since we are moving the book straight into the shelf, we do
            % not worry about angle RMRC control
            manipulability = zeros(fix(stepsTotal),1);   % Measure of manipulability at each step     fix() cuts off the decimals, making the number an integer
            qMatrix = zeros(fix(stepsTotal),7);          % qMatrix for arm motion
            qDotMatrix = zeros(fix(stepsTotal),7);       % derivitive of above matrix
%             thetaMatrix = zeros(3,stepsTotal);      % roll-pitch-yaw matrix
            xyzMatrix = zeros(3,fix(stepsTotal));        % x-y-z motion Matrix
            positionError = zeros(3,fix(stepsTotal));    % Linear Error due to singularity avoision
%             angleError = zeros(3,stepsTotal);       % Angle Error due to singularity avoision
            % Here we setup the cartesian trajectory
            trapezoidalScalar = lspb(0,1,stepsTotal);
            for i=1:stepsTotal
                xyzMatrix(1,i) = (1-trapezoidalScalar(i))*A(1,4) + trapezoidalScalar(i)*B(1,4);
                xyzMatrix(2,i) = (1-trapezoidalScalar(i))*A(2,4) + trapezoidalScalar(i)*B(2,4);
                xyzMatrix(3,i) = (1-trapezoidalScalar(i))*A(3,4) + trapezoidalScalar(i)*B(3,4);
            end
            % Here we track the trajectory with RMRC.
            T = A;                                          % Initial EF transform
            qMatrix(1,:) = qInitial;                        % Set initial q values as row 1 of qMatrix
            for i=1:stepsTotal-1
                T = self.model.fkine(qMatrix(i,:));         % Get current EF transform
                deltaX = xyzMatrix(:,i+1) - T(1:3,4);       % Difference between NEXT desired cartesian point, and our current EF transform (3x1 array)
                J = self.model.jacob0(qMatrix(i,:));        % Get current Jacobian
                manipulability(i) = sqrt(det(J*J'));        % Get maanipulability
                linearVelocity = deltaX/deltaT;             % Get linear velocity to next point
                angularVelocity = 0*linearVelocity;         % Placeholder for the angular velocity (we do not perform RMRC on angle
                xDot = W*[linearVelocity;angularVelocity];            % We do not worry about the angle change
                if manipulability(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - manipulability(i)/epsilon)*0.001;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda*eye(7))*J';        % DLS inverse
                qDotMatrix(i,:) = invJ * xDot;
                % We do not check if joint limits are maintained
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qDotMatrix(i,:);
                positionError(:,i) = xyzMatrix(:,i+1) - T(1:3,4);
            end
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
        function CollisionStop(self)
            disp("Collision detected!");
            self.running = false;
            self.stopVariable(2) = true;
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
        function TriggerGripper(self,inputBool)
            if  inputBool %If gripper is open
                self.gripperBool = true;
                disp("Gripper closed");
            else
                self.gripperBool = false;
                disp("Gripper released");
            end
        end
        
    end
end


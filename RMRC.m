function RMRC()
close all;
clear all;
clc;

%% 1.1) Set parameters for the simulation

L1 = Link('d',(54+66)/1000,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);
L2 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-105),deg2rad(105)]);
L3 = Link('d',140.80/1000,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-150),deg2rad(150)]);
L4 = Link('d',0,'a',71.8/1000,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L5 = Link('d',0,'a',71.8/1000,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-105),deg2rad(105)]);
L6 = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L7 = Link('d',129.6/1000,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-150),deg2rad(150)]);

LbrBot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',"LibraryBot"); % 
robot=LbrBot;
robot.teach();

links=7;

t = 5;             % Total time
steps = 100;         % No. of steps
% deltaT = t/steps;   % Discrete time step
% delta = 2*pi/steps; % Small angle change
% epsilon = 0.1;       % Threshold value for manipulability/Damped Least Squares
% W = diag([...]);    % Weighting matrix for the velocity vector



 
%% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qM_X = zeros(steps,links);       % Array for joint angles
qM_Y = zeros(steps,links);
qM_Z = zeros(steps,links);

qdot = zeros(steps,links);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles

% Array for x-y-z trajectory
x = zeros(3,steps);
y = zeros(3,steps);
z = zeros(3,steps);             

 %% 1.3) Set up trajectory, initial pose
%   A = [Ax Ay Az]   %starting cartesian
%   B = [Bx By Bz]   %destination cartesian
Ax=-0.15
Ay=-0.2
Az=-0.1
rpy_A = [3.1416   -0.0000   -1.5708]

Bx=0.15
By=0
Bz=0.3
rpy_B = [-1.5881    0.7853    1.5830]

s = lspb(0,1,steps);                % Trapezoidal trajectory scalar

for i=1:steps
    z(1,i) = Ax; % Points in x
    z(2,i) = Ay; % Points in y
    z(3,i) = (1-s(i))*Az + s(i)*Bz; % Points in z
   
end

for i=1:steps
    y(1,i) = Ax; % Points in y
    y(2,i) = (1-s(i))*Ay + s(i)*By; % Points in y
    y(3,i) = Bz; % Points in z
   
end

for i=1:steps
    x(1,i) = (1-s(i))*Ax + s(i)*Bx; % Points in x
    x(2,i) = By; % Points in y
    x(3,i) = Bz; % Points in z
   
end
 hold on
 figure(1)
plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
plot3(y(1,:),y(2,:),y(3,:),'k.','LineWidth',1)
plot3(z(1,:),z(2,:),z(3,:),'k.','LineWidth',1)

for i=1:steps
theta(1,i) = pi/2;                 % Roll angle 
theta(2,i) = 0;            % Pitch angle
theta(3,i) = 0;                 % Yaw angle
end

for i=1:steps
theta2(1,i) = 0;                 % Roll angle 
theta2(2,i) = 0;            % Pitch angle
theta2(3,i) = 0;                 % Yaw angle
end


rot_mtx=rpy2r(theta(:,1)');
%%
qM_Z = myRMRC(robot,z,rot_mtx,theta,steps);
qM_Y = myRMRC(robot,y,rot_mtx,theta,steps);
qM_X = myRMRC(robot,x,rot_mtx,theta,steps);


 %% 1.5) Plot the results

% figure(1)
% plot3(z(1,:),z(2,:),z(3,:),'k.','LineWidth',1)
robot.plot(qM_Z,'trail','r-')

% plot3(y(1,:),y(2,:),y(3,:),'k.','LineWidth',1)
robot.plot(qM_Y,'trail','r-')

% plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1)
robot.plot(qM_X,'trail','r-')

%% 

end

%% 1.4) Track the trajectory with RMRC
 function [qMatrix] = myRMRC (robot,w,rot_mtx,theta,steps) 
t = 5;
deltaT = t/steps;   % Discrete time step

epsilon = 0.1;       % Threshold value for manipulability/Damped Least Squares
 
 qMatrix = zeros(steps,7); 
 qdot= zeros(steps,7);

T = [rot_mtx [w(:,1)];zeros(1,3) 1];             % Create transformation of first point and angle

qMatrix(1,:) = robot.ikcon(T);  % Solve joint angles to achieve first waypoint

for i = 1:steps-1
    T = robot.fkine(qMatrix(i,:))                 % Get forward transformation at current joint state
    deltaX = w(:,i)-T(1:3,4);            % Get position error from next waypoint
    Rd = rpy2r(theta(:,i+1)');      % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                % Current end-effector rotation matrix
    
    Rdot = (1/deltaT)*(Rd - Ra);       % Calculate rotation matrix error (see RMRC lectures)
        S = Rdot*Ra'                      % Skew symmetric! S(\omega)
        linear_velocity = (1/deltaT)*deltaX;
        angular_velocity = [S(3,2);S(1,3);S(2,1)];  % Check the structure of Skew Symmetric matrix! Extract the angular velocities. (see RMRC lectures)
        
        deltaR = rpy2r(theta(:,i)') - Ra        % Calculate rotation matrix error
        deltaTheta = tr2rpy(deltaR);% Convert rotation matrix to RPY angles
        xdot = [(1/deltaT)*(w(:,i+1) - w(:,i)); angular_velocity];              % Calculate end-effector velocity to reach next waypoint.(Try using a weighting matrix to (de)emphasize certain dimensions)
        K = robot.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
%         invJ = pinv(K);
%         N = null (J);
        
        m(i,:)= sqrt(det(K*K'));  
    if m(i,:)<epsilon  % If manipulability is less than given threshold
        lambda = (1-((m(i,:)/epsilon)^2))*0.001 % Damping coefficient (try scaling it)
        invJ = inv(K'*K + lambda*eye(7))*K'% Apply Damped Least Squares pseudoinverse
    else
        invJ = pinv(K); % Don't use DLS
    end

    qdot(i,:) = invJ*xdot % Solve the RMRC equation (you may need to transpose thevector)
%     for  ... % Loop through joints 1 to 6
%         if ... % If next joint angle is lower than joint limit...
%             ... % Stop the motor
%         else  ... %if If next joint angle is greater than joint limit ...
%             ... % Stop the motor
%         end
%     end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:) % Update next joint state based on joint velocities
%     m(i) = ...  % Record manipulability
%     positionError(:,i) = deltaX;  % For plotting
%     angleError(:,i) = deltaTheta; % For plotting
end

end
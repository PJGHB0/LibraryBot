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
clf;
clear;
clc;
myHC = HansCute();
pause();
myHC.EStop();
pause();
myHC.StartRobot();
pause();
myHC.EStop();
pause();
myHC.StartRobot();

%% Testing prism construction
clf;
clear;
clc;
myprism = RectangularPrism();
myprism.ConstructWithCorners([0 0 0],[1 1 1]);
a=myprism.face
b=myprism.faceNormals
c=myprism.vertex

%% Testing collision detection
clf;
clear;
clc;
myHC = HansCute();
myObsticle = RectangularPrism();
myObsticle.ConstructWithCorners([0.1 0.1 1],[-0.1 -0.1 0.3]);
v = myObsticle.vertex;
f = myObsticle.face;
fN = myObsticle.faceNormals;

myObsticle.PlotEdges;
myHC.model.teach();
qNoCollision = [0 pi/2 0 0 0 0 0];
qCollision = [0 0 0 0 0 0 0]
EFCheck = myHC.model.fkine(qCollision)
tNoCollision = myHC.GetArmVerticies(qNoCollision);
tCollision = myHC.GetArmVerticies(qCollision);

collisionBool_NoCollision = myHC.CheckInterception(tNoCollision,v,f,fN)
collisionBool_Collision = myHC.CheckInterception(tCollision,v,f,fN)


%% 
myRobot = HansCute();
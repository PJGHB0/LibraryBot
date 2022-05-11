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

%% And plotting an object
clf;
clear;
clc;
myHC = HansCute;
myBrick = RectangularPrism();
myBrick.ConstructWithCorners([0.1 0.1 0.1],[0 0 0]);
myBrick.PlotEdges();
hold on;
myBrick.PlotModel(transl(0.1,0.2,0)*trotz(pi/4),'Brick.ply');
%% Function for animating robot and object along a qMatrix (must already be checked for interception)
%Function requires qMatrix, and name of book file
myHC = HansCute();
qMatrix = jtraj([0 0 0 0 0 0 0],[ 1.1 1.2 1.3 1.4 1.5 1.6 1.7],50); %An example, placeholder qMatrix
qMatrixSize = size(qMatrix,1)
myBook = Book(myHC.model.fkine(qMatrix(1,:)),'book_1.ply');
for i= 1:qMatrixSize
    myHC.model.animate(qMatrix(i,:));
    EFTransform = myHC.model.fkine(qMatrix(i,:));
    myBook.Animate(EFTransform);
end

%% Collision detection for an entire qMatrix (and all objects)
clf;
clear;
clc;

myHC = HansCute();
collisionDetected = false; % Bool for is a check occurs
CossisionDetectedIndex = 0; % Index for qMatrix row where collision occurs

% Input parameters below
qMatrix = jtraj([pi/2 pi/2 0 0 0 0 0],[ 0 pi/2 0 0 0 0 0],50); %An example, placeholder qMatrix
qMatrixSize = size(qMatrix,1)

% Make first obsticle
myObsticle1 = RectangularPrism();
myObsticle1.ConstructWithCorners([0.1 0.1 1],[-0.1 -0.1 0.3]);
v1 = myObsticle1.vertex;
f1 = myObsticle1.face;
fN1 = myObsticle1.faceNormals;
% Make second obsticle
myObsticle2 = RectangularPrism();
myObsticle2.ConstructWithCorners([0.1 0.1 1],[-0.1 -0.1 0.3]);
v2 = myObsticle2.vertex;
f2 = myObsticle2.face;
fN2= myObsticle2.faceNormals;
% Make third obsticle
myObsticle3 = RectangularPrism();
myObsticle3.ConstructWithCorners([0.1 0.1 1],[-0.1 -0.1 0.3]);
v3 = myObsticle3.vertex;
f3 = myObsticle3.face;
fN3= myObsticle3.faceNormals;
% Make fourth obsticle
myObsticle4 = RectangularPrism();
myObsticle4.ConstructWithCorners([0.1 0.1 1],[-0.1 -0.1 0.3]);
v4 = myObsticle4.vertex;
f4 = myObsticle4.face;
fN4= myObsticle4.faceNormals;

for i = 1:qMatrixSize
    tr = myHC.GetArmVerticies(qMatrix(i,:));
    ob1 = myHC.CheckInterception(tr,v1,f1,fN1);
    ob2 = myHC.CheckInterception(tr,v2,f2,fN2);
    ob3 = myHC.CheckInterception(tr,v3,f3,fN3);
    ob4 = myHC.CheckInterception(tr,v4,f4,fN4);
    if ob1 || ob2 || ob3 || ob4 %%If interception at ANY object
        collisionDetected = true;
        break
    end
    collisionDetected = false;
end
display(collisionDetected);
%% 
myRobot = HansCute();
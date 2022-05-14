%% Here we initialise our environment and objects
% Add in our robot model
clc;
clf;
clear;
myHC = HansCute();
% axis off;
% Add in the wall and floor textures (no roof)
surf([-1,-1;1,1],[-1,1;-1,1],[0.001,0.001;0.001,0.001],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([1,1;1,1],[1,-1;1,-1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([1,-1;1,-1],[1,1;1,1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% Add in our shelves and table. Construct the mesh AND plot the model
shelf_1 = RectangularPrism();
shelf_2 = RectangularPrism();
shelf_3 = RectangularPrism();
table_0 = RectangularPrism();
shelf_1.PlotModel(transl(0, 0.4, 0.25),'bookshelf_2.PLY');
shelf_2.PlotModel(transl(0,-0.4,0.25)*trotz(pi),'bookshelf_2.PLY');
shelf_3.PlotModel(transl(0.4,0,0.25)*trotz(-pi/2),'bookshelf_2.PLY');
table_0.PlotModel(transl(-0.4,0,0.1)*trotz(-pi/2),'table.PLY');
shelf_1.ConstructWithCorners([0.2,0.3,0],[-0.2,0.55,0.5]);
shelf_2.ConstructWithCorners([0.2,-0.3,0],[-0.2,-0.55,0.5]);
shelf_3.ConstructWithCorners([0.55,-0.185,0],[0.3,0.185,0.5]);
table_0.ConstructWithCorners([-0.5,0.25,0],[-0.275,-0.25,0.18]);
shelf_1.PlotEdges();
shelf_2.PlotEdges();
shelf_3.PlotEdges();
table_0.PlotEdges();
% Add in our books
book_1 = Book(transl(-0.3627,-0.05,0.2476)*trotz(pi/2),'book_2.PLY',false);
book_2 = Book(transl(-0.3627,0,0.2476)*trotz(pi/2),'book_1.PLY',false);
book_3 = Book(transl(-0.3627,0.05,0.2476)*trotz(pi/2),'book_2.PLY',false);
% Add in our barrier (as a rectangular prism)
barrier_1 = RectangularPrism();
barrier_2 = RectangularPrism();
barrier_3 = RectangularPrism();
barrier_4 = RectangularPrism();
barrier_1.PlotModel(transl(-0.6,0,0.160)*trotz(-pi/2),'barrier.PLY');
barrier_2.PlotModel(transl(0.6,0,0.160)*trotz(-pi/2),'barrier.PLY');
barrier_3.PlotModel(transl(0,-0.9,0.160),'barrier.PLY');
barrier_4.PlotModel(transl(0,0.9,0.160),'barrier.PLY');
% Add in Estop button and camera
button_1 = Book(transl(-0.8,1,0.5)*trotz(-pi/2),'button.ply',false);
% button_2 = Book(transl(1,-0.8,0.5),'button.ply');
camera = Book(transl(1,0,0.8)*trotz(-pi),'camera.PLY',false);
%% Now we setup the animation parameters
% We list the begining 'shelf poses' of the robot
qInitial = [0 0 0 0 0 0 0];
qTable_0_q1 = deg2rad([-90 0 0 0 0 0 0]);
qShelf_1_q1 = deg2rad([-180 0 0 0 0 0 0]);
qShelf_2_q1 = deg2rad([0 0 0 0 0 0 0]);
qShelf_3_q1 = deg2rad([90 0 0 0 0 0 0]);
% We find a good q set for upper, mid and lower shelf (can apply to any
% shelf by changing q1 by 90 degrees)
qShelf_Upper = deg2rad([0 30 0 0 0 -60 0]); % Difficult to reach, near singularity
qShelf_Mid = deg2rad([0 -30 0 -90 0 -30 0]);
qShelf_Lower = deg2rad([0 35 0 -140 0 85 0]);
% We can choose which shelf (or table), and which level, by summing the q
% matricies
%% Here we run our main loop
while myHC.running == true             % Only when robot is 'operating'
    % Once robot is started, it puts away all of the books
    
    % -----Put away book 1 first-----
    % Go to table
    qMatrix_1 = jtraj(qInitial, qTable_0_q1 + qShelf_Mid, 50);
    for i = 1:size(qMatrix_1,1)
        myHC.model.animate(qMatrix_1(i,:));
        myHC.qCurrent = qMatrix_1(i,:);
        armVerticies = myHC.GetArmVerticies(qMatrix_1(i,:));
        collisionFlag = myHC.CheckInterception(armVerticies,shelf_1.vertex,shelf_1.face,shelf_1.faceNormals) || myHC.CheckInterception(armVerticies,shelf_2.vertex,shelf_2.face,shelf_2.faceNormals) || myHC.CheckInterception(armVerticies,shelf_3.vertex,shelf_3.face,shelf_3.faceNormals) || myHC.CheckInterception(armVerticies,table_0.vertex,table_0.face,table_0.faceNormals);
        if collisionFlag
            disp("Collision detected!");
            myHC.EStop();
            break
        end
    end
    % Grab book one
    qMatrix_2 = myHC.LinearRMRC(myHC.qCurrent,book_1.currentTransform*transl(0,-0.03,0));
    for i = 1:size(qMatrix_2,1)
        myHC.model.animate(qMatrix_2(i,:));
        myHC.qCurrent = qMatrix_2(i,:);
        armVerticies = myHC.GetArmVerticies(qMatrix_2(i,:));
        collisionFlag = myHC.CheckInterception(armVerticies,shelf_1.vertex,shelf_1.face,shelf_1.faceNormals) || myHC.CheckInterception(armVerticies,shelf_2.vertex,shelf_2.face,shelf_2.faceNormals) || myHC.CheckInterception(armVerticies,shelf_3.vertex,shelf_3.face,shelf_3.faceNormals) || myHC.CheckInterception(armVerticies,table_0.vertex,table_0.face,table_0.faceNormals);
        if collisionFlag
            disp("Collision detected!");
            myHC.EStop();
            break
        end
    end
    myHC.gripperBool = true; % Grab the book
    % Move back with book 1
    for i = size(qMatrix_2,1):-1:1
        myHC.model.animate(qMatrix_2(i,:));
        myHC.qCurrent = qMatrix_2(i,:);
        EFTransform = myHC.model.fkine(qMatrix_2(i,:));
        book_1.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
        armVerticies = myHC.GetArmVerticies(qMatrix_2(i,:));
        collisionFlag = myHC.CheckInterception(armVerticies,shelf_1.vertex,shelf_1.face,shelf_1.faceNormals) || myHC.CheckInterception(armVerticies,shelf_2.vertex,shelf_2.face,shelf_2.faceNormals) || myHC.CheckInterception(armVerticies,shelf_3.vertex,shelf_3.face,shelf_3.faceNormals) || myHC.CheckInterception(armVerticies,table_0.vertex,table_0.face,table_0.faceNormals);
        if collisionFlag
            disp("Collision detected!");
            myHC.EStop();
            break
        end
    end
    % Move to in front of shelf 1 (at the bottom)
    qMatrix_3 = jtraj(myHC.qCurrent, qShelf_1_q1 + qShelf_Lower, 50);
    for i = 1:size(qMatrix_3,1)
        myHC.model.animate(qMatrix_3(i,:));
        myHC.qCurrent = qMatrix_3(i,:);
        EFTransform = myHC.model.fkine(qMatrix_3(i,:));
        book_1.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
        armVerticies = myHC.GetArmVerticies(qMatrix_3(i,:));
        collisionFlag = myHC.CheckInterception(armVerticies,shelf_1.vertex,shelf_1.face,shelf_1.faceNormals) || myHC.CheckInterception(armVerticies,shelf_2.vertex,shelf_2.face,shelf_2.faceNormals) || myHC.CheckInterception(armVerticies,shelf_3.vertex,shelf_3.face,shelf_3.faceNormals) || myHC.CheckInterception(armVerticies,table_0.vertex,table_0.face,table_0.faceNormals);
        if collisionFlag
            disp("Collision detected!");
            myHC.EStop();
            break
        end
    end
    % Place in middle of shelf
    qMatrix_4 = myHC.LinearRMRC(myHC.qCurrent,book_1.currentTransform*transl(0,0.07-0.03,0));
    for i = 1:size(qMatrix_4,1)
        myHC.model.animate(qMatrix_4(i,:));
        myHC.qCurrent = qMatrix_4(i,:);
        EFTransform = myHC.model.fkine(qMatrix_4(i,:));
        book_1.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
        armVerticies = myHC.GetArmVerticies(qMatrix_4(i,:));
        collisionFlag = myHC.CheckInterception(armVerticies,shelf_1.vertex,shelf_1.face,shelf_1.faceNormals) || myHC.CheckInterception(armVerticies,shelf_2.vertex,shelf_2.face,shelf_2.faceNormals) || myHC.CheckInterception(armVerticies,shelf_3.vertex,shelf_3.face,shelf_3.faceNormals) || myHC.CheckInterception(armVerticies,table_0.vertex,table_0.face,table_0.faceNormals);
        if collisionFlag
            disp("Collision detected!");
            myHC.EStop();
            break
        end
    end
    book_1.onShelf = true;
    myHC.gripperBool = false;
    % Pull arm back from shelf
    for i = size(qMatrix_4,1):-1:1
        myHC.model.animate(qMatrix_4(i,:));
        myHC.qCurrent = qMatrix_4(i,:);
        armVerticies = myHC.GetArmVerticies(qMatrix_4(i,:));
        collisionFlag = myHC.CheckInterception(armVerticies,shelf_1.vertex,shelf_1.face,shelf_1.faceNormals) || myHC.CheckInterception(armVerticies,shelf_2.vertex,shelf_2.face,shelf_2.faceNormals) || myHC.CheckInterception(armVerticies,shelf_3.vertex,shelf_3.face,shelf_3.faceNormals) || myHC.CheckInterception(armVerticies,table_0.vertex,table_0.face,table_0.faceNormals);
        if collisionFlag
            disp("Collision detected!");
            myHC.EStop();
            break
        end
    end
end








































function getOrPutBook(bookNumber,getOrPut)
%     getOrPut is 1 for retreiving book, 2 for placing it on the shelf
    
end
function goToMainLocation( locationNumber)
    % 0 for table, 1 for shelf 1, 2 for shelf 2...
    % Function only applies when we are NOT holding a book
    if locationNumber == 0
        qMatrix_ = jtraj(myHC.qCurrent,deg2rad([-90 -30 0 -90 0 -30 0]),50);
        
        
    end
    if locationNumber == 1

    end
    if locationNumber == 2

    end
    if locationNumber == 3

    end
end

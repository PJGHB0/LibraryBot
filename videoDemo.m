clf;
clc
clear;

%% Here we initialise our environment and objects
% Add in our robot model
myHC = HansCute();
axis off;
% Add in the wall and floor textures (no roof)
surf([-1,-1;1,1],[-1,1;-1,1],[0.001,0.001;0.001,0.001],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([1,1;1,1],[1,-1;1,-1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([1,-1;1,-1],[1,1;1,1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
% Add in our shelves and table. Construct the mesh AND plot the model
shelf_1 = RectangularPrism();
shelf_2 = RectangularPrism();
shelf_3 = RectangularPrism();
table_1 = RectangularPrism();
% !!! WE NEED TO CONFIRM THE MESH DIMENSIONS OF THE SHELVES (they are
% currently wrong)
shelf_1.ConstructWithCentre([0 0.4 0.25], 0.5, 0.5, 0.5);
shelf_2.ConstructWithCentre([0 -0.4 0.25], 0.5, 0.5, 0.5);
shelf_3.ConstructWithCentre([0.4 0 0.25], 0.5, 0.5, 0.5);
table_1.ConstructWithCentre([-0.4 0 0.1], 0.3, 0.3, 0.3);
% Remove mesh outlines for now
% shelf_1.PlotEdges();
% shelf_2.PlotEdges();
% shelf_3.PlotEdges();
% table_1.PlotEdges();
shelf_1.PlotModel(transl(0, 0.4, 0.25),'bookshelf_2.PLY');
shelf_2.PlotModel(transl(0,-0.4,0.25)*trotz(pi),'bookshelf_2.PLY');
shelf_3.PlotModel(transl(0.4,0,0.25)*trotz(-pi/2),'bookshelf_2.PLY');
table_1.PlotModel(transl(-0.4,0,0.1)*trotz(-pi/2),'table.PLY');
% Add in our books
book_1 = Book(transl(-0.3627,0,0.2476)*trotz(-pi/2),'book_1.PLY');
book_2 = Book(transl(0.4,0,0.40167)*trotz(-pi/2),'book_2.PLY');
book_3 = Book(transl(0,0.4,0.24833),'book_2.PLY');
book_4 = Book(transl(0,-0.4,0.04),'book_2.PLY');
% Add in our barrier (as a rectangular prism)
barrier_1 = RectangularPrism();
barrier_2 = RectangularPrism();
barrier_3 = RectangularPrism();
barrier_4 = RectangularPrism();
% !!! WE NEED TO CONFIRM THE MESH DIMENSIONS OF THE BARRIERS (they are
% currently wrong)
% !!! Dont need to do collision detection of barrier
% barrier_1.PlotEdges();
% barrier_2.PlotEdges();
% barrier_3.PlotEdges();
% barrier_4.PlotEdges();
% barrier_1.ConstructWithCentre([-0.6 0 0.360], 0.3, 0.3, 0.3);
% barrier_2.ConstructWithCentre([-0.4 0 0.15], 0.3, 0.3, 0.3);
% barrier_3.ConstructWithCentre([-0.4 0 0.15], 0.3, 0.3, 0.3);
% barrier_4.ConstructWithCentre([-0.4 0 0.15], 0.3, 0.3, 0.3);
barrier_1.PlotModel(transl(-0.6,0,0.160)*trotz(-pi/2),'barrier.PLY');
barrier_2.PlotModel(transl(0.6,0,0.160)*trotz(-pi/2),'barrier.PLY');
barrier_3.PlotModel(transl(0,-0.9,0.160),'barrier.PLY');
barrier_4.PlotModel(transl(0,0.9,0.160),'barrier.PLY');
% Add in Estop button and camera
button_1 = Book(transl(-0.8,1,0.5)*trotz(-pi/2),'button.ply');
% button_2 = Book(transl(1,-0.8,0.5),'button.ply');
camera = Book(transl(1,0,0.8)*trotz(-pi),'camera.PLY');
%% Now we setup the animation parameters
% We list the begining 'shelf poses' of the robot
qInitial = [0 0 0 0 0 0 0];
qTable_0_q1 = deg2rad([-90 0 0 0 0 0 0]);
qShelf_1_q1 = deg2rad([180 0 0 0 0 0 0]);
qShelf_2_q1 = deg2rad([0 0 0 0 0 0 0]);
qShelf_3_q1 = deg2rad([90 0 0 0 0 0 0]);
% We find a good q set for upper, mid and lower shelf (can apply to any
% shelf by changing q1 by 90 degrees)
qShelf_Upper = deg2rad([0 30 0 0 0 -60 0]); % Difficult to reach, near singularity
qShelf_Mid = deg2rad([0 -30 0 -90 0 -30 0]);
qShelf_Lower = deg2rad([0 15 0 -140 0 65 0]);
% We can choose which shelf (or table), and which level, by summing the q
% matricies

%% Now we actually animate
% Go to book one table
qMatrix = jtraj(qInitial, qTable_0_q1 + qShelf_Mid, 50);
pause();
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
    view(280+0.4*i,20+0.1*i);
end
% THIS SHOULD BE DONE WITH RMRC, BUT USE HARDCODED Q VALUES FOR NOW
qGetBook1 = deg2rad([-90 25 0 -65 0 0 0]);
qMatrix = jtraj(qTable_0_q1 + qShelf_Mid, qGetBook1, 30);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
    view(300,25+0.25*i); % Will be 300, 32.5 after
end
% Move book to bottom of shelf 1
qMatrix = jtraj(qGetBook1, qTable_0_q1 + qShelf_Mid, 30);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
    EFTransform = myHC.model.fkine(qMatrix(i,:));
    book_1.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
    view(300+0.25*i,32.5+0.1*i); % Will be 307.5, 35.5
end
qMatrix = jtraj(qTable_0_q1 + qShelf_Mid, - qShelf_1_q1 + qShelf_Lower, 50);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
    EFTransform = myHC.model.fkine(qMatrix(i,:));
    book_1.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
    view(315.5-(8/2500)*(50-i)^2,35.5); % will be 315.5,35.5 after
end
qPlaceBook1 = deg2rad([-180 55 0 -100 0 65 0]);
qMatrix = jtraj(-qShelf_1_q1 + qShelf_Lower, qPlaceBook1, 30);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
    EFTransform = myHC.model.fkine(qMatrix(i,:));
    book_1.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
end
% After placing, move robot arm back
qMatrix = jtraj(qPlaceBook1, -qShelf_1_q1 + qShelf_Lower, 30);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
end
% Move to shelf 2
qMatrix = jtraj(- qShelf_1_q1 + qShelf_Lower, qShelf_2_q1 + qShelf_Lower, 65);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
    view(315.5-0.3*i,35.5-0.1*i); % Will be 296,29 after
end
% Move to upper shelf 3
qMatrix = jtraj(qShelf_2_q1 + qShelf_Lower, qShelf_3_q1 + qShelf_Upper, 50);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
end
% Move to table
qMatrix = jtraj(qShelf_3_q1 + qShelf_Upper, qTable_0_q1 + qShelf_Mid, 65);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
end
qMatrix = jtraj(qTable_0_q1 + qShelf_Mid, qGetBook1, 30);
for i = 1:size(qMatrix,1)
    myHC.model.animate(qMatrix(i,:));
end


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
book_1 = Book(transl(-0.3627,-0.1,0.2476),'book_1.PLY');
book_2 = Book(transl(-0.3627,0,0.2476),'book_2.PLY');
book_3 = Book(transl(-0.3627,0.1,0.2476),'book_2.PLY');
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
%% Here we run our main loop
while myHC.running == true             % Only when robot is 'operating'
    % Once robot is started, it puts away all of the books
    % Put away book 1 first
    break
end

function getOrPutBook(bookNumber,getOrPut)
%     getOrPut is 1 for retreiving book, 2 for placing it on the shelf
    
end
function goToMainLocation(locationNumber)
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

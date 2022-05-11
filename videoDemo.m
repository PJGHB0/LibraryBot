clf;
clc
clear;

%% Here we initialise our environment and objects
% Add in our robot model
myHC = HansCute();
% axis off; Add this back in for video
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
shelf_1.PlotEdges();
shelf_2.PlotEdges();
shelf_3.PlotEdges();
table_1.PlotEdges();
shelf_1.PlotModel(transl(0, 0.4, 0.25),'bookshelf_2.PLY');
shelf_2.PlotModel(transl(0,-0.4,0.25)*trotz(pi),'bookshelf_2.PLY');
shelf_3.PlotModel(transl(0.4,0,0.25)*trotz(-pi/2),'bookshelf_2.PLY');
table_1.PlotModel(transl(-0.4,0,0.1)*trotz(-pi/2),'table.PLY');
% Add in our books
book_1 = Book(transl(-0.4,0,0.285)*trotz(-pi/2),'book_1.PLY');
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

%% Now we animate the robot moving some books




%Setting the workspace
axis([-1 1 -1 1 0 0.6]);

% Setting the floor and wall
surf([-1,-1;1,1],[-1,1;-1,1],[0,0;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
hold on
surf([1,1;1,1],[1,-1;1,-1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
surf([1,-1;1,-1],[1,1;1,1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

%Setting up the first bookshelf
[f,v,data] = plyread('bookshelf_2.PLY','tri');
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255; %Setting the color
location_1= [0 0.4 0.25]; %Setting the location where we want the bookshelf
 Bookshelf_1 = trisurf(f,v(:,1)+location_1(1,1),v(:,2)+location_1(1,2), v(:,3)+location_1(1,3) ... %load the bookshelf into the simulation 
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');

 %Setting the view, and camera
 view(3);
 camlight;
% hold on;
%Setting for the second bookshelf
 [f,v,data] = plyread('bookshelf_2.PLY','tri');
 VertexCount = size(v,1); %Find the number of vertex
%  MidPoint = sum(v)/VertexCount; 
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
 Bookshelf_1_Position = zeros(4) 
location_2= transl(0,-0.4,0.25)*trotz(pi); %Setting the location of the second bookshelf, change this to change the location and orientation
Update_point = [location_2*[v,ones(VertexCount,1)]']'
 Bookshelf_2 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 Bookshelf_2.Vertices=Update_point(:,1:3);


 %Setting for the third bookshelf
  [f,v,data] = plyread('bookshelf_2.PLY','tri');
 VertexCount_bookshelf = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_3= transl(0.4,0,0.25)*trotz(-pi/2); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_bookshelf = [location_3*[v,ones(VertexCount_bookshelf,1)]']'
 Bookshelf_3 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 Bookshelf_3.Vertices=Update_point_bookshelf(:,1:3);

  %Setting for the table
  [f,v,data] = plyread('table.PLY','tri');
 VertexCount_table = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_table= transl(-0.4,0,0.15)*trotz(-pi/2); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_table = [location_table*[v,ones(VertexCount_table,1)]']'
 table = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 table.Vertices=Update_point_table(:,1:3);

 % INFO FOR DIMENSION, 
 % IF YOU WANT THE BOOK TO BE ON THE THIRD ROW, CHANGE Z TO 0.40167
 %IF YOU WANT THE BOOK TO BE ON THE SECOND ROW, CHANGE Z TO 0.24833
 %IF YOU WANT THE BOOK TO BE ON THE FIRST ROW, CHANGE Z TO 0.04
 %IF YOU WANT THE BOOK TO BE ON THE TABLE, CHANGE Z TO 0.215
 %fOR THE CHANGE IN LENGTH FOR THE BOOK, CHANGE X OR Y FROM -0.16 TO 0.16
 %fOR THE CHANGE IN LENGTH FOR THE BOOK, CHANGE X OR Y FROM -0.24 TO 0.24

  %Setting for the book 1
  [f,v,data] = plyread('book_1.PLY','tri');
 VertexCount_book_1 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_book_1= transl(-0.4,0,0.215)*trotz(-pi/2); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_book_1 = [location_book_1*[v,ones(VertexCount_book_1,1)]']'
 book_1 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 book_1.Vertices=Update_point_book_1(:,1:3);

 %Setting for book 2
   [f,v,data] = plyread('book_2.PLY','tri');
 VertexCount_book_2 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_book_2= transl(0.4,0,0.40167)*trotz(-pi/2);; %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_book_2 = [location_book_2*[v,ones(VertexCount_book_2,1)]']'
 book_2 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 book_2.Vertices=Update_point_book_2(:,1:3);

  %Setting for book 3
   [f,v,data] = plyread('book_2.PLY','tri');
 VertexCount_book_3 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_book_3= transl(0,0.4,0.24833); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_book_3 = [location_book_3*[v,ones(VertexCount_book_3,1)]']'
 book_3 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 book_3.Vertices=Update_point_book_3(:,1:3);


  %Setting for book 4
   [f,v,data] = plyread('book_2.PLY','tri');
 VertexCount_book_4 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_book_4= transl(0,-0.4,0.04); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_book_4 = [location_book_4*[v,ones(VertexCount_book_4,1)]']'
 book_4 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 book_4.Vertices=Update_point_book_4(:,1:3);

% setting for Barrier 1
   [f,v,data] = plyread('barrier.PLY','tri');
 VertexCount_barrier_1 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_barrier_1= transl(-0.6,0,0.360)*trotz(-pi/2); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_barrier_1 = [location_barrier_1*[v,ones(VertexCount_barrier_1,1)]']'
 barrier_1 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 barrier_1.Vertices=Update_point_barrier_1(:,1:3);


 % setting for Barrier 2
   [f,v,data] = plyread('barrier.PLY','tri');
 VertexCount_barrier_2 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_barrier_2= transl(0.6,0,0.360)*trotz(-pi/2); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_barrier_2 = [location_barrier_2*[v,ones(VertexCount_barrier_2,1)]']'
 barrier_2 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 barrier_2.Vertices=Update_point_barrier_2(:,1:3);

 % setting for Barrier 3
   [f,v,data] = plyread('barrier.PLY','tri');
 VertexCount_barrier_3 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_barrier_3= transl(0,-0.9,0.360); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_barrier_3 = [location_barrier_3*[v,ones(VertexCount_barrier_1,1)]']'
 barrier_3 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 barrier_3.Vertices=Update_point_barrier_3(:,1:3);


  % setting for Barrier 4
   [f,v,data] = plyread('barrier.PLY','tri');
 VertexCount_barrier_4 = size(v,1);  %Find the number of vertex
%  MidPoint = sum(v)/VertexCount;
 
 vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

location_barrier_4= transl(0,0.9,0.360); %Setting the location of the third bookshelf, change this to change the location and orientation
Update_point_barrier_4 = [location_barrier_4*[v,ones(VertexCount_barrier_4,1)]']'
 barrier_4 = trisurf(f,v(:,1),v(:,2), v(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','none','EdgeLighting','none');
 barrier_4.Vertices=Update_point_barrier_4(:,1:3);

 %Importing the Hanscute robot
 a= HansCute
%  a.model.teach
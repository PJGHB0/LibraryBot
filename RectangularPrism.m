%% Class for obsticles - anything we collision detect the robot with
% Class will include model of object, as well as it's current global
% transform. Class will accept input variables of 

% This is very similar to the code from week 5 which Gavin made,
% essentially inputting two diagonal verticies of the object, and returning
% the verticies, faces, and face normals. Also has the option to input the
% centre of the prism, as well as x,y, and z widths.

classdef RectangularPrism < handle
    properties
        vertex;
        face;
        faceNormals;
        % A model for the object (is not to change location)
        model;
        base = transl(0,0,0); %Base is the set location of the model (not to change)
    end
    methods
        function self = RectangularPrism()
        end
        function ConstructWithCorners(self,upper,lower)
            %Set all verticies
            self.vertex(1,:) = lower;
            self.vertex(2,:) = [upper(1),lower(2:3)];
            self.vertex(3,:) = [upper(1:2),lower(3)];
            self.vertex(4,:) = [upper(1),lower(2),upper(3)];
            self.vertex(5,:) = [lower(1),upper(2:3)];
            self.vertex(6,:) = [lower(1:2),upper(3)];
            self.vertex(7,:) = [lower(1),upper(2),lower(3)];
            self.vertex(8,:) = upper;
            %Set faces
            self.face= [1,2,3;1,3,7;
                        1,6,5;1,7,5;
                        1,6,4;1,4,2;
                        6,4,8;6,5,8;
                        2,4,8;2,3,8;
                        3,7,5;3,8,5];
            %Set faceNormals        
            self.faceNormals = zeros(size(self.face,1),3);
            for faceIndex = 1:size(self.face,1)
                v1 = self.vertex(self.face(faceIndex,1)',:);
                v2 = self.vertex(self.face(faceIndex,2)',:);
                v3 = self.vertex(self.face(faceIndex,3)',:);
                self.faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
            
        end
        function ConstructWithCentre(self,centre,length_x,length_y,length_z)
            %Set all verticies
            lower = [centre(1)+0.5*length_x centre(2)+0.5*length_y centre(3)+0.5*length_z];
            upper = [centre(1)-0.5*length_x centre(2)-0.5*length_y centre(3)-0.5*length_z];
            self.vertex(1,:) = lower;
            self.vertex(2,:) = [upper(1),lower(2:3)];
            self.vertex(3,:) = [upper(1:2),lower(3)];
            self.vertex(4,:) = [upper(1),lower(2),upper(3)];
            self.vertex(5,:) = [lower(1),upper(2:3)];
            self.vertex(6,:) = [lower(1:2),upper(3)];
            self.vertex(7,:) = [lower(1),upper(2),lower(3)];
            self.vertex(8,:) = upper;
            %Set faces
            self.face =[1,2,3;1,3,7;
                        1,6,5;1,7,5;
                        1,6,4;1,4,2;
                        6,4,8;6,5,8;
                        2,4,8;2,3,8;
                        3,7,5;3,8,5];
            %Set faceNormals        
            self.faceNormals = zeros(size(self.face,1),3);
            for faceIndex = 1:size(self.face,1)
                v1 = self.vertex(self.face(faceIndex,1)',:);
                v2 = self.vertex(self.face(faceIndex,2)',:);
                v3 = self.vertex(self.face(faceIndex,3)',:);
                self.faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
            end
        end
        function [vertex,face,faceNormals] = GetVertexFaceNormals(self)
            %We can individually find these properties, however this is
            %easier if we want to pass to a function
            vertex = self.vertex;
            face = self.face;
            faceNormals = self.faceNormals;
        end
        function PlotEdges(self) % Plots the mesh of the object
            hold on;
            links=[1,2;2,3;3,7;7,1;1,6;5,6;5,7;4,8;5,8;6,4;4,2;8,3];
            for i=1:size(links,1)
                plot3(gca,[self.vertex(links(i,1),1),self.vertex(links(i,2),1)],...
                [self.vertex(links(i,1),2),self.vertex(links(i,2),2)],...
                [self.vertex(links(i,1),3),self.vertex(links(i,2),3)],'k')
            end
        end
        function PlotModel(self, baseLocationIn, fileName)
            self.base = baseLocationIn;
            [f,v,data] = plyread(fileName,'tri');
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            vertexCount = size(v,1);
            % Move center point to origin
            midPoint = sum(v)/vertexCount;
            verts = v - repmat(midPoint,vertexCount,1);
            mesh_h = trisurf(f,verts(:,1),verts(:,2), verts(:,3) ...
                            ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            updatedPoints = [self.base * [verts,ones(vertexCount,1)]']';  
            % Now update the Vertices
            mesh_h.Vertices = updatedPoints(:,1:3);
            drawnow();  
        end
    end
        
end
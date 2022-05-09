%% Class for the book object, which needs to be animated
classdef Book < handle
    properties
    currentTransform;
    fileName;
    model;
    f;
    v
    data;
    vertexColours;
    vertexCount;
    midPoint;
    verts;
    mesh_h;
    updatedPoints;
    end
    methods
        function self = Book(startingTransform, fileName)
            self.currentTransform = startingTransform;
            self.fileName = fileName;
            [self.f,self.v,self.data] = plyread(fileName,'tri');
            self.vertexColours = [self.data.vertex.red, self.data.vertex.green, self.data.vertex.blue] / 255;
            self.vertexCount = size(self.v,1);
            % Move center point to origin
            self.midPoint = sum(self.v)/self.vertexCount;
            self.verts = self.v - repmat(self.midPoint,self.vertexCount,1);
            self.mesh_h = trisurf(self.f,self.verts(:,1),self.verts(:,2), self.verts(:,3) ...
                            ,'FaceVertexCData',self.vertexColours,'EdgeColor','interp','EdgeLighting','flat');
            Animate(self, startingTransform);
        end
        function Animate(self, newTransform)
            self.currentTransform = newTransform;
            self.updatedPoints = [newTransform * [self.verts,ones(self.vertexCount,1)]']';  
            % Now update the Vertices
            self.mesh_h.Vertices = self.updatedPoints(:,1:3);
            drawnow();  
        end
    end

    
end

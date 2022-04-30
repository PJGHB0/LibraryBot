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
        
    end
    methods
        function self = RectangularPrism()
        end
        function ConstructWithCorners(self,upper,lower)
            self.vertex(1,:) = lower;
            self.vertex(2,:) = [upper(1),lower(2:3)];
            self.vertex(3,:) = [upper(1:2),lower(3)];
            self.vertex(4,:) = [upper(1),lower(2),upper(3)];
            self.vertex(5,:) = [lower(1),upper(2:3)];
            self.vertex(6,:) = [lower(1:2),upper(3)];
            self.vertex(7,:) = [lower(1),upper(2),lower(3)];
            self.vertex(8,:) = upper;
        end
        function GetVertexFaceNormals
        function ConstructWithCentre(self,centre,length_x,length_y,length_z)
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
        end
    end
        
end
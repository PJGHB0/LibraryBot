%% This is a class for our front end interface (pretty much does everything)
classdef InterfaceClass < handle
    properties
        HansCute; %The instance of our hans cute model
        % Our environment objects
        table_0;
        shelf_1;
        shelf_2;
        shelf_3;
        books;
        barrier_1;
        barrier_2;
        barrier_3;
        barrier_4;
        button_1;
        camera_1;
        % We list the shelf poses of the robot
        % We can choose which shelf (or table), and which level, by summing the q
        % matricies below
        qInitial = [0 0 0 0 0 0 0];
        qtable_0_q1 = deg2rad([-90 0 0 0 0 0 0]);
        qshelf_1_q1 = deg2rad([-180 0 0 0 0 0 0]);
        qshelf_2_q1 = deg2rad([0 0 0 0 0 0 0]);
        qshelf_3_q1 = deg2rad([90 0 0 0 0 0 0]);
        qShelf_Upper = deg2rad([0 30 0 0 0 -60 0]);     % Difficult to reach, near singularity
        qShelf_Mid = deg2rad([0 -40 0 -120 0 -10 0]);
        qTable_level = deg2rad([0 -30 0 -90 0 -30 0]);
        qShelf_Lower = deg2rad([0 25 0 -150 0 85 0]);
        % all the following are in terms of end effector pose (have to
        % translate along z to get book pose)
        qBooksShelfPosition; % This is th position IN FRONT OF the books shelf location (not the final drop off point)
        xyzBookShelfDepositLocation; % This is the position of the book when it is to be released
        speedMultiplier;
    end
    methods
        function self = InterfaceClass()        % Our constructor
            self.HansCute = HansCute();
            self.BuildEnvironment();
            self.qBooksShelfPosition{1} = self.qshelf_1_q1 + self.qShelf_Lower; % Book one has home on lower shelf 1
            self.qBooksShelfPosition{2} = self.qshelf_2_q1 + self.qShelf_Mid;
            self.qBooksShelfPosition{3} = self.qshelf_3_q1 + self.qShelf_Mid;
            self.xyzBookShelfDepositLocation{1} = transl(0,0.06,-0.01);
            self.xyzBookShelfDepositLocation{2} = transl(0,0.08,0);
            self.xyzBookShelfDepositLocation{3} = transl(0,0.08,-0.01);
            self.speedMultiplier = 2;
            self.HansCute.speed = 0.1*self.speedMultiplier;
        end
        function BuildEnvironment(self)
            % Add in the wall and floor textures (no roof)
            surf([-1,-1;1,1],[-1,1;-1,1],[0.001,0.001;0.001,0.001],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([1,1;1,1],[1,-1;1,-1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([1,-1;1,-1],[1,1;1,1],[0,0;1,1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            % Add in our shelves and table. Construct the mesh AND plot the model
            self.shelf_1 = RectangularPrism();
            self.shelf_2 = RectangularPrism();
            self.shelf_3 = RectangularPrism();
            self.table_0 = RectangularPrism();
            self.shelf_1.PlotModel(transl(0, 0.4, 0.25),'bookshelf_2.PLY');
            self.shelf_2.PlotModel(transl(0,-0.4,0.25)*trotz(pi),'bookshelf_2.PLY');
            self.shelf_3.PlotModel(transl(0.4,0,0.25)*trotz(-pi/2),'bookshelf_2.PLY');
            self.table_0.PlotModel(transl(-0.4,0,0.1)*trotz(-pi/2),'table.PLY');
            self.shelf_1.ConstructWithCorners([0.2,0.3,0],[-0.2,0.55,0.5]);
            self.shelf_2.ConstructWithCorners([0.2,-0.3,0],[-0.2,-0.55,0.5]);
            self.shelf_3.ConstructWithCorners([0.55,-0.185,0],[0.3,0.185,0.5]);
            self.table_0.ConstructWithCorners([-0.5,0.25,0],[-0.275,-0.25,0.18]);
            self.shelf_1.PlotEdges();
            self.shelf_2.PlotEdges();
            self.shelf_3.PlotEdges();
            self.table_0.PlotEdges();
            % Add in our books
            self.books{1} = Book(transl(-0.3627,-0.07,0.2476)*trotz(pi/2),'book_2.PLY',false);
            self.books{2} = Book(transl(-0.3627,0,0.2476)*trotz(pi/2),'book_1.PLY',false);
            self.books{3} = Book(transl(-0.3627,0.07,0.2476)*trotz(pi/2),'book_2.PLY',false);
            % Add in our barrier (as a rectangular prism)
            self.barrier_1 = RectangularPrism();
            self.barrier_2 = RectangularPrism();
            self.barrier_3 = RectangularPrism();
            self.barrier_4 = RectangularPrism();
            self.barrier_1.PlotModel(transl(-0.6,0,0.160)*trotz(-pi/2),'barrier.PLY');
            self.barrier_2.PlotModel(transl(0.6,0,0.160)*trotz(-pi/2),'barrier.PLY');
            self.barrier_3.PlotModel(transl(0,-0.9,0.160),'barrier.PLY');
            self.barrier_4.PlotModel(transl(0,0.9,0.160),'barrier.PLY');
            % Add in Estop button and camera
            self.button_1 = Book(transl(-0.8,1,0.5)*trotz(-pi/2),'button.ply',false);
            % button_2 = Book(transl(1,-0.8,0.5),'button.ply');
            self.camera_1 = Book(transl(1,0,0.8)*trotz(-pi),'camera.PLY',false);
        end
        function EmergencyProcedure(self,emergencyType)
            %Upon a collision or estop being pressed, this is excecuted
            %Emergency type - 0 for EStop, 1 for collision
            if emergencyType == 1
            % Collision procedure    
            end
            if emergencyType == 0
            % EStop procedure
            end
        end
        function ReturnBook(self,bookNumber)
            % We check if book is already returned
            if self.books{bookNumber}.onShelf
                X = ['ERROR: Book ',num2str(bookNumber),' is already on the shelf'];
                disp(X);
                return
            end
            X = ['----------Returning Book ',num2str(bookNumber),'----------'];
            disp(X);
            % First go in front of book on table (and make sure EF is open)
            self.HansCute.TriggerGripper(false);
            qMatrix = jtraj(self.HansCute.qCurrent,self.qtable_0_q1+self.qTable_level,50/self.speedMultiplier);
            MoveWithoutBook(self,qMatrix);
            % Next we slowly move the arm to the book location
            qMatrix = self.HansCute.LinearRMRC(self.HansCute.qCurrent,self.books{bookNumber}.currentTransform*transl(0,-0.03,0));
            MoveWithoutBook(self,qMatrix);
            % Now we go backwards away from the book (while holding book)
            self.HansCute.TriggerGripper(true);
            MoveWithBook(self,flip(qMatrix,1),bookNumber);
            % Now we go to in front of the appropriate shelf location
            qMatrix = jtraj(self.HansCute.qCurrent, self.qBooksShelfPosition{bookNumber}, 50/self.speedMultiplier);
            MoveWithBook(self,qMatrix,bookNumber);
            % Now we use RMRC to place the book (opportunity for visual
            % servoing)
            qMatrix = self.HansCute.LinearRMRC(self.HansCute.qCurrent,self.books{bookNumber}.currentTransform*self.xyzBookShelfDepositLocation{bookNumber});
            MoveWithBook(self,qMatrix,bookNumber);
            % Back out from the book location (after releasing book)
            self.HansCute.TriggerGripper(false);
            MoveWithoutBook(self,flip(qMatrix,1));
%             And return to upright position (WE SKIP THIS RIGHT NOW, AS
%             ITS UNNECESSARY)
%             qMatrix = jtraj(self.HansCute.qCurrent, [0 0 0 0 0 0 0], 75/self.speedMultiplier);
%             MoveWithoutBook(self,qMatrix);
            self.books{bookNumber}.onShelf = true;
            X = ['----------Book ',num2str(bookNumber),' Returned----------'];
            disp(X);
        end
        function GetBook(self,bookNumber)
            % We check if book is already retreived
            if ~self.books{bookNumber}.onShelf
                X = ['ERROR: Book ',num2str(bookNumber),' is already on the table'];
                disp(X);
                return
            end
            X = ['----------Getting Book ',num2str(bookNumber),'----------'];
            disp(X);
            % First we go to the book location
            self.HansCute.gripperBool = false;
            qMatrix = jtraj(self.HansCute.qCurrent,self.qBooksShelfPosition{bookNumber},50/self.speedMultiplier);
            MoveWithoutBook(self,qMatrix);
            % Now we retreive the book
            qMatrix = self.HansCute.LinearRMRC(self.HansCute.qCurrent,self.HansCute.model.fkine(self.HansCute.qCurrent)*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2)*self.xyzBookShelfDepositLocation{bookNumber});
            MoveWithoutBook(self,qMatrix);
            % Back out from the book location (after gripping book)
            self.HansCute.TriggerGripper(true);
            MoveWithBook(self,flip(qMatrix,1),bookNumber);
            % Now we go to the desired table location
            qMatrix = jtraj(self.HansCute.qCurrent, self.qtable_0_q1+self.qTable_level, 50/self.speedMultiplier);
            MoveWithBook(self,qMatrix,bookNumber);
            % Now we slowly move the arm to place the book
            qMatrix = self.HansCute.LinearRMRC(self.HansCute.qCurrent,self.books{bookNumber}.tableTransform*transl(0,-0.03,0));
            MoveWithBook(self,qMatrix,bookNumber);
            % We release the book, and move the arm back
            self.HansCute.TriggerGripper(false);
            MoveWithoutBook(self,flip(qMatrix,1));
            self.books{bookNumber}.onShelf = false;
            X = ['----------Book ',num2str(bookNumber),' Retreived----------'];
            disp(X);
        end
        function MoveWithBook(self,qMatrix,bookNumber)
            % Does not work if we are currently stopped
            if ~self.HansCute.running
                disp("Cannot run system");
                return
            end
            % Move end effector along a qMatrix (first q must be current
            % robot q)m while holding the specified book. 
            % Checks for collision along the way
            for i = 1:size(qMatrix,1)
                armVerticies = self.HansCute.GetArmVerticies(qMatrix(i,:));
                collisionFlag = self.HansCute.CheckInterception(armVerticies,self.shelf_1.vertex,self.shelf_1.face,self.shelf_1.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.shelf_2.vertex,self.shelf_2.face,self.shelf_2.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.shelf_3.vertex,self.shelf_3.face,self.shelf_3.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.table_0.vertex,self.table_0.face,self.table_0.faceNormals);
                if collisionFlag
                    self.HansCute.CollisionStop();
                    break
                end
                self.HansCute.model.animate(qMatrix(i,:));
                self.HansCute.qCurrent = qMatrix(i,:);
                EFTransform = self.HansCute.model.fkine(qMatrix(i,:));
                self.books{bookNumber}.Animate(EFTransform*transl(0,0,0.03)*trotx(pi/2)*troty(pi/2));
            end

        end
        function MoveWithoutBook(self,qMatrix)
            % Does not work if we are currently stopped
            if ~self.HansCute.running
                disp("Cannot run system");
                return
            end
            % Move end effector along a qMatrix (first q must be current
            % robot q)
            % Checks for collision along the way
            for i = 1:size(qMatrix,1)
                armVerticies = self.HansCute.GetArmVerticies(qMatrix(i,:));
                collisionFlag = self.HansCute.CheckInterception(armVerticies,self.shelf_1.vertex,self.shelf_1.face,self.shelf_1.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.shelf_2.vertex,self.shelf_2.face,self.shelf_2.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.shelf_3.vertex,self.shelf_3.face,self.shelf_3.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.table_0.vertex,self.table_0.face,self.table_0.faceNormals);
                if collisionFlag
                    disp("Collision detected!");
                    self.HansCute.EStop();
                    self.HansCute.stopVariable = [true true];
                    break
                end
                self.HansCute.model.animate(qMatrix(i,:));
                self.HansCute.qCurrent = qMatrix(i,:);
            end
        end
        function [collisionPredicted,index] = CollisionPrediction(self,qMatrix)
            % Predicts a collision along a path, and the q values
            % associated
            for i=1:size(qMatrix,1)
                armVerticies = self.HansCute.GetArmVerticies(qMatrix(i,:));
                collisionFlag = self.HansCute.CheckInterception(armVerticies,self.shelf_1.vertex,self.shelf_1.face,self.shelf_1.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.shelf_2.vertex,self.shelf_2.face,self.shelf_2.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.shelf_3.vertex,self.shelf_3.face,self.shelf_3.faceNormals) || self.HansCute.CheckInterception(armVerticies,self.table_0.vertex,self.table_0.face,self.table_0.faceNormals);
                if collisionFlag
                    index = i;
                    collisionPredicted = collisionFlag;
                    return
                end
            end
        end
    end
end

%% Setup our world
clc;
clf;
clear
myInterface = InterfaceClass();

%% Testing book get and return
myInterface.ReturnBook(3);
% myInterface.ReturnBook(2);
% myInterface.GetBook(3);
%% Testing control (obselete)
% qMatrix = jtraj([0 0 0 0 0 0 0],myInterface.qtable_0_q1+myInterface.qShelf_Mid,50);
% myInterface.MoveWithoutBook(qMatrix);
% qMatrix = jtraj(myInterface.HansCute.qCurrent,[0 0 0 0 0 0 0],50);
myInterface.MoveWithBook(qMatrix,1);
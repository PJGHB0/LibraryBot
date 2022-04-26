%HansCute / Cyton300E for 41013 assignment2
%built by QND
clear all
clf    
clc

L1 = Link('d',(54+66)/1000,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-150),deg2rad(150)]);
L2 = Link('d',0,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-105),deg2rad(105)]);
L3 = Link('d',140.80/1000,'a',0,'alpha',pi/2,'offset',0,'qlim',[deg2rad(-150),deg2rad(150)]);
L4 = Link('d',0,'a',71.8/1000,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L5 = Link('d',0,'a',71.8/1000,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-105),deg2rad(105)]);
L6 = Link('d',0,'a',0,'alpha',pi/2,'offset',pi/2,'qlim',[deg2rad(-105),deg2rad(105)]);
L7 = Link('d',129.6/1000,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-150),deg2rad(150)]);

LbrBot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',"LibraryBot"); % 

LbrBot.plot(zeros(1,7)) %
LbrBot.teach()

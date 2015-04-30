%% Initialization.m
% MEAM620 Project 3A
% Group 2:    Caio Mucchiani, Jay Davey, Eduardo Garcia
%%

%%This file initializes the variables needed to dothe CAPT algorithm for 2D
%%Robots.




%% 
%Variable Definitions
Num_Dim = 2;
Max_X = 1500;
May_Y = 1500;

N = 20; %Number of robots
M = N;  %Number of goals

R = 1; %Radius of Robots

X = zeros(N, Num_Dim);%Nxnum_dim State Vector X (Robot positions in X, Y)
G = zeros(N, Num_Dim);%Nxnum_dim State Vector G (Goal positions in X, Y)


%%Map Generation
%The start and the end (goals) must be delta spaced apart! We have to
%generate this the old way (for loops)...
















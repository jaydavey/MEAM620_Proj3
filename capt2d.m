% 2D implementation of CAPT algorithm
% Jay Davey, Eduardo Garcia, Caio 

% M. Turpin, N. Michael, and V. Kumar, \Capt: Concurrent assignment and 
% planning of trajectories for multiple robots," 
% The International Journal of Robotics Research, 2014.

close all; clearvars; clc;

% Program variables
Nr = 100; %Number of robots
Ns = Nr;   %Number of starting locations
Ng = 100; %Number of goal locations

% Define the extents of the robot's stage area (the area that the robots
% will fly in
xmax = 10; xmin = -10; %meters
ymax = 5; ymin = -5; %meters
% zmax = 5; zmin = 0; %meters

% Setup the figure for showing the alogorithm's output
h = figure(1);
axis([xmin xmax ymin ymax]); % zmin zmax]);
axis square
hold on

% Generate random positions for the start and goal locations in the
% allocated stage area

% As starting locations are generated, make sure they are spaced out enough
% that they aren't on top of each other in 2D



for i = 1:Ns
    newLoc = rand(1,2);
    
    %test for distance to other locations




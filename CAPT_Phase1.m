% 3D implementation of CAPT algorithm
% Jay Davey, Eduardo Garcia, Caio 

% M. Turpin, N. Michael, and V. Kumar, \Capt: Concurrent assignment and 
% planning of trajectories for multiple robots," 
% The International Journal of Robotics Research, 2014.

close all; clearvars; clc;

%% Program variables
Nr = 20; %Number of robots
Ns = Nr;   %Number of starting locations
Ng = 20; %Number of goal locations

R = 0.08; % Radius of robot in meters

% Define the extents of the robot's stage area (the area that the robots
% will fly in
xmax = 4; xmin = -2; %meters
ymax = 1; ymin = -2; %meters
zmax = 1; zmin = 0; %meters

%% Setup the figure for showing the alogorithm's output
h = figure(1);
bound = [xmin xmax ymin ymax zmin zmax];
offset=R*sqrt(2);
axis equal
grid on
if bound(6)==0
    %we're doing it in 2D
    axis(bound(1:4));
    pt_bound = [bound(1)+offset, bound(2)-offset, bound(3)+offset, bound(4)-offset];
else
    %we're doing it in 3D
    axis(bound);
    pt_bound = [bound(1)+offset, bound(2)-offset, bound(3)+offset, bound(4)-offset, bound(5)+offset, bound(6)-offset];
end
set(h, 'Position', [50, 250, 1400, 700]);
hold on

%% CAPT Part 1, generate random start and end locations
% Generate random positions for the start and goal locations in the
% allocated stage area
[S,G] = getRandomPoints(Ns,Ng,pt_bound,R);

if size(S,2)==2
    %make the points 3D points with zero height
    S = [S, zeros(size(S,1),1)];
	G = [G, zeros(size(G,1),1)];
end

% make robots little spheres
[Sx,Sy,Sz] = sphere();
colorS = [1 0 0]; % Start point color (red)
for i = 1:Ns
    s(i) = surf(S(i,1)+Sx*R,S(i,2)+Sy*R,S(i,3)+Sz*R);
    set(s(i),'EdgeAlpha',0,'FaceAlpha',0.3,'FaceColor', colorS);
    str = strcat('S',num2str(i));
    th = text(S(i,1),S(i,2),S(i,3),str);
    set(th,'FontSize',8,'FontWeight','bold');
end
    
% make goals spheres to ensure we don't have collisions at the end 
colorG = [0 0 1]; % Goal color (green)
for i = 1:Ns
    g(i) = surf(G(i,1)+Sx*R,G(i,2)+Sy*R,G(i,3)+Sz*R);
    set(g(i),'EdgeAlpha',0,'FaceAlpha',0.3,'FaceColor', colorG);
    str = strcat('G',num2str(i));
    th = text(G(i,1),G(i,2),G(i,3),str);
    set(th,'FontSize',8,'FontWeight','bold');
end

%% CAPT Part 2, 







% 3D implementation of CAPT algorithm for quadrotor
% Jay Davey, Eduardo Garcia, Caio 

% M. Turpin, N. Michael, and V. Kumar, \Capt: Concurrent assignment and 
% planning of trajectories for multiple robots," 
% The International Journal of Robotics Research, 2014.

%Base on our "CAPT_Phase1" code

close all; clearvars; clc;
addpath('./Quadrotor');
addpath('./Quadrotor/utils')
addpath('./Quadrotor/trajectories')

%% Program variables
Nr = 15; %Number of quadrotors
Ns = Nr;   %Number of starting locations
Ng = Nr*2; %Number of goal locations

R = 0.10; % Radius of robot in meters

% Define the extents of the robot's stage area (the area that the robots
% will fly in
xmax = 3; xmin = -1; %meters
ymax = 3; ymin = -1; %meters
zmax = 1; zmin = 0; %meters

%% Setup the figure for showing the alogorithm's output
h = figure(1);
bound = [xmin xmax ymin ymax zmin zmax]; %Change of variable for quadrotor
offset=R*sqrt(2);
axis equal
grid on
if bound(6)==0
    %we're doing it in 2D
    axis(bound(1:4));
    pt_bound = [bound(1)+offset, bound(2)-offset, bound(3)+offset, bound(4)-offset];
else
    %we're doing it in 3D
    bound = [xmin xmax ymin ymax zmin*4 zmax*4]; %Change of variable for quadrotor, here just for ploting reasons
    axis(bound);
    bound = [xmin xmax ymin ymax zmin zmax]; %
    pt_bound = [bound(1)+offset, bound(2)-offset, bound(3)+offset, bound(4)-offset, bound(5)+offset, bound(6)-offset];
end
%set(h, 'Position', [25, 150, 1400, 600]);
hold on

%% CAPT Part 1, generate random start and end locations
% Generate random positions for the start and goal locations in the
% allocated stage area
[S,G] = getRandomPoints(Ns,Ng,pt_bound,R);

%The minimum distance in the Z direction must be 8R! We do the variable
%change z_hat = z/4

if size(S,2)==2
    %make the points 3D points with zero height
    S = [S, zeros(size(S,1),1)];
	G = [G, zeros(size(G,1),1)];
else
   S(:,3) =  S(:,3)*4;
   G(:,3) =  G(:,3)*4;
    
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
for i = 1:Ng
    g(i) = surf(G(i,1)+Sx*R,G(i,2)+Sy*R,G(i,3)+Sz*R);
    set(g(i),'EdgeAlpha',0,'FaceAlpha',0.3,'FaceColor', colorG);
    str = strcat('G',num2str(i));
    th = text(G(i,1),G(i,2),G(i,3),str);
    set(th,'FontSize',8,'FontWeight','bold');
end

%% CAPT Part 2, Cost matrix

% Find D matrix for the start and goal points (D = the distance^2 matrix)
A = pdist2(S,G);
D = A.*A;

Nquad = [10 15]; %Number of quadrotors for Phase 2 Part1**

% find assignments between start and goals
[assignment,cost] = munkres(D);

% Draw lines between the matching start-goals
for i = 1:Ns
    if ~(assignment(i)==0)
        pl = plot3([S(i,1),G(assignment(i),1)],[S(i,2),G(assignment(i),2)],[S(i,3),G(assignment(i),3)],'-');
        set(pl, 'color',[0,0.8,0]);
    end
end


%Start Locations (meters):
S

%Goal Locations (meters):
G




if(Nr < Ng)
    num_left = 1: Ng;
    visited = sort(assignment);
    goals_left_index = ~ismember(num_left, visited); %Get the logical indices of the goals that are not visited on the first run
    start2 = G(assignment,:);
    goal2 = G(goals_left_index,:);
    
    
    % Find D matrix for the start and goal points (D = the distance^2 matrix)
    A2 = pdist2(start2,goal2);
    D2 = A2.*A2;
    
    % find assignments between start and goals
    [assignment2,cost2] = munkres(D2);
    
    
    % Draw lines between the matching start-goals
    for i = 1:Ns
        if ~(assignment2(i)==0)
            pl = plot3([start2(i,1),goal2(assignment2(i),1)],[start2(i,2),goal2(assignment2(i),2)],[start2(i,3),goal2(assignment2(i),3)],'-');
            set(pl, 'color',[0.8,0.8,0]);
        end
    end
    
    
    
end


if(size(S,1)<=size(G,1))
    start = S;
    goal = G(assignment,:);
else
    assignment = assignment(assignment~=0);
       start = S;
       goal = S;
    goal(assignment,:) = G(assignment,:); 
end


%% Quadrotor Simulation

%Set up system
nquad = Nr;
if(Nr < Ng)
    trajectory_generator( -1, 2, start, goal, start2, goal2);
    time_exit = 21;
else
    trajectory_generator( -1, 2, start, goal);
    time_exit = 11;
end
runsim;







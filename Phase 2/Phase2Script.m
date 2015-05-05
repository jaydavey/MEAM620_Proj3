% Phase 2 Project 3 MEAM620
% Jay Davey, Eduardo Garcia, Caio 

% M. Turpin, N. Michael, and V. Kumar, \Capt: Concurrent assignment and 
% planning of trajectories for multiple robots," 
% The International Journal of Robotics Research, 2014.


% Extend the CAPT algorithm to robots operating in known obstacle-filled environments using the Goal
% Assignment and Trajectory Planning (GAP) algorithm found in this paper: M. Turpin, K. Mohta, N.
% Michael, and V. Kumar, “Goal assignment and trajectory planning for large teams of interchangeable
% robots,” Robotics: Science and Systems 2013.


close all; clearvars; clc;

%% Include Necesarry Paths
%We include the paths of the things we will need for our program

addpath('./Maps');
addpath('./A_Star')




%% Program variables

R = 0.1; % Radius of robot in meters
vel = 1; % m/s velocity for robots

%% Read Map
%We read the maps with all the information available at
%/Maps directory
map = load_map('map1.txt', 2*R*sqrt(2), 5, 0); % map = load_map(filename, xy_res, z_res, margin)


occp_grid = map{1};

%map{3} = [bound_xi bound_yi bound_zi bound_xf bound_yf bound_zf xy_res z_res margin];
xmin = map{3}(1);
ymin = map{3}(2);
xmax = map{3}(4);
ymax = map{3}(5);

%map{4} = [map_size_x, map_size_y, map_size_z];
map_size_x = map{4}(1);
map_size_y = map{4}(2);
map_size_z = map{4}(3);


Nr = map{5}(1);
Ns = Nr;   %Number of starting locations
Ng = map{5}(2);
start = map{6};
goals = map{7};
start_val = map{8};
goals_val = map{9};
astar = 0; %No A-STAR
robot_pos = start;
% Define the extents of the robot's stage area (the area that the robots
% will fly in
zmax = 0; zmin = 0; %meters


%Plot2DGrid( occp_grid );
%PlotObjectives( plot_handler, start, goals );
%PlotRobots(plot_handler, robot_pos, R);


%% Do Dijkstra Algorithm
% We do this path-planning algorithm to determine the shortest path between
% all nodes!

%We can get the cost of all robots to goals
prev = cell(Nr, Ng);
D = zeros(Nr, Ng);
for i=1:Nr
    for j=1:Ng

        [p, cost] = dijkstra(map, start(i,:), goals(j,:), 0);
        prev{i,j} = p;
        D(i,j) = cost;
        
    end
end

%% Setup the figure for showing the alogorithm's output
% h = plot_handler
% bound = [xmin xmax ymin ymax zmin zmax];
% offset=R*sqrt(2);
% axis equal
% grid on
% if bound(6)==0
%     %we're doing it in 2D
%     axis(bound(1:4));
%     pt_bound = [bound(1)+offset, bound(2)-offset, bound(3)+offset, bound(4)-offset];
% else
%     %we're doing it in 3D
%     axis(bound);
%     pt_bound = [bound(1)+offset, bound(2)-offset, bound(3)+offset, bound(4)-offset, bound(5)+offset, bound(6)-offset];
% end
% set(h, 'Position', [25, 150, 1400, 600]);
% hold on

%% CAPT Part 1, generate random start and end locations
% Generate random positions for the start and goal locations in the
% allocated stage area
%[S,G] = getRandomPoints(Ns,Ng,pt_bound,R);
S = start(:,1:2);
G = goals(:,1:2);

if size(S,2)==2
    %make the points 3D points with zero height
    S = [S, zeros(size(S,1),1)];
	G = [G, zeros(size(G,1),1)];
end



%% CAPT Part 2, Cost matrix

% Find D matrix for the start and goal points (D = the distance^2 matrix)





% find assignments between start and goals
[assignment,cost] = munkres(D);

%From assignments, get path for each robot
robot_paths = cell(Nr,1);
robot_paths_xy = cell(Nr,1);
for(i=1:Nr)
    
    if(assignment(i) ~=0)
        robot_paths{i} = prev{i,assignment(i)};
        [I J] = ind2sub(size(map{1}),robot_paths{i}) ;
        robot_paths_xy{i} = [I J];
    else
        robot_paths{i} = sub2ind(size(map{1}), start(i,1), start(i,2));
        robot_paths_xy{i} = [start(i,1), start(i,2)];
    end
end


%We now get the distance between each point! First we get the vector from
%points A to B. We also get the time for each trayectory

robot_vector = cell(Nr,1);
robot_vector_dist = cell(Nr,1);
robot_time_traj = cell(Nr,1); %Cummulative time of all path to point X
robot_indv_time = cell(Nr,1); %Individual time of each paths
for(i=1:Nr)
    robot_vector{i} = diff(robot_paths_xy{i});
    robot_vector_dist{i} = sqrt(sum(diff(robot_paths_xy{i}).^2,2));
    robot_indv_time{i} = robot_vector_dist{i}./vel;
    robot_time_traj{i} = [0; cumsum(robot_indv_time{i})]; 
end


%Start Locations (meters):
S;

%Goal Locations (meters):
G;



%% Parameters


move_time_flag = ones(Nr,1);
move_time = zeros(Nr,1);

%% Animation
max_iter = 5000;
real_time = 1;
cstep     = 0.05;      % image capture time interval
time = 0;
t = 0;
for iter = 1:max_iter

     tic;
     if iter == 1
         plot_handler = figure();
         figure(plot_handler);
         Plot2DGrid( occp_grid,xmin,ymin );
         PlotObjectives( plot_handler, start, goals, xmin,ymin );
         [robot_handles, text_handles] = PlotRobots(plot_handler, robot_pos, R, Nr);
         h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
     end
     
        %Update Robot Positions
        figure(plot_handler);
         %Plot2DGrid( occp_grid );
         %PlotObjectives( plot_handler, start, goals );
         
         
         for (i=1:Nr)
            UpdateRobotPlot(time, i, robot_vector{i}, robot_vector_dist{i}, robot_time_traj{i},robot_indv_time{i}, robot_paths_xy{i}, robot_handles{i}, text_handles{i} );
         end
         
        drawnow();
     % Update plot

     set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    
    
    
        time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> inf*cstep*50000)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end
    
    
    
end







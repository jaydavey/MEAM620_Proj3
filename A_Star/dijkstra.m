function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if nargin < 4
    astar = false;
end


%%%Add code HW0%%%

%% Map Creation (Adjecency)

%Map should be a 2 cell variable with an occupancy grid on cell (1).



%I am basing my algorithm on what is shown on
%http://en.wikipedia.org/wiki/Dijkstra's_algorithm#Using_a_priority_queue


%% Variable Declaration
%Extract Data from map
occp_grid = map{1};

map_sizes = map{4};
map_size_x = map_sizes(1);
map_size_y = map_sizes(2);
map_size_z = map_sizes(3);

bounds = map{3};
bound_xi = bounds(1);
bound_yi = bounds(2);
bound_zi = bounds(3);
bound_xf = bounds(4);
bound_yf = bounds(5);
bound_zf = bounds(6);
xy_res   = bounds(7);
z_res    = bounds(8);
margin   = bounds(9);
%End Extract data from map{}


%Convert start and goal from XYZ coordinates to occupancy grid coordinates
start_grid_x = floor((start(1)/xy_res) - (bound_xi/xy_res)) + 1;
start_grid_y = floor((start(2)/xy_res) - (bound_yi/xy_res)) + 1;
start_grid_z = floor((start(3)/z_res) -  (bound_zi/z_res)) + 1;

goal_grid_x = floor((goal(1)/xy_res) - (bound_xi/xy_res)) + 1;
goal_grid_y = floor((goal(2)/xy_res) - (bound_yi/xy_res)) + 1;
goal_grid_z = floor((goal(3)/z_res) -  (bound_zi/z_res)) + 1;

start_i = sub2ind(size(occp_grid),start_grid_x, start_grid_y, start_grid_z);
goal_i  = sub2ind(size(occp_grid),goal_grid_x, goal_grid_y, goal_grid_z);


%Initialize Dijkstra variables
num_nodes = map_size_x * map_size_y * map_size_z;
%pQueue = PriorityQueue(num_nodes); %Create a priority queue based on #nodes
%dist = ones(1,num_nodes).*inf;
dist = ones(map_size_x, map_size_y, map_size_z).*inf;
dist_heur = ones(map_size_x, map_size_y, map_size_z).*inf;
alt = ones(map_size_x, map_size_y, map_size_z).*inf;
alt_heur = ones(map_size_x, map_size_y, map_size_z).*inf;
prev = zeros(map_size_x, map_size_y, map_size_z);
scanned = logical(ones(map_size_x, map_size_y, map_size_z));

%Make the neighbors offset matrix! Linear index
% north_offset = -1;
% south_offset = 1;
% east_offset  = map_size_x;
% west_offset  = -map_size_x;
% up_offset    =  map_size_x * map_size_y;  %Go up in Z
% down_offset  = -map_size_x * map_size_y; %Go down on Z

%Make the neighbors offset matrix! Subscript index
north_offset = -1;
south_offset = 1;
east_offset  = 1;
west_offset  = -1;
up_offset    =  1;  %Go up in Z
down_offset  = -1; %Go down on Z

dy = [west_offset, 0, east_offset ]';
dy = repmat(dy, [9,1]);

dx = [north_offset 0  south_offset]';
dx = repmat(dx,[3,3])';
dx = dx(:);

dz = [up_offset 0  down_offset]';
dz = repmat(dz,[1,9])';
dz = dz(:);

%neigh_offset = dz + dx + dy;
neigh_offset = [dx dy dz];
%
num_expanded = 0;



%%Code to replace ind2sub
%Adapted from https://labangels.wordpress.com/tag/ind2sub/
dim = size(occp_grid);

[Cols, Rows, W] = meshgrid( 1:dim(2), 1:dim(1), 1:size(occp_grid,3));


%Make an num_nodesx3 array that has the XYZ coordinates of each point
a = Rows;
b = Cols;
c = W;

    a = (a-1)*xy_res + 0.5*xy_res + bound_xi; 
    b = (b-1)*xy_res + 0.5*xy_res + bound_yi;
    c = (c-1)*z_res + 0.5*z_res + bound_zi;
    
    a(start_i) = start(1); 
    b(start_i) = start(2);
    c(start_i) = start(3);
    
    a(goal_i) = goal(1);
    b(goal_i) = goal(2);
    c(goal_i) = goal(3);
    
    a(a>bound_xf) = bound_xf;
    a(a<bound_xi) = bound_xi;
    
    b(b>bound_yf) = bound_yf;
    b(b<bound_yi) = bound_yi;
   
    c(c>bound_zf) = bound_zf;
    c(c<bound_zi) = bound_zi;

disc_cor = [a,b,c];
%% Initialization
dist(start_i) = 0;
dist_heur(start_i) = 0;

bUpdate = false(size(alt)); % allocate the vector

%Try #2: Vectorizing

%%Dijkstra
if (astar == false)
    for i=1:num_nodes
        num_expanded = num_expanded +1;
        %u := vertex in Q with min dist[u]
        %dist_left(scanned) = NaN; %We use Nan so min ignores the nodes we have been to!
        [~,u1] = min(dist_heur(:)); %Get minimum unvisited node
        u = (u1);
        
        scanned(u) = 0; %Remove u from Q
        if(scanned(goal_i) == 0)
            break;
        end
        
        neighbors = getNeighbors(u, occp_grid, neigh_offset , num_nodes, map_sizes, Cols, Rows, W);  %Gets list of neighbors
        
        
%         if(~isempty(neighbors))
%             dist_u = dist(u);
%             current_loc = [a(u), b(u), c(u)];
%             neigh_loc = [a(neighbors),b(neighbors),c(neighbors)];
%             cost = sqrt(sum((bsxfun(@minus, neigh_loc,current_loc)).^2,2)');
%             alt(neighbors) = dist_u + cost;%dist_u + map(neighbors,u);
%             %alt(~neighbors)=inf;
%             bUpdate = (alt(neighbors) < dist(neighbors));
%             bUpdate = neighbors(bUpdate);
%             dist(bUpdate) = alt(bUpdate);
%             prev(bUpdate) = u;
%         end

        if(~isempty(neighbors))
            dist_u = dist(u); %Get cost of lin ind
            current_loc = [a(u), b(u), c(u)]; %Geth XYZ [m] from linear index ind2sub substitute
            neigh_loc = [a(neighbors),b(neighbors),c(neighbors)];%XYZ Neigh [m]
            
            
            
            cost = sqrt(sum((bsxfun(@minus, neigh_loc,current_loc)).^2,2)');
            
            
            heur = 0;%Euclidean Distance
            alt(neighbors) = dist_u + cost;%dist_u + map(neighbors,u);
            
            
            
            alt_heur(neighbors) = alt(neighbors) + heur';
            %alt(~neighbors)=inf;
            bUpdate = (alt(neighbors) < dist(neighbors));
            
            
            bUpdate = neighbors(bUpdate);
            
            dist(bUpdate) = alt(bUpdate);
            prev(bUpdate) = u;
            
            bUpdate = (alt_heur(neighbors) < dist_heur(neighbors));
            bUpdate = neighbors(bUpdate);
            dist_heur(bUpdate) = alt_heur(bUpdate);
            
        end

        
        dist_heur(u) = NaN;
    end
    
else
    %%A*
    for i=1:num_nodes
        num_expanded = num_expanded +1;
        %u := vertex in Q with min dist[u]
        %dist_left(scanned) = NaN; %We use Nan so min ignores the nodes we have been to!
        [~,u1] = min(dist_heur(:)); %Get minimum unvisited node
        u = (u1);
        
        scanned(u) = 0; %Remove u from Q
        if(scanned(goal_i) == 0)
            break;
        end
        
        neighbors = getNeighbors(u, occp_grid, neigh_offset , num_nodes, map_sizes, Cols, Rows, W);  %Gets list of neighbors
        
        
        if(~isempty(neighbors))
            dist_u = dist(u);
            current_loc = [a(u), b(u), c(u)];
            neigh_loc = [a(neighbors),b(neighbors),c(neighbors)];
            cost = sqrt(sum((bsxfun(@minus, neigh_loc,current_loc)).^2,2)');
            heur = sqrt(sum((bsxfun(@minus, neigh_loc,goal)).^2,2)');%Euclidean Distance
            alt(neighbors) = dist_u + cost;%dist_u + map(neighbors,u);
            alt_heur(neighbors) = alt(neighbors) + heur';
            %alt(~neighbors)=inf;
            bUpdate = (alt(neighbors) < dist(neighbors));
            bUpdate = neighbors(bUpdate);
            dist(bUpdate) = alt(bUpdate);
            prev(bUpdate) = u;
            
            bUpdate = (alt_heur(neighbors) < dist_heur(neighbors));
            bUpdate = neighbors(bUpdate);
            dist_heur(bUpdate) = alt_heur(bUpdate);
            
        end
        
        dist_heur(u) = NaN;
    end
    
end



path = constructPath(prev,start_i,goal_i);

[a,b,c] = ind2sub(map_sizes, path);

if(~isempty(path))
    %Convert from occupancy grid to XYZ coords
    a = (a-1)*xy_res + 0.5*xy_res + bound_xi; 
    b = (b-1)*xy_res + 0.5*xy_res + bound_yi;
    c = (c-1)*z_res + 0.5*z_res + bound_zi;
    
    a(a>bound_xf) = bound_xf;
    a(a<bound_xi) = bound_xi;
    
    b(b>bound_yf) = bound_yf;
    b(b<bound_yi) = bound_yi;
   
    c(c>bound_zf) = bound_zf;
    c(c<bound_zi) = bound_zi;
    
   path = [a,b,c];
   path(1,:) = start;
   path(end,:) = goal; 
end


cost = dist(goal_i);

%%%End code HW0%%%


end













%function getNeighbors(voxel, map)
%This function returns the neighbors of a node on a given occupancy grid.
function neighbors = getNeighbors(voxel, occp_grid, neigh_offset, max_ind, sz, Cols, Rows, W)




% Get neighbors indices. I based this on the ideas presented at
%http://blogs.mathworks.com/steve/2007/03/28/neighbor-indexing/
%I assume the neigh matrix, with the indices offset of the neighbors, is
%already made!


%[x,y,z] = ind2sub(sz, voxel);
x = Rows(voxel);
y = Cols(voxel);
z = W(voxel);

nx = neigh_offset(:,1) + x;
ny = neigh_offset(:,2) + y;
nz = neigh_offset(:,3) + z;

out = nx <= 0 | nx > sz(1) | ny <= 0 | ny > sz(2) | nz <= 0 | nz > sz(3);

nx(out) = x;
ny(out) = y;
nz(out) = z;



%neigh_ind = voxel + neigh_offset;

%Check out of bounds indices

%neigh_ind = sub2ind(sz, nx, ny, nz);

neigh_ind = (nz-1).*(sz(1)*sz(2))+(ny-1).*(sz(1))+nx;

%Get only indices that are not u
valid_neighbors = neigh_ind(neigh_ind ~= voxel);
neighbors = valid_neighbors(occp_grid(valid_neighbors) ~= 1);

end




%function constructPath(prev,start,goal);
%This function constructs the part from start to goal given an Nx1 vector 
%that states the predecesor of a node on a path.
function path = constructPath(prev,start,goal)

path = [goal];
current = path(end);
previous = -1;
while(previous ~= start && previous ~= 0 && previous ~= current )
    current = path(end);
    previous = prev(current);
    path = [path; previous];
    
end
if(previous == 0 || previous == current)
    %No path!
    path = [];
    return;
end
path = flipud(path);


end

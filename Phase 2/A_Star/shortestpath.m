function [path, cost] = shortestpath(Graph, start, goal)
% SHORTESTPATH Find the shortest path from start to goal on the given Graph.
%   PATH = SHORTESTPATH(Graph, start, goal) returns an M-by-1 matrix, where each row
%   consists of the node on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-1 matrix.
%
%   [PATH, COST] = SHORTESTPATH(...) returns the path as well as
%   the total cost of the path.
%   Arguments:
%      Graph is a N-by-3 matrix, representing all edges in the given graph.
%      Each row has the format of [U, V, W] where U and V are two nodes that
%      this edge connects and W is the weight(cost) of traveling through
%      the edge. For example, [1 5 20.2] means this edge connects
%      node 1 and 5 with cost 20.2. Please note that all edges are
%      undirected and you can assume that the nodes in Graph are indexed by
%      1, ..., n where n is the number of nodes in the Graph.
%

% Hint: You may consider constructing a different graph structure to speed up
% you code.


%% Map Creation (Adjecency)

%First, we create an sparse adjacency  matrix to store the node and path
%information

if(isempty(Graph))
    path = [];
    cost = inf; 
    return
end
[map] = createAdjacencyMatrix(Graph);



%I am basing my algorithm on what is shown on
%http://en.wikipedia.org/wiki/Dijkstra's_algorithm#Using_a_priority_queue


%% Variable Declaration

num_nodes = size(map,1);
%pQueue = PriorityQueue(num_nodes); %Create a priority queue based on #nodes
dist = ones(1,num_nodes).*inf;
alt = ones(1,num_nodes).*inf;
prev = zeros(1,num_nodes);
scanned = logical(ones(1,num_nodes));

%% Initialization
dist(start) = 0;

nod = logical(map');
bUpdate = false(size(alt)); % allocate the vector
%nod.*a;

%Try #2: Vectorizing
for i=1:num_nodes
    %u := vertex in Q with min dist[u]
%     [dist_ind] = find(Q(:,2) == 0)'; %ind of who is left  
%     dist_left = dist(dist_ind); %Get only unvisited nodes
%     [~,u1] = min(dist_left); %Get minimum unvisited node
%     u = dist_ind(u1);
    
    
    
    %dist_left(scanned) = NaN; %We use Nan so min ignores the nodes we have been to!
    [~,u1] = min(dist); %Get minimum unvisited node
    u = (u1);
    
    
    
    scanned(u) = 0; %Remove u from Q
    if(scanned(goal) == 0)
        break;
    end
    
    %neighbors = (getNeighbors(edges, map, u,num_nodes))';  %Gets list of neighbors
    neighbors = (nod(:,u));
   
    if(~isempty(neighbors))
        dist_u = dist(u);
        alt(neighbors) = dist_u + map(neighbors,u);
        bUpdate = (alt(neighbors) < dist(neighbors));
        ne = find(neighbors);
        bUpdate = ne(bUpdate);
        dist(bUpdate) = alt(bUpdate);
        prev(bUpdate) = u;
    end
    
    dist(u) = NaN;

end





%Try #1: I tried using a priority queue, but it was too slow! Now let's try
%vectorizing...
% %Add al nodes to queue with infinite distance (except start node!)
% 
% for i=1:num_nodes
%     pQueue.insert([dist(i),i]);
%     
% end
% 
% 
% 
% while(~pQueue.empty())
%     minNode = pQueue.removeMin();   % Remove and return best vertex
%     u = minNode(2);                 % Get node
%     scanned(minNode(2)) = 1;        % mark u as scanned
%     
%     if(scanned(goal) == 1)
%         break;
%     end
%     
%     neighbors = getNeighbors(map,u,num_nodes);  %Gets list of neighbors
%     neighbor_num = size(neighbors,2);
%     
%     for i=1:neighbor_num           % for each neighbor v of u:
%         current_n = neighbors(i);
%         if(scanned(current_n) == 0) % if v is not yet scanned:
%             alt = dist(u) + map(current_n,u);
%             if(alt < dist(current_n))
%                 dist(current_n) = alt;
%                 prev(current_n) = u;
%                 pQueue.decreasePriority(current_n,alt);
%             end
%         end
%     end
%     
%     
% end



path = constructPath(prev,start,goal);
cost = dist(goal);
end








%function createAdjacencyMatrix(Graph)
%Create an adjecency matrix from a supplied list of vertex-to-vertex
%information in the following form:
%      Input: "Graph" is a N-by-3 matrix, representing all edges in the given graph.
%      Each row has the format of [U, V, W] where U and V are two nodes that
%      this edge connects and W is the weight(cost) of traveling through
%      the edge. For example, [1 5 20.2] means this edge connects
%      node 1 and 5 with cost 20.2. Please note that all edges are
%      undirected and you can assume that the nodes in Graph are indexed by
%      1, ..., n where n is the number of nodes in the Graph.

function [adjMatrix] = createAdjacencyMatrix(Graph)

%First, create the matrix. To do this, we need to find the maximum
%number of nodes. We do this by finding the maximum number on columns
%#1 and #2 of the Graph Nx3 Matrix.

num_nodes = max(max(Graph(:,1:2)));

%The following is done to add each element on thei diagonal, since the
%paths are undirected.



A = [Graph(:,1:3);[fliplr(Graph(:,1:2)),Graph(:,3)]];
[uGraph,ind] = unique(A(:,1:2),'rows');
duplicate_ind = setdiff(1:size(A, 1), ind);
duplicate_ind = duplicate_ind';
top = size(duplicate_ind,1);
val = A(ind,3);


x_index = [uGraph(:,1);uGraph(:,2)];
y_index = [uGraph(:,2);uGraph(:,1)];
value = [val/2;val/2];

adjMatrix = sparse(x_index, y_index, value, num_nodes, num_nodes);




if(~isempty(duplicate_ind))
    rGraph = A(duplicate_ind,:);
    
    for i=1:top
        x = rGraph(i,1);
        y = rGraph(i,2);
        c = rGraph(i,3);
        if(adjMatrix(x,y) > c)
            adjMatrix(x,y) = c;
            adjMatrix(y,x) = c;
        end
    end
    
end
% [r,c] = find(adjMatrix);
% edges = [r,c];



end

%function getNeighbors(map,node)
%This function returns the neighbors of a node o a given sparse matrix
%adjecency map.
function neighbors = getNeighbors(edges,graph, node, num_node)
% c=(edges(fliplr(edges == node)));
neighbors = logical(graph(node,:));
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

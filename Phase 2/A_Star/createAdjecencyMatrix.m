
function adjMatrix = createAdjecencyMatrix(Graph)

%First, create the matrix. To do this, we need to find the maximum
%number of nodes. We do this by finding the maximum number on columns
%#1 and #2 of the Graph Nx3 Matrix.

num_nodes = max(max(Graph(:,1:2)));
%The following is done to add each element on thei diagonal, since the
%paths are undirected.
x_index = [Graph(:,1);Graph(:,2)];
y_index = [Graph(:,2);Graph(:,1)];
value = [Graph(:,3);Graph(:,3)];


adjMatrix = sparse(x_index, y_index, value, num_nodes, num_nodes);




end
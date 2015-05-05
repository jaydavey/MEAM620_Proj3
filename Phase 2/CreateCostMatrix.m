function [ D ] = CreateCostMatrix( start, goals, prev )
%CREATECOSTMATRIX Creates the cost matrix for N robots to M goals given the
%result of Dijkstra's algorithm.

num_robots = size(start,1);
num_goals = size(goals,1);

D = zeros(num_robots, num_goals);

for i=1:num_robots
    for j=1:num_goals
        
    %Get cost of robot i going to goal j
    cost = GetCost(start(i,:), goals(j,:), prev);
    
    %Store cost
    D(i,j) = cost;
        
    end
end


end



function [c] = GetCost(start, goal, prev)


c = 0;

start_ind = sub2ind(size(prev), start(1), start(2));
goal_ind = sub2ind(size(prev), goal(1), goal(2));

previous = goal_ind;
while(previous ~= 0 && previous ~= start_ind)
    %Get Current node
    current = prev(previous);
    %Distance between previous and current
    [xp,yp] = ind2sub(size(prev),previous);
    [xc,yc] = ind2sub(size(prev),current);
    dist = sqrt((xp-xc)^2 + (yp-yc)^2);
    c = c + dist;
    previous = current;

end

if(previous == 0)
   c = inf; 
end


end
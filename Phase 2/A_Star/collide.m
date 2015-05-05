function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.

%For this function I assume that the map is the same output variable as my
%load_map.m, which has the foolowing structure:
% * It is a cell with two fields. The first oneis the occupancy grid, the
% other one is the x,y,z of the vertices of the obstacles with margin
% included. The first row is the boundary.

%To be inside, it must comply with the following laws:
%Its X (or Y or Z) coordinate must be between Xi and Xf.

%If we find points where this is true for each coordinate axis then we have
%found points inside it.
C = logical(zeros(size(points,1),1));
cor = map{2};
xi = cor(:,1);
yi = cor(:,2);
zi = cor(:,3);
xf = cor(:,4);
yf = cor(:,5);
zf = cor(:,6);


in_range = zeros(size(points,1), size(points,2));

for j=1:size(points,1)
    for i=2:size(xf)
        if(points(j,1) >= xi(i) && points(j,1) <= xf(i) &&...
                points(j,2) >= yi(i) && points(j,2) <= yf(i) && ...
                points(j,3) >= zi(i) && points(j,3) <= zf(i))
            C(j) = true;
        end
    end
end
end

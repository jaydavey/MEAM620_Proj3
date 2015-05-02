function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.


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


xi = map{2}(:,1); %yi, zi, xf, yf, zf, r, g, b];
yi = map{2}(:,2);
zi = map{2}(:,3);
xf = map{2}(:,4);
yf = map{2}(:,5);
zf = map{2}(:,6);
r  = map{2}(:,7);
g  = map{2}(:,8);
b  = map{2}(:,9);

%Plot boundary
figure(1);
hold on;
axis equal;
%Left Face
plot3([bound_xi,bound_xi,bound_xi,bound_xi,bound_xi],[bound_yi, bound_yi, bound_yf, bound_yf,bound_yi ],[bound_zi, bound_zf, bound_zf, bound_zi, bound_zi]);
hold on;
%Right Face
plot3([bound_xf,bound_xf,bound_xf,bound_xf,bound_xf],[bound_yi, bound_yi, bound_yf, bound_yf,bound_yi ],[bound_zi, bound_zf, bound_zf, bound_zi, bound_zi]);
%Bottom Face
plot3([bound_xi,bound_xi,bound_xf,bound_xf,bound_xi],[bound_yi, bound_yi, bound_yi, bound_yi,bound_yi ],[bound_zi, bound_zf, bound_zf, bound_zi, bound_zi]);
%Top Face
plot3([bound_xi,bound_xi,bound_xf,bound_xf,bound_xi],[bound_yf, bound_yf, bound_yf, bound_yf,bound_yf ],[bound_zi, bound_zf, bound_zf, bound_zi, bound_zi]);

%draw

for i=2 : size(xi)
   x1 = xi(i);
   x2 = xf(i);
   y1 = yi(i);
   y2 = yf(i);
   z1 = zi(i);
   z2 = zf(i);
   r1 = r(i)/255;
   g1 = g(i)/255;
   b1 = b(i)/255;
    
   %Left face
   patch([x1,x1,x1,x1],[y1 y1 y2 y2],[z1 z2 z2 z1], [r1, g1, b1]); 
   %Right face
   patch([x2,x2,x2,x2],[y1 y1 y2 y2],[z1 z2 z2 z1], [r1, g1, b1]);  
   
   %Bottom face
   patch([x1,x1,x2,x2],[y1 y1 y1 y1],[z1 z2 z2 z1], [r1, g1, b1]);
   %Top face
   patch([x1,x1,x2,x2],[y2 y2 y2 y2],[z1 z2 z2 z1], [r1, g1, b1]);  
   
   %Front face
   patch([x1,x1,x2,x2],[y1 y2 y2 y1],[z2 z2 z2 z2], [r1, g1, b1]); 
   %Back face
   patch([x1,x1,x2,x2],[y1 y2 y2 y1],[z1 z1 z1 z1], [r1, g1, b1]);
end


plot3(path(:,1),path(:,2), path(:,3) )
end
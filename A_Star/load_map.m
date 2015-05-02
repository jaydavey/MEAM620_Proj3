function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.


map = cell(3);
%0.) Read the textfile. This function will exclude comment and read the
%data we want. I am assuming that the 1st sentence read will be the
%boundary option, and in general that the files are well-behaved (they have
%no errors in them!).

[type, xi, yi, zi, xf, yf, zf, r, g, b] = textread(filename, ...
'%s %f %f %f %f %f %f %f %f %f', -1, 'commentstyle', 'shell');


%1.) Create sparse matrix with the resolution provided. This will be an XYZ
%grid representation of the envirioment described by 'filename'. For this,
%we need the data on the 'boundary' command.

boundary_ind = strfind(type, 'boundary');
%Get Data
bound_xi = xi(boundary_ind{1});
bound_yi = yi(boundary_ind{1});
bound_zi = zi(boundary_ind{1});
bound_xf = xf(boundary_ind{1});
bound_yf = yf(boundary_ind{1});
bound_zf = zf(boundary_ind{1});

map_size_x = ceil((bound_xf/xy_res) - (bound_xi/xy_res));
map_size_y = ceil((bound_yf/xy_res) - (bound_yi/xy_res));
map_size_z = ceil((bound_zf/z_res) - (bound_zi/z_res));



occp_grid = zeros(map_size_x, map_size_y, map_size_z);


%2.) Now we add the margin of safety to the obstacles. This should be described by
%rows 2 to the end of the vectors. Since the data is organized in
%increasing coordinates (initial coord is less than final coords) we can
%easily take the margin into account by substracting it from the initial
%coords and assing it to the final coords.

xi = xi - margin;
yi = yi - margin;
zi = zi - margin;
xf = xf + margin;
yf = yf + margin;
zf = zf + margin;

xi(xi <= bound_xi) = bound_xi;
xf(xf >= bound_xf) = bound_xf;

yi(yi <= bound_yi) = bound_yi;
yf(yf >= bound_yf) = bound_yf;

zi(zi <= bound_zi) = bound_zi;
zf(zf >= bound_zf) = bound_zf;

%3.) Now we will try to move these box coordinates to fit into our discrete
%grid. For this, we need to see which boxes of our grid are occupied
%complete or partially by the corners of the box.

%To make this easy, we find the closest coordinates of our grid to each
%point, and depending on the direction the box opens we can choose from
%which grid point the obstacle starts.


%For initial coordinates, the obstacle starts from the largest or equal
%coordinate (because the boxes are described from the smallest to the
%largets coordinate). The reverse is used for the final coordinates 
%(smallest or equal).


%Xi, Yi, Zi
%m_xi holds the index of the point in our discrete grid
%v_xi holds the value of the point in our discrete grid
m_xi = xi;
v_xi = xi;
%eq = xi;
m_xi = floor((xi./xy_res) - (bound_xi/xy_res));
v_xi = m_xi.*xy_res + bound_xi; 
%eq = abs(v_xi - xi) <= eps(xi)*4;
%m_xi(~eq) = m_xi(~eq) + 1; %0-index 
m_xi = m_xi + 1; %1-index

m_yi = yi;
v_yi = yi;
m_yi = floor((yi./xy_res) - (bound_yi/xy_res));
v_yi = m_yi.*xy_res + bound_yi; 
%eq = abs(v_yi - yi) <= eps(yi)*4;
%m_yi(~eq) = m_yi(~eq) + 1; %0-index 
m_yi = m_yi + 1; %1-index


m_zi = zi;
v_zi = zi;
m_zi = floor((zi./z_res) - (bound_zi/z_res));
v_zi = m_zi.*z_res + bound_zi; 
%eq = abs(v_zi - zi) <= eps(zi)*4;
%m_zi(~eq) = m_zi(~eq) + 1; %0-index 
m_zi = m_zi + 1; %1-index

%Xf, Yf, Zf
%m_xi holds the index of the point in our discrete grid
%v_xi holds the value of the point in our discrete grid

m_xf = xf;
v_xf = xf;
eq = xf;
m_xf = floor((xf./xy_res) - (bound_xi/xy_res));
v_xf = m_xf.*xy_res + bound_xi; 
eq = abs(v_xf - xf) <= eps(xf)*4;
m_xf(eq) = m_xf(eq) - 1; %0-index 
m_xf = m_xf + 1; %1-index


m_yf = yf;
v_yf = yf;
eq = yf;
m_yf = floor((yf./xy_res) - (bound_yi/xy_res));
v_yf = m_yf.*xy_res + bound_yi; 
eq = abs(v_yf - yf) <= eps(yf)*4;
m_yf(eq) = m_yf(eq) - 1; %0-index 
m_yf = m_yf + 1; %1-index


m_zf = zf;
v_zf = zf;
eq = zf;
m_zf = floor((zf./z_res) - (bound_zi/z_res));
v_zf = m_zf.*z_res + bound_zi; 
eq = abs(v_zf - zf) <= eps(zf)*4;
m_zf(eq) = m_zf(eq) - 1; %0-index 
m_zf = m_zf + 1; %1-index



m_xi(m_xi <= 1) = 1;
m_xf(m_xf >= map_size_x) = map_size_x;

m_yi(m_yi <= 1) = 1;
m_yf(m_yf >= map_size_y) = map_size_y;

m_zi(m_zi <= 1) = 1;
m_zf(m_zf >= map_size_z) = map_size_z;



%4.) Now we add the occupied voxel to the occupancy grid. We go obstacle bu
%obstacle (from 2nd row onwards).


for i=2:size(xf)
    
    occp_grid(m_xi(i):m_xf(i),m_yi(i):m_yf(i), m_zi(i):m_zf(i)) = 1;
    
end



map{1} = occp_grid;
map{2} = [xi, yi, zi, xf, yf, zf, r, g, b];
map{3} = [bound_xi bound_yi bound_zi bound_xf bound_yf bound_zf xy_res z_res margin];
map{4} = [map_size_x, map_size_y, map_size_z];
end



%function roundToGrid(ci,cf)
%This function gets the coordinates of a obstacle and decides to which
%coordinate on a discrete grid it must be assigned to. To determine this,
%we need the initial and enc coordinate (ci and cf), the initial and end
%boundary (init and end) and the resolution of the system.

function roundToGrid(ci, cf,init, dend, res)


end
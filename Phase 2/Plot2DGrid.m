function [  ] = Plot2DGrid( occup_grid , ix, iy)
%PLOT2DGRID Summary of this function goes here
%   Detailed explanation goes here



xf = size(occup_grid,1);
yf = size(occup_grid,2);


%% Draw Grid
x = linspace(ix,xf+1,xf-ix+2)
y = linspace(iy,yf+1,yf-iy+2)

% Horizontal grid 
for k = 1:length(x)
  line([x(1) x(end)], [y(k) y(k)])
end

% Vertical grid
for k = 1:length(y)
  line([x(k) x(k)], [y(1) y(end)])
end

axis square

%% Draw Squares

for (i = 1: xf)
    for (j = 1: yf)
        
        if(occup_grid(i,j) == 1)
            rectangle('Position',[i,j,1,1],...
                'FaceColor','r');
        end
    end
end


end


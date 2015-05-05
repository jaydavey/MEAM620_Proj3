function [] = PlotObjectives( plot_handler, start, goals, xmin,ymin )
%PLOTOBJECTIVES Plots the start and end goal of the robots
%   

%Plot start
for i=1:size(start,1)
   rectangle('Position',[start(i,1),start(i,2),1,1], 'Curvature',[0.2,0.2], 'FaceColor', 'blue') 
   s = strcat('S',int2str(i));
   text(start(i,1)+0.5,start(i,2)+0.5, s);
    
end

%Plot goals
for i=1:size(goals,1)
   rectangle('Position',[goals(i,1),goals(i,2),1,1], 'Curvature',[0.2,0.2], 'FaceColor', 'yellow') 
   s = strcat('G',int2str(i));
   text(goals(i,1)+0.5,goals(i,2)+0.5, s);
    
end



end


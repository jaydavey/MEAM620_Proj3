function [robot_handles, text_handles] = PlotRobots( plot_handler, robot_pos, R, Nr)
%PLOTROBOTS Plots the current position of the robots
%   

a = (1 - 1/sqrt(2))/2;
b = 1/sqrt(2);
c = 1/(2*sqrt(2));

robot_handles = cell(Nr,1);
text_handles = cell(Nr,1);


for(i=1:size(robot_pos,1))
    
   robot_handles{i} = rectangle('Position',[robot_pos(i,1)+ a,robot_pos(i,2)+a,b,b],'Curvature',[1 1]);
   s = strcat('R',int2str(i));
   text_handles{i} = text(robot_pos(i,1)+0.5,robot_pos(i,2)+0.5, s);
    
end




end


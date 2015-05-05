function [] = UpdateRobotPlot(t, i, robot_vector, robot_vector_dist, robot_time_traj, robot_indv_time,robot_paths_xy, robot_handles, text_handles )
% Updates the plot of the robot

a = (1 - 1/sqrt(2))/2;
b = 1/sqrt(2);
c = 1/(2*sqrt(2));
x = 1;
y = 1;

%First we need to determine in which path of the robot we are on!

num_path = GetNumPathRobot(t, robot_time_traj);
num_elem = size(robot_paths_xy,1);

if(num_path == 0 || num_elem == 1)
    x = robot_paths_xy(1,1);
    y = robot_paths_xy(1,2);
else if(num_path == inf)
        x = robot_paths_xy(end,1);
        y = robot_paths_xy(end,2);
    else
        
        x = robot_paths_xy(num_path,1) + (robot_vector(num_path,1)/robot_indv_time(num_path))*(t-robot_time_traj(num_path)); %robot_time_traj has one more argument because it begins with initial time!
        y = robot_paths_xy(num_path,2) + (robot_vector(num_path,2)/robot_indv_time(num_path))*(t-robot_time_traj(num_path));
        
    end
end

   set(robot_handles,'Position',[x+ a,y+a,b,b],'Curvature',[1 1], 'FaceColor', 'green');
   s = strcat('R',int2str(i));
   set(text_handles, 'Position', [x+0.5,y+0.5]);




end



function [numPath] = GetNumPathRobot(t, robot_time_traj)

num_of_paths = size(robot_time_traj,1);

if(t <= robot_time_traj(1))
    numPath = 0;
    return;
end

for(i=1:num_of_paths-1)
    if(t >= robot_time_traj(i) && t < robot_time_traj(i+1) )
        numPath = i;
        break;
    end
end

if(t >= robot_time_traj(end))
    numPath = inf;
end

end

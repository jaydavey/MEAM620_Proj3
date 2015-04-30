function [ Start, Goals ] = Generate2DPoints( num_start, num_goals, max_x, max_y, delta )
%GENERATE2DPOINTS Generates the start and end points for a 2D map following
%the rules of the paper CAPT: Concurrent assignment and planning of trajectories for multiple
%robots
%the
%

%We must satisfy
%||xi(to) - xj(to)|| > delta
%||gi - gj|| > delta
%||xi(to) - gj|| > delta (if num_goals > num_start)

%At first I was going to perfectly generate random stuff, but found it too
%complex. It is easier to create a grid of positions we know are ok, and
%just randomly grab from this bag of positions that are known to be ok!



[x_pos,y_pos] = meshgrid(0:delta+eps:max_x,0:delta+eps:max_y);


num_points = size(x_pos,1)*size(y_pos,2);
if(num_points < num_start + num_goals) % Should realistically never happen! We should not have so much points that they don't fit in our system!
    [x_pos,y_pos] = meshgrid(0:delta+eps:(num_start + num_goals)*(delta+eps),0:delta+eps:(num_start + num_goals)*(delta+eps));
    num_points = size(x_pos,1)*size(y_pos,2);
end


p = randperm(num_points,num_start + num_goals); %Grab num_start + num_goals random points from num_points

p_start = p(1:num_start);
p_goal = p(num_start+1:end);

Start = [x_pos(p_start)',y_pos(p_start)'];
Goals = [x_pos(p_goal)',y_pos(p_goal)'];
    
   
    
end

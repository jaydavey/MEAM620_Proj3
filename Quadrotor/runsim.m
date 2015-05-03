% NOTE: This srcipt will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** MEAM 620 QUADROTOR SIMULATION *****************
close all
%clear all
addpath('utils')
addpath('trajectories')

% You need to implement trajhandle and controlhandle




% trajectory generator
trajhandle = @trajectory_generator;

% controller
controlhandle = @controller;

% real-time 
real_time = true;



% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********
% number of quadrotors
%nquad = 4; Now defined on CAPT_Phase_QUADROTOR.M

% max time
time_tol = 30;

% parameters for simulation
params = nanoplus();

%% **************************** FIGURES *****************************
fprintf('Initializing figures...\n')
h_fig = figure;
h_3d = gca;
axis equal
grid on
view(3);
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);

set(gcf,'Renderer','OpenGL')




    %we're doing it in 3D
    bound = [xmin xmax ymin ymax zmin*4 zmax*4]; %Change of variable for quadrotor, here just for ploting reasons
    axis(bound);
    axis equal
grid on
hold on
% make robots little spheres
[Sx,Sy,Sz] = sphere();
R_draw=0.04;
colorS = [1 0 0]; % Start point color (red)
for i = 1:Ns
    s(i) = surf(S(i,1)+Sx*R_draw,S(i,2)+Sy*R_draw,S(i,3)+Sz*R_draw);
    set(s(i),'EdgeAlpha',0,'FaceAlpha',0.3,'FaceColor', colorS);
    str = strcat('S',num2str(i));
    th = text(S(i,1),S(i,2),S(i,3),str);
    set(th,'FontSize',8,'FontWeight','bold');
end
    
% make goals spheres to ensure we don't have collisions at the end 
colorG = [0 0 1]; % Goal color (green)
for i = 1:Ng
    g(i) = surf(G(i,1)+Sx*R_draw,G(i,2)+Sy*R_draw,G(i,3)+Sz*R_draw);
    set(g(i),'EdgeAlpha',0,'FaceAlpha',0.3,'FaceColor', colorG);
    str = strcat('G',num2str(i));
    th = text(G(i,1),G(i,2),G(i,3),str);
    set(th,'FontSize',8,'FontWeight','bold');
end


%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
max_iter  = 500;      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
nstep     = cstep/tstep;
time      = starttime; % current time
err = []; % runtime errors
for qn = 1:nquad
    % Get start and stop position
    des_start = trajhandle(0, qn);
    des_stop  = trajhandle(inf, qn);
    stop{qn}  = des_stop.pos;
    x0{qn}    = init_state( des_start.pos );
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

x         = x0;        % state

pos_tol   = 0.01;
vel_tol   = 0.01;

writerObj = VideoWriter('B_l10.avi');
open(writerObj);



%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....')
% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;

    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.2, 0.3, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time, qn);
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end

        % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn}    = xsave(end, :)';
        
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

        % Update quad plot
        desired_state = trajhandle(time + cstep, qn);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
        set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    end
    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> inf*cstep*50000)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Get out if trayectory time is done. Set on CAPT_Phase_Quadrotor
    if (time > time_exit)
        break;
    end

    
    % Check termination criteria
    if terminate_check(x, time, stop, pos_tol, vel_tol, time_tol)
        break
    end
    
    %Get Movie Frame
    frame = getframe(gcf);
    writeVideo(writerObj,frame);
    set(gca,'nextplot','replacechildren');
end




fps = 20; sec = 15;
for i=1:fps*sec
    iter = iter + 1;
    camorbit(0.9,-0.1);
    drawnow;
    frame = getframe(gcf);
    writeVideo(writerObj,frame);
    set(gca,'nextplot','replacechildren');
end
close(writerObj);

% %% ************************* POST PROCESSING *************************
% % Truncate xtraj and ttraj
% for qn = 1:nquad
%     xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
%     ttraj{qn} = ttraj{qn}(1:iter*nstep);
% end
% 
% % Plot the saved position and velocity of each robot
% for qn = 1:nquad
%     % Truncate saved variables
%     QP{qn}.TruncateHist();
%     % Plot position for each quad
%     h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
%     plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
%     plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
%     % Plot velocity for each quad
%     h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
%     plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
%     plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
% end
% if(~isempty(err))
%     error(err);
% end

fprintf('finished.\n')
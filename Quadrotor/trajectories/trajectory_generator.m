function [ desired_state ] = trajectory_generator( t, qn, varargin )
% Trajectory generator for Project 3, Quadrotors

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables

%

%The movement on Z can be described linearly as:
% Z = A*t

%PERSISTENT VARIABLES
persistent aa_x calc_arg start goal start2 goal2 aa_x2;



%% Path Variables

if(t == -1)
    if(nargin == 4)
        start = varargin{1};
        goal = varargin{2};
        fprintf('Start and Goal Set...\n');
        desired_state = 0;
        return;
    else  if(nargin == 6)
            start = varargin{1};
            goal = varargin{2};
            fprintf('Start and Goal Set...\n');
            start2 = varargin{3};
            goal2 = varargin{4};
            fprintf('Start2 and Goal2 Set...\n');
            desired_state = 0;
            return;
        end
    end
end





t_total = 10; %13 [s]
freq = 1/t_total;
to = 0;
tf  = t_total;
tf2 = t_total * 2;


%% Generate Path (only 1 time!)


%We define the A matrix

if(isempty(calc_arg))
    calc_arg = 1;
    
    %We define the A matrix
    A = [ 1, to, to^2, to^3, to^4, to^5, to^6, to^7; %Initial Position T0
        1, tf, tf^2, tf^3, tf^4, tf^5, tf^6, tf^7; %Final Position TF
        
        0, 1, 2*to, 3*to^2, 4*to^3, 5*to^4, 6*to^5, 7*to^6; %Initial Vel T0
        0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4, 6*tf^5, 7*tf^6; %Final Vel TF
        
        
        
        0, 0, 2, 6*to^1, 12*to^2, 20*to^3, 30*to^4, 42*to^5; %Initial Acc T0
        0, 0, 2, 6*tf^1, 12*tf^2, 20*tf^3, 30*tf^4, 42*tf^5; %Final Acc TF
        
        
        0, 0, 0, 6, 24*to^1, 60*to^2, 120*to^3, 210*to^4; %Initial Jerk T0
        0, 0, 0, 6, 24*tf^1, 60*tf^2, 120*tf^3, 210*tf^4; %Final Jerk TF
        
        ];
    
    
    to;
    %We define a B! It is parametrized to the line! 0=start, 1=end!
    Bx = [0 1 0 0 0 0 0 0]';%[xo xf xdot0 xdotf xddot0 xddotf xjerk0 xjerk1]';
    
    %Solve
    
    aa_x = mldivide(A,Bx); % Ax = b
    
    
    if(~isempty(start2))
        %We define the A2 matrix
        A2 = [ 1, tf, tf^2, tf^3, tf^4, tf^5, tf^6, tf^7; %Initial Position T0
            1, tf2, tf2^2, tf2^3, tf2^4, tf2^5, tf2^6, tf2^7; %Final Position TF
            
            0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4, 6*tf^5, 7*tf^6; %Initial Vel T0
            0, 1, 2*tf2, 3*tf2^2, 4*tf2^3, 5*tf2^4, 6*tf2^5, 7*tf2^6; %Final Vel TF
            
            
            
            0, 0, 2, 6*tf^1, 12*tf^2, 20*tf^3, 30*tf^4, 42*tf^5; %Initial Acc T0
            0, 0, 2, 6*tf2^1, 12*tf2^2, 20*tf2^3, 30*tf2^4, 42*tf2^5; %Final Acc TF
            
            
            0, 0, 0, 6, 24*tf^1, 60*tf^2, 120*tf^3, 210*tf^4; %Initial Jerk T0
            0, 0, 0, 6, 24*tf2^1, 60*tf2^2, 120*tf2^3, 210*tf2^4; %Final Jerk TF
            
            ];
        
        
        to;
        %We define a B! It is parametrized to the line! 0=start, 1=end!
        Bx2 = [0 1 0 0 0 0 0 0]';%[xo xf xdot0 xdotf xddot0 xddotf xjerk0 xjerk1]';
        
        %Solve
        
        aa_x2 = mldivide(A2,Bx2); % Ax = b
    end
end

%% Calculate Path
if(t<=0)
    x = start(qn,1);
    y = start(qn,2);
    z = start(qn,3);
    vel_x = 0;
    vel_y = 0;
    vel_z = 0;
    ac_x = 0;
    ac_y = 0;
    ac_z = 0;
else if(t > t_total)
        
        if(~isempty(start2))
            if(t>tf2)
                x = goal2(qn,1);
                y = goal2(qn,2);
                z = goal2(qn,3);
                vel_x = 0;
                vel_y = 0;
                vel_z = 0;
                ac_x = 0;
                ac_y = 0;
                ac_z = 0;
            else
                
                pos_l = aa_x2'*[1 t t^2  t^3     t^4      t^5        t^6      t^7]'; %1, to, to^2, to^3, to^4, to^5, to^6, to^7;
                vel_l = aa_x2'*[0 1 2*t  3*t^2   4*t^3    5*t^4      6*t^5    7*t^6]';
                acc_l = aa_x2'*[0 0 2    6*t     12*t^2   20*t^3     30*t^4   42*t^5]';
                
                line_unit = (goal2(qn,:) - start2(qn,:));
                line_unit_vec = line_unit./norm(line_unit);
                
                pos_v = line_unit * pos_l + start2(qn,:);
                vel_v = line_unit * vel_l;
                acc_v = line_unit * acc_l;

                x = pos_v(1);
                y = pos_v(2);
                z = pos_v(3);
                vel_x = vel_v(1);
                vel_y = vel_v(2);
                vel_z = vel_v(3);
                ac_x = acc_v(1);
                ac_y = acc_v(2);
                ac_z = acc_v(3);
            end
            
            
            
        else
            x = goal(qn,1);
            y = goal(qn,2);
            z = goal(qn,3);
            vel_x = 0;
            vel_y = 0;
            vel_z = 0;
            ac_x = 0;
            ac_y = 0;
            ac_z = 0;
            
        end
    else
        
        pos_l = aa_x'*[1 t t^2  t^3     t^4      t^5        t^6      t^7]'; %1, to, to^2, to^3, to^4, to^5, to^6, to^7;
        vel_l = aa_x'*[0 1 2*t  3*t^2   4*t^3    5*t^4      6*t^5    7*t^6]';
        acc_l = aa_x'*[0 0 2    6*t     12*t^2   20*t^3     30*t^4   42*t^5]';
        
        line_unit = (goal(qn,:) - start(qn,:));
        line_unit_vec = line_unit./norm(line_unit);
        
        pos_v = line_unit * pos_l + start(qn,:);
        vel_v = line_unit * vel_l;
        acc_v = line_unit * acc_l;
        
        
        
        x = pos_v(1);
        y = pos_v(2);
        z = pos_v(3);
        vel_x = vel_v(1);
        vel_y = vel_v(2);
        vel_z = vel_v(3);
        ac_x = acc_v(1);
        ac_y = acc_v(2);
        ac_z = acc_v(3);
        
        
    end
    
end
pos = [x; y; z];
vel = [vel_x; vel_y; vel_z];
acc = [ac_x; ac_y; ac_z];
yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================


desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end



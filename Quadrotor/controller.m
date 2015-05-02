function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================



%%Control Parameters
kdx = 5;
kpx = 10;
kdy = 5;
kpy = 10;

kdz = 30;
kpz = 80;


kd_phi = 0.025;
kp_phi = 0.4;

kd_theta = 0.025;
kp_theta =  0.4;

kd_psi = 0.1;
kp_psi = 1;



%Get Nano parameters

m = params.mass;
g = 9.81; %[m/s^2]

Ixx = params.I(1,1);
Iyy = params.I(2,2);
Izz = params.I(3,3);

%Get Current states
x       = qd{qn}.pos(1);
xdot    = qd{qn}.vel(1);

y       = qd{qn}.pos(2);
ydot    = qd{qn}.vel(2);

z       = qd{qn}.pos(3);
zdot    = qd{qn}.vel(3);

phi     = qd{qn}.euler(1);
theta   = qd{qn}.euler(2);
psi     = qd{qn}.euler(3);

p = qd{qn}.omega(1);
q = qd{qn}.omega(2);
r = qd{qn}.omega(3);


%Get desired states

xdesddot    = qd{qn}.acc_des(1);
xdesdot     = qd{qn}.vel_des(1);
xdes        = qd{qn}.pos_des(1);

ydesddot    = qd{qn}.acc_des(2);
ydesdot     = qd{qn}.vel_des(2);
ydes        = qd{qn}.pos_des(2);

zdesddot  = qd{qn}.acc_des(3);
zdesdot   = qd{qn}.vel_des(3);
zdes    = qd{qn}.pos_des(3);

yaw_des = qd{qn}.yaw_des;
yaw_desdot = qd{qn}.yawdot_des;


% Desired roll, pitch and yaw
phi_des = 0;
theta_des = 0;
psi_des = 0;


%Get commanded states
xcomm_ddot = xdesddot + kdx*(xdesdot - xdot) + kpx*(xdes - x);
ycomm_ddot = ydesddot + kdy*(ydesdot - ydot) + kpy*(ydes - y);

phi_comm    = (1/g) * (xcomm_ddot * sin(yaw_des) - ycomm_ddot * cos(yaw_des));
theta_comm  = (1/g) * (xcomm_ddot * cos(yaw_des) + ycomm_ddot * sin(yaw_des));





%Get u1 and u2
u1 = m*g + m *(zdesddot + kdz * (zdesdot - zdot) + kpz*(zdes - z));

u2_1 = (kp_phi*(phi_comm - phi) + kd_phi*(0 - p));
u2_2 = (kp_theta*(theta_comm - theta) + kd_theta*(0 - q));
u2_3 = (kp_psi*(yaw_des - psi) + kd_psi*(yaw_desdot-r)); %Yaw does not matter!

u2 = [u2_1;u2_2;u2_3];

% Thurst
F    = u1;

if (F > params.maxF)
   F = params.maxF; 
end

if (F < params.minF)
   F = params.minF; 
end




% Moment
M    = u2; % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end

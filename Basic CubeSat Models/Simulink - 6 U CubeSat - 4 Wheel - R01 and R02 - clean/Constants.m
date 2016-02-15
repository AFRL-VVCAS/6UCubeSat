%% 6U CubeSat Attitude Simulator and RWA script
% This is the constants file for a 6U Cubesat with a RWA in which 4
% wheels are arranged in a pyramid structure.
% 14 Aug 2015
clear all;
close all;
clc;

%% Simulation Properties
delta_t = 0.1; % [s] time step
t = 0;         % [s] initial time
fprintf('\nThe rate of the controller is %g Hz',delta_t);
tf = 200;        % [s] final time of the simulation
steps = tf/delta_t; % total number of time steps for the simulation

%% CubeSat Properties
fprintf('\n6U CubeSat Properties:');
mass_sat = 6; % From MECH 632 --> Acutal Mass of a 6U cubesat may be 8 Kg?
fprintf('\n    Mass =  %g kg',mass_sat);
% Moment of Inertia in principal body frame fo the s/c
Ixx = 1/12*mass_sat*(0.1^2+0.3^2); % [kg-m^2] along body frame x axis
Iyy = 1/12*mass_sat*(0.2^2+0.3^2); % [kg-m^2] along body frame y axis
Izz = 1/12*mass_sat*(0.1^2+0.2^2);% [kg-m^2] along body frame z axis
I = [Ixx 0 0; 0 Iyy 0; 0 0 Izz];   % [kg-m^2] Inertial matrix
Inv_I = inv(I);  % Inverse of Inertia matrix - compute once here
fprintf('\n    Ixx =  %g kg-m^2',Ixx);
fprintf('\n    Iyy =  %g kg-m^2',Ixx);
fprintf('\n    Izz =  %g kg-m^2',Ixx);

%%  Reaction Wheels and Motor Properties
% 3 wheel RWA--assume RWA is aligned with body axis
D = 9.07E-6; % MOI of one wheel (kg-m^2)
fprintf('\nSingle Reaction Wheel Properties:');
fprintf('\n    Moment of Inertia (D) =  %g kg-m^2',D);
max_rpm = 1000; % max rev/min of motor - (rpm)
fprintf('\nRW Maximum Angular Velocity =  %g rpm',max_rpm);
psi_max = max_rpm/60*2*pi % max angular velocity (rad/s)
fprintf('\nRW Maximum Angular Velocity =  %g rad/s',psi_max);
motor_torque = 1*1E-3; %(hdot_max) torque from motor - (N-m)
psi_dot_max = motor_torque/D % max angular acceleration of wheel (rad/s^2)
fprintf('\nRW Maximum Angular Acceleration =  %g rad/s^2',psi_dot_max);
% orientation of reaction wheels in 4 wheel RWA
alpha1 = 315*pi/180;
alpha2 = 45*pi/180;
alpha3 = 135*pi/180;
alpha4 = 225*pi/180;
beta = 45*pi/180;
%Calculate these values only once
ca1 = cos(alpha1);
ca2 = cos(alpha2);
ca3 = cos(alpha3);
ca4 = cos(alpha4);
sa1 = sin(alpha1);
sa2 = sin(alpha2);
sa3 = sin(alpha3);
sa4 = sin(alpha4);
cb = cos(beta);
sb = sin(beta);
S =  D*[-cos(alpha1)*sin(beta) -cos(alpha2)*sin(beta) -cos(alpha3)*sin(beta) -cos(alpha4)*sin(beta);...
       sin(alpha1)*sin(beta)  sin(alpha2)*sin(beta)  sin(alpha3)*sin(beta)  sin(alpha4)*sin(beta);...
       cos(beta)              cos(beta)              cos(beta)              cos(beta)];
%psuedoinverse S matrix
S_pinv = S'*inv(S*S');  
% sc_accel = motor_torque/MOI(end,end); % constant angular acceleration of s/c (rad/s^2)
% slope = .104753770724;
% bias = 1325.711503755787;
% slope_dc_to_rpm = 9.50537207196;
% bias_dc_to_rpm = -12591.21684473440;
%% Initial Conditions
% Initial orientation
theta_10 = 0/180*pi;  % rotation about 1-axis
theta_20 = 0/180*pi;  % rotation about 2-axis
theta_30 = 0/180*pi;   % rotation about 3-axis
fprintf('\nInitial orientation of the 6U CubeSat');
fprintf('\n    Theta 1 =  %g deg',theta_10*180/pi);
fprintf('\n    Theta 2 =  %g deg',theta_20*180/pi);
fprintf('\n    Theta 3 =  %g deg',theta_30*180/pi);
R1_0 = [ 1 0 0 ; 0 cos(theta_10) -sin(theta_10) ; 0 sin(theta_10) cos(theta_10) ]; % 1-axis R matrix
R2_0 = [ cos(theta_20) 0 sin(theta_20) ; 0 1 0 ; -sin(theta_20) 0 cos(theta_20)];% 2-axis R matrix
R3_0 = [ cos(theta_30)  -sin(theta_30) 0 ; sin(theta_30) cos(theta_30) 0 ; 0 0 1 ];% 3-axis R matrix
R_0 = transpose(R1_0*R2_0*R3_0);  % Rotation matrix R
[a_0,phi_0,q_initial] = R2Q(R_0);% compute initial quaternions 

% Initial Input Torque for reaction wheels
psi_dot_0 = [0;0;0;0]; 

% Initial External Torques on the S/C
m = [0;0;0]; 

% Initial angular velocity of the reaction wheels
psi_0 = [0;0;0;0];

% Initial spacecraft angular velocity
omega_b_0 = [0;0;0]; %initial angular velocity of the s/c

% Initial State Vector
x0=[q_initial' omega_b_0' psi_0'];% [quaternions(1-4) omegas(5-7) wheelspeeds(8-11) wheelaccels(11-13), externaltoques (14-16)]

% Initial Angular momentums
H_b = [0 0 0]; %set initial angular momentum in the body frame
H_i = [0 0 0]; %set initial angular momentum in the inertial frame
H_rwa = [0 0 0]; %set initial angular momentum in the inertial frame

%% Final Conditions
% Final Orientation
theta_1f = 90/180*pi;  % desired rotation about 1-axis
theta_2f = 45/180*pi;  % desired rotation about 2-axis
theta_3f = -45/180*pi; % desired rotation about 3-axis
fprintf('\nFinal desired orientation of the 6U CubeSat');
fprintf('\n    Theta 1 =  %g deg',theta_1f*180/pi);
fprintf('\n    Theta 2 =  %g deg',theta_2f*180/pi);
fprintf('\n    Theta 3 =  %g deg\n',theta_3f*180/pi);
R1 = [ 1 0 0 ; 0 cos(theta_1f) -sin(theta_1f) ; 0 sin(theta_1f) cos(theta_1f) ]; % 1-axis R matrix
R2 = [ cos(theta_2f) 0 sin(theta_2f) ; 0 1 0 ; -sin(theta_2f) 0 cos(theta_2f)];% 2-axis R matrix
R3 = [ cos(theta_3f)  -sin(theta_3f) 0 ; sin(theta_3f) cos(theta_3f) 0 ; 0 0 1 ];% 3-axis R matrix
R = transpose(R1*R2*R3);  % Rotation matrix R
[a,phi,q_final] = R2Q(R); % compute final quaternions 

M_tilde = [q_final(4) -q_final(3) q_final(2) q_final(1);...
            q_final(3) q_final(4) -q_final(1) q_final(2);...
            -q_final(2) q_final(1) q_final(4) q_final(3);...
            -q_final(1) -q_final(2) -q_final(3) q_final(4)];
q_commanded = M_tilde; %initialize q_commanded
inv_M_tilde = inv(M_tilde);

%% PID control
Kp = 0.1;
Kd = 0.1;
Ki = 0.1;

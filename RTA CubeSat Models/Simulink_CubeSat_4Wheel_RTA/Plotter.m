% Plotter
% AUTHOR: Kerianne Gross 
% Updated: 10 Nov 2015
% Output
% This is a plotting file used to create the following 3 plots:
% 1) Orienation, Angular Velocity, Wheel Angular Velocity, and Wheel
% Angular Acceleration (q, omega, psi, and psi_dot)

close all;
time = t:delta_t:tf;

subplot(221)
plot(quaternions)
legend('q_1','q_2','q_3','q_4','location','northeastoutside')
title('Spacecraft Orientation')
ylabel('Orientation (quaternion)');
xlabel('Time (s)');

subplot(222)
plot(omegas)
legend('\omega_1','\omega_2','\omega_3','location','northeastoutside');
title('Spacecraft Angular Velocity');
ylabel('Angular Velocity (rad/sec)');
xlabel('Time (s)');

subplot(223)
plot(psi)
line([t,tf],[-psi_max,-psi_max])
line([t,tf],[psi_max,psi_max])
legend('\psi_1','\psi_2','\psi_3','\psi_4','max \psi','min \psi','location','northeastoutside')
title('Wheel Angular Velocities')
xlabel('Time (s)');
ylabel('Angular Velocity (rad/sec)');

subplot(224)
plot(psi_dot)
line([t,tf],[-psi_dot_max,-psi_dot_max])
line([t,tf],[psi_dot_max,psi_dot_max])
I = legend('$\dot{\psi}_1$','$\dot{\psi}_2$','$\dot{\psi}_3$','$\dot{\psi}_4$','max $\dot{\psi}$','min $\dot{\psi}$','location','northeastoutside');
set(I,'interpreter','latex')
title('Wheel Angular Accelerations')
ylabel('Angular Acceleration (rad/sec^2)');
xlabel('Time (s)');
%% Dennis Wai
%% 
clear all, clc
cd /Users/dwai/Documents/smc_quadcopter

%% Define system variables
m = .5; %kg
g = 9.81; %m/s2
C = [0 0 0];
Cd = [0 0 0];
l = .25; %m
Ix =1/12*(.4*m)*(2*l)^2; Iy = Ix; Iz = m*(l^2);
Iz = m*(l^2);
I = [Ix Iy Iz]; %kg m2
Kp = [.01 .01 1 .035 .035 60]; %P gain for the three channels
Kd = [.01 .01 .6 .005 .005 0]; %D gain for the three channels
Km = 1; %Reaction Torque Gain
Kf = 100; %Thrust Gain

x0 = [0 0 ... x0, xdot0
      0 0 ...
      .2 0 ...
      0 0 ...
      0 0 ...
      0 0 ...
      ];
step_time = [0 0 0];    
ref = [.1 0 .2 0 0 0]; %x,y,z,phi,th,psi
t_end = 10;
    
%% Simulate PID 
sim pid_quadcopter
disp('Done Simulating')

%% Plot Data
t = h_sim.x.Time;
x = h_sim.x.Data;     
y = h_sim.y.Data;
z = h_sim.z.Data;
phi = h_sim.phi.Data;
th = h_sim.th.Data;
psi = h_sim.psi.Data;

eX = h_err.Data(:,1);     
eY = h_err.Data(:,2);
eZ = h_err.Data(:,3);

w1 = h_rotor.w1.Data;
w2 = h_rotor.w2.Data;
w3 = h_rotor.w3.Data;
w4 = h_rotor.w4.Data;

figure(1), clf
plot(t,[x y z]); hold on; plot(t,.1*sin(1*t),'k--');
legend('x','y','z','Reference'); xlabel('Time'); ylabel('m')
title('PD - States vs Time')

figure(2)
plot(t,Kf*[w1 w2 w3 w4].^2)
legend('F1','F2','F3','F4'); xlabel('Time'); ylabel('Ang Vel')
title('PD - Rotor Thrust')

figure(3)
plot(t,[phi th psi]./pi*180)
legend('\phi','\theta','\psi'); xlabel('Time'); ylabel('Degrees')
title('PD - Body Angles vs Time')

figure(4)
plot(t,[eX eY eZ])
legend('eX','eY','eZ'); xlabel('Time'); ylabel('Error')
title('PD - Error vs Time')

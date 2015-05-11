%% Dennis Wai
%% 
clear all, clc
cd /Users/dwai/Documents/SMC_quadcopter

%% Define system variables
m = .5; %kg
m_nom = m;
g = 9.81; %m/s2
C = [0 0 0]; Cd = [0 0 0];
C_nom = [1 1 1].*C; Cd_nom = [1 1 1].*Cd;
l = .25; %m
Ix =1/12*(.1*m)*(2*l)^2; Iy = Ix; Iz = m*(l^2);
Ix_nom = Ix; Iy_nom = Iy; Iz_nom = Iz;
I = [Ix Iy Iz]; %kg m2
H_nom = diag([m_nom,m_nom,m_nom,Ix_nom,Iy_nom,Iz_nom]);
CC_nom = diag([C_nom,Cd_nom]);

lamb = [5 100 5 5 5 5];
PSI = [.005 .005 .005 .005 .005 .005]';
eta = [1 1 1 1 1 1]';
Km = 1; %Reaction Torque Gain
Kf = 100; %Thrust Gain

x0 = [0 0 ... x0, xdot0
      0 0 ...
      .1 0 ...
      0 0 ...
      0 0 ...
      0 0 ...
      ];
step_time = [0 0 0];    
ref = [0 2 1 0 0 0]; %x,y,z,phi,th,psi
t_end = .5;
    
%% Simulate PID 
sim sliding_quadcopter
disp('Done Simulating')

%% Plot Data
t = h_states.x.Time;
x = h_states.x.Data;     
y = h_states.y.Data;
z = h_states.z.Data;
phi = h_states.phi.Data;
th = h_states.th.Data;
psi = h_states.psi.Data;

eX = h_err.Data(:,1);     
eY = h_err.Data(:,4);
eZ = h_err.Data(:,7);

w1 = h_rotor.w1.Data;
w2 = h_rotor.w2.Data;
w3 = h_rotor.w3.Data;
w4 = h_rotor.w4.Data;

figure(1)
plot(t,[x y z])
legend('x','y','z'); xlabel('Time'); ylabel('m')
title('States vs Time')

figure(2)
plot(t,[w1 w3 w2 w4])
legend('w1','w3','w2','w4'); xlabel('Time'); ylabel('Ang Vel')
title('Rotor Speed')

figure(3)
plot(t,[phi th psi]./pi*180)
legend('\phi','\theta','\psi'); xlabel('Time'); ylabel('Degrees')
title('Body Angles vs Time')

figure(4)
plot(t,[eX eY eZ])
legend('eX','eY','eZ'); xlabel('Time'); ylabel('Error')
title('Error vs Time')
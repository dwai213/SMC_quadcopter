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
Ix =1/12*(.4*m)*(2*l)^2; Iy = Ix; Iz = m*(l^2);
Ix_nom = Ix; Iy_nom = Iy; Iz_nom = Iz;
I = [Ix Iy Iz]; %kg m2
H_nom = diag([m_nom,m_nom,m_nom,Ix_nom,Iy_nom,Iz_nom]);
CC_nom = diag([C_nom,Cd_nom]);

Kp = 1.0*[1 1]; %x, y
Kd = .95*[1 1]; %x, y
lamb = [1 20 20 1];
PSI = [.01 .0005 .0005 .001]';
eta = [1 1 1 1]';
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
% set_param('gen_smc_quadcopter','AlgebraicLoopSolver','LineSearch')
set_param('gen_smc_quadcopter','AlgebraicLoopSolver','TrustRegion')
sim gen_smc_quadcopter
disp('Done Simulating')

%% Plot Data
t = h_states.x.Time;
x = h_states.x.Data(:);     
y = h_states.y.Data(:);
z = h_states.z.Data(:);
phi = h_states.phi.Data(:);
th = h_states.th.Data(:);
psi = h_states.psi.Data(:);
th_ref = h_th_ref.Data(:);

eX = h_err.x_err.Data(:);     
eY = h_err.y_err.Data(:);     
eZ = h_err.z_err.Data(:);     

u1 = h_rotor_thrust.Data(:,1);
u2 = h_rotor_thrust.Data(:,2);
u3 = h_rotor_thrust.Data(:,3);
u4 = h_rotor_thrust.Data(:,4);
F1 = .25*u1 - .5*u2 + .25*u4;
F2 = .25*u1 - .5*u3 - .25*u4;
F3 = .25*u1 + .5*u2 + .25*u4;
F4 = .25*u1 + .5*u3 - .25*u4;

figure(1), clf
% line([0 4],[.1,.1],'LineStyle','--','Color',[0 0 0 ])
plot(t,[x y z]); hold on; plot(t,.1*sin(1*t),'k--');
legend('x','y','z','Reference'); xlabel('Time'); ylabel('m')
title('SMC - States vs Time')

figure(2)
plot(t,[F1 F2 F3 F4])
legend('F1','F2','F3','F4'); xlabel('Time'); ylabel('Ang Vel')
title('SMC - Rotor Thrust')
axis([0 4 .1 2.5])
% axis([0 4 1 1.35])

figure(3), clf
plot(t,[phi th psi]./pi*180); hold on; plot(t,th_ref/pi*180,'k--');
legend('\phi','\theta','\psi','Reference'); xlabel('Time'); ylabel('Degrees')
title('SMC - Body Angles vs Time')

figure(4)
plot(t,[eX eY eZ])
legend('eX','eY','eZ','Location','SouthEast'); xlabel('Time'); ylabel('Error')
title('SMC - Error vs Time')

%%
A = [1 1 1 1;-1 0 1 0; 0 -1 0 1; 1 -1 1 -1];
B = inv(A)

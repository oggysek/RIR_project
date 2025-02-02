clc
clear

% Position tolerance
positionTolerance = 0.1;    % [m]

% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [100,200,300,400,500]*0.2; % [s]

% Waypoints [X, Y, Z][m] - these points must be flown by quadcopter
wayPoints = [0 0 -6;
             1 1 -10;
             1 2 -4;
             2 2 -5;
             3 1 -6];

% sample time
ts = 0.01;

% Initial States
Euler_0 = [0;0;0];
XYZ_0 = wayPoints(1,1:3)';


%% State space system
% Parametre
Ix = 0.023;
Iy = 0.023;
Iz = 0.047;
ArmLenght = 0.27; % [m]
m = 1.3;
g = -9.81;
IC = [Euler_0;
      XYZ_0;
      0;
      0;
      0;
      0;
      0;
      0];

A = [0 0 0 0 0 0 1 0 0 0 0 0;  %phi
     0 0 0 0 0 0 0 1 0 0 0 0;  %theta
     0 0 0 0 0 0 0 0 1 0 0 0;  %psi
     0 0 0 0 0 0 0 0 0 1 0 0;  %x
     0 0 0 0 0 0 0 0 0 0 1 0;  %y
     0 0 0 0 0 0 0 0 0 0 0 1;  %z
     0 0 0 0 0 0 0 0 0 0 0 0;  %p
     0 0 0 0 0 0 0 0 0 0 0 0;  %q
     0 0 0 0 0 0 0 0 0 0 0 0;  %r
     0 -g 0 0 0 0 0 0 0 0 0 0; %u
     g 0 0 0 0 0 0 0 0 0 0 0;  %v
     0 0 0 0 0 0 0 0 0 0 0 0]; %w

B = [0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   0    0    0;
     0   1/Ix    0    0;
     0   0    1/Iy    0;
     0   0    0    1/Iz;
     0   0    0    0;
     0   0    0    0;
     1/m   0    0    0];

C = eye(12);
D = zeros(12,4);

% controlability, obsv
rank(ctrb(A,B));
obs =  rank(obsv(A,C));

% vypocet hodnot do regulatoru
Aaa = A(1:6,1:6);
Aab = A(1:6,7:12);
Aba = A(7:12,1:6);
Abb = A(7:12,7:12);
Ba = B(1:6,:);
Bb = B(7:12,:);

J = [-15+i -15-i -20 -20 -10 -10 -6 -14 -8+i -8-i -18+i -18-i];
Je = [-20 -18 -8 -12 -22 -16];

K = place(A,B,J);  %4*12
Ka = K(1:4,1:6);
Kb = K(1:4,7:12);
Ke = place(Abb',Aab',Je)';

As = Abb-Ke*Aab;
Bs = As*Ke+Aba-Ke*Aaa;
Cs = [zeros(6,6); eye(6)];
Ds = [zeros(6,6)+1; Ke];    
Fs = Bb - Ke*Ba;

% transfer fucntion
At = As-Fs*Kb;
Bt = Bs-Fs*(Ka+Kb*Ke);
Ct = -Kb;
Dt = -(Ka+Kb*Ke);

sys = ss(At,Bt,-Ct,-Dt);
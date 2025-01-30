clc
clear

% Position tolerance
positionTolerance = 0.1;    % [m]

% At these times, the quadcopter must pass the appropriate waypoint
% timeForWaypointPasage = [100,200,300,400,500]; % [s]
timeForWaypointPasage = [100,200,300,400,500]*0.2;

% Waypoints [X, Y, Z][m] - these points must be flown by quadcopter
wayPoints = [0 0 6;
             1 1 10;
             1 2 4;
             2 2 5;
             3 1 6];
%% Setup Drone
m = 1.3;
I = [[0.023,0,0];[0,0.023,0];[0,0,0.047]];

% sample time
ts = 0.01;

% Initial States (Initial XYZ is generated by XYZsignal script)
Euler_0 = [0;0;0];
XYZ_0 = wayPoints(1,1:3);
body_rate_0 = [0;0;0];

% Environment
g = [0;0;-9.8];

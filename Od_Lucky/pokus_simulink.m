clc;
clear;
clf;
close all;
format compact

%% Zadani

% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [100,200,300,400,500]; % [s]
pokus = timeForWaypointPasage(1)
% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1 1 -10;
             1 2 -4;
             2 2 -5;
             3 1 -6];

    % Počet waypointů
    numWaypoints = size(wayPoints, 1);

    % Defaultní waypoint, pokud čas simulace přesáhne všechny časy
    waypoint_now = wayPoints(end, :);
    timeForWaypoints_now = 0;

    % Najdeme odpovídající waypoint podle času
time = 510;
    for i = 1:numWaypoints
            if time <= timeForWaypointPasage(i)
                waypoint_now = wayPoints(i, :)
                timeForWaypoints_now = timeForWaypointPasage(i)
                return;
            end
    end
    %%
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
deltaT = 0.01;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]

% Interpolate trajectory
timeSim = 0:deltaT:simulationTime;
XTrajectory = -interp1(timeForWaypointPasage, wayPoints(:, 1), timeSim, 'pchip');
YTrajectory = -interp1(timeForWaypointPasage, wayPoints(:, 2), timeSim, 'pchip');
ZTrajectory = -interp1(timeForWaypointPasage, wayPoints(:, 3), timeSim, 'pchip');

% Constants
% Radians to degree
RadianToDegree = 180/pi; 
% Degree to radians
DegreeToRadian = pi/180;

% Quadcopter parameters
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]
g = 9.81;

% Stavové proměnné (12 stavů: pozice, rychlosti, úhly, úhlové rychlosti)
A = zeros(12); 
A(1:6, 7:12) = eye(6); 
A(7,5)=g;
A(8,4)=-g;

% Vstupy
B = zeros(12, 4); % (T, Mx, My, Mz)
B(9, 1) = 1 / Mass; % Tah ovlivňuje Z-rychlost
B(10, 2) = 1 / XMomentOfInertia; % Moment ovlivňuje rotaci kolem X
B(11, 3) = 1 / YMomentOfInertia; % Moment ovlivňuje rotaci kolem Y
B(12, 4) = 1 / ZMomentOfInertia; % Moment ovlivňuje rotaci kolem Z

% Výstupy
C = eye(12); % Měříme x,y,z,uhly
D = zeros(12, 4); % Přímý přenos není

% Stavový model
sys = ss(A, B, C, D);

Init_State_vect = [0 0 -6 0 0 0 0 0 0 0 0.1 0];

% Initial state of quadcopter
quadcopterInitState.BodyXYZPosition.X = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Y = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Z = -6; % [m]           
quadcopterInitState.BodyXYZVelocity.X = 0;            
quadcopterInitState.BodyXYZVelocity.Y = 0;            
quadcopterInitState.BodyXYZVelocity.Z = 0;
quadcopterInitState.BodyEulerAngle.Phi = 0;
quadcopterInitState.BodyEulerAngle.Theta = 0;
quadcopterInitState.BodyEulerAngle.Psi = 0;
quadcopterInitState.BodyAngularRate.dPhi = 0;
quadcopterInitState.BodyAngularRate.dTheta = 0;
quadcopterInitState.BodyAngularRate.dPsi = 0;

% Control variables - total thrust and moments on each control axis
quadcopterInitControlInputs = [0, 0, 0, 0]';     % (T, M1, M2, M3)
                           
% Initiate quadcopter
global quadcopter;
quadcopter = Quadcopter(Mass, ...               
               XMomentOfInertia, ...
               YMomentOfInertia, ...
               ZMomentOfInertia, ...
               quadcopterInitState, ...
               quadcopterInitControlInputs,...
               deltaT);

%% Simulation
for i = 1 : deltaT : simulationTime
    
    % Action for total thrust
    quadcopter.TotalThrustControlAction(0);
    % Action for attitude
    quadcopter.AttitudeControlAction(0,0,0);
   
    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState();
    
    % Crash check
    if (quadcopterActualState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end

    % Waypoint check
    if (CheckWayPointTrack(...
                quadcopterActualState.BodyXYZPosition,...
                i * deltaT,...
                timeForWaypointPasage,...
                wayPoints,...
                positionTolerance))
        msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
        break;
    end
end                              
clc;
clear;
clf;
close all;
format compact

% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [100,200,300,400,500]; % [s]

% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1 1 -10;
             1 2 -4;
             2 2 -5;
             3 1 -6];
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
deltaT = 0.01;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]

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

% Initial state of quadcopter
quadcopterInitState.BodyXYZPosition.X = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Y = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Z = -6; % [m]           
quadcopterInitState.BodyXYZVelocity.X = 0;    % change to 0        
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
quadcopter = Quadcopter(Mass, ...               
               XMomentOfInertia, ...
               YMomentOfInertia, ...
               ZMomentOfInertia, ...
               quadcopterInitState, ...
               quadcopterInitControlInputs,...
               deltaT);

% Define PIDs
X_PID = PIDRegulator(1, 0, 0); % PID gains for position control
Y_PID = PIDRegulator(1, 0.0, 0);
Z_PID = PIDRegulator(1e-3, 0, 0.5);
Roll_PID = PIDRegulator(1e-9, 0, 0);
Pitch_PID = PIDRegulator(1.0, 0.0, 0);
Yaw_PID = PIDRegulator(1.0, 0.0, 0);

% For plot
i_p = 0;
Path = zeros(simulationTime/deltaT,3);

quadcopterActualState = quadcopter.GetState();

% Simulation
for i = 0 :deltaT: simulationTime

    % Trajectory and control calculations
    [X_desired, Y_desired, Z_desired] = TrajectoryPlanner(i, timeForWaypointPasage, wayPoints);

    % Apply control actions
    quadcopter.TotalThrustControlAction(Mass * quadcopter.g); % !vyresit, aby to kopenzovalo i kdyz je natoceny (neco s promitnutim totalthrust do Z)
    quadcopter.AttitudeControlAction(1e-7, 0, 1e-10); % (x-rot, y-rot, z-rot)

    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState();

    % Crash check
%     if (quadcopterActualState.BodyXYZPosition.Z >= 0)
%         msgbox('Quadcopter Crashed!', 'Error', 'error');
%         break;
%     end

    % Waypoint check
%     if (CheckWayPointTrack(...
%                 quadcopterActualState.BodyXYZPosition,...
%                 i * deltaT,...
%                 timeForWaypointPasage,...
%                 wayPoints,...
%                 positionTolerance))
%         msgbox('Quadcopter did not passed waypoint', 'Error', 'error');
%         break;
%     end

% Ploting
i_p = i_p+1;    % index for ploting
Path(i_p,:) = quadcopter.state(1:3)';
end

plot3(Path(:,1),Path(:,2),Path(:,3))
xlabel("x")
ylabel("y")
zlabel("z")
hold on
plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),"*")
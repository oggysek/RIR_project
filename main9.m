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
P = 1e-3;
D = 1;
X_PID = PIDRegulator(P, 0, 0); % PID gains for position control
Y_PID = PIDRegulator(P, 0, 0);

Z_PID = PIDRegulator(1e-3, 0, 0.5);
Phi_PID = PIDRegulator(P, 0, 0);
Theta_PID = PIDRegulator(P, 0, 0);
Psi_PID = PIDRegulator(P, 0, 0);

M1_PID = PIDRegulator(P, 0, D);
M2_PID = PIDRegulator(P, 0, D);
M3_PID = PIDRegulator(P, 0, D);

% For plot
i_p = 0;
Path = zeros(simulationTime/deltaT,3);

% Simulation
quadcopterActualState = quadcopter.GetState();
for i = 0 :deltaT: simulationTime

    % Trajectory and control calculations
%     [X_desired, Y_desired, Z_desired] = TrajectoryPlanner(i, timeForWaypointPasage, wayPoints);
    X_desired = 1;
    Y_desired = 1;
    Z_desired = -10;

    % Translational position
    X_PID.CalculateAction(quadcopterActualState.BodyXYZPosition.X, X_desired, deltaT);
    Y_PID.CalculateAction(quadcopterActualState.BodyXYZPosition.Y, Y_desired, deltaT);

    Phi_desired = -Y_PID.GetCurrentAction();
    Theta_desired = X_PID.GetCurrentAction(); 
    % saturation
    Phi_desired = max(min(Phi_desired, 1), -1);
    Theta_desired = max(min(Theta_desired, 1), -1);

    % Attitude/Altitude
    Z_PID.CalculateAction(quadcopterActualState.BodyXYZPosition.Z, Z_desired, deltaT);
    Phi_PID.CalculateAction(quadcopterActualState.BodyEulerAngle.Phi, Phi_desired, deltaT);
    Theta_PID.CalculateAction(quadcopterActualState.BodyEulerAngle.Theta, Theta_desired, deltaT);
    Psi_PID.CalculateAction(quadcopterActualState.BodyEulerAngle.Psi, 0, deltaT);   % We dont want no rotation in Z axis

    TotalThrust_command = -Z_PID.GetCurrentAction() + Mass*quadcopter.g;
    dPhi_desired = Phi_PID.GetCurrentAction();
    dTheta_desired = Theta_PID.GetCurrentAction();
    dPsi_desired = Psi_PID.GetCurrentAction();

    TotalThrust_body = TotalThrust_command / ...
    (cos(quadcopterActualState.BodyEulerAngle.Phi) * cos(quadcopterActualState.BodyEulerAngle.Theta));

    % Angular Velocity
    M1_PID.CalculateAction(quadcopterActualState.BodyAngularRate.dPhi, dPhi_desired, deltaT);
    M2_PID.CalculateAction(quadcopterActualState.BodyAngularRate.dTheta, dTheta_desired, deltaT);
    M3_PID.CalculateAction(quadcopterActualState.BodyAngularRate.dPsi, dPsi_desired, deltaT);

    M1_command = M1_PID.GetCurrentAction();
    M2_command = M2_PID.GetCurrentAction();
    M3_command = M3_PID.GetCurrentAction();
    
    % Apply control actions
    quadcopter.TotalThrustControlAction(TotalThrust_body); % !vyresit, aby to kopenzovalo i kdyz je natoceny (neco s promitnutim totalthrust do Z)
    quadcopter.AttitudeControlAction(M1_command, 0, 0); % (x-rot, y-rot, z-rot)

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
function [X_desired, Y_desired, Z_desired] = TrajectoryPlanner(time, timeForWaypoints, wayPoints)
    for j = 1:length(timeForWaypoints)
        if time <= timeForWaypoints(j)
            X_desired = wayPoints(j, 1);
            Y_desired = wayPoints(j, 2);
            Z_desired = wayPoints(j, 3);
            return;
        end
    end
    X_desired = wayPoints(end, 1);
    Y_desired = wayPoints(end, 2);
    Z_desired = wayPoints(end, 3);
end
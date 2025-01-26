function isSuccesfullyPassed = CheckWayPointTrack(bodyXYZPosition, actualTime, timeForWaypointPasage, wayPoints, positionTolerance)
    % Function that determines whether the quadcopter will pass a specified point in a given time within a given tolerance
    % Find the waypoint corresponding to the current time
    [~, idx] = min(abs(timeForWaypointPasage - actualTime));
    targetPoint = wayPoints(idx, :);

    % Check if the position is within tolerance
    distance = norm([bodyXYZPosition.X, bodyXYZPosition.Y, bodyXYZPosition.Z] - targetPoint);
    isSuccesfullyPassed = (distance <= positionTolerance);
% Pobezi timer do prvniho time for waypoint 
% pokud bude behem toho pozice drona v toleranci, zakaze se po uplynuti
% timeru alarm.
% az se dosahne prvniho time for waypoint, tak vyhodnoti, jestli byl na
% waypointu a zacne overovat pro dalsi waypoint.
end
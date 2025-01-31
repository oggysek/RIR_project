%% Generate signals
% [x,y,z,t]

WayPts = [wayPoints, timeForWaypointPasage'];

dX = [];
dY = [];
dZ = [];

% urci prumernou rychlost v osach pro kazdy usek
for i = 1:length(WayPts)-1
    dX(i) = (WayPts(i+1,1)-WayPts(i,1))/(WayPts(i+1,4)-WayPts(i,4));
    dY(i) = (WayPts(i+1,2)-WayPts(i,2))/(WayPts(i+1,4)-WayPts(i,4));
    dZ(i) = (WayPts(i+1,3)-WayPts(i,3))/(WayPts(i+1,4)-WayPts(i,4));
end

Tfinal = WayPts(length(WayPts),4) + 10;
TOFtime = 1;
tss = 0.1;
t = 0:tss:Tfinal;

% Superpose signals + initial conditions
x_sum = 0*t + WayPts(1,1);
y_sum = 0*t + WayPts(1,2);
z_sum = 0*t + WayPts(1,3);

for k = 2:length(WayPts)
    section = [WayPts(k-1,4),WayPts(k,4)];
    start_section = section(1);
    end_section = section(2);
        
    for i = 1:length(z_sum)               % pro kazdy interval pricita linearne souradnice
        if t(i) > start_section && t(i) <= end_section
            z_sum(i) = z_sum(i-1) + tss*dZ(k-1);
            y_sum(i) = y_sum(i-1) + tss*dY(k-1);
            x_sum(i) = x_sum(i-1) + tss*dX(k-1);
        end
    end
end

% Po poslednim bodu zustane v poslednim bodu jeste nejakou chvili (+10)
start_section = WayPts(length(WayPts),4);
end_section = Tfinal;
for i = 1:length(z_sum)
    if t(i) > start_section && t(i) <= end_section
        z_sum(i) = z_sum(i-1);
        y_sum(i) = y_sum(i-1);
        x_sum(i) = x_sum(i-1);
    end
end

% Generate timeseries cmd
sigZ = [t;z_sum];
Zcmd = timeseries(sigZ(2:end,:),sigZ(1,:));
sigY = [t;y_sum];
Ycmd = timeseries(sigY(2:end,:),sigY(1,:));
sigX = [t;x_sum];
Xcmd = timeseries(sigX(2:end,:),sigX(1,:));

clear section
clear start_section
clear end_section
clear y_sum
clear z_sum
clear x_sum
clear i
clear k



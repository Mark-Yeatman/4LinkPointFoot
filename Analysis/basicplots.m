%% Colors
blue = [0, 0.4470, 0.7410];
orange = [0.8500, 0.3250, 0.0980];
yellow = [0.9290, 0.6940, 0.1250];
purple = [0.4940, 0.1840, 0.5560];
green = [0.4660, 0.6740, 0.1880];
red = [0.6350, 0.0780, 0.1840];
dim = flowdata.Parameters.dim;

set(0,'DefaultFigureWindowStyle','docked')

%% Positions
figure('Name','Positions','NumberTitle','off')
subplot(2,1,1)
plot(tout,rad2deg(xout(:,3:7)))
title('Joint Positions')
legend("thigh_{1}","knee_{1}","thigh_{2}","knee_{2}","torso",'Location', 'eastoutside')
xlabel("time")
ylabel("Angle (deg)")

subplot(2,1,2)
plot(xout(:,1),xout(:,2))
xlabel("x pos")
ylabel("y pos")
title('Hip Trajectory')

%% Velocities
figure('Name','Velocities','NumberTitle','off')
plot(tout,xout(:,10:14))
title('velocities')
legend("thigh_{1}","knee_{1}","thigh_{2}","knee_{2}","torso",'Location', 'eastoutside')
xlabel("time")
ylabel("vel (deg/s)")

set(0,'DefaultAxesXGrid','off')
set(0,'DefaultAxesYGrid','off')
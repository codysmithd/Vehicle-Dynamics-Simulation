% RIT P16241: Autonomous People Mover, Phase III
% Main program to simulate Vehicle Dynamics through path-finding solution
% Eric Schulken

clear; clc

%% Inputs: GPS coordinates, Longitudinal Force at Wheels (Throtle and Brake)
% Tire force inputs in ode45_APM.m Function File (Pf, Pr)

% Initial Conditions
thetadot = 0; % Turning Rate
Vlat = 0; % Horizontal Speed (ft/s)
Vlon = 5; % Forward Speed (ft/s)
X = 0; % X Position (ft)
Y = 0; % Y Position (ft)
theta = 0; % Heading Angle (deg)
theta = theta/57.3; % to rad
s = 0; % Steering diflection

% GPS coordinates
Xpts = [50,100,150,200,250];
Ypts = [50,70,50,50,50];
prox = 5; % proximity to point before proceeding to next

%% GPS Iteration
n = 1; % Which iteration we are on (which point we are going to)
count = 1; % How many time steps we have gone through
tstart = 0;
L = 0; % The index (in time units) in the final data set we are at
x_out(:,1) = [thetadot,Vlat,Vlon,X,Y,theta,s,0,0];

while n<6

    % Time (for this iteration)
    time = tstart:0.1:tstart+20;

    % Initial condition vector
    IC = [x_out(1,count),x_out(2,count),x_out(3,count),x_out(4,count),...
        x_out(5,count),x_out(6,count),x_out(7,count),Xpts(n),Ypts(n)];

    % Run simulation
    [t_o,x_o] = ode45(@APM_GPS,time,IC);
    x_o = x_o';

    for i=1:length(time)

        % If we are within the prox of the point, move to next
        % point/iteration
        if Xpts(n)>(x_o(4,i)-prox) & Xpts(n)<(x_o(4,i)+prox) & ...
                Ypts(n)>(x_o(5,i)-prox) & Ypts(n)<(x_o(5,i)+prox)
            n = n+1;
            break
        end

        % If we are NOT within the prox of the point, save data and move to
        % next time step
        x_out(:,count) = x_o(:,i);
        time_out(count) = t_o(i);
        count = L + i;
    end

    count = count - 1; % Set count to end
    tstart = time_out(count); % Set start time for next simulation to end time for last
    L = length (time_out);
end



%% Plots
figure(1)
subplot(2,2,1); plot(x_out(4,:),x_out(5,:)); xlabel('ft'); ylabel('ft');
title('Cart Position'); grid on; axis equal; hold on
plot(Xpts,Ypts,'*'); hold off
subplot(2,2,2); plot(time_out,x_out(7,:)*57.3); xlabel('Time (s)');
ylabel('Steering Angle (deg)'); title('Steering Input'); grid on
subplot(2,2,3); plot(time_out,x_out(2,:)); xlabel('Time (s)');
ylabel('Lateral Velocity (ft/s)'); grid on
subplot(2,2,4); plot(time_out,x_out(3,:)); xlabel('Time (s)');
ylabel('Longitudinal Velocity (ft/s)'); grid on
% Found online to create centered title over subplots
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],'Box','off',...
    'Visible','off','Units','normalized', 'clipping' , 'off');
text(0.5, 1,'Simulated Vehicle Response','HorizontalAlignment','center',...
    'VerticalAlignment', 'top')

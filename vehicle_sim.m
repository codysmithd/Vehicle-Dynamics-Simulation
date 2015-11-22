% RIT P16241: Autonomous People Mover, Phase III
% Vehicle Dynamics Simulation module

% Eric Schulken

function dx = APM_GPS(t,x)

% Vehicle Properties:
m = 500; % Mass (lbs)
a = 2.5; b = 1.5; h = 1.5; % Center of Gravity Location (ft)
cf = 0.4*m*0.165*57.3; cr = 0.6*m*0.165*57.3; % Cornering Stiffness of Tires (lbs/rad)
% http://www.mchenrysoftware.com/medit32/readme/msmac/default.htm?turl=examplestirecorneringstiffnesscalculation.htm
Pf = 0; Pr = 12; % Longitudinal Force of Tires (lbs)
mu = 0.7; % Friction Coefficient
T = 4; % Track Width (ft)
Ixx = 250; % Estimated (lb-ft-s^2/rad)

% External Forces:
g = 32.17561865; % ft/s^2

% Throttle to maintain speed
%Pr = zeros(length(t));
%if x(3) < 4.9
%    Pr = 200;
%else
%    Pr = 0;
%end

% Lateral Force Model:
I = Ixx+m*(T^2/4+h^2);
Fzf = ((m*g*b)-(Pf+Pr)*h)/(a+b);
Fzr = ((m*g*a)-(Pf+Pr)*h)/(a+b);
af = x(7)-((a*x(1)+x(2))/x(3));
ar = (b*x(1)-x(2))/x(3);
afs = ((cf*af)/(mu*Fzf));
ars = ((cr*ar)/(mu*Fzr));
Fef = mu*Fzf*(afs-((afs*abs(afs))/3)+(afs^3/27))*sqrt(1-(Pf^2/(mu^2*Fzf^2))+(Pf^2/cf^2));
Fer = mu*Fzr*(ars-((ars*abs(ars))/3)+(ars^3/27))*sqrt(1-(Pr^2/(mu^2*Fzr^2))+(Pr^2/cr^2));
xx = x(2)*sin(x(6))+x(3)*cos(x(6)); % X velocity
yy = x(3)*sin(x(6))-x(2)*cos(x(6)); % Y velocity


%% Define 6 elements of ODE
dx = zeros(9,1);
dx(1) = (a*Pf*x(7)+b*Fef-b*Fer)/I; % Yaw Rate
dx(2) = ((Pf*x(7)+Fef+Fer)/m)-(x(3)*x(1)); % Lateral Velocity
dx(3) = ((Pf+Pr-Fef*x(7))/m)-(x(2)*x(1)); % Longitudinal Velocity
dx(4) = xx; % Longitudinal Position
dx(5) = yy; % Lateral Position
dx(6) = x(1); % yaw angle
dx(7) = -x(1)+2*((xx*(x(9)-x(5))+(x(4)-x(8))*yy)/...
    (-2*x(8)*x(4)+x(4)^2-2*x(9)*x(5)+x(5)^2+x(8)^2+x(9)^2)); % Steering Diffy-Q
dx(8) = 0; % X GPS coordinate
dx(9) = 0; % Y GPS coordinate
% dx(10) = -((Pf+Pr-Fef*x(7))/m)-(x(2)*x(1)); % Throttle force at rear wheels

end

function [Theta, DTheta] = MyRotorDTU10MW (TimeSteps, DT, Rot0)

% TimeSteps --> Number of time steps (Input argument)
% DT --> Time steps (Input argument)
% Rot0 --> Initial rotor angle [º] (Input argument)

% GAP --> Time gap to increase the angular velocity from 0 to WNominal
% WNominal --> Nominal angular velocity in rad/sec

GAP          = fix ((TimeSteps+1) / 20);
WNominal_RPM = -9.60;                          % Angular velocity in RPM
WNominal     = WNominal_RPM * 2 * pi / 60;     % Angular velocity in rad/sec
Rot0         = Rot0 *pi / 180;

TIME = linspace (0, TimeSteps*DT, TimeSteps+1);

DTheta = zeros (1,TimeSteps+1);

for i = 1:GAP+1
    HH = HermitePoly (DT*(i-1), 0, DT*GAP);
    DTheta(i) = HH.h3*(WNominal);
end

DTheta(GAP+2:TimeSteps+1) = WNominal;
Theta(1:TimeSteps+1)      = Rot0 + DTheta(1:TimeSteps+1).*TIME(1:TimeSteps+1);


end

%%

function [H] = HermitePoly (x, xi, xj)

h = xj - xi;

H.h1 = 1 - 3*(x - xi)^2 / (h^2) + 2*(x - xi)^3/(h^3);
H.h2 = (x - xi) - 2/h*(x - xi)^2 + 1/(h^2)*(x - xi)^3;
H.h3 = 3/h^2*(x - xi)^2 - 2/(h^3)*(x - xi)^3;
H.h4 = -1/h*(x - xi)^2 + 1/(h^2)*(x - xi)^3;


end
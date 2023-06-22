function [GROUND_AERO] = AeroMesh_Ground (DATA, GROUND_AERO, Indx)

%% Auxiliar Data

Grd_DB   = DATA(Indx).RTowGround * 2;   % Tower bottom diameter
Grd_DOut = DATA(Indx).LGround;          % Ground extension 

Grd_NodR = DATA(Indx).NRadGround;       % Number of aerodynamic nodes along the radial direction of the ground
Grd_NodC = DATA(Indx).NCircGround;      % Number of aerodynamic nodes along the circunferential direction of the ground (to be equal to DATA.Tow_NodC)

%% Square coordinates (From circle to square) / FG-Squircular mapping

% x = a sgn(cost)/(s sqrt(2) abs(sint)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))
% y = a sgn(sint)/(s sqrt(2) abs(cost)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))

% where s is a parameter which is used to blend the circle and the square smoothly
% Reference: Analytical Methods for Squaring the Disc (Chamberlain Fong)

Theta = linspace(0, 2*pi, Grd_NodC);                                       % Circunferential division along the hub-blade hole

XX = Grd_DB/2 * cos(Theta);                                                  % X-coordinate along the hub-blade hole
YY = Grd_DB/2 * sin(Theta);                                                  % Y-coordinate along the hub-blade hole

DAng = linspace (0, pi, Grd_NodR);                                         % Incremental angle used to make a radial division (sine-shape) from ellipse to square

a1 = Grd_DB/2 + (Grd_DOut/2 - Grd_DB/2)*(1 + cos(DAng))/2;                          % Major semi-axis for different radial coordinates
b1 = Grd_DB/2 + (Grd_DOut/2 - Grd_DB/2)*(1 + cos(DAng))/2;                       % Minor semi-axis for different radial coordinates

a1 = sort(a1, 'ascend');
b1 = sort(b1, 'ascend');

ss1 = linspace (0, pi/2, Grd_NodR);                                        

ss1 = sin(ss1);                                                            % incremental FG-squircicurlar parameter (s = 0: circle/ellipse // s = 1: square)

for k = 1:Grd_NodR
    
    [AUX_X, AUX_Y] = FG_Squircular_Mapping (a1(k), b1(k), ss1(k), Theta, 1);
    
    XX2(k,:) = AUX_X;
    YY2(k,:) = AUX_Y;
    
end

%% Patch coordinates on the Ground

NN = length(Theta);

COORD(1:NN,1) = transpose(XX);
COORD(1:NN,2) = transpose(YY);
COORD(1:NN,3) = 0;

for i = 1:Grd_NodR - 4                                                      % Hub_NodR - 4 is to eliminate the second and second-to-last radial divisions
    
    I1 = NN*(i) + 1;
    I2 = NN*(i+1);
    
    COORD(I1:I2,1) = transpose(XX2(i+2,:));
    COORD(I1:I2,2) = transpose(YY2(i+2,:));
    COORD(I1:I2,3) = 0;
    
end

I1 = NN*(i+1) + 1;
I2 = NN*(i+2);

COORD(I1:I2,1) = transpose(XX2(end,:));
COORD(I1:I2,2) = transpose(YY2(end,:));
COORD(I1:I2,3) = 0;

%--------------------------------------------------------------------------
%                    Ground connectivities (ICON)
%--------------------------------------------------------------------------

ICON_Ground = Connectivities_1 (Grd_NodR-2, NN);

%% Control points and normalls

XYZ_CP = ControlPoints (COORD, ICON_Ground, size(ICON_Ground,1));
NOR    = NormallVector (COORD, ICON_Ground, size(ICON_Ground,1));

%%

GROUND_AERO(Indx).XYZ     = COORD;
GROUND_AERO(Indx).XYZCP   = XYZ_CP;
GROUND_AERO(Indx).Normal  = NOR;

GROUND_AERO(Indx).ICON    = ICON_Ground;

GROUND_AERO(Indx).NN      = size(COORD,1);
GROUND_AERO(Indx).NP      = size(ICON_Ground,1);
    
GROUND_AERO(Indx).NNR     = Grd_NodR-2;
GROUND_AERO(Indx).NNC     = NN;

    
end

%% CONTROL POINTS 

function XYZCP = ControlPoints (XYZ, ICON, NP)

XYZCP = zeros(NP,3);

for i = 1:NP
    
    XYZCP (i, 1:3) =  0.25D+0 * ( XYZ(ICON(i,1), 1:3) + ...
        XYZ(ICON(i,2), 1:3) + XYZ(ICON(i,3), 1:3) + ...
        XYZ(ICON(i,4), 1:3) );
    
end


end

%% NORMALLS 

function VectN = NormallVector (XYZ, ICON, NP)

      V1 (1:3, 1:NP) = transpose ( XYZ (ICON (1:NP,2), 1:3) ) - ...
                         transpose ( XYZ (ICON (1:NP,4), 1:3) );
      V2 (1:3, 1:NP) = transpose ( XYZ (ICON (1:NP,1), 1:3) ) - ...
                       transpose ( XYZ (ICON (1:NP,3), 1:3) );
     
      VN =  cross (V1, V2);

      VectN = zeros (NP,3);
     
      for i = 1:NP
     
          V = norm ( VN(1:3, i), 2);
         
          VectN (i, 1:3) = transpose (VN(1:3, i) / V);
         
      end              


end


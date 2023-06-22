function [HUB_AERO] = AeroMesh_Hub (DATA, HUB_AERO, Indx)

%% Auxiliar Data

D_Rot    = DATA(Indx).HubRad * 2;       % Distance from the rotation axis to the blade root
Num_BD   = DATA(Indx).NumBld;           % Number of blades
D_Hub    = DATA(Indx).HubInnerR * 2;    % Hub Diameter
L_Hub    = DATA(Indx).LCylHub / 2;      % Hub length (Cylinder part)
Beta     = DATA(Indx).PreCone;          % Precone angle
D_Blade  = DATA(Indx).DBladeHub;        % Diameter of the hub-blade coupling (Equal to the blade root diameter)
LNose    = DATA(Indx).LNoseHub;         % Length of the hub nose


Hub_NodC = DATA(Indx).NCircHub;      % Number of nodes along the circunferential direction (on the coupling)
Hub_NodR = DATA(Indx).NRadHub;       % Number of nodes along the radial direction (division based on concentric squares)
Hub_NodZ = DATA(Indx).NZcoupHub;     % Number of nodes along the hub-blade coupling zone
Hub_NodN = DATA(Indx).NNoseHub;      % Number of nodes along the nose of the Hub

Nose_Shape = DATA(Indx).ShapeNosHub;       % Shape of the hub nose
PerTrim    = DATA(Indx).TrimNoseHub / 100; % Percentaje of trim


%% Hub patch data

Beta   = Beta * pi / 180;

H_Square = (D_Hub/2 * (2*pi/Num_BD)) / 2;                             % Height of the square containing the hole (hub-blade intersection)

a_axis = D_Blade / 2 * (cos(Beta) + sin(Beta)*tan(Beta));             % Major semi-axis of the ellipse on the hub-blade intersection
b_axis = asin(D_Blade/D_Hub)*Num_BD*H_Square/pi;                      % Minor semi-axis of the ellipse on the hub-blade intersection


%% Square coordinates (From circle to square) / FG-Squircular mapping

% x = a sgn(cost)/(s sqrt(2) abs(sint)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))
% y = a sgn(sint)/(s sqrt(2) abs(cost)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))

% where s is a parameter which is used to blend the circle and the square smoothly
% Reference: Analytical Methods for Squaring the Disc (Chamberlain Fong)

Theta = linspace(0, 2*pi, Hub_NodC);                                       % Circunferential division along the hub-blade hole

XX = a_axis * cos(Theta);                                                  % X-coordinate along the hub-blade hole
YY = b_axis * sin(Theta);                                                  % Y-coordinate along the hub-blade hole

DAng = linspace (0, pi, Hub_NodR);                                         % Incremental angle used to make a radial division (sine-shape) from ellipse to square

a1 = a_axis + (L_Hub - a_axis)*(1 + cos(DAng))/2;                          % Major semi-axis for different radial coordinates
b1 = b_axis + (H_Square - b_axis)*(1 + cos(DAng))/2;                       % Minor semi-axis for different radial coordinates

a1 = sort(a1, 'ascend');
b1 = sort(b1, 'ascend');

ss1 = linspace (0, pi/2, Hub_NodR);                                        

ss1 = sin(ss1);                                                            % incremental FG-squircicurlar parameter (s = 0: circle/ellipse // s = 1: square)

for k = 1:Hub_NodR
    
    [AUX_X, AUX_Y] = FG_Squircular_Mapping (a1(k), b1(k), ss1(k), Theta, 1);
    
    XX2(k,:) = AUX_X;
    YY2(k,:) = AUX_Y;
    
end

%% Patch coordinates on the Hub (Cylindrical part)

NN = length(Theta);

COORD(1:NN,1) = transpose(XX);
COORD(1:NN,2) = transpose(YY);
COORD(1:NN,3) = 0;

for i = 1:Hub_NodR - 4                                                      % Hub_NodR - 4 is to eliminate the second and second-to-last radial divisions
    
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
%            Rolling up the patches on the cylinder
%--------------------------------------------------------------------------

PATCH = PatchRollingUp (Num_BD, COORD, D_Hub, H_Square);

%--------------------------------------------------------------------------
%                    Patches connectivities (ICON)
%--------------------------------------------------------------------------

ICON_Patch = Connectivities_1 (Hub_NodR-2, NN);

%% Coordinates of the Hub-Blade connection

X_Connect = D_Blade/2*cos(Theta);
Y_Connect = D_Blade/2*sin(Theta);

LCylinder = (D_Rot - D_Hub) / 2;

DZ        = linspace (0, LCylinder, Hub_NodZ);

k = 1;

for i = 1:length(DZ)
    
    for j = 1:length(X_Connect)
        
        COORD_1(k,1:3) = [X_Connect(j), Y_Connect(j), DZ(i)];
        k = k + 1;
        
    end 
    
end

%--------------------------------------------------------------------------
%                    Displacement of the connection part
%--------------------------------------------------------------------------

T2 = RotationMatrix( 2 , Beta );

for i = 1:size(COORD_1,1)
    
    CONNECT(1).XYZ(i,1:3) = transpose (T2 * transpose(COORD_1(i,1:3)) + [0; 0; D_Hub/2]);
    
end

CONNECT(1).XYZ(1:NN,1:3) = PATCH(1).XYZ(1:NN,1:3);

%--------------------------------------------------------------------------
%       Coordinates of the connections (according number of blades)
%--------------------------------------------------------------------------

DGamma = 2 * pi / Num_BD;

for i = 2:Num_BD

    TNB = RotationMatrix( 1 , DGamma*(i-1) );
    for j = 1:size(COORD_1,1)
        CONNECT(i).XYZ(j,1:3) = transpose (TNB * transpose(CONNECT(1).XYZ(j,1:3)));
    end

end

%--------------------------------------------------------------------------
%                Hub-blade connnection connectivities (ICON)
%--------------------------------------------------------------------------

ICON_HubBlade = Connectivities (length(DZ), length(X_Connect));

%% Coordinates of the Hub Nose

NN1  = ((Hub_NodC + 3) / 4);                                 % Number of nodes on the half patch of the hub (cylinder part) 
NN2  = (((Hub_NodC + 3) / 4) - 1) / 2 + 1; 

LOC1 = ((Hub_NodR - 3) * Hub_NodC + 1):((Hub_NodR - 3) * Hub_NodC + NN2);     % Position of the nodes on the half patch of the hub
LOC2 = ((Hub_NodR - 2) * Hub_NodC - NN2 + 1):(Hub_NodR - 2) * Hub_NodC;

LOC = [LOC2, LOC1(2:end)];

NOSEAUX(1).XYZ = PATCH(1).XYZ(LOC,:);
NOSEEND = NOSEAUX(1).XYZ;
NOSEAUX(1).XYZ(end,:) = [];
NOSE = NOSEAUX(1).XYZ;

DGamma = -2 * pi / Num_BD;

for i = 2:Num_BD

    TNB = RotationMatrix( 1 , DGamma*(i-1) );
    if i == Num_BD
        for j = 1:size(NOSEEND,1)
            NOSEAUX(i).XYZ(j,1:3) = transpose (TNB * transpose(NOSEEND(j,1:3)));
        end
    else
        for j = 1:size(NOSEAUX(1).XYZ,1)
            NOSEAUX(i).XYZ(j,1:3) = transpose (TNB * transpose(NOSEAUX(1).XYZ(j,1:3)));
        end
    end
    NOSE = [NOSE; NOSEAUX(i).XYZ];

end

if Nose_Shape == 1
    
    DX = (D_Hub / 2 - D_Hub/2*PerTrim) * sin(linspace(0, pi/2, Hub_NodN));
    NNHUB = size(NOSE,1);
    for i = 1:length(DX)
        I1 = 1 + NNHUB*(i-1);
        I2 = NNHUB + NNHUB*(i-1);
        RR = sqrt((D_Hub/2)^2 - (DX(i))^2);
        Factor = RR / (D_Hub/2);
        NOSEFINAL.XYZ(I1:I2, 2:3) = NOSE(:,2:3)*Factor;
        NOSEFINAL.XYZ(I1:I2, 1)   = DX(i);
    end
    NOSEFINAL.XYZ(:,1) = NOSEFINAL.XYZ(:,1) + L_Hub;
    
elseif Nose_Shape == 2
    
    a = (D_Hub/2)^2;
    b = a / (LNose);
    DX = (LNose - LNose*PerTrim) * sin(linspace(0,pi/2,Hub_NodN));
    NNHUB = size(NOSE,1);
    for i = 1:length(DX)
        I1 = 1 + NNHUB*(i-1);
        I2 = NNHUB + NNHUB*(i-1);
        RR = sqrt(a - b*DX(i));
        Factor = RR / (D_Hub/2); 
        NOSEFINAL.XYZ(I1:I2, 2:3) = NOSE(:,2:3)*Factor;
        NOSEFINAL.XYZ(I1:I2, 1) = DX(i);  
    end
    NOSEFINAL.XYZ(:,1) = NOSEFINAL.XYZ(:,1) + L_Hub;  
    
elseif Nose_Shape == 3
    
    DX = (LNose - LNose*PerTrim) * sin(linspace(0,pi/2,Hub_NodN));
    NNHUB = size(NOSE,1);
    for i = 1:length(DX) 
        I1 = 1 + NNHUB*(i-1);
        I2 = NNHUB + NNHUB*(i-1);
        [HH] = Hermite (DX(i), 0, LNose);
        RR = HH.h1*(D_Hub/2) + HH.h4*(-pi/2); 
        Factor = RR / (D_Hub/2);
        NOSEFINAL.XYZ(I1:I2, 2:3) = NOSE(:,2:3)*Factor;
        NOSEFINAL.XYZ(I1:I2, 1) = DX(i); 
    end
    NOSEFINAL.XYZ(:,1) = NOSEFINAL.XYZ(:,1) + L_Hub;    
    
end

%--------------------------------------------------------------------------
%                    Hub nose connectivities (ICON)
%--------------------------------------------------------------------------

ICON_HubNose = Connectivities_1 (length(DX), NNHUB);

%--------------------------------------------------------------------------
%% Control points and normals

XYZ_CP = ControlPoints (NOSEFINAL.XYZ, ICON_HubNose, size(ICON_HubNose,1));
NOR    = NormallVector (NOSEFINAL.XYZ, ICON_HubNose, size(ICON_HubNose,1));

HUB_AERO(Indx).XYZCPNose  = XYZ_CP;
HUB_AERO(Indx).NormalNose = NOR;

for i = 1:Num_BD

    XYZ_CP = ControlPoints (PATCH(i).XYZ, ICON_Patch, size(ICON_Patch,1));
    NOR    = NormallVector (PATCH(i).XYZ, ICON_Patch, size(ICON_Patch,1));

    HUB_AERO(Indx).PATCH(i).XYZCP  = XYZ_CP;
    HUB_AERO(Indx).PATCH(i).Normal = NOR;

    XYZ_CP = ControlPoints (CONNECT(i).XYZ, ICON_HubBlade, size(ICON_HubBlade,1));
    NOR    = NormallVector (CONNECT(i).XYZ, ICON_HubBlade, size(ICON_HubBlade,1));

    HUB_AERO(Indx).CONNECT(i).XYZCP  = XYZ_CP;
    HUB_AERO(Indx).CONNECT(i).Normal = NOR;

end


%%

for i = 1:Num_BD
    HUB_AERO(Indx).PATCH(i).XYZ = PATCH(i).XYZ;
    HUB_AERO(Indx).CONNECT(i).XYZ = CONNECT(i).XYZ;
end

HUB_AERO(Indx).XYZNose  = NOSEFINAL.XYZ;

HUB_AERO(Indx).ICONPatch     = ICON_Patch;
HUB_AERO(Indx).ICONConnect   = ICON_HubBlade;
HUB_AERO(Indx).ICONNose      = ICON_HubNose;

HUB_AERO(Indx).NNPatch = size(PATCH(1).XYZ,1);
HUB_AERO(Indx).NPPatch = size(ICON_Patch,1);

HUB_AERO(Indx).NNConnect = size(CONNECT(1).XYZ,1);
HUB_AERO(Indx).NPConnect = size(ICON_HubBlade,1);

HUB_AERO(Indx).NNHubNose = size(NOSEFINAL.XYZ,1);
HUB_AERO(Indx).NPHubNose = size(ICON_HubNose,1);

HUB_AERO(Indx).NNPatchC   = NN;
HUB_AERO(Indx).NNPatchR   = Hub_NodR-2;
HUB_AERO(Indx).NNConnectC = Hub_NodC;
HUB_AERO(Indx).NNConnectZ = Hub_NodZ;
HUB_AERO(Indx).NNNoseC    = NNHUB;
HUB_AERO(Indx).NNNoseX    = length(DX);


    
end

%% PATCH ROLL-UP FUNCTION

function PATCH = PatchRollingUp (Num_BD, COORD, D_Hub, H_Square)

AuxAngle = 2*pi / Num_BD / 2;

NN = size(COORD,1);

for i = 1:NN

    PATCH(1).XYZ(i,1) = COORD(i,1);
    PATCH(1).XYZ(i,2) = D_Hub/2 * sin (COORD(i,2)*AuxAngle*(1/H_Square));
    PATCH(1).XYZ(i,3) = D_Hub/2 * cos (COORD(i,2)*AuxAngle*(1/H_Square));

end

DGamma = 2 * pi / Num_BD;

for i = 2:Num_BD

    Gamma = DGamma * (i-1);
    TNB   = [1 0 0; 0 cos(Gamma) -sin(Gamma); 0 sin(Gamma) cos(Gamma)];
    for j = 1:NN
        PATCH(i).XYZ(j,1:3) = transpose (TNB * transpose(PATCH(1).XYZ(j,1:3)));
    end


end

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
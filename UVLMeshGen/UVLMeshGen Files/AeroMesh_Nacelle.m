function [NACELLE_AERO] = AeroMesh_Nacelle (DATA, NACELLE_AERO, Indx)

%% Auxiliar Data

D_Nac     = DATA(Indx).RadNac * 2;       % Nacelle Diameter
D_NacRear = DATA(Indx).RTailNac * 2;     % Nacelle rear diameter
L_Nac     = DATA(Indx).LCylNac/2;        % Nacelle length (Cylindrical part)
L_Tail    = DATA(Indx).LTailNac;         % Length of the nacelle tail
L_CTN     = DATA(Indx).LConTNac;         % Length of the nacelle-tower coupling
Gamma     = DATA(Indx).Tilt;             % Tilt angle
D_Tower   = DATA(Indx).RConTNac * 2;     % Diameter of the nacelle-tower coupling (tower top diameter)

Tail_Shape   = DATA(Indx).ShapeTailNac;  % Shape of the nacelle tail

Nac_NodC     = DATA(Indx).NCircNac;      % Number of nodes along the circunferential direction (on the coupling)
Nac_NodR     = DATA(Indx).NRadNac;       % Number of nodes along the radial direction (division based on concentric squares)
Nac_NodC_Cyl = DATA(Indx).NCircCylNac;   % Number of nodes along the circunferential direction in the cylindrical part of the nacelle
Nac_NodTail  = DATA(Indx).NTailNac;      % Number of nodes along the longitudinal direction of nacelle tail
Nac_NodZ     = DATA(Indx).NZCoupNac;     % Number of nodes along the nacelle-tower coupling zone

%% Hub patch data

Gamma   = -1.0 * Gamma * pi /180;

H_Square = (D_Nac/2 * (2*pi/3)) / 2;                                  % Height of the square containing the hole (hub-blade intersection)

a_axis = D_Tower / 2 * (cos(Gamma) + sin(Gamma)*tan(Gamma));          % Major semi-axis of the ellipse on the hub-blade intersection
b_axis = asin(D_Tower/D_Nac) * 3 * H_Square/pi;  

% Minor semi-axis of the ellipse on the hub-blade intersection

%% Square coordinates (From circle to square) / FG-Squircular mapping

% x = a sgn(cost)/(s sqrt(2) abs(sint)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))
% y = a sgn(sint)/(s sqrt(2) abs(cost)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))

% where s is a parameter which is used to blend the circle and the square smoothly
% Reference: Analytical Methods for Squaring the Disc (Chamberlain Fong)

Theta = linspace(0, 2*pi, Nac_NodC);                                       % Circunferential division along the nacelle-tower hole

XX = a_axis * cos(Theta);                                                  % X-coordinate along the hub-blade hole
YY = b_axis * sin(Theta);                                                  % Y-coordinate along the hub-blade hole

DAng = linspace (0, pi, Nac_NodR);                                         % Incremental angle used to make a radial division (sine-shape) from ellipse to square

a1 = a_axis + (L_Nac - a_axis)*(1 + cos(DAng))/2;                          % Major semi-axis for different radial coordinates
b1 = b_axis + (H_Square - b_axis)*(1 + cos(DAng))/2;                       % Minor semi-axis for different radial coordinates

a1 = sort(a1, 'ascend');
b1 = sort(b1, 'ascend');

ss1 = linspace (0, pi/2, Nac_NodR);                                        

ss1 = sin(ss1);                                                            % incremental FG-squircicurlar parameter (s = 0: circle/ellipse // s = 1: square)

for k = 1:Nac_NodR
    
    [AUX_X, AUX_Y] = FG_Squircular_Mapping (a1(k), b1(k), ss1(k), Theta, 1);
    
    XX2(k,:) = AUX_X;
    YY2(k,:) = AUX_Y;           
        
end

%% Patch coordinates on the Nacelle (Cylindrical part)

NN = length(Theta);

COORD(1:NN,1) = transpose(XX);
COORD(1:NN,2) = transpose(YY);
COORD(1:NN,3) = 0;

for i = 1:Nac_NodR - 4                                                      % Nac_NodR - 4 is to eliminate the second and second-to-last radial divisions
    
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
     
for i = 1:size(COORD,1)
    
    XYZ(i,1) = COORD(i,1);
    XYZ(i,2) = D_Nac/2 * sin (COORD(i,2)*pi/3*(1/H_Square));
    XYZ(i,3) = D_Nac/2 * cos (COORD(i,2)*pi/3*(1/H_Square));
    
end
        
%--------------------------------------------------------------------------
%                    Patches connectivities (ICON)
%--------------------------------------------------------------------------

ICON_Patch = Connectivities_1 (Nac_NodR-2, NN);

NACELLE_AERO(Indx).NNPatchR = Nac_NodR - 2;
NACELLE_AERO(Indx).NNPatchC = NN;

%% Coordinates on the Nacelle (Cylindrical part) out of the patch

N_Aux = (((Nac_NodC + 3) / 4) - 1) / 2 + 1;                                              % Number of nodes on the half patch of the nacelle (cylinder part)

LOC   = ((Nac_NodR - 3) * Nac_NodC + N_Aux):((Nac_NodR - 3) * Nac_NodC + 2*N_Aux-1);     % Position of the nodes on the half patch of the nacelle

XYZ_AUX = XYZ(LOC,:);
AUX     = XYZ_AUX(end-1:-1:1,:);
AUX(:,1)= -1.0 * AUX(:,1);
XYZ_AUX = [XYZ_AUX; AUX];

DX = XYZ_AUX;

Alpha = linspace(5*pi/6, 13*pi/6, Nac_NodC_Cyl);

Y_CylNac = D_Nac/2*cos(Alpha);
Z_CylNac = D_Nac/2*sin(Alpha);

NNNAC    = length(Y_CylNac);

for i = 1:length(DX)

    I1 = 1 + NNNAC*(i-1);
    I2 = NNNAC + NNNAC*(i-1);

    XYZ_CYLNAC(I1:I2, 2:3) = [transpose(Y_CylNac), transpose(Z_CylNac)];
    XYZ_CYLNAC(I1:I2, 1) = DX(i);

end

%--------------------------------------------------------------------------
%                  Cylindrical Nacelle connectivities (ICON)
%--------------------------------------------------------------------------

ICON_CYLNAC = Connectivities_1 (length(DX), NNNAC);

NACELLE_AERO(Indx).NNCylL = length(DX);
NACELLE_AERO(Indx).NNCylC = NNNAC;

%% Coordinates of the nacelle-tower connection

X_Connect = D_Tower/2*cos(Theta);
Y_Connect = D_Tower/2*sin(Theta);

DZ        = linspace (0, L_CTN, Nac_NodZ);

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

T2 = RotationMatrix( 2 , Gamma );

for i = 1:size(COORD_1,1)
    
    XYZ_Connect(i,1:3) = transpose (T2 * transpose(COORD_1(i,1:3)) + [0; 0; D_Nac/2]);
    
end

XYZ_Connect(1:NN,1:3) = XYZ(1:NN,1:3);

%--------------------------------------------------------------------------
%                Nacelle-tower connnection connectivities (ICON)
%--------------------------------------------------------------------------

ICON_NacTower = Connectivities (length(DZ), length(X_Connect));

NACELLE_AERO(Indx).NNConnectZ = length(DZ);
NACELLE_AERO(Indx).NNConnectC = length(X_Connect);

%% Coordinates of the Nacelle Tail

N_Aux = (((Nac_NodC + 3) / 4) - 1) / 2 + 1;                                      % Number of nodes on the half patch of the hub (cylinder part)

LOC   = ((Nac_NodR - 3) * Nac_NodC + 1):((Nac_NodR - 3) * Nac_NodC + N_Aux);     % Position of the nodes on the half patch of the hub

XYZ_AUX      = XYZ(LOC,:);
XYZ_AUX      = XYZ_AUX(end:-1:1,:);
XYZ_AUX(:,1) = -1.00 * XYZ_AUX(:,1);

XYZ_AUX_1      = XYZ_AUX(end-1:-1:1,:);
XYZ_AUX_1(:,2) = -1.00 * XYZ_AUX_1(:,2);

XYZ_AUX_2    = XYZ_CYLNAC(2:Nac_NodC_Cyl,:);
XYZ_AUX_2(:,1) = -1.00 * XYZ_AUX_2(:,1);

XYZ_AUX_3 = [XYZ_AUX; XYZ_AUX_1; XYZ_AUX_2];

switch Tail_Shape

%--------------------------------------------------------------------------
%            PARABOLIC SHAPE OF THE NACELLE (CIRCULAR ENDING)
%--------------------------------------------------------------------------

    case (1)

        DX = L_Tail * sin(linspace(0,pi/2, Nac_NodTail));

        a = (D_Nac/2)^2;
        b = (a - (D_NacRear/2)^2) / (L_Tail);

        NNNAC    = size(XYZ_AUX_3,1);

        for i = 1:length(DX)

            I1 = 1 + NNNAC*(i-1);
            I2 = NNNAC + NNNAC*(i-1);

            RR = sqrt(a - b*DX(i));

            Factor = RR / (D_Nac/2);

            XYZ_NACELLE(I1:I2, 2:3) = [XYZ_AUX_3(:,2), XYZ_AUX_3(:,3)]*Factor;
            XYZ_NACELLE(I1:I2, 1) = DX(i);

        end

        XYZ_NACELLE(:,1) = XYZ_NACELLE(:,1);
        XYZ_NACELLE(:,1) = -1.00 * XYZ_NACELLE(:,1) - L_Nac;

        %--------------------------------------------------------------------------
        %            PARABOLIC SHAPE OF THE NACELLE (SQUARE ENDING)
        %--------------------------------------------------------------------------

    case (2)

        DX = L_Tail * sin(linspace(0,pi/2, Nac_NodTail));

        a = (D_Nac/2)^2;
        b = (a - (D_NacRear/2)^2) / (L_Tail);

        NNNAC    = size(XYZ_AUX_3,1);

        ss = linspace(0, 1, length(DX));

        for i = 1:length(DX)

            I1 = 1 + NNNAC*(i-1);
            I2 = NNNAC + NNNAC*(i-1);

            RR = sqrt(a - b*DX(i));

            if (i == length(DX)-1)

                RR = RR * 0.9;

            elseif (i == length(DX))

                RR = RR * 0.7;

            end

            [Ang] = AngularConversion (XYZ_AUX_3(:,2:3));

            [AUX_Y, AUX_Z] = FG_Squircular_Mapping (RR, RR, ss(i), Ang, 1);

            XYZ_NACELLE(I1:I2, 2:3) = [transpose(AUX_Y), transpose(AUX_Z)];
            XYZ_NACELLE(I1:I2, 1) = DX(i);

        end

        XYZ_NACELLE(:,1) = XYZ_NACELLE(:,1);
        XYZ_NACELLE(:,1) = -1.00 * XYZ_NACELLE(:,1) - L_Nac;

        %--------------------------------------------------------------------------
        %            CUBIC SHAPE OF THE NACELLE (CIRCULAR ENDING)
        %--------------------------------------------------------------------------

    case (3)

        DX = L_Tail * sin(linspace(0,pi/2, Nac_NodTail));

        if L_Tail > (D_Nac-D_NacRear)

            BCond_Angle = -pi/(L_Tail/((D_Nac-D_NacRear)/2));

        else

            BCond_Angle = -pi/2;

        end

        NNNAC    = size(XYZ_AUX_3,1);

        for i = 1:length(DX)

            I1 = 1 + NNNAC*(i-1);
            I2 = NNNAC + NNNAC*(i-1);

            [HH] = Hermite (DX(i), 0, L_Tail);
            RR = HH.h1*(D_Nac/2-D_NacRear/2) + HH.h4*(BCond_Angle);
            RR = RR + D_NacRear/2;

            Factor = RR / (D_Nac/2);

            XYZ_NACELLE(I1:I2, 2:3) = [XYZ_AUX_3(:,2), XYZ_AUX_3(:,3)]*Factor;
            XYZ_NACELLE(I1:I2, 1) = DX(i);

        end

        XYZ_NACELLE(:,1) = XYZ_NACELLE(:,1);
        XYZ_NACELLE(:,1) = -1.00 * XYZ_NACELLE(:,1) - L_Nac;

        %--------------------------------------------------------------------------
        %               CUBIC SHAPE OF THE NACELLE (SQUARE ENDING)
        %--------------------------------------------------------------------------

    case (4)

        DX = L_Tail * sin(linspace(0,pi/2, Nac_NodTail));

        if L_Tail > (D_Nac-D_NacRear)
            BCond_Angle = -pi/(L_Tail/((D_Nac-D_NacRear)/2));
        else
            BCond_Angle = -pi/2;
        end

        NNNAC    = size(XYZ_AUX_3,1);
        ss = linspace(0, 1, length(DX));

        for i = 1:length(DX)

            I1 = 1 + NNNAC*(i-1);
            I2 = NNNAC + NNNAC*(i-1);

            [HH] = Hermite (DX(i), 0, L_Tail);
            RR = HH.h1*(D_Nac/2-D_NacRear/2) + HH.h4*(BCond_Angle);
            RR = RR + D_NacRear/2;

            if (i == length(DX)-1)
                RR = RR * 0.9;
            elseif (i == length(DX))
                RR = RR * 0.7;
            end

            [Ang] = AngularConversion (XYZ_AUX_3(:,2:3));

            [AUX_Y, AUX_Z] = FG_Squircular_Mapping (RR, RR, ss(i), Ang, 1);

            XYZ_NACELLE(I1:I2, 2:3) = [transpose(AUX_Y), transpose(AUX_Z)];
            XYZ_NACELLE(I1:I2, 1) = DX(i);

        end

        XYZ_NACELLE(:,1) = XYZ_NACELLE(:,1);
        XYZ_NACELLE(:,1) = -1.00 * XYZ_NACELLE(:,1) - L_Nac;

end

%--------------------------------------------------------------------------
%                Nacelle tail connectivities (ICON)
%--------------------------------------------------------------------------

ICON_TAIL = Connectivities_1 (length(DX), NNNAC);

NACELLE_AERO(Indx).NNTailL = length(DX);
NACELLE_AERO(Indx).NNTailC = NNNAC;

%%-------------------------------------------------------------------------
% Control points and Normalls

XYZCPPatch  = ControlPoints (XYZ, ICON_Patch, size(ICON_Patch,1));
NORPatch    = NormallVector (XYZ, ICON_Patch, size(ICON_Patch,1));

XYZCPCylnac = ControlPoints (XYZ_CYLNAC, ICON_CYLNAC, size(ICON_CYLNAC,1));
NORCylnac    = NormallVector (XYZ_CYLNAC, ICON_CYLNAC, size(ICON_CYLNAC,1));

XYZCPConnect = ControlPoints (XYZ_Connect, ICON_NacTower, size(ICON_NacTower,1));
NORConnect   = NormallVector (XYZ_Connect, ICON_NacTower, size(ICON_NacTower,1));

XYZCPTail = ControlPoints (XYZ_NACELLE, ICON_TAIL, size(ICON_TAIL,1));
NORTail   = NormallVector (XYZ_NACELLE, ICON_TAIL, size(ICON_TAIL,1));

%%

NACELLE_AERO(Indx).XYZPatch   = XYZ;
NACELLE_AERO(Indx).XYZPatchFlat = COORD;
NACELLE_AERO(Indx).XYZCyl     = XYZ_CYLNAC;
NACELLE_AERO(Indx).XYZConnect = XYZ_Connect;
NACELLE_AERO(Indx).XYZTail    = XYZ_NACELLE;

NACELLE_AERO(Indx).ICONPatch    = ICON_Patch;
NACELLE_AERO(Indx).ICONCyl      = ICON_CYLNAC;
NACELLE_AERO(Indx).ICONConnect  = ICON_NacTower;
NACELLE_AERO(Indx).ICONTail     = ICON_TAIL;

NACELLE_AERO(Indx).NNPatch = size(XYZ,1);
NACELLE_AERO(Indx).NPPatch = size(ICON_Patch,1);

NACELLE_AERO(Indx).NNTail  = size(XYZ_NACELLE,1);
NACELLE_AERO(Indx).NPTail  = size(ICON_TAIL,1);

NACELLE_AERO(Indx).NNCyl   = size(XYZ_CYLNAC,1);
NACELLE_AERO(Indx).NPCyl   = size(ICON_CYLNAC,1);

NACELLE_AERO(Indx).NNConnect = size(XYZ_Connect,1);
NACELLE_AERO(Indx).NPConnect = size(ICON_NacTower,1);

T1 = RotationMatrix( 1 , pi );
T2 = RotationMatrix( 2 , Gamma );

for i = 1:NACELLE_AERO(Indx).NNPatch
    NACELLE_AERO(Indx).XYZPatch(i,:) = transpose(T2 * T1 * transpose(NACELLE_AERO(Indx).XYZPatch(i,:)));
end

for i = 1:NACELLE_AERO(Indx).NNTail
    NACELLE_AERO(Indx).XYZTail(i,:) = transpose(T2 * T1 * transpose(NACELLE_AERO(Indx).XYZTail(i,:)));
end

for i = 1:NACELLE_AERO(Indx).NNCyl
    NACELLE_AERO(Indx).XYZCyl(i,:) = transpose(T2 * T1 * transpose(NACELLE_AERO(Indx).XYZCyl(i,:)));
end

for i = 1:NACELLE_AERO(Indx).NNConnect
    NACELLE_AERO(Indx).XYZConnect(i,:) = transpose(T2 * T1 * transpose(NACELLE_AERO(Indx).XYZConnect(i,:)));
end

XYZCPPatch  = ControlPoints (NACELLE_AERO(Indx).XYZPatch, ICON_Patch, size(ICON_Patch,1));
NORPatch    = NormallVector (NACELLE_AERO(Indx).XYZPatch, ICON_Patch, size(ICON_Patch,1));

XYZCPCylnac = ControlPoints (NACELLE_AERO(Indx).XYZCyl, ICON_CYLNAC, size(ICON_CYLNAC,1));
NORCylnac   = NormallVector (NACELLE_AERO(Indx).XYZCyl, ICON_CYLNAC, size(ICON_CYLNAC,1));

XYZCPConnect = ControlPoints (NACELLE_AERO(Indx).XYZConnect, ICON_NacTower, size(ICON_NacTower,1));
NORConnect   = NormallVector (NACELLE_AERO(Indx).XYZConnect, ICON_NacTower, size(ICON_NacTower,1));

XYZCPTail = ControlPoints (NACELLE_AERO(Indx).XYZTail, ICON_TAIL, size(ICON_TAIL,1));
NORTail   = NormallVector (NACELLE_AERO(Indx).XYZTail, ICON_TAIL, size(ICON_TAIL,1));

NACELLE_AERO(Indx).XYZCPPatch   = XYZCPPatch;
NACELLE_AERO(Indx).XYZCPCyl     = XYZCPCylnac;
NACELLE_AERO(Indx).XYZCPConnect = XYZCPConnect;
NACELLE_AERO(Indx).XYZCPTail    = XYZCPTail;

NACELLE_AERO(Indx).NormalPatch   = NORPatch;
NACELLE_AERO(Indx).NormalCyl     = NORCylnac;
NACELLE_AERO(Indx).NormalConnect = NORConnect;
NACELLE_AERO(Indx).NormalTail    = NORTail;
    
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

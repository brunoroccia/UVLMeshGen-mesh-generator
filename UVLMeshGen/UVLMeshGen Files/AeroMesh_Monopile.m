function [MONOPILE_AERO] = AeroMesh_Monopile (DATA, MONOPILE_AERO, Indx)

%% Auxiliar Data

Monop_H  = DATA(Indx).LMon;             % Monopile height
Monop_DT = DATA(Indx).RWaterMon * 2;    % Monopile water line diameter
Monop_DB = DATA(Indx).RDeepMon * 2;     % Monopile bottom diameter

Monop_NodZ = DATA(Indx).NZMon;          % Number of aerodynamic nodes along the longitudinal direction of the monopile
Monop_NodC = DATA(Indx).NCircMon;       % Number of aerodynamic nodes along the circunferential direction of the monopile


%% Coordinates on the Tower
    
DZ = linspace (0, Monop_H, Monop_NodZ);

Alpha = linspace(0, 2*pi, Monop_NodC);
        
k = 1;

XYZ_Monopile = zeros (Monop_NodC*Monop_NodZ, 3);

for i = 1:Monop_NodZ
    
    DD = DZ(i)/(2*Monop_H) * (Monop_DT - Monop_DB);    % Computation of the diameter as function of coordinate Z (linear distribution)
    Di = 2*DD + Monop_DB;
    
    for j = 1:Monop_NodC

        XYZ_Monopile(k,1) = Di*cos(Alpha(j))/2;       % Tangential division
        XYZ_Monopile(k,2) = Di*sin(Alpha(j))/2;
        XYZ_Monopile(k,3) = DZ(i);
        
        k = k + 1;
        
    end
    
end

%--------------------------------------------------------------------------
%                    Monopile connectivities (ICON)
%--------------------------------------------------------------------------

ICON_Monopile = Connectivities (length(DZ), Monop_NodC);

%%--------------------------------------------------------------------------
% Control poits and normalls

XYZ_CP = ControlPoints (XYZ_Monopile, ICON_Monopile, size(ICON_Monopile,1));
NOR    = NormallVector (XYZ_Monopile, ICON_Monopile, size(ICON_Monopile,1));

%%

MONOPILE_AERO(Indx).XYZ    = XYZ_Monopile;
MONOPILE_AERO(Indx).XYZCP  = XYZ_CP;
MONOPILE_AERO(Indx).Normal = NOR;

MONOPILE_AERO(Indx).ICON   = ICON_Monopile;

MONOPILE_AERO(Indx).NN     = size(XYZ_Monopile,1);
MONOPILE_AERO(Indx).NP     = size(ICON_Monopile,1);
    
MONOPILE_AERO(Indx).NNC    = Monop_NodC;
MONOPILE_AERO(Indx).NNZ    = length(DZ);    

    
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

function [TOWER_AERO] = AeroMesh_Tower (DATA, TOWER_AERO, Indx)

%% Auxiliar Data

Tower_H  = DATA(Indx).TowHeight;        % Tower altitude
Tower_DT = DATA(Indx).RConTTow * 2;     % Tower top diameter
Tower_DB = DATA(Indx).RGroundTow * 2;   % Tower bottom diameter

Tower_NodZ = DATA(Indx).NZTow;          % Number of aerodynamic nodes along the longitudinal direction of the tower
Tower_NodC = DATA(Indx).NCircTow;       % Number of aerodynamic nodes along the circunferential direction of the tower


%% Coordinates on the Tower
    
DZ = linspace (0, Tower_H, Tower_NodZ);

Alpha = linspace(0, 2*pi, Tower_NodC);
        
k = 1;

XYZ_Tower = zeros (Tower_NodC*Tower_NodZ, 3);

for i = 1:Tower_NodZ
    
    DD = DZ(i)/(2*Tower_H) * (Tower_DT - Tower_DB);    % Computation of the diameter as function of coordinate Z (linear distribution)
    Di = 2*DD + Tower_DB;
    
    for j = 1:Tower_NodC

        XYZ_Tower(k,1) = Di*cos(Alpha(j))/2;       % Tangential division
        XYZ_Tower(k,2) = Di*sin(Alpha(j))/2;
        XYZ_Tower(k,3) = DZ(i);
        
        k = k + 1;
        
    end
    
end

%--------------------------------------------------------------------------
%                    Tower connectivities (ICON)
%--------------------------------------------------------------------------

ICON_Tower = Connectivities (length(DZ), Tower_NodC);

%%-------------------------------------------------------------------------
% Control points and normalls

XYZ_CP = ControlPoints (XYZ_Tower, ICON_Tower, size(ICON_Tower,1));
NOR    = NormallVector (XYZ_Tower, ICON_Tower, size(ICON_Tower,1));

%%

TOWER_AERO(Indx).XYZ    = XYZ_Tower;

TOWER_AERO(Indx).XYZCP  = XYZ_CP;
TOWER_AERO(Indx).Normal = NOR;

TOWER_AERO(Indx).ICON   = ICON_Tower;

TOWER_AERO(Indx).NN     = size(XYZ_Tower,1);
TOWER_AERO(Indx).NP     = size(ICON_Tower,1);

TOWER_AERO(Indx).NNC    = Tower_NodC;
TOWER_AERO(Indx).NNR    = Tower_NodZ;
 
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

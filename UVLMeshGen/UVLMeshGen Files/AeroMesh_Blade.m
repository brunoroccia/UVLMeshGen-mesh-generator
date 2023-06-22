%**************************************************************************
% Function AeroMesh_Blade is called from MainProgram.m
% This function builds the blade-related UVLM mesh, including shedding
% zones and the characteristic length LC to be used in the UVLM code. The LC
% calculated here is an automatic alternative to the user-defined value.
% UNRC - UNC - CONICET (July 07, 2020)
% broccia@ing.unrc.edu.ar
%**************************************************************************

function [BLADE_AERO, CodError] = AeroMesh_Blade (DATA, BLADE, BLADE_AERO, CodError, Indx)

LBlade = BLADE(Indx).Zcoord(length(BLADE(Indx).Zcoord));
Zcoord = BLADE(Indx).Zcoord;

%% Blade length modification

Chord99  = ppval( BLADE(Indx).SChord, 0.99*LBlade );   % Chord at 99% of the span
Chord100 = ppval( BLADE(Indx).SChord,      LBlade );   % Chord at the blade tip

if( Chord100 < 0.5*Chord99 )
    inds = find( Zcoord > 0.99*LBlade );
    Zcoord(inds) = [];
    LBlade = max(Zcoord);     % Total length of the modified blade
end

%% Division along the blade spanwise (Z Coordinate)

% (Lifting surface Zone)

LSNN      = 1:DATA(Indx).NBldS-3;                                                       % Number -3 is to avoid small panels at the blade tip
LS_ZCoord = DATA(Indx).LSLBLd * sin( pi*(LSNN-1)/(2*DATA(Indx).NBldS) );
LastCoord = LS_ZCoord(length(LS_ZCoord));                                               % Last node located in the sine discretization section
Tip_Sec   = DATA(Indx).LSLBLd - LastCoord;                                              % Section located at the blade tip without discretization
LastPanel = LastCoord - LS_ZCoord(length(LS_ZCoord)-1);                                 % Size of the last panel in the sine discretization section
Tip_Coord = linspace(LastCoord, DATA(Indx).LSLBLd, ceil(Tip_Sec/LastPanel)+1 );         % Discretization of the blade tip section
LS_ZCoord = [LS_ZCoord, Tip_Coord(2:end)];                                              % Concatenation of the coordinate vector (without repit coordinates)
LSNN      = length(LS_ZCoord);                                                          % Number of nodes on the lifting surface

% (Blade Root Zone)

FirstPanel = LS_ZCoord(2) - LS_ZCoord(1);                                               % Size of the first panel in the sine discretization zone
Root_Coord = linspace(0, DATA(Indx).RLBld, ceil(DATA(Indx).RLBld/FirstPanel)+1 );       % Discretization of the blade root zone
BRNN       = length(Root_Coord);                                                        % Number of nodes on the blade root

% (Blade Transition Zone)

LTB        = Zcoord(length(Zcoord)) - DATA(Indx).LSLBLd - DATA(Indx).RLBld;             % Transition zone length
Tran_Coord = linspace(DATA(Indx).RLBld, DATA(Indx).RLBld+LTB, ceil(LTB/FirstPanel)+1 ); % Discretization of the blade transition zone
BTNN       = length(Tran_Coord);                                                        % Number of nodes on the blade transition zone

% (Data concerning the whole blade)

BLADE_ZCOORD = [Root_Coord, Tran_Coord(2:length(Tran_Coord)), DATA(Indx).RLBld+LTB+LS_ZCoord];    % Blade Z-coordinate without duplicate nodes at the root-transition section
NNBLADE      = BRNN + (BTNN - 1) + LSNN;                                                          % Blade nodes without duplicate nodes at the root-transition section

LS_ZCoord    = LS_ZCoord + LTB + DATA(Indx).RLBld;                                      % Lifting surface Z-coordinate
RT_ZCoord    = [Root_Coord, Tran_Coord(2:length(Tran_Coord))];                          % Root + Transition Z-coordinates

% (Data to be used for aerodynamic meshing)

LS_ZCoord_TOT   = kron (LS_ZCoord, ones(1, DATA(Indx).NBldC) );                         % Total Z coordinate for lifting surface
RT_ZCoord_TOT   = kron (RT_ZCoord, ones(1, 2*DATA(Indx).NBldC-1) );                     % Total Z coordinate for Root + Transition section
LS_ZCoord_TOT_1 = kron (LS_ZCoord, ones(1, 2*DATA(Indx).NBldC-1) );                     % Total Z coordinate for lifting surface considering thickness


%% Division along the blade chordwise (X-Y Coordinates)

% (Computation of X-Y coordinates of the lifting surface)

for i = 1 : DATA(Indx).NBldC

    CC(i,1) = (1/(DATA(Indx).NBldC-1))*(i-1);                                     % Chord Percentage 
    
    for k = 1 : length (BLADE(Indx).Zcoord)
        
        Extrados(k) = ppval(BLADE(Indx).PP_Airfoil{k,1}, CC(i,1));                 % Extrados thick at chord percentage CC and for all the span
        Intrados(k) = ppval(BLADE(Indx).PP_Airfoil{k,2}, CC(i,1));                 % Intrados thick at chord percentage CC and for all the span

    end
    
    PP_Ext_Span = spline( BLADE(Indx).Zcoord, Extrados );                         % Spline along the spanwise for chord percentage CC (Extrados)
    PP_Int_Span = spline( BLADE(Indx).Zcoord, Intrados );                         % Spline along the spanwise for chord percentage CC (Intrados)
    
    X_Ext       = ppval(PP_Ext_Span,BLADE(Indx).Zcoord);                          % Spline evaluation at chord percentage CC along the spanwise Extrados
    X_Int       = ppval(PP_Int_Span,BLADE(Indx).Zcoord);                          % Spline evaluation at chord percentage CC along the spanwise Intrados
    X_LS        = (X_Ext + X_Int) / 2;                                            % Determination of the mean surface coordinates
    PP_LS       = spline(BLADE(Indx).Zcoord, X_LS);                               % Spline along the spanwise for chord percentage CC (mean surface)
    
    XCoord_Ext(i,:)  = ppval(PP_Ext_Span, BLADE_ZCOORD(1+BRNN+(BTNN-1):end));   % Interpolated X coordinate (Extrados) - only for the lifting surface
    XCoord_Int(i,:)  = ppval(PP_Int_Span, BLADE_ZCOORD(1+BRNN+(BTNN-1):end));   % Interpolated X coordinate (Intrados) - only for the lifting surface
    XCoord_LS(i,:)   = ppval(PP_LS, BLADE_ZCOORD(1+BRNN+(BTNN-1):end));         % Interpolated X coordinate (mean surface) - only for the lifting surface
    
    XCoord_Cy_Ext(i,:) = ppval(PP_Ext_Span, BLADE_ZCOORD(1:BRNN));              % Interpolated X coordinate (Extrados) - Root section
    XCoord_Cy_Int(i,:) = ppval(PP_Int_Span, BLADE_ZCOORD(1:BRNN));              % Interpolated X coordinate (Intrados) - Root section
    
    XCoord_Tr_Ext(i,:) = ppval(PP_Ext_Span, BLADE_ZCOORD(1+BRNN:BRNN+(BTNN-1)));    % Interpolated X coordinate (Extrados) - Transition section
    XCoord_Tr_Int(i,:) = ppval(PP_Int_Span, BLADE_ZCOORD(1+BRNN:BRNN+(BTNN-1)));    % Interpolated X coordinate (Intrados) - Transition section    
    
end

LS_YCoord = kron (ones(1,size(XCoord_LS,2)), transpose(CC));               % Lifting surface Y coordinate (for all nodes in LS)
LS_XCoord = reshape (XCoord_LS, 1, numel(XCoord_LS));                      % Lifting surface X coordinate (for all nodes in LS)

%% X-Y Computation on the blade root section

AUX = [transpose(CC), transpose( CC(size(CC,1)-1:-1:1, 1) )];

Root_YCoord = kron (ones(1,size(XCoord_Cy_Ext,2)), AUX);

Root_XCoord = [];

for i=1:BRNN
    
    AUX_1 = [transpose(XCoord_Cy_Ext(:,i)), ...
            transpose(XCoord_Cy_Int(size(XCoord_Cy_Int,1)-1:-1:1,i))];
        
    Root_XCoord = [Root_XCoord, AUX_1];    
    
end

%% X-Y Computation on the blade transition section (for Aerodynamic Mesh)
% Linear interpolation from the root section to lifting surface

SEC0_X = Root_XCoord (end-(2*DATA(Indx).NBldC-1)+1:end); 
SEC0_Y = Root_YCoord (end-(2*DATA(Indx).NBldC-1)+1:end);

SEC1_X = [LS_XCoord(1:DATA(Indx).NBldC), LS_XCoord(DATA(Indx).NBldC-1:-1:1)]; 
SEC1_Y = AUX;

for i = 1:2*DATA(Indx).NBldC - 1

    Tran_X(i,:) = interp1([Tran_Coord(1) Tran_Coord(length(Tran_Coord))],[SEC0_X(i) SEC1_X(i)],  ...
                Tran_Coord);
            
    Tran_Y(i,:) = interp1([Tran_Coord(1) Tran_Coord(length(Tran_Coord))],[SEC0_Y(i) SEC1_Y(i)],  ...
                Tran_Coord);            
    
end

Tran_XCoord = reshape (Tran_X(:,2:end), 1, numel(Tran_X(:,2:end)));
Tran_YCoord = reshape (Tran_Y(:,2:end), 1, numel(Tran_Y(:,2:end)));

%% X-Y Computation on the blade transition section 
% Only used for a full blade with thickness along all the span

Tran_XCoord_1 = [];
Tran_YCoord_1 = [];

for i=1:BTNN-1
    
    AUX_1 = [transpose(XCoord_Tr_Ext(:,i)), ...
            transpose(XCoord_Tr_Int(size(XCoord_Tr_Int,1)-1:-1:1,i))];
        
    Tran_XCoord_1 = [Tran_XCoord_1, AUX_1];    
    
end

Tran_YCoord_1 = kron (ones(1,size(XCoord_Tr_Ext,2)), AUX);

%% X-Y Computation on the Lifting Surface section 
% Only used for a full blade with thickness along all the span

LS_XCoord_1 = [];
LS_YCoord_1 = [];

for i=1:LSNN
    
    AUX_1 = [transpose(XCoord_Ext(:,i)), ...
            transpose(XCoord_Int(size(XCoord_Int,1)-1:-1:1,i))];
        
    LS_XCoord_1 = [LS_XCoord_1, AUX_1];    
    
end

LS_YCoord_1 = kron (ones(1,size(XCoord_Ext,2)), AUX);

%% Blade fit to real dimensions (MESH 1: suitable for UVLM)

XYZ_1 = [Root_XCoord, Tran_XCoord; Root_YCoord, Tran_YCoord; ...
       RT_ZCoord_TOT];
   
XYZ_2 = [LS_XCoord; LS_YCoord; LS_ZCoord_TOT]; 

ZZCoord = [RT_ZCoord, LS_ZCoord];

[DEFORM, LBlade, CodError] = BendSweep_Blade (DATA, BLADE, ZZCoord, CodError, Indx);

if CodError.BldBend == 1
    return
end

for k = 1:BRNN + (BTNN - 1)
    
    I1     = 1 + (2*DATA(Indx).NBldC-1)*(k-1);
    I2     = (2*DATA(Indx).NBldC-1) + (2*DATA(Indx).NBldC-1)*(k-1);
   
    Beta   = ppval(BLADE(Indx).SBeta, XYZ_1(3,I1));
    Chord  = ppval(BLADE(Indx).SChord, XYZ_1(3,I1));
    Offset = ppval(BLADE(Indx).SOffset, XYZ_1(3,I1));
    
    XYZ_1(1,I1:I2)   = XYZ_1(1,I1:I2) * Chord;
    XYZ_1(2,I1:I2)   = XYZ_1(2,I1:I2) * Chord;
    
    XYZ_1(2,I1:I2)   = XYZ_1(2,I1:I2) - Offset * Chord;
    
    T3 = RotationMatrix ( 3, deg2rad( Beta ) );
    T1 = RotationMatrix ( 1, -1.0*DEFORM.Theta2(k) );
    T2 = RotationMatrix ( 2, DEFORM.Theta1(k) );
    
    TNB = T1*T2*T3;
    
    XYZ_1(3,I1:I2)   = 0.0;
    
    XYZ_1(1:3,I1:I2) = TNB * XYZ_1(1:3,I1:I2) + [DEFORM.Deflec_1(k); DEFORM.Deflec_2(k); DEFORM.RC(k)];
    
end

kkFlag = k;

for k = 1:LSNN
    
    I1     = 1 + DATA(Indx).NBldC*(k-1);
    I2     = DATA(Indx).NBldC + DATA(Indx).NBldC*(k-1);
   
    Beta   = ppval(BLADE(Indx).SBeta, XYZ_2(3,I1));
    Chord  = ppval(BLADE(Indx).SChord, XYZ_2(3,I1));
    Offset = ppval(BLADE(Indx).SOffset, XYZ_2(3,I1));
    
    XYZ_2(1,I1:I2)   = XYZ_2(1,I1:I2) * Chord;
    XYZ_2(2,I1:I2)   = XYZ_2(2,I1:I2) * Chord;
    
    XYZ_2(2,I1:I2)   = XYZ_2(2,I1:I2) - Offset * Chord;    
    
    T3 = RotationMatrix ( 3, deg2rad( Beta ) );
    T1 = RotationMatrix ( 1, -1.0*DEFORM.Theta2(k+kkFlag) );
    T2 = RotationMatrix ( 2, DEFORM.Theta1(k+kkFlag) );
    
    TNB = T1*T2*T3;
    
    XYZ_2(3,I1:I2)   = 0.0;
    
    XYZ_2(1:3,I1:I2) = TNB * XYZ_2(1:3,I1:I2) + [DEFORM.Deflec_1(k+kkFlag); DEFORM.Deflec_2(k+kkFlag); DEFORM.RC(k+kkFlag)];    
    
end

%% Blade fit to real dimensions (MESH 2: with LS thickness)

XYZ_1_1 = [Root_XCoord, Tran_XCoord_1; Root_YCoord, Tran_YCoord_1; ...
       RT_ZCoord_TOT];
   
XYZ_2_2 = [LS_XCoord_1; LS_YCoord_1; LS_ZCoord_TOT_1]; 

for k = 1:BRNN + (BTNN - 1)
    
    I1     = 1 + (2*DATA(Indx).NBldC-1)*(k-1);
    I2     = (2*DATA(Indx).NBldC-1) + (2*DATA(Indx).NBldC-1)*(k-1);
   
    Beta   = ppval(BLADE(Indx).SBeta, XYZ_1_1(3,I1));
    Chord  = ppval(BLADE(Indx).SChord, XYZ_1_1(3,I1));
    Offset = ppval(BLADE(Indx).SOffset, XYZ_1_1(3,I1));
    
    XYZ_1_1(1,I1:I2)   = XYZ_1_1(1,I1:I2) * Chord;
    XYZ_1_1(2,I1:I2)   = XYZ_1_1(2,I1:I2) * Chord;
    
    XYZ_1_1(2,I1:I2)   = XYZ_1_1(2,I1:I2) - Offset * Chord;
    
    TNB = RotationMatrix ( 3, deg2rad( Beta ) );
    
    XYZ_1_1(1:3,I1:I2) = TNB * XYZ_1_1(1:3,I1:I2);
    
end

for k = 1:LSNN
    
    I1     = 1 + (2*DATA(Indx).NBldC-1)*(k-1);
    I2     = (2*DATA(Indx).NBldC-1) + (2*DATA(Indx).NBldC-1)*(k-1);
   
    Beta   = ppval(BLADE(Indx).SBeta, XYZ_2_2(3,I1));
    Chord  = ppval(BLADE(Indx).SChord, XYZ_2_2(3,I1));
    Offset = ppval(BLADE(Indx).SOffset, XYZ_2_2(3,I1));
    
    XYZ_2_2(1,I1:I2)   = XYZ_2_2(1,I1:I2) * Chord;
    XYZ_2_2(2,I1:I2)   = XYZ_2_2(2,I1:I2) * Chord;
    
    XYZ_2_2(2,I1:I2)   = XYZ_2_2(2,I1:I2) - Offset * Chord;
    
    TNB = RotationMatrix ( 3, deg2rad( Beta ) );
    
    XYZ_2_2(1:3,I1:I2) = TNB * XYZ_2_2(1:3,I1:I2);    
    
end

%% CONNECTIVITIES (suitable for UVLM)

ICON_LS = Connectivities (LSNN, DATA(Indx).NBldC);
ICON_RT = Connectivities (BRNN+(BTNN - 1), 2*DATA(Indx).NBldC-1);

%% CONNECTIVITIES (suitable Blade with thickness)

ICON_LS_1 = Connectivities (LSNN, 2*DATA(Indx).NBldC-1);
ICON_RT_1 = Connectivities (BRNN+(BTNN - 1), 2*DATA(Indx).NBldC-1);

%% SHEDDING ZONES && Nodes located at the free edges

[Nod_Shed, Panel_Shed, NNShed, NPShed] = SheddingZones (DATA, LSNN, Indx);

if DATA(Indx).ShedBld == 1
    
    for i = 1:(LSNN)        
        FE_Nodes(i) = 1 + (DATA(Indx).NBldC)*(i-1);        
    end    
    AUX = LSNN*DATA(Indx).NBldC:-1:(LSNN*DATA(Indx).NBldC-DATA(Indx).NBldC+2);
    
    FE_Nodes = [FE_Nodes, AUX(length(AUX):-1:1)];
    
elseif DATA(Indx).ShedBld == 2
    
    for i = 1:(LSNN)        
        FE_Nodes(i) = 1 + (DATA(Indx).NBldC)*(i-1);        
    end
    
end

%% Control points and normalls
NPRootTran      = (BRNN + (BTNN - 2)) * (2*DATA(Indx).NBldC - 2);
NPLS            = (LSNN - 1) * (DATA(Indx).NBldC - 1);
XYZ_CP_RootTran = ControlPoints (transpose (XYZ_1), ICON_RT, NPRootTran);
XYZ_CP_LS       = ControlPoints (transpose (XYZ_2), ICON_LS, NPLS);

NOR_RootTran    = NormallVector (transpose (XYZ_1), ICON_RT, NPRootTran);
NOR_LS          = NormallVector (transpose (XYZ_2), ICON_LS, NPLS);

%% Output Data

BLADE_AERO(Indx).XYZRootTran       = transpose (XYZ_1);
BLADE_AERO(Indx).XYZLS             = transpose (XYZ_2);
BLADE_AERO(Indx).XYZCPRootTran     = XYZ_CP_RootTran;
BLADE_AERO(Indx).XYZCPLS           = XYZ_CP_LS;

BLADE_AERO(Indx).NormalRootTran     = NOR_RootTran;
BLADE_AERO(Indx).NormalLS           = NOR_LS;

BLADE_AERO(Indx).XYZRootTranThick  = transpose (XYZ_1_1);
BLADE_AERO(Indx).XYZLSThick        = transpose (XYZ_2_2);

BLADE_AERO(Indx).ICONLS            = ICON_LS;
BLADE_AERO(Indx).ICONRootTran      = ICON_RT;

BLADE_AERO(Indx).ICONLSThick       = ICON_LS_1;
BLADE_AERO(Indx).ICONRootTranThick = ICON_RT_1;

BLADE_AERO(Indx).NNSpanBlade       = NNBLADE;
BLADE_AERO(Indx).NNSpanLS          = LSNN;
BLADE_AERO(Indx).NNSpanRoot        = BRNN;
BLADE_AERO(Indx).NNSpanTran        = BTNN;
BLADE_AERO(Indx).NNSpanRootTran    = BRNN + (BTNN - 1);
BLADE_AERO(Indx).NNChordRootTran   = 2*DATA(Indx).NBldC - 1;
BLADE_AERO(Indx).NNLS              = LSNN * DATA(Indx).NBldC;
BLADE_AERO(Indx).NPLS              = (LSNN - 1) * (DATA(Indx).NBldC - 1);
BLADE_AERO(Indx).NNRootTran        = (BRNN + (BTNN - 1)) * (2*DATA(Indx).NBldC - 1);
BLADE_AERO(Indx).NPRootTran        = (BRNN + (BTNN - 2)) * (2*DATA(Indx).NBldC - 2);

BLADE_AERO(Indx).NNChordLSThick    = 2*DATA(Indx).NBldC - 1;
BLADE_AERO(Indx).NNLSThick         = LSNN * (2*DATA(Indx).NBldC-1);
BLADE_AERO(Indx).NPLSThick         = (LSNN - 1) * (2*DATA(Indx).NBldC - 2);

BLADE_AERO(Indx).NNShed            = NNShed;
BLADE_AERO(Indx).NPShed            = NPShed;
BLADE_AERO(Indx).ShedNode          = Nod_Shed;
BLADE_AERO(Indx).ShedPanel         = Panel_Shed;

BLADE_AERO(Indx).FE_Nodes          = FE_Nodes;

BLADE_AERO(Indx).LBlade            = LBlade;
BLADE_AERO(Indx).DEFORM            = DEFORM;
BLADE_AERO(Indx).InitialSpan       = ZZCoord(end);

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

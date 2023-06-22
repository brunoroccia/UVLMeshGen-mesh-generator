function [KINEMATICS, SEMatrices, DATA] = KinematicsProcessor (KINEMATICS, SEMatrices, DATA, WIND_TURBINE, BLADE_AERO, HUB_AERO, ...
            NACELLE_AERO, TOWER_AERO, GROUND_AERO, GROUND_FARM, MONOPILE_AERO, Kinematic_FLAG, Ground_FLAG, ...
            WTNames, Indx)

TimeSteps = DATA(Indx).UVLMSteps;
DT        = DATA(Indx).DT;

%% SIGNAL CALCULATION

if strcmpi(Kinematic_FLAG{1,2}, 'RotorOn')
    Fcn = str2func(DATA(Indx).NameRot);
    [Theta, DTheta] = Fcn (TimeSteps, DT, DATA(Indx).Rot0WT);
else
    Theta  = (DATA(Indx).Rot0WT * pi / 180) * ones (1,TimeSteps+1);
    DTheta = zeros (1,TimeSteps+1);
end

if strcmpi(Kinematic_FLAG{1,3}, 'YawOn')
    Fcn = str2func(DATA(Indx).NameYaw);
    [Yaw, DYaw] = Fcn (TimeSteps, DT, DATA(Indx).Yaw0WT);
else
    Yaw  = (DATA(Indx).Yaw0WT * pi / 180) * ones (1,TimeSteps+1);
    DYaw = zeros (1,TimeSteps+1);
end

if strcmpi(Kinematic_FLAG{1,4}, 'PitchOn')
    Fcn = str2func(DATA(Indx).NamePitch);
    [Pitch, DPitch] = Fcn (TimeSteps, DT, DATA(Indx).Pitch0WT);
else
    Pitch  = ones (DATA(Indx).NumBld,TimeSteps+1);
    for  i = 1: DATA(Indx).NumBld
        Pitch(i,:)  = Pitch(i,:) * (DATA(Indx).Pitch0WT(i)* pi / 180);
    end
    DPitch = zeros (DATA(Indx).NumBld,TimeSteps+1);
end

%% KINEMATICS

Rposition(1,1) = WTNames{Indx,2};
Rposition(2,1) = WTNames{Indx,3};
Rposition(3,1) = WTNames{Indx,4};

for i = 1:TimeSteps

%----------------------------------------------------------------------------------------
%                   G R O U N D   F A R M   K I N E M A T I C S
%----------------------------------------------------------------------------------------    
    if strcmpi(Ground_FLAG{1,1}, 'ON') && length(DATA) > 1

        for k = 1:GROUND_FARM.NumBox
            KINEMATICS(i).GroundFarm.PATCH(k).XYZ   = GROUND_FARM.PATCH(k).XYZ;
            KINEMATICS(i).GroundFarm.PATCH(k).XYZCP = GROUND_FARM.PATCH(k).XYZCP;
            KINEMATICS(i).GroundFarm.PATCH(k).VCP   = zeros(size(GROUND_FARM.PATCH(k).XYZCP,1));
        end

    end
%----------------------------------------------------------------------------------------
%                       G R O U N D   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Ground == 1

        KINEMATICS(i).WT(Indx).XYZGround   = WIND_TURBINE(Indx).XYZGround;
        KINEMATICS(i).WT(Indx).XYZCPGround = GROUND_AERO(Indx).XYZCP;

        for k = 1:size(GROUND_AERO(Indx).XYZCP,1)
            KINEMATICS(i).WT(Indx).XYZCPGround(k,:) = KINEMATICS(i).WT(Indx).XYZCPGround(k,:) + transpose (Rposition);
        end

        KINEMATICS(i).WT(Indx).XYZVCPGround = zeros(size(GROUND_AERO(Indx).XYZCP,1), 3);

        R0Grd = Rposition;
        b1    = [1;0;0];
        b2    = [0;1;0];
        b3    = [0;0;1];
        SEGrd = SEMatrixConstruction (R0Grd, b1, b2, b3);
        SEMatrices(Indx).HGrd{i} = SEGrd;

    end

%----------------------------------------------------------------------------------------
%                          T O W E R   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Tower == 1

        KINEMATICS(i).WT(Indx).XYZTower   = WIND_TURBINE(Indx).XYZTower;
        KINEMATICS(i).WT(Indx).XYZCPTower = TOWER_AERO(Indx).XYZCP;

        for k = 1:size(TOWER_AERO(Indx).XYZCP,1)
            KINEMATICS(i).WT(Indx).XYZCPTower(k,:) = KINEMATICS(i).WT(Indx).XYZCPTower(k,:) + transpose (Rposition);
        end

        KINEMATICS(i).WT(Indx).XYZVCPTower = zeros(size(TOWER_AERO(Indx).XYZCP,1), 3);

        R0Tow = Rposition;
        b1    = [1;0;0];
        b2    = [0;1;0];
        b3    = [0;0;1];
        SETow = SEMatrixConstruction (R0Tow, b1, b2, b3);
        SEMatrices(Indx).HTow{i} = SETow;

    end

%----------------------------------------------------------------------------------------
%                        M O N O P I L E   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Monopile == 1

        KINEMATICS(i).WT(Indx).XYZMonopile   = WIND_TURBINE(Indx).XYZMON;
        KINEMATICS(i).WT(Indx).XYZCPMonopile = MONOPILE_AERO(Indx).XYZCP;

        R0Mon = [0.0; 0.0; -DATA(Indx).LMon] + Rposition;

        for k = 1:size(MONOPILE_AERO(Indx).XYZCP,1)
            KINEMATICS(i).WT(Indx).XYZCPMonopile(k,:) = KINEMATICS(i).WT(Indx).XYZCPMonopile(k,:) + transpose (R0Mon);
        end

        KINEMATICS(i).WT(Indx).XYZVCPMonopile = zeros(size(MONOPILE_AERO(Indx).XYZCP,1), 3);        

        b1    = [1;0;0];
        b2    = [0;1;0];
        b3    = [0;0;1];
        SEMon = SEMatrixConstruction (R0Mon, b1, b2, b3);
        SEMatrices(Indx).HMon{i} = SEMon;

    end

%----------------------------------------------------------------------------------------
%                        N A C E L L E   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Nacelle == 1

        Gamma  = -1.00 * DATA(Indx).Tilt * pi / 180;

        R0 = [0.0; 0.0; DATA(Indx).TowHeight];

        Offset_X = DATA(Indx).HubInnerR*sin(Gamma);
        Offset_Z = DATA(Indx).HubInnerR*cos(Gamma) + DATA(Indx).LConTNac;
        OFFSET   = [Offset_X; 0.0; Offset_Z];
        R         = R0 + OFFSET;

        KINEMATICS(i).WT(Indx).NACPART(1).XYZ = NACELLE_AERO(Indx).XYZPatch;
        KINEMATICS(i).WT(Indx).NACPART(2).XYZ = NACELLE_AERO(Indx).XYZCyl;
        KINEMATICS(i).WT(Indx).NACPART(3).XYZ = NACELLE_AERO(Indx).XYZTail;
        KINEMATICS(i).WT(Indx).NACPART(4).XYZ = NACELLE_AERO(Indx).XYZConnect;

        KINEMATICS(i).WT(Indx).NACPART(1).XYZCP = NACELLE_AERO(Indx).XYZCPPatch;
        KINEMATICS(i).WT(Indx).NACPART(2).XYZCP = NACELLE_AERO(Indx).XYZCPCyl;
        KINEMATICS(i).WT(Indx).NACPART(3).XYZCP = NACELLE_AERO(Indx).XYZCPTail;
        KINEMATICS(i).WT(Indx).NACPART(4).XYZCP = NACELLE_AERO(Indx).XYZCPConnect;

        T3  = RotationMatrix( 3 , Yaw(i) );
        DT3 = RotationMatrix_D ( 3 , Yaw(i), DYaw(i) );

        for j = 1:4
            for k = 1:size(WIND_TURBINE(Indx).NACPART(j).XYZ,1)
                KINEMATICS(i).WT(Indx).NACPART(j).XYZ(k,:) = transpose (T3 * (transpose(KINEMATICS(i).WT(Indx).NACPART(j).XYZ(k,:)) + OFFSET) + ...
                    R0 + Rposition);
            end
        end
        R0Nac = R0 + Rposition + T3 * OFFSET;
        b1    = T3 * [1;0;0];
        b2    = T3 * [0;1;0];
        b3    = T3 * [0;0;1];
        SENac = SEMatrixConstruction (R0Nac, b1, b2, b3);
        SEMatrices(Indx).HNac{i} = SENac;

        for j = 1:4
            for k = 1:size(KINEMATICS(i).WT(Indx).NACPART(j).XYZCP,1)
                KINEMATICS(i).WT(Indx).NACPART(j).VCP(k,:)   = transpose (DT3 * (transpose(KINEMATICS(i).WT(Indx).NACPART(j).XYZCP(k,:)) + OFFSET) );
                KINEMATICS(i).WT(Indx).NACPART(j).XYZCP(k,:) = transpose (T3 * (transpose(KINEMATICS(i).WT(Indx).NACPART(j).XYZCP(k,:)) + OFFSET) + ...
                    R0 + Rposition);
            end
        end

    end

%----------------------------------------------------------------------------------------
%                             H U B   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Hub == 1

        R0 = [0.0; 0.0; DATA(Indx).TowHeight];
        E3 = [0; 0; 1];
        n1 = [1; 0; 0];

        Gamma  = -1.00 * DATA(Indx).Tilt * pi / 180;

        Offset_X = DATA(Indx).HubInnerR*sin(Gamma);
        Offset_Z = DATA(Indx).HubInnerR*cos(Gamma) + DATA(Indx).LConTNac;

        H1 = Offset_Z + tan(Gamma)*Offset_X;
        a1 = DATA(Indx).LCylHub/2 + DATA(Indx).LCylNac/2 + Offset_X/cos(Gamma);
        R  = R0 + H1*E3;

        T2 = RotationMatrix( 2 , Gamma );
        T3 = RotationMatrix( 3 , Yaw(i) );
        T1 = RotationMatrix( 1 , Theta(i) );

        DT3 = RotationMatrix_D( 3 , Yaw(i), DYaw(i) );
        DT1 = RotationMatrix_D( 1 , Theta(i), DTheta(i) );

        DDT = DT3*T2*T1 + T3*T2*DT1;

        R0Hub = R + Rposition + T3*T2*T1*(a1*n1);
        b1    = T3*T2*T1 * [1;0;0];
        b2    = T3*T2*T1 * [0;1;0];
        b3    = T3*T2*T1 * [0;0;1];
        SEHub = SEMatrixConstruction (R0Hub, b1, b2, b3);
        SEMatrices(Indx).HHub{i} = SEHub;

        KINEMATICS(i).WT(Indx).HUBPART(1).XYZ    = HUB_AERO(Indx).XYZNose;
        KINEMATICS(i).WT(Indx).HUBPART(1).XYZCP  = HUB_AERO(Indx).XYZCPNose;

        k = 2;
        for j = 1:DATA(Indx).NumBld
            KINEMATICS(i).WT(Indx).HUBPART(k).XYZ     = HUB_AERO(Indx).PATCH(j).XYZ;
            KINEMATICS(i).WT(Indx).HUBPART(k+1).XYZ   = HUB_AERO(Indx).CONNECT(j).XYZ;
            KINEMATICS(i).WT(Indx).HUBPART(k).XYZCP   = HUB_AERO(Indx).PATCH(j).XYZCP;
            KINEMATICS(i).WT(Indx).HUBPART(k+1).XYZCP = HUB_AERO(Indx).CONNECT(j).XYZCP;
            k = k + 2;
        end

        for j = 1:DATA(Indx).NumBld*2+1
            for k = 1:size(KINEMATICS(i).WT(Indx).HUBPART(j).XYZ,1)
                KINEMATICS(i).WT(Indx).HUBPART(j).XYZ(k,:) = transpose (R + Rposition + T3*T2*T1*( transpose(KINEMATICS(i).WT(Indx).HUBPART(j).XYZ(k,:)) + ...
                    a1*n1));
            end
        end

        for j = 1:DATA(Indx).NumBld*2+1
            for k = 1:size(KINEMATICS(i).WT(Indx).HUBPART(j).XYZCP,1)
                KINEMATICS(i).WT(Indx).HUBPART(j).VCP(k,:) = transpose (DDT*( transpose(KINEMATICS(i).WT(Indx).HUBPART(j).XYZCP(k,:)) + a1*n1 ) );
                KINEMATICS(i).WT(Indx).HUBPART(j).XYZCP(k,:) = transpose (R + Rposition + T3*T2*T1*( transpose(KINEMATICS(i).WT(Indx).HUBPART(j).XYZCP(k,:)) + ...
                    a1*n1));
            end
        end

    end

%----------------------------------------------------------------------------------------
%                              B L A D E   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Blade == 1

        R0 = [0.0; 0.0; DATA(Indx).TowHeight];
        E3 = [0; 0; 1];
        e3 = [0; 0; 1];
        n1 = [1; 0; 0];

        L_Connect = DATA(Indx).HubRad - DATA(Indx).HubInnerR;

        Gamma  = -1.00 * DATA(Indx).Tilt * pi / 180;
        Beta   = DATA(Indx).PreCone * pi / 180;

        Offset_X = DATA(Indx).HubInnerR*sin(Gamma);
        Offset_Z = DATA(Indx).HubInnerR*cos(Gamma) + DATA(Indx).LConTNac;

        H1 = Offset_Z + tan(Gamma)*Offset_X;
        a1 = DATA(Indx).LCylHub/2 + DATA(Indx).LCylNac/2 + Offset_X/cos(Gamma);

        R = R0 + H1*E3;

        KINEMATICS(i).WT(Indx).BLDPART(1).XYZ   = BLADE_AERO(Indx).XYZRootTran;
        KINEMATICS(i).WT(Indx).BLDPART(2).XYZ   = BLADE_AERO(Indx).XYZLS;
        KINEMATICS(i).WT(Indx).BLDPART(1).XYZCP = BLADE_AERO(Indx).XYZCPRootTran;
        KINEMATICS(i).WT(Indx).BLDPART(2).XYZCP = BLADE_AERO(Indx).XYZCPLS;

        k = 3;
        for j = 2:DATA(Indx).NumBld
            KINEMATICS(i).WT(Indx).BLDPART(k).XYZ     = BLADE_AERO(Indx).XYZRootTran;
            KINEMATICS(i).WT(Indx).BLDPART(k+1).XYZ   = BLADE_AERO(Indx).XYZLS;
            KINEMATICS(i).WT(Indx).BLDPART(k).XYZCP   = BLADE_AERO(Indx).XYZCPRootTran;
            KINEMATICS(i).WT(Indx).BLDPART(k+1).XYZCP = BLADE_AERO(Indx).XYZCPLS;
            k = k + 2;
        end

        T2  = RotationMatrix( 2 , Gamma );
        T2p = RotationMatrix( 2 , Beta );

        T3y  = RotationMatrix( 3 , Yaw(i) );
        T1r  = RotationMatrix( 1 , Theta(i) );
        DT3y = RotationMatrix_D( 3 , Yaw(i), DYaw(i) );
        DT1r = RotationMatrix_D( 1 , Theta(i), DTheta(i) );

        ss = 1;
        for j = 1:DATA(Indx).NumBld
            PitchAux  = pi - Pitch(j,i);
            DPitchAux = - DPitch(j,i);
            T3p   = RotationMatrix( 3 , PitchAux );
            DT3p  = RotationMatrix_D( 3 , DPitch(j,i), DPitchAux );
            DeltaAng = 0.0 + 2*pi/DATA(Indx).NumBld*(j-1);
            T1 = RotationMatrix( 1 , DeltaAng );

            R0Bld = R + Rposition + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3);
            b1    = T3y*T2*T1r*T1*T2p*T3p * [1;0;0];
            b2    = T3y*T2*T1r*T1*T2p*T3p * [0;1;0];
            b3    = T3y*T2*T1r*T1*T2p*T3p * [0;0;1];
            SEBld = SEMatrixConstruction (R0Bld, b1, b2, b3);
            SEMatrices(Indx).HBld{j}{i} = SEBld;

            for k = 1:size(KINEMATICS(i).WT(Indx).BLDPART(ss).XYZ,1)
                KINEMATICS(i).WT(Indx).BLDPART(ss).XYZ(k,:) = transpose (R + Rposition + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3) + ...
                    T3y*T2*T1r*T1*T2p*T3p*transpose(KINEMATICS(i).WT(Indx).BLDPART(ss).XYZ(k,:)));
            end
            for k = 1:size(KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZ,1)
                KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZ(k,:) = transpose (R + Rposition + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3) + ...
                    T3y*T2*T1r*T1*T2p*T3p*transpose(KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZ(k,:)));
            end
            % Control point velocity
            DDT1 = DT3y * T2;
            DDT2 = DT3y*T2*T1r*T1 + T3y*T2*DT1r*T1;
            DDT3 = DT3y*T2*T1r*T1*T2p + T3y*T2*DT1r*T1*T2p;
            DDT4 = DT3y*T2*T1r*T1*T2p*T3p + T3y*T2*DT1r*T1*T2p*T3p + T3y*T2*T1r*T1*T2p*DT3p;
            for k = 1:size(KINEMATICS(i).WT(Indx).BLDPART(ss).XYZCP,1)
                KINEMATICS(i).WT(Indx).BLDPART(ss).VCP(k,:) = transpose (DDT1*(a1)*n1 + DDT2*(DATA(Indx).HubInnerR)*e3 + DDT3*(L_Connect*e3) + ...
                    DDT4*transpose(KINEMATICS(i).WT(Indx).BLDPART(ss).XYZCP(k,:)));
                KINEMATICS(i).WT(Indx).BLDPART(ss).XYZCP(k,:) = transpose (R + Rposition + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3) + ...
                    T3y*T2*T1r*T1*T2p*T3p*transpose(KINEMATICS(i).WT(Indx).BLDPART(ss).XYZCP(k,:)));
            end
            for k = 1:size(KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZCP,1)
                KINEMATICS(i).WT(Indx).BLDPART(ss+1).VCP(k,:) = transpose (DDT1*(a1)*n1 + DDT2*(DATA(Indx).HubInnerR)*e3 + DDT3*(L_Connect*e3) + ...
                    DDT4*transpose(KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZCP(k,:)));
                KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZCP(k,:) = transpose (R + Rposition + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3) + ...
                    T3y*T2*T1r*T1*T2p*T3p*transpose(KINEMATICS(i).WT(Indx).BLDPART(ss+1).XYZCP(k,:)));
            end
            ss = ss + 2;
        end

        KINEMATICS(i).WT(Indx).MPOINT = R;
        KINEMATICS(i).WT(Indx).MAxis  = T3y*T2*[1;0;0];

    end

%----------------------------------------------------------------------------------------------------------------------------------------------

end

%% First derivative computation of SE matrices

if DATA(Indx).Ground == 1
    DSE = FiniteDiference (SEMatrices(Indx).HGrd, DT, TimeSteps, 2);
    SEMatrices(Indx).DHGrd = DSE;
end
if DATA(Indx).Tower == 1
    DSE = FiniteDiference (SEMatrices(Indx).HTow, DT, TimeSteps, 2);
    SEMatrices(Indx).DHTow = DSE;
end
if DATA(Indx).Monopile == 1
    DSE = FiniteDiference (SEMatrices(Indx).HMon, DT, TimeSteps, 2);
    SEMatrices(Indx).DHMon = DSE;
end
if DATA(Indx).Nacelle == 1
    DSE = FiniteDiference (SEMatrices(Indx).HNac, DT, TimeSteps, 2);
    SEMatrices(Indx).DHNac = DSE;
end
if DATA(Indx).Hub == 1
    DSE = FiniteDiference (SEMatrices(Indx).HHub, DT, TimeSteps, 2);
    SEMatrices(Indx).DHHub = DSE;
end
if DATA(Indx).Blade == 1
    AUX = [];
    for j = 1:DATA(Indx).NumBld
        DSE = FiniteDiference ({SEMatrices(Indx).HBld{j}{:}}, DT, TimeSteps, 2);
        SEMatrices(Indx).DHBld{j} = DSE;
    end
    
end


end

%% SE(3) Matrix Construction

function SEM = SEMatrixConstruction (R, b1, b2, b3)

n = eye (3);
b = [b1, b2, b3];

R0 = [0;0;0];

r  = R - R0;

SEM = zeros (4,4);
T   = zeros (3,3);

for i = 1:3
    for j = 1:3
        T(i,j) = dot(b(:,j),n(:,i));
    end
end

SEM (1:3,1:3) = T;
SEM (1:3,4)   = r;
SEM (4,4)     = 1;

end

%% Finite difference 2nd Orden

function [DSE] = FiniteDiference (SE, DT, TimeSteps, Order)

switch Order

    case (2)

        for i = 1:TimeSteps
            if i == 1
                Mi  = SE{i};
                Mi1 = SE{i+1};
                Mi2 = SE{i+2};
                DSE{i} = (-3*Mi + 4*Mi1 - Mi2) / (2*DT);
            elseif i == TimeSteps
                Mi  = SE{i};
                Mi1 = SE{i-1};
                Mi2 = SE{i-2};
                DSE{i} = (3*Mi - 4*Mi1 + Mi2) / (2*DT);
            else
                Mi1 = SE{i-1};
                Mi2 = SE{i+1};
                DSE{i} = (Mi2 - Mi1) / (2*DT);
            end
        end
    
    otherwise


end

end
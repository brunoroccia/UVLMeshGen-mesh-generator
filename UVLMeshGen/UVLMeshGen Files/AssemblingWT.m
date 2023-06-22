function [WIND_TURBINE, CONNECT] = AssemblingWT (DATA, WIND_TURBINE, CONNECT, BLADE_AERO, HUB_AERO, ...
                                   NACELLE_AERO, TOWER_AERO, GROUND_AERO, MONOPILE_AERO, ...
                                   WTNames, Indx)
                 
% Assembling function

%% Ground Assembling

if DATA(Indx).Ground == 1

    for i = 1:size(GROUND_AERO(Indx).XYZ,1)
        WIND_TURBINE(Indx).XYZGround(i,:)  = GROUND_AERO(Indx).XYZ(i,:);
    end

    CONNECT(Indx).GROUND.Icon = GROUND_AERO(Indx).ICON;

end

%% Tower Assmbling

if DATA(Indx).Tower == 1

    for i = 1:size(TOWER_AERO(Indx).XYZ,1)
        WIND_TURBINE(Indx).XYZTower(i,:) = TOWER_AERO(Indx).XYZ(i,:);
    end

    CONNECT(Indx).TOWER.Icon = TOWER_AERO(Indx).ICON;

end

%% Nacelle Assmbling

if DATA(Indx).Nacelle == 1

    Gamma  = -1.00 * DATA(Indx).Tilt * pi / 180;

    R0 = [0.0; 0.0; DATA(Indx).TowHeight];

    Offset_X = DATA(Indx).HubInnerR*sin(Gamma);
    Offset_Z = DATA(Indx).HubInnerR*cos(Gamma) + DATA(Indx).LConTNac;
    OFFSET   = [Offset_X; 0.0; Offset_Z];
    R         = R0 + OFFSET;

    WIND_TURBINE(Indx).NACPART(1).XYZ = NACELLE_AERO(Indx).XYZPatch;
    WIND_TURBINE(Indx).NACPART(2).XYZ = NACELLE_AERO(Indx).XYZCyl;
    WIND_TURBINE(Indx).NACPART(3).XYZ = NACELLE_AERO(Indx).XYZTail;
    WIND_TURBINE(Indx).NACPART(4).XYZ = NACELLE_AERO(Indx).XYZConnect;

    CONNECT(Indx).NACPART(1).ICON = NACELLE_AERO(Indx).ICONPatch;
    CONNECT(Indx).NACPART(2).ICON = NACELLE_AERO(Indx).ICONCyl;
    CONNECT(Indx).NACPART(3).ICON = NACELLE_AERO(Indx).ICONTail;
    CONNECT(Indx).NACPART(4).ICON = NACELLE_AERO(Indx).ICONConnect;    

    T3 = RotationMatrix( 3 , DATA(Indx).Yaw0WT*pi/180 );

    for j = 1:4
        for k = 1:size(WIND_TURBINE(Indx).NACPART(j).XYZ,1)
            WIND_TURBINE(Indx).NACPART(j).XYZ(k,:) = transpose (T3 * (transpose(WIND_TURBINE(Indx).NACPART(j).XYZ(k,:)) + OFFSET) + R0);
        end
    end

    WIND_TURBINE(Indx).AxisNac   = R0(3) + OFFSET(3);
    WIND_TURBINE(Indx).OffsetNac = OFFSET;

end

%% Hub Assmbling

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
    T3 = RotationMatrix( 3 , DATA(Indx).Yaw0WT * pi/180 );
    T1 = RotationMatrix( 1 , DATA(Indx).Rot0WT * pi/180 );

    WIND_TURBINE(Indx).HUBPART(1).XYZ  = HUB_AERO(Indx).XYZNose;
    CONNECT(Indx).HUBPART(1).ICON = HUB_AERO(Indx).ICONNose;
    k = 2;
    for j = 1:DATA(Indx).NumBld
        WIND_TURBINE(Indx).HUBPART(k).XYZ = HUB_AERO(Indx).PATCH(j).XYZ;
        WIND_TURBINE(Indx).HUBPART(k+1).XYZ = HUB_AERO(Indx).CONNECT(j).XYZ;

        CONNECT(Indx).HUBPART(k).ICON = HUB_AERO(Indx).ICONPatch;
        CONNECT(Indx).HUBPART(k+1).ICON = HUB_AERO(Indx).ICONConnect;

        k = k + 2;
    end

    for j = 1:DATA(Indx).NumBld*2+1
        for k = 1:size(WIND_TURBINE(Indx).HUBPART(j).XYZ,1)
            WIND_TURBINE(Indx).HUBPART(j).XYZ(k,:) = transpose (R + T3*T2*T1*( transpose(WIND_TURBINE(Indx).HUBPART(j).XYZ(k,:)) + ...
                                                     a1*n1));
        end
    end

    WIND_TURBINE(Indx).ClampPointHub = H1;
    WIND_TURBINE(Indx).RootHub       = a1;
    WIND_TURBINE(Indx).MPOINT        = R;
    WIND_TURBINE(Indx).MAxis         = T3*T2*[1;0;0];

end

%% Blade Assmbling

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

    WIND_TURBINE(Indx).BLDPART(1).XYZ = BLADE_AERO(Indx).XYZRootTran;
    WIND_TURBINE(Indx).BLDPART(2).XYZ = BLADE_AERO(Indx).XYZLS;

    CONNECT(Indx).BLDPART(1).ICON = BLADE_AERO(Indx).ICONRootTran;
    CONNECT(Indx).BLDPART(2).ICON = BLADE_AERO(Indx).ICONLS;

    k = 3;
    for j = 2:DATA(Indx).NumBld
        WIND_TURBINE(Indx).BLDPART(k).XYZ = BLADE_AERO(Indx).XYZRootTran;
        WIND_TURBINE(Indx).BLDPART(k+1).XYZ = BLADE_AERO(Indx).XYZLS;

        CONNECT(Indx).BLDPART(k).ICON = BLADE_AERO(Indx).ICONRootTran;
        CONNECT(Indx).BLDPART(k+1).ICON = BLADE_AERO(Indx).ICONLS;
        
        k = k + 2;
    end

    T2  = RotationMatrix( 2 , Gamma );
    T2p = RotationMatrix( 2 , Beta );

    T3y = RotationMatrix( 3 , DATA(Indx).Yaw0WT * pi/180 );
    T1r = RotationMatrix( 1 , DATA(Indx).Rot0WT * pi/180 );    

    ss = 1;
    for j = 1:DATA(Indx).NumBld
        Pitch = ( 180 - DATA(Indx).Pitch0WT(j) ) * pi/180;
        T3p  = RotationMatrix( 3 , Pitch );
        DeltaAng = 0.0 + 2*pi/DATA(Indx).NumBld*(j-1);
        T1 = RotationMatrix( 1 , DeltaAng );
        for k = 1:size(WIND_TURBINE(Indx).BLDPART(ss).XYZ,1)
            WIND_TURBINE(Indx).BLDPART(ss).XYZ(k,:) = transpose (R + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3) + ...
                T3y*T2*T1r*T1*T2p*T3p*transpose(WIND_TURBINE(Indx).BLDPART(ss).XYZ(k,:)));
        end
        for k = 1:size(WIND_TURBINE(Indx).BLDPART(ss+1).XYZ,1)
            WIND_TURBINE(Indx).BLDPART(ss+1).XYZ(k,:) = transpose (R + T3y*T2*(a1)*n1 + T3y*T2*T1r*T1*(DATA(Indx).HubInnerR)*e3 + T3y*T2*T1r*T1*T2p*(L_Connect*e3) + ...
                T3y*T2*T1r*T1*T2p*T3p*transpose(WIND_TURBINE(Indx).BLDPART(ss+1).XYZ(k,:)));
        end
        ss = ss + 2;
    end

    WIND_TURBINE(Indx).BladeR    = R;
    WIND_TURBINE(Indx).BladeH    = H1;
    WIND_TURBINE(Indx).BladeHubC = a1;
    WIND_TURBINE(Indx).BladeLCon = L_Connect;

end

%% Monopile Assmbling

if DATA(Indx).Monopile == 1
    
    R = [0.0; 0.0; -DATA(Indx).LMon];
    
    WIND_TURBINE(Indx).XYZMON = MONOPILE_AERO(Indx).XYZ;
    
    for i = 1:size(WIND_TURBINE(Indx).XYZMON,1)
        WIND_TURBINE(Indx).XYZMON(i,:) = WIND_TURBINE(Indx).XYZMON(i,:) + transpose(R);
    end

    CONNECT(Indx).MONOPILE.Icon = MONOPILE_AERO(Indx).ICON;
    
end


end

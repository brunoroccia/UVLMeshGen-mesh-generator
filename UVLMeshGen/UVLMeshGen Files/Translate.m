function [WIND_TURBINE] = Translate (WIND_TURBINE, DATA, WTNames, Indx)

Rposition(1,1) = WTNames{Indx,2};
Rposition(1,2) = WTNames{Indx,3};
Rposition(1,3) = WTNames{Indx,4};

if DATA(Indx).Ground == 1
    for i = 1:size(WIND_TURBINE(Indx).XYZGround,1)
        WIND_TURBINE(Indx).XYZGround(i,1:3) = WIND_TURBINE(Indx).XYZGround(i,1:3) + Rposition;
    end
end

if DATA(Indx).Tower == 1
    for i = 1:size(WIND_TURBINE(Indx).XYZTower,1)
        WIND_TURBINE(Indx).XYZTower(i,1:3) = WIND_TURBINE(Indx).XYZTower(i,1:3) + Rposition;
    end
end

if DATA(Indx).Monopile == 1
    for i = 1:size(WIND_TURBINE(Indx).XYZMON,1)
        WIND_TURBINE(Indx).XYZMON(i,1:3) = WIND_TURBINE(Indx).XYZMON(i,1:3) + Rposition;
    end
end

if DATA(Indx).Hub == 1
    for i = 1:length(WIND_TURBINE(Indx).HUBPART)
        for j = 1:size(WIND_TURBINE(Indx).HUBPART(i).XYZ)
            WIND_TURBINE(Indx).HUBPART(i).XYZ(j,1:3) = WIND_TURBINE(Indx).HUBPART(i).XYZ(j,1:3) + Rposition;
        end
    end
end

if DATA(Indx).Nacelle == 1
    for i = 1:length(WIND_TURBINE(Indx).NACPART)
        for j = 1:size(WIND_TURBINE(Indx).NACPART(i).XYZ)
            WIND_TURBINE(Indx).NACPART(i).XYZ(j,1:3) = WIND_TURBINE(Indx).NACPART(i).XYZ(j,1:3) + Rposition;
        end
    end
end

if DATA(Indx).Blade == 1
    for i = 1:length(WIND_TURBINE(Indx).BLDPART)
        for j = 1:size(WIND_TURBINE(Indx).BLDPART(i).XYZ)
            WIND_TURBINE(Indx).BLDPART(i).XYZ(j,1:3) = WIND_TURBINE(Indx).BLDPART(i).XYZ(j,1:3) + Rposition;
        end
    end
end

end
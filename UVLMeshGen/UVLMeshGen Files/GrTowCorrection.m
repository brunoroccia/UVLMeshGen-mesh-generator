function WIND_TURBINE = GrTowCorrection (DATA, WIND_TURBINE, GROUND_FARM, EQWT_FLAG, Flag)

switch Flag

    case (0)
        I1  = 1; 
        I2  = DATA(1).NCircGround;
        WIND_TURBINE(1).XYZTower(I1:I2, 1:3) = WIND_TURBINE(1).XYZGround(I1:I2, 1:3);
    case (1) 
        for i = 1:length(GROUND_FARM.PATCH)
            Indx = GROUND_FARM.IDWT(i);
            if strcmpi(EQWT_FLAG, 'ON')
                Indx1 = 1;
            else
                Indx1 = Indx;
            end
            if Indx ~= 0
                I1  = 1; 
                I2  = DATA(Indx1).NCircGround;
                WIND_TURBINE(Indx).XYZTower(I1:I2, 1:3) = GROUND_FARM.PATCH(i).XYZ(I1:I2, 1:3);
                if DATA(Indx1).Monopile == 1
                    WIND_TURBINE(Indx).XYZMON(end-I2+1:end, 1:3) = GROUND_FARM.PATCH(i).XYZ(I1:I2, 1:3);
                end
            end
        end
end


end
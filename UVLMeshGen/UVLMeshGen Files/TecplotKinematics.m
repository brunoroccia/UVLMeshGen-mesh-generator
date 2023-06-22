

%% *******************************************************************************************************************
%                                K I N E M A T I C   W I N D   T U R B I N E
%*********************************************************************************************************************

if ( strcmp(Kinematic_FLAG{1,1}, 'ON') )

    NameTecplot = fullfile (Folders{4,1}, strcat(OutputName,'-Kinematic.tec') );
    FID = fopen( NameTecplot, 'wt' );

    for Time = 1:DATA(1).UVLMSteps

        for i = 1:NumWT

%-----------------------------------------------------------------------------------------------------------------------
%                                                  GROUND PART
%-----------------------------------------------------------------------------------------------------------------------

            if DATA(i).Ground == 1

                fprintf(FID, 'ZONE T = "WT %i - Ground", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', Time, GROUND_AERO(i).NN, GROUND_AERO(i).NP);
                fprintf( FID, '\n');

                for j = 1:GROUND_AERO(i).NN
                    fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', KINEMATICS(Time).WT(i).XYZGround(j,1:3));
                end

                for j = 1:GROUND_AERO(i).NP
                    fprintf(FID, '%i\t %i\t %i\t %i\n', GROUND_AERO(i).ICON(j,1:4));
                end

            end

%-----------------------------------------------------------------------------------------------------------------------
%                                                  TOWER PART
%-----------------------------------------------------------------------------------------------------------------------

            if DATA(i).Tower == 1

                fprintf(FID, 'ZONE T = "WT %i - Tower", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', Time, TOWER_AERO(i).NN, TOWER_AERO(i).NP);
                fprintf( FID, '\n');

                for j = 1:TOWER_AERO(i).NN
                    fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', KINEMATICS(Time).WT(i).XYZTower(j,1:3));
                end

                for j = 1:TOWER_AERO(i).NP
                    fprintf(FID, '%i\t %i\t %i\t %i\n', TOWER_AERO(i).ICON(j,1:4));
                end

            end

%-----------------------------------------------------------------------------------------------------------------------
%                                                  MONOPILE PART
%-----------------------------------------------------------------------------------------------------------------------

            if DATA(i).Monopile == 1

                fprintf(FID, 'ZONE T = "WT %i - Monopile", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', Time, MONOPILE_AERO(i).NN, MONOPILE_AERO(i).NP);
                fprintf( FID, '\n');

                for j = 1:MONOPILE_AERO(i).NN
                    fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', KINEMATICS(Time).WT(i).XYZMonopile(j,1:3));
                end

                for j = 1:MONOPILE_AERO(i).NP
                    fprintf(FID, '%i\t %i\t %i\t %i\n', MONOPILE_AERO(i).ICON(j,1:4));
                end

            end

%-----------------------------------------------------------------------------------------------------------------------
%                                                  NACELLE PART
%-----------------------------------------------------------------------------------------------------------------------

            if DATA(i).Nacelle == 1

                for j = 1:4

                    fprintf(FID, 'ZONE T = "WT %i - Part %i - Nacelle", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', Time, j, size(WIND_TURBINE(i).NACPART(j).XYZ,1), size(CONNECT(i).NACPART(j).ICON,1));
                    fprintf( FID, '\n');

                    for k = 1:size(KINEMATICS(Time).WT(i).NACPART(j).XYZ,1)
                        fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', KINEMATICS(Time).WT(i).NACPART(j).XYZ(k,1:3));
                    end

                    for k = 1:size(CONNECT(i).NACPART(j).ICON,1)
                        fprintf(FID, '%i\t %i\t %i\t %i\n', CONNECT(i).NACPART(j).ICON(k,1:4));
                    end

                end

            end

%-----------------------------------------------------------------------------------------------------------------------
%                                                  HUB PART
%-----------------------------------------------------------------------------------------------------------------------

            if DATA(i).Hub == 1

                for j = 1:length(WIND_TURBINE(i).HUBPART)

                    fprintf(FID, 'ZONE T = "WT %i - Part %i - Hub", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', Time, j, size(WIND_TURBINE(i).HUBPART(j).XYZ,1), size(CONNECT(i).HUBPART(j).ICON,1));
                    fprintf( FID, '\n');

                    for k = 1:size(KINEMATICS(Time).WT(i).HUBPART(j).XYZ,1)
                        fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', KINEMATICS(Time).WT(i).HUBPART(j).XYZ(k,1:3));
                    end

                    for k = 1:size(CONNECT(i).HUBPART(j).ICON,1)
                        fprintf(FID, '%i\t %i\t %i\t %i\n', CONNECT(i).HUBPART(j).ICON(k,1:4));
                    end

                end

            end

%-----------------------------------------------------------------------------------------------------------------------
%                                                  BLADE PART
%-----------------------------------------------------------------------------------------------------------------------

            if DATA(i).Blade == 1

                for j = 1:length(WIND_TURBINE(i).BLDPART)

                    fprintf(FID, 'ZONE T = "WT %i - Part %i - Blade", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', i, j, size(WIND_TURBINE(i).BLDPART(j).XYZ,1), size(CONNECT(i).BLDPART(j).ICON,1));
                    fprintf( FID, '\n');

                    for k = 1:size(KINEMATICS(Time).WT(i).BLDPART(j).XYZ,1)
                        fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', KINEMATICS(Time).WT(i).BLDPART(j).XYZ(k,1:3));
                    end

                    for k = 1:size(CONNECT(i).BLDPART(j).ICON,1)
                        fprintf(FID, '%i\t %i\t %i\t %i\n', CONNECT(i).BLDPART(j).ICON(k,1:4));
                    end

                end

            end


        end

%-----------------------------------------------------------------------------------------------------------------------
%                                               GROUND FARM
%-----------------------------------------------------------------------------------------------------------------------

        if ~isempty(GROUND_FARM)

            for j = 1:GROUND_FARM.NumBox

                fprintf(FID, 'ZONE T = "Ground Farm - Part %i", N = %i , E = %i , F = FEPOINT, ET = QUADRILATERAL, C = BLUE \n', j, GROUND_FARM.PATCH(j).NN, GROUND_FARM.PATCH(j).NP);
                fprintf( FID, '\n');

                for k = 1:GROUND_FARM.PATCH(j).NN
                    fprintf(FID, '%12.8f\t  %12.8f\t  %12.8f\n', GROUND_FARM.PATCH(j).XYZ(k,1:3));
                end

                for k = 1:GROUND_FARM.PATCH(j).NP
                    fprintf(FID, '%i\t %i\t %i\t %i\n', GROUND_FARM.PATCH(j).ICON(k,1:4));
                end


            end


        end

    end

    fclose (FID);

else

    CodError.OptTecp = 1;

end



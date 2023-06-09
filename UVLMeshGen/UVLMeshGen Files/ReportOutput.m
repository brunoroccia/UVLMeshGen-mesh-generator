function ReportOutput (NumWT, OutputName, DATA, BLADE_AERO, ...
   HUB_AERO, NACELLE_AERO, TOWER_AERO, GROUND_AERO, ...
   MONOPILE_AERO, Ground_FLAG, Nodes, Elements)

NComps = 0;

for i = 1:NumWT
    if DATA(i).Blade == 1
        NComps = NComps + DATA(i).NumBld;
    end
    if DATA(i).Hub == 1
        NComps = NComps + 1;
    end
    if DATA(i).Nacelle == 1
        NComps = NComps + 1;
    end
    if DATA(i).Tower == 1
        NComps = NComps + 1;
    end
    if DATA(i).Monopile == 1
        NComps = NComps + 1;
    end
    if DATA(i).Ground == 1
        NComps = NComps + 1;
    end
    if (NumWT > 1 && strcmpi(Ground_FLAG{1,1}, 'ON'))
        NComps = NComps + 1;
    end
end

%% Report file

FID = fopen( ['Report Files/' OutputName '-Report.dat'], 'wt' );

fprintf (FID, 'Mesh Generator ver. 1.00 - Wind Farms\n' );
fprintf (FID, 'Developed by Dr. Ing. Bruno Roccia\n' );
fprintf (FID, '             Date: June 22, 2023\n' );
fprintf (FID, '             BOW, UiB\n' );
fprintf (FID, '             Contact: bruno.roccia@uib.no\n' );

fprintf (FID, '\n');

fprintf (FID, '*************************************************************************************\n');
fprintf (FID, '                            G E N E R A L   D A T A\n');
fprintf (FID, '*************************************************************************************\n');
fprintf (FID, '\n');
fprintf (FID, 'Number of Wind Farm Components: %2i\n', NComps);
fprintf (FID, 'Total Number of Nodes:  %5i\n', Nodes);
fprintf (FID, 'Total Number of Panels: %5i\n', Elements);

fprintf (FID, '\n');
fprintf (FID, 'Hereafter the global reference frame will be referenced as GRF\n');
fprintf (FID, 'For onshore and offshore wind turbines, the GRF origin is located at\n');
fprintf (FID, 'the center of Ground component.\n');
fprintf (FID, '\n');

for i = 1:NumWT

    fprintf (FID, '*************************************************************************************\n');
    fprintf (FID, '                    W I N D   T U R B I N E   (%i)   \n', i);
    fprintf (FID, '*************************************************************************************\n');
    fprintf (FID, '\n');
    fprintf (FID, 'Tilt angle [�]:      %8.5f\n', DATA(i).Tilt);
    fprintf (FID, 'Pre-cone angle [�]:  %8.5f\n', DATA(i).PreCone);
    fprintf (FID, 'Number of blades:    %i\n', DATA(i).NumBld);
    fprintf (FID, 'Tower height:        %8.5f\n', DATA(i).TowHeight);
    fprintf (FID, '\n');

    if DATA(i).Ground == 1

        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '                    G R O U N D   C O M P O N E N T   D A T A\n');
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes:  %5i\n', GROUND_AERO(i).NN);
        fprintf (FID, 'Number of Panels: %5i\n', GROUND_AERO(i).NP);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes along the circunf. direction:  %5i\n', GROUND_AERO(i).NNC);
        fprintf (FID, 'Number of Nodes along the radial direction: %5i\n', GROUND_AERO(i).NNR);
        fprintf (FID, '\n');

    end

    if DATA(i).Tower == 1

        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '                    T O W E R   C O M P O N E N T   D A T A\n');
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes                               : %5i\n', TOWER_AERO(i).NN);
        fprintf (FID, 'Number of Panels                              : %5i\n', TOWER_AERO(i).NP);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes along the circunf. direction  : %5i\n', TOWER_AERO(i).NNC);
        fprintf (FID, 'Number of Nodes along the Z(Height) direction : %5i\n', TOWER_AERO(i).NNR);
        fprintf (FID, '\n');

    end

    if DATA(i).Nacelle == 1

        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '                  N A C E L L E   C O M P O N E N T   D A T A\n');
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Patch component               : %5i\n', NACELLE_AERO(i).NNPatch);
        fprintf (FID, 'Number of Panels on the Patch component              : %5i\n', NACELLE_AERO(i).NPPatch);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Cylindrical component         : %5i\n', NACELLE_AERO(i).NNCyl);
        fprintf (FID, 'Number of Panels on the Cylindrical component        : %5i\n', NACELLE_AERO(i).NPCyl);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Connection N-T component      : %5i\n', NACELLE_AERO(i).NNConnect);
        fprintf (FID, 'Number of Panels on the Connection N-T component     : %5i\n', NACELLE_AERO(i).NPConnect);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Tail component                : %5i\n', NACELLE_AERO(i).NNTail);
        fprintf (FID, 'Number of Panels on the Tail component               : %5i\n', NACELLE_AERO(i).NPTail);
        fprintf (FID, '\n');

    end

    if DATA(i).Hub == 1

        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '                       H U B   C O M P O N E N T   D A T A\n');
        fprintf (FID, '-------------------------------------------------------------------------------------\n');

        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Patch component              : %5i\n', HUB_AERO(i).NNPatch);
        fprintf (FID, 'Number of Panels on the Patch component             : %5i\n', HUB_AERO(i).NPPatch);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Connection H-B component     : %5i\n', HUB_AERO(i).NNConnect);
        fprintf (FID, 'Number of Panels on the Connection H-B component    : %5i\n', HUB_AERO(i).NPConnect);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Nose component               : %5i\n', HUB_AERO(i).NNHubNose);
        fprintf (FID, 'Number of Panels on the Nose component              : %5i\n', HUB_AERO(i).NPHubNose);
        fprintf (FID, '\n');

    end

    if DATA(i).Blade == 1

        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '                     B L A D E   C O M P O N E N T   D A T A\n');
        fprintf (FID, '-------------------------------------------------------------------------------------\n');

        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the Lifting Surface component    : %5i\n', BLADE_AERO(i).NNLS);
        fprintf (FID, 'Number of Panels on the Lifting Surface             : %5i\n', BLADE_AERO(i).NPLS);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes on the blade root component         : %5i\n', BLADE_AERO(i).NNRootTran);
        fprintf (FID, 'Number of Panels on the blade root component        : %5i\n', BLADE_AERO(i).NPRootTran);
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes to be shed:   %5i\n', BLADE_AERO(i).NNShed);
        fprintf (FID, 'Number of Panels to be shed:  %5i\n', BLADE_AERO(i).NPShed);
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, 'Shedding Nodes\n');
        fprintf (FID, '%5i\n',BLADE_AERO(i).ShedNode);
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, 'Shedding Panels\n');
        fprintf (FID, '%5i\n',BLADE_AERO(i).ShedPanel);
        fprintf (FID, '\n');
        fprintf (FID, 'Blade Pre-bend                  : %s\n', BLADE_AERO(i).DEFORM.Label1);
        fprintf (FID, 'Max Pre-bend displacement (TIP) : %10.6f\n', BLADE_AERO(i).DEFORM.Deflec_1(end));
        fprintf (FID, 'Max Pre-bend rotation (TIP)     : %10.6f\n', BLADE_AERO(i).DEFORM.Theta1(end));
        fprintf (FID, 'Blade Sweep                     : %s\n', BLADE_AERO(i).DEFORM.Label2);
        fprintf (FID, 'Max Pre-bend displacement (TIP) : %10.6f\n', BLADE_AERO(i).DEFORM.Deflec_2(end));
        fprintf (FID, 'Max Pre-bend rotation (TIP)     : %10.6f\n', BLADE_AERO(i).DEFORM.Theta2(end));
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, 'Satinity Test\n');
        fprintf (FID, 'Original blade length : %10.6f\n', DATA(i).LBld);
        fprintf (FID, 'Final blade length    : %10.6f\n', BLADE_AERO(i).LBlade);
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '\n');

    end

    if DATA(i).Monopile == 1

        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '                M O N O P I L E   C O M P O N E N T   D A T A\n');
        fprintf (FID, '-------------------------------------------------------------------------------------\n');
        fprintf (FID, '\n');
        fprintf (FID, 'Number of Nodes                               : %5i\n', MONOPILE_AERO(i).NN);
        fprintf (FID, 'Number of Panels                              : %5i\n', MONOPILE_AERO(i).NP);
        fprintf (FID, '\n');
    end
end

%%

fclose (FID);



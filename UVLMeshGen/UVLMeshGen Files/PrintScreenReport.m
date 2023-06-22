% cOMPUTING NUMBER OF TOTAL NODES, ELEMENTS AND ZONES
Nodes    = 0;
Elements = 0;
Zones    = 0;
for i = 1:NumWT
    if DATA(i).Ground == 1
        Nodes = Nodes + GROUND_AERO(i).NN;
        Elements = Elements + GROUND_AERO(i).NP;
        Zones = Zones + 1;
    end
    if DATA(i).Tower == 1
        Nodes = Nodes + TOWER_AERO(i).NN;
        Elements = Elements + TOWER_AERO(i).NP;
        Zones = Zones + 1;
    end
    if DATA(i).Monopile == 1
        Nodes = Nodes + MONOPILE_AERO(i).NN;
        Elements = Elements + MONOPILE_AERO(i).NP;
        Zones = Zones + 1;
    end
    if DATA(i).Nacelle == 1
        Nodes = Nodes + NACELLE_AERO(i).NNPatch + NACELLE_AERO(i).NNCyl + ...
                        NACELLE_AERO(i).NNConnect + NACELLE_AERO(i).NNTail;
        Elements = Elements + NACELLE_AERO(i).NPPatch + NACELLE_AERO(i).NPCyl + ...
                        NACELLE_AERO(i).NPConnect + NACELLE_AERO(i).NPTail;
        Zones = Zones + 4;
    end
    if DATA(i).Hub == 1
        Nodes = Nodes + HUB_AERO(i).NNPatch*DATA(i).NumBld + HUB_AERO(i).NNConnect*DATA(i).NumBld + ...
                        HUB_AERO(i).NNHubNose;
        Elements = Elements + HUB_AERO(i).NPPatch*DATA(i).NumBld + HUB_AERO(i).NPConnect*DATA(i).NumBld + ...
                        HUB_AERO(i).NPHubNose;
        Zones = Zones + 1 + 2*DATA(i).NumBld;
    end
    if DATA(i).Blade == 1
        Nodes = Nodes + BLADE_AERO(i).NNLS*DATA(i).NumBld + BLADE_AERO(i).NNRootTran*DATA(i).NumBld;
        Elements = Elements + BLADE_AERO(i).NPLS*DATA(i).NumBld + BLADE_AERO(i).NPRootTran*DATA(i).NumBld;
        Zones = Zones + 2*DATA(i).NumBld;
    end
end
if (NumWT > 1 && strcmpi(Ground_FLAG{1,1}, 'ON'))
    for i = 1:GROUND_FARM.NumBox
        Nodes = Nodes + GROUND_FARM.PATCH(i).NN;
        Elements = Elements + GROUND_FARM.PATCH(i).NP;
    end
    Zones = Zones + GROUND_FARM.NumBox;
end

%-----------------------------------------------------------------------------------------------------


fprintf('\n')
cprintf('*black','******************************************************************************\n')
cprintf('*black','                                   REPORT\n')
cprintf('*black','******************************************************************************\n')
fprintf('\n')
fprintf('Number of Wind Turbines: %i\n', NumWT)
fprintf('\n')
for i = 1:NumWT
    fprintf('\t Wind Turbine %i: %s, Coordinate (x,y,z) = (%5.1f, %5.1f, %5.1f)\n', i, ...
             extractBefore(DATA(i).NameBld, '.'), WTNames{i,2}, WTNames{i,3}, WTNames{i,4})
end
fprintf('\n')
fprintf('Homogeneous wind farm activated: %s\n', EQWT_FLAG)
fprintf('Ground Farm activated: %s\n', Ground_FLAG{1,1})
if strcmpi(Ground_FLAG{1,2}, 'ON')
    fprintf('Terrain level activated: %s / Mode: %s \n', Ground_FLAG{1,2}, Ground_FLAG{1,3})
else
    fprintf('Terrain level activated: %s / Mode: - \n', Ground_FLAG{1,2})
end
if strcmpi(Ground_FLAG{1,1}, 'ON')
    fprintf('Number of Patches to generate the Ground Farm: %i\n', (length(GroundDivision{1,1})-1)*(length(GroundDivision{2,1})-1));
end
fprintf('Kinematics activated: %s\n', Kinematic_FLAG{1,1})
fprintf('\n')
cprintf('*black','-----------------------------------------------------------------------------\n')
cprintf('*black','                            Aerodynamics Data\n')
cprintf('*black','-----------------------------------------------------------------------------\n')
fprintf('Total number of aerodynamic nodes: %i\n', Nodes)
fprintf('Total number of aerodynamic panels: %i\n', Elements)
fprintf('Total number of Zones (for Tecplot purposes): %i\n', Zones)
if strcmpi(Kinematic_FLAG{1,1}, 'ON')
    for i = 1:NumWT
        fprintf('\t Characteristic length - Wind Turbine %i: %10.6f\n', [i DATA(i).LC])
    end

    fprintf('\t Characteristic Time (Time step): %10.6f (Mode: %s)\n', DTm, Kinematic_FLAG{1,5})
end
cprintf('*black','-----------------------------------------------------------------------------\n')

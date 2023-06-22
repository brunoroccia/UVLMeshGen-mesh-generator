%*********************************************************************************
% Main Program script is called from Mesh_Generator.m
% This script contains all the calls to build onshore/offshore wind farms meshes
% UiB, June 22, 2023
% bruno.roccia@uib.no
%*********************************************************************************

CurrentFolder = pwd;
Folders = SetFolderName;
Path1 = fullfile (CurrentFolder, Folders{6,1});
Path2 = fullfile (CurrentFolder, Folders{8,1});
Path3 = fullfile (CurrentFolder, Folders{9,1});
addpath(Path1, Path2, Path3);

cprintf('*black','-----------------------------------------------------------------------------\n')
cprintf ('*black','UVLMeshGen - onshore/offshore wind farms\n');
cprintf ('blue','Creative Commons Attribution 4.0 International License.\n');
cprintf ('black','Version 1.0 - First released June 22, 2023\n');
cprintf ('black','\t - Bergen Offshore Wind Centre (BOW), UiB, Norway\n');
cprintf ('black','\t - Group of Applied Mathematics (GMA), UNRC, Argentina\n');
cprintf ('blue','Repository: https://github.com/brunoroccia/UVLMeshGen-mesh-generator\n');
cprintf('*black','-----------------------------------------------------------------------------\n')
cprintf ('*black','UVLMeshGen is running....\n');
cprintf ('black','\n');

SetInitialVariables;

%% ------------------------------------------------------------------------

if strcmpi(EQWT_FLAG, 'ON')

    [DATA, CodError] = ReadWindTurbinesFiles(DATA, 1, Folders{3,1}, WTNames{1,1}, CodError);
    if CodError.WTFile == 1
        cprintf ('*red','Error: WT Datasheet file Name: %s Not valid\n',WTNames{1,1});
        return
    else
        fprintf ('Wind Turbine datasheet file successfully read ... !\n');
    end

    Fields = fieldnames (DATA(1));
    for i = 2:NumWT
        for j = 1:numel(Fields)
            DATA(i).(string(Fields(j))) = DATA(1).(string(Fields(j)));
        end
    end

    % Airfoil blade Analysis & Aerodynamic Mesh

    if DATA(1).Blade == 1
        [BLADE, CodError] = BladeGeometry (BLADE, Folders, DATA(1).NameBld, CodError, 1);

        if (CodError.BldBlade == 1)
            fprintf ('\n');
            cprintf ('*red','Process... ABORTED \n');
            cprintf ('*red','Blade file: %s does not exist\n', CodError.BladeName);
            return
        end

        if (CodError.BldAirfoil == 1)
            fprintf ('\n');
            cprintf ('*red','Process... ABORTED \n');
            cprintf ('*red','Airfoil file: %s does not exist\n', CodError.AirfoilName);
            return
        end

        [BLADE_AERO, CodError] = AeroMesh_Blade (DATA, BLADE, BLADE_AERO, CodError, 1);
        
        if (CodError.BldBend == 1)
            fprintf ('\n');
            cprintf ('*red','Process... ABORTED \n');
            cprintf ('*red','Tip blade displacement too big for the Z0 start bending point considered ... (When using Zuteck formula) \n');
            return
        end

        Fields = fieldnames (BLADE_AERO(1));
        for i = 2:NumWT
            for j = 1:numel(Fields)
                BLADE_AERO(i).(string(Fields(j))) = BLADE_AERO(1).(string(Fields(j)));
            end
        end
    end

    if DATA(1).Hub == 1
        [HUB_AERO] = AeroMesh_Hub (DATA, HUB_AERO, 1);
        Fields = fieldnames (HUB_AERO(1));
        for i = 2:NumWT
            for j = 1:numel(Fields)
                HUB_AERO(i).(string(Fields(j))) = HUB_AERO(1).(string(Fields(j)));
            end
        end
    end
    

    if DATA(1).Nacelle == 1
        [NACELLE_AERO] = AeroMesh_Nacelle (DATA, NACELLE_AERO, 1);
        Fields = fieldnames (NACELLE_AERO(1));
        for i = 2:NumWT
            for j = 1:numel(Fields)
                NACELLE_AERO(i).(string(Fields(j))) = NACELLE_AERO(1).(string(Fields(j)));
            end
        end
    end

    if DATA(1).Tower == 1
        [TOWER_AERO] = AeroMesh_Tower (DATA, TOWER_AERO, 1);
        Fields = fieldnames (TOWER_AERO(1));
        for i = 2:NumWT
            for j = 1:numel(Fields)
                TOWER_AERO(i).(string(Fields(j))) = TOWER_AERO(1).(string(Fields(j)));
            end
        end
    end

    if DATA(1).Ground == 1
        [GROUND_AERO] = AeroMesh_Ground (DATA, GROUND_AERO, 1);
         Fields = fieldnames (GROUND_AERO(1));
        for i = 2:NumWT
            for j = 1:numel(Fields)
                GROUND_AERO(i).(string(Fields(j))) = GROUND_AERO(1).(string(Fields(j)));
            end
        end
    end

    if DATA(1).Monopile == 1
        [MONOPILE_AERO] = AeroMesh_Monopile (DATA, MONOPILE_AERO, 1);
        Fields = fieldnames (MONOPILE_AERO(1));
        for i = 2:NumWT
            for j = 1:numel(Fields)
                MONOPILE_AERO(i).(string(Fields(j))) = MONOPILE_AERO(1).(string(Fields(j)));
            end
        end
    end

elseif strcmpi(EQWT_FLAG, 'OFF')

    for i = 1:NumWT

        [DATA, CodError] = ReadWindTurbinesFiles(DATA, i, Folders{3,1}, WTNames{i,1}, CodError);

        if CodError.WTFile == 1
            cprintf ('*red','Error: WT Datasheet file Name: %s Not valid\n',WTNames{i,1});
            return
        else
            fprintf ('Wind Turbine datasheet file number: %i successfully read ... !\n', i);
        end

        DATA(i).NCircTow    = DATA(1).NCircTow;
        DATA(i).NCircGround = DATA(1).NCircGround;
        DATA(i).NCircMon    = DATA(1).NCircMon;

        if DATA(i).Blade == 1
            [BLADE, CodError] = BladeGeometry (BLADE, Folders, DATA(i).NameBld, CodError, i);
            
            if (CodError.BldBlade == 1)
                fprintf ('\n');
                cprintf ('*red','Process... ABORTED \n');
                cprintf ('*red','Blade file: %s does not exist\n', CodError.BladeName);
                return
            end

            if (CodError.BldAirfoil == 1)
                fprintf ('\n');
                cprintf ('*red','Process... ABORTED \n');
                cprintf ('*red','Airfoil file: %s does not exist\n', CodError.AirfoilName);
                return
            end
            [BLADE_AERO, CodError] = AeroMesh_Blade (DATA, BLADE, BLADE_AERO, CodError, i);

        end

        if (CodError.BldBend == 1)
            fprintf ('\n');
            cprintf ('*red','Process... ABORTED \n');
            cprintf ('*red','Tip blade displacement too big for the Z0 start bending point considered ... (When using Zuteck formula) \n');
            return
        end

        if DATA(i).Hub == 1
            [HUB_AERO] = AeroMesh_Hub (DATA, HUB_AERO, i);
        end

        if DATA(i).Nacelle == 1
            [NACELLE_AERO] = AeroMesh_Nacelle (DATA, NACELLE_AERO, i);
        end

        if DATA(i).Tower == 1
            [TOWER_AERO] = AeroMesh_Tower (DATA, TOWER_AERO, i);
        end

        if DATA(i).Ground == 1
            [GROUND_AERO] = AeroMesh_Ground (DATA, GROUND_AERO, i);
        end

        if DATA(i).Monopile == 1
            [MONOPILE_AERO] = AeroMesh_Monopile (DATA, MONOPILE_AERO, i);
        end        

    end

else
    cprintf ('*red','Error: Not valid entry for the variable EQWT_FLAG\n');
    return
end

%% GROUND Generation

if (NumWT > 1 && strcmpi(Ground_FLAG{1,1}, 'ON'))
    for i = 1:NumWT
        DATA(i).Ground = 0;
        WTNames{i,4} = 0.0;
    end
    [GROUND_FARM, CONNECT] = AeroMesh_GroundFarm (DATA, NumWT, EQWT_FLAG, WTNames, GroundDivision);
    fprintf ('Ground local component has been turned OFF because NumWT > 1 and Farm Ground is ON... !\n');
    fprintf ('FARM GROUND generated successfully... ! \n');
else
    GROUND_FARM = [];
end

if strcmpi(Ground_FLAG{1,2}, 'ON')
    [GROUND_AERO, GROUND_FARM, WTNames, CodError] = GroundLevelGeneration (DATA, GROUND_AERO, GROUND_FARM, NumWT, WTNames,...
        Folders, Ground_FLAG, CodError);
    if (CodError.GrGen == 0 && CodError.GrFlag == 0)
        fprintf ('Terrain altitude generated successfully... !\n');
    else
        if (CodError.GrGen == 1)
            cprintf ('*red','Error: Ground component disabled while Ground level is activated...\n');
            return
        end
        if (CodError.GrFlag == 1)
            cprintf ('*red','Input argument: %s Not valid...\n',Ground_FLAG{1,3});
            return
        end
    end
else
    fprintf ('Terrain level not activated...\n');
end

%% ASSEMBLING WIND FARM

if strcmpi(EQWT_FLAG, 'ON')
    [WIND_TURBINE, CONNECT] = AssemblingWT (DATA, WIND_TURBINE, CONNECT, BLADE_AERO, HUB_AERO, ...
        NACELLE_AERO, TOWER_AERO, GROUND_AERO, MONOPILE_AERO, ...
        WTNames, 1);

    Fields1 = fieldnames (WIND_TURBINE(1));
    Fields2 = fieldnames (CONNECT(1));
    for i = 2:NumWT
        for j = 1:numel(Fields2)
            WIND_TURBINE(i).(string(Fields1(j))) = WIND_TURBINE(1).(string(Fields1(j)));
            CONNECT(i).(string(Fields2(j))) = CONNECT(1).(string(Fields2(j)));
        end
    end
    fprintf ('%i Wind turbines assembled successfully... !\n', NumWT);
else
    for i = 1:NumWT
        [WIND_TURBINE, CONNECT] = AssemblingWT (DATA, WIND_TURBINE, CONNECT, BLADE_AERO, HUB_AERO, ...
            NACELLE_AERO, TOWER_AERO, GROUND_AERO, MONOPILE_AERO, ...
            WTNames, i);
    end
    fprintf ('%i Wind turbines assembled successfully... !\n', NumWT);
end

for i = 1: NumWT
    [WIND_TURBINE] = Translate (WIND_TURBINE, DATA, WTNames, i);
end

if strcmpi(Ground_FLAG{1,1}, 'ON')
    if NumWT == 1
        WIND_TURBINE = GrTowCorrection (DATA, WIND_TURBINE, GROUND_FARM, EQWT_FLAG, 0);
    else
        WIND_TURBINE = GrTowCorrection (DATA, WIND_TURBINE, GROUND_FARM, EQWT_FLAG, 1);
    end
end

%% KINEMATIC

if strcmpi(Kinematic_FLAG{1,1}, 'ON')
    for i = 1:NumWT
        AUX(i) = DATA(i).UVLMSteps;
    end
    AUX = (AUX == DATA(1).UVLMSteps);
    if ~all(AUX)
        cprintf ('*red','Warning: Kinematic generation aborted / Number of time steps must be equal for all WTs...\n');
        return
    end

    for i = 1:NumWT
        if DATA(i).UVLMLC == 0
            LC = Characteristiclength (BLADE_AERO(i).XYZLS, BLADE_AERO(i).ICONLS);
            DATA(i).LC = LC;
        else
            DATA(i).LC = DATA(i).UVLMLC;
        end
        DT(i) = DATA(i).LC / DATA(i).UVLMVC;
    end

    if strcmpi(Kinematic_FLAG{1,5}, 'Average')
        DTm = mean (DT);
    elseif strcmpi(Kinematic_FLAG{1,5}, 'Max')
        DTm = max(DT);
    elseif strcmpi(Kinematic_FLAG{1,5}, 'Min')
        DTm = min(DT);
    else
        cprintf ('*red','Input argument: %s Not valid...\n',Kinematic_FLAG{1,5});
        return
    end

    AUXEr =[];
    for i = 1:NumWT
        DATA(i).DT = DTm;

        if strcmpi(Kinematic_FLAG{1,2}, 'RotorON')
            Name = fullfile (Folders{8,1}, strcat(DATA(i).NameRot,'.m'));
            Flag1 = exist (Name,'file');
            if Flag1 == 0
                cprintf ('*red','Rotor Kinematic file: %s does not exist...\n',DATA(i).NameRot);
                return
            end
        end

        if strcmpi(Kinematic_FLAG{1,3}, 'YawON')
            Name = fullfile (Folders{8,1}, strcat(DATA(i).NameYaw,'.m'));
            Flag1 = exist (Name,'file');
            if Flag1 == 0
                cprintf ('*red','Rotor Kinematic file: %s does not exist...\n',DATA(i).NameYaw);
                return
            end
        end

        if strcmpi(Kinematic_FLAG{1,4}, 'PitchON')
            Name = fullfile (Folders{8,1}, strcat(DATA(i).NamePitch,'.m'));
            Flag1 = exist (Name,'file');
            if Flag1 == 0
                cprintf ('*red','Rotor Kinematic file: %s does not exist...\n',DATA(i).NamePitch);
                return
            end
        end        

        [KINEMATICS, SEMatrices, DATA] = KinematicsProcessor (KINEMATICS, SEMatrices, DATA, WIND_TURBINE, BLADE_AERO, HUB_AERO, ...
            NACELLE_AERO, TOWER_AERO, GROUND_AERO, GROUND_FARM, MONOPILE_AERO, Kinematic_FLAG, Ground_FLAG, ...
            WTNames, i);
    end
    

end


%% OUTPUTs

if strcmp(OPT_FLAG{1}, 'ON')
    Flag1 = exist (fullfile('UVLMeshGen Files/',OPT_FLAG{1,2}),'file');
    Flag2 = exist (fullfile('Output Scripts/',OPT_FLAG{1,2}),'file');
    if Flag1 == 0 && Flag2 == 0
        fprintf ('\n');
        cprintf ('*red','Script name: %s Not valid...\n',OPT_FLAG{1,2});
    else
        Fcn = str2func(OPT_FLAG{1,2});
        Fcn();
    end
    if CodError.OptTecp == 1
        fprintf ('\n');
        cprintf ('*red','Not possible to write kinematics TECPLOT file\n');
        cprintf ('*red','Kinematics processor is not activated...\n');
    end
end

%%  OUTPUT Reports

PrintScreenReport;

ReportOutput (NumWT, OutputName, DATA, BLADE_AERO, ...
   HUB_AERO, NACELLE_AERO, TOWER_AERO, GROUND_AERO, ...
   MONOPILE_AERO, Ground_FLAG, Nodes, Elements);

%%
function Folders = SetFolderName ()
    Folders{1,1}  = 'Airfoils';
    Folders{2,1}  = 'BladeDataSheetFiles';
    Folders{3,1}  = 'WTDataSheetFiles';
    Folders{4,1}  = 'Tecplot Files';
    Folders{5,1}  = 'Report Files';
    Folders{6,1}  = 'Terrain Data';
    Folders{7,1}  = 'VLMSim Solver';
    Folders{8,1}  = 'Kinematic Files';
    Folders{9,1}  = 'UVLMeshGen Files';
    Folders{10,1} = 'Output Scripts';
end

%% CHARACTERISTIC LENGTH

function LC = Characteristiclength (XYZ, ICON)

NPLS = size(ICON,1);
A  = 0.00;

for i = 1:NPLS
    
    r1 = XYZ ( ICON (i,2), 1:3 ) - XYZ ( ICON (i,1), 1:3 );
    r2 = XYZ ( ICON (i,3), 1:3 ) - XYZ ( ICON (i,2), 1:3 );
    rk = cross (r1, r2);
    JJ  = norm(rk, 2);
    A = A + JJ;
    
end

LC = sqrt (A / NPLS);

end




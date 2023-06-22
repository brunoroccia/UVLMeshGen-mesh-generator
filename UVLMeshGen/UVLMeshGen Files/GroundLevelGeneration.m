function [GROUND_AERO, GROUND_FARM, WTNames, CodError] = GroundLevelGeneration (DATA, GROUND_AERO, GROUND_FARM, NumWT, WTNames,...
    Folders, Ground_FLAG, CodError)

if strcmpi(Ground_FLAG{1,3}, 'externaldata')
    XYZTerrain = ReadGround (Folders{6,1}, Ground_FLAG{1,5});
    GFunction = fit (XYZTerrain(:,1:2), XYZTerrain(:,3), Ground_FLAG{1,6});
    if (NumWT == 1) && (DATA(1).Ground==1)
        XYZ = GROUND_AERO(1).XYZ;
        XYZ = [XYZ; [WTNames{1,2}, WTNames{1,3}, WTNames{1,4}]];
        NNTotal = GROUND_AERO(1).NN + 1;
        XYZOutput = zeros(NNTotal,3);
        for i = 1:NNTotal
            XYZOutput(i,1) = XYZ(i,1);
            XYZOutput(i,2) = XYZ(i,2);
            XYZOutput(i,3) = GFunction(XYZOutput(i,1),XYZOutput(i,2));
        end
        GROUND_AERO(1).XYZ = XYZOutput(1:end-1,1:3);
        WTNames{1,2} = XYZOutput(end,1);
        WTNames{1,3} = XYZOutput(end,2);
        WTNames{1,4} = XYZOutput(end,3);
        NNCP = GROUND_AERO(1).NP;
        XYZOutput =  zeros(NNCP,1:3);
        for i = 1:NNCP
            XYZOutput(i,1) = GROUND_AERO(1).XYZCP(i,1);
            XYZOutput(i,2) = GROUND_AERO(1).XYZCP(i,2);
            XYZOutput(i,3) = GFunction(XYZOutput(i,1),XYZOutput(i,2));
        end
        GROUND_AERO(1).XYZCP  = XYZOutput(1:end,1:3);
        GROUND_AERO(1).Normal = NormallVector (GROUND_AERO(1).XYZ, GROUND_AERO(1).ICON, GROUND_AERO(1).NP);
    elseif NumWT > 1
        XYZ   = [];
        XYZCP = [];
        NNTotal = 0;
        NPTotal = 0;
        for i = 1:length(GROUND_FARM.PATCH)
            XYZ     = [XYZ; GROUND_FARM.PATCH(i).XYZ];
            XYZCP   = [XYZCP; GROUND_FARM.PATCH(i).XYZCP];
            NNTotal = NNTotal + GROUND_FARM.PATCH(i).NN;
            NPTotal = NPTotal + GROUND_FARM.PATCH(i).NP;
        end
        for i = 1:NumWT
            XYZ = [XYZ; [WTNames{i,2}, WTNames{i,3}, WTNames{i,4}] ];
            NNTotal = NNTotal + 1;
        end
        XYZOutput = zeros(NNTotal,3);
        for i = 1:NNTotal
            XYZOutput(i,1) = XYZ(i,1);
            XYZOutput(i,2) = XYZ(i,2);
            XYZOutput(i,3) = GFunction(XYZOutput(i,1),XYZOutput(i,2));
        end
        I1 = 1;
        I2 = GROUND_FARM.PATCH(1).NN;
        GROUND_FARM.PATCH(1).XYZ = XYZOutput(I1:I2,1:3);
        for i = 2:length(GROUND_FARM.PATCH)
            I1 = I2 + 1;
            I2 = I2 + GROUND_FARM.PATCH(i).NN;
            GROUND_FARM.PATCH(i).XYZ = XYZOutput(I1:I2,1:3);
        end
        for i = NumWT:-1:1
            WTNames{i,2} = XYZOutput(end-(NumWT-i),1);
            WTNames{i,3} = XYZOutput(end-(NumWT-i),2);
            WTNames{i,4} = XYZOutput(end-(NumWT-i),3);
        end
        XYZOutput  = zeros(NPTotal,3);
        for i = 1:NPTotal
            XYZOutput(i,1) = XYZCP(i,1);
            XYZOutput(i,2) = XYZCP(i,2);
            XYZOutput(i,3) = GFunction(XYZOutput(i,1),XYZOutput(i,2));
        end
        I1 = 1;
        I2 = GROUND_FARM.PATCH(1).NP;
        GROUND_FARM.PATCH(1).XYZCP = XYZOutput(I1:I2,1:3);
        GROUND_FARM.PATCH(1).Normal = NormallVector (GROUND_FARM.PATCH(1).XYZ, GROUND_FARM.PATCH(1).ICON, GROUND_FARM.PATCH(1).NP);
        for i = 2:length(GROUND_FARM.PATCH)
            I1 = I2 + 1;
            I2 = I2 + GROUND_FARM.PATCH(i).NP;
            GROUND_FARM.PATCH(i).XYZCP = XYZOutput(I1:I2,1:3);
            GROUND_FARM.PATCH(i).Normal = NormallVector (GROUND_FARM.PATCH(i).XYZ, GROUND_FARM.PATCH(i).ICON, ...
                                          GROUND_FARM.PATCH(i).NP);
        end
    else
        CodError.GrGen = 1;
        return
    end
elseif strcmpi(Ground_FLAG{1,3}, 'userfunction')
    Fcn = str2func(Ground_FLAG{1,4});
    if (NumWT == 1) && (DATA(1).Ground==1)
        XYZ = GROUND_AERO(1).XYZ;
        XYZ = [XYZ; [WTNames{1,2}, WTNames{1,3}, WTNames{1,4}]];
        NNTotal = GROUND_AERO(1).NN + 1;
        XYZOutput = Fcn (XYZ, NNTotal);
        GROUND_AERO(1).XYZ = XYZOutput(1:end-1,1:3);
        WTNames{1,2} = XYZOutput(end,1);
        WTNames{1,3} = XYZOutput(end,2);
        WTNames{1,4} = XYZOutput(end,3);
        XYZOutput = Fcn (GROUND_AERO(1).XYZCP, GROUND_AERO(1).NP);
        GROUND_AERO(1).XYZCP  = XYZOutput(1:end,1:3);
        GROUND_AERO(1).Normal = NormallVector (GROUND_AERO(1).XYZ, GROUND_AERO(1).ICON, GROUND_AERO(1).NP);
    elseif NumWT > 1
        XYZ   = [];
        XYZCP = [];
        NNTotal = 0;
        NPTotal = 0;
        for i = 1:length(GROUND_FARM.PATCH)
            XYZ     = [XYZ; GROUND_FARM.PATCH(i).XYZ];
            XYZCP   = [XYZCP; GROUND_FARM.PATCH(i).XYZCP];
            NNTotal = NNTotal + GROUND_FARM.PATCH(i).NN;
            NPTotal = NPTotal + GROUND_FARM.PATCH(i).NP;
        end
        for i = 1:NumWT
            XYZ = [XYZ; [WTNames{i,2}, WTNames{i,3}, WTNames{i,4}] ];
            NNTotal = NNTotal + 1;
        end
        XYZOutput = Fcn (XYZ, NNTotal);
        I1 = 1;
        I2 = GROUND_FARM.PATCH(1).NN;
        GROUND_FARM.PATCH(1).XYZ = XYZOutput(I1:I2,1:3);
        for i = 2:length(GROUND_FARM.PATCH)
            I1 = I2 + 1;
            I2 = I2 + GROUND_FARM.PATCH(i).NN;
            GROUND_FARM.PATCH(i).XYZ = XYZOutput(I1:I2,1:3);
        end
        for i = NumWT:-1:1
            WTNames{i,2} = XYZOutput(end-(NumWT-i),1);
            WTNames{i,3} = XYZOutput(end-(NumWT-i),2);
            WTNames{i,4} = XYZOutput(end-(NumWT-i),3);
        end
        XYZOutput = Fcn (XYZCP, NPTotal);
        I1 = 1;
        I2 = GROUND_FARM.PATCH(1).NP;
        GROUND_FARM.PATCH(1).XYZCP = XYZOutput(I1:I2,1:3);
        NOR = NormallVector (GROUND_FARM.PATCH(1).XYZ, GROUND_FARM.PATCH(1).ICON, ...
                                      GROUND_FARM.PATCH(1).NP);
        GROUND_FARM.PATCH(1).Normal = NOR;
        for i = 2:length(GROUND_FARM.PATCH)
            I1 = I2 + 1;
            I2 = I2 + GROUND_FARM.PATCH(i).NP;
            GROUND_FARM.PATCH(i).XYZCP = XYZOutput(I1:I2,1:3);
            GROUND_FARM.PATCH(i).Normal = NormallVector (GROUND_FARM.PATCH(i).XYZ, GROUND_FARM.PATCH(i).ICON, ...
                                          GROUND_FARM.PATCH(i).NP);
        end
    else
        CodError.GrGen = 1;
        return
    end

else
    CodError.GrFlag = 1;
    return
end

end

%%

function XYZ = ReadGround (Folder, NameT)
    Name = fullfile (Folder,NameT);    
    FID = fopen (Name, 'r');
    for i = 1:11
        aux = fgets(FID);
    end
    XYZ = fscanf (FID, '%g %g %g', [3 inf]);
    XYZ = transpose(XYZ);
    fclose (FID);
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
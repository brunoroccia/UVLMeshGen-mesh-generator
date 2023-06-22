function AUXEr = TestKinematics (AUXEr, KINEMATICS, SEMatrices, NACELLE_AERO, HUB_AERO, BLADE_AERO, DATA, Indx)

TimeSteps = DATA(Indx).UVLMSteps;
DT        = DATA(Indx).DT;

for i = 1:TimeSteps

%----------------------------------------------------------------------------------------
%                        N A C E L L E   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Nacelle == 1

        SEM  = SEMatrices(Indx).HNac{i};
        DSEM = SEMatrices(Indx).DHNac{i};

        AUX1(i).WT(Indx).NACPART(1).XYZ = NACELLE_AERO(Indx).XYZPatch;
        AUX1(i).WT(Indx).NACPART(2).XYZ = NACELLE_AERO(Indx).XYZCyl;
        AUX1(i).WT(Indx).NACPART(3).XYZ = NACELLE_AERO(Indx).XYZTail;
        AUX1(i).WT(Indx).NACPART(4).XYZ = NACELLE_AERO(Indx).XYZConnect;

        AUX1(i).WT(Indx).NACPART(1).XYZCP = NACELLE_AERO(Indx).XYZCPPatch;
        AUX1(i).WT(Indx).NACPART(2).XYZCP = NACELLE_AERO(Indx).XYZCPCyl;
        AUX1(i).WT(Indx).NACPART(3).XYZCP = NACELLE_AERO(Indx).XYZCPTail;
        AUX1(i).WT(Indx).NACPART(4).XYZCP = NACELLE_AERO(Indx).XYZCPConnect;

        AUX1(i).WT(Indx).NACPART(1).XYZ(:,4) = 1;
        AUX1(i).WT(Indx).NACPART(2).XYZ(:,4) = 1;
        AUX1(i).WT(Indx).NACPART(3).XYZ(:,4) = 1;
        AUX1(i).WT(Indx).NACPART(4).XYZ(:,4) = 1;

        AUX1(i).WT(Indx).NACPART(1).XYZCP(:,4) = 1;
        AUX1(i).WT(Indx).NACPART(2).XYZCP(:,4) = 1;
        AUX1(i).WT(Indx).NACPART(3).XYZCP(:,4) = 1;
        AUX1(i).WT(Indx).NACPART(4).XYZCP(:,4) = 1;        

        for j = 1:4
            for k = 1:size(AUX1(i).WT(Indx).NACPART(j).XYZ,1)
                AUX1(i).WT(Indx).NACPART(j).XYZ(k,:) = transpose (SEM * (transpose(AUX1(i).WT(Indx).NACPART(j).XYZ(k,:))) );
            end
            for k = 1:size(AUX1(i).WT(Indx).NACPART(j).XYZCP,1)
                AUX1(i).WT(Indx).NACPART(j).VCP(k,:)   = transpose (DSEM * (transpose(AUX1(i).WT(Indx).NACPART(j).XYZCP(k,:))) );
                AUX1(i).WT(Indx).NACPART(j).XYZCP(k,:) = transpose (SEM * (transpose(AUX1(i).WT(Indx).NACPART(j).XYZCP(k,:))) );
            end

            AUXEr(i).WT(Indx).NACPART(j).XYZ   = AUX1(i).WT(Indx).NACPART(j).XYZ(:,1:3) - KINEMATICS(i).WT(Indx).NACPART(j).XYZ;
            AUXEr(i).WT(Indx).NACPART(j).XYZCP = AUX1(i).WT(Indx).NACPART(j).XYZCP(:,1:3) - KINEMATICS(i).WT(Indx).NACPART(j).XYZCP;
            AUXEr(i).WT(Indx).NACPART(j).VCP   = AUX1(i).WT(Indx).NACPART(j).VCP(:,1:3) - KINEMATICS(i).WT(Indx).NACPART(j).VCP;
            AUXEr(i).WT(Indx).NACPART(j).ErrorNN  = norm(AUXEr(i).WT(Indx).NACPART(j).XYZ,inf);
            AUXEr(i).WT(Indx).NACPART(j).ErrorCP  = norm(AUXEr(i).WT(Indx).NACPART(j).XYZCP,inf);
            AUXEr(i).WT(Indx).NACPART(j).ErrorVCP = norm(AUXEr(i).WT(Indx).NACPART(j).VCP,inf);
        end


    end

%----------------------------------------------------------------------------------------
%                             H U B   K I N E M A T I C S
%----------------------------------------------------------------------------------------

    if DATA(Indx).Hub == 1    

        SEM  = SEMatrices(Indx).HHub{i};
        DSEM = SEMatrices(Indx).DHHub{i};
        
        AUX1(i).WT(Indx).HUBPART(1).XYZ  = HUB_AERO(Indx).XYZNose;
        AUX1(i).WT(Indx).HUBPART(1).XYZ(:,4) = 1;

        AUX1(i).WT(Indx).HUBPART(1).XYZCP  = HUB_AERO(Indx).XYZCPNose;
        AUX1(i).WT(Indx).HUBPART(1).XYZCP(:,4) = 1;

        k = 2;
        for j = 1:DATA(Indx).NumBld
            AUX1(i).WT(Indx).HUBPART(k).XYZ     = HUB_AERO(Indx).PATCH(j).XYZ;
            AUX1(i).WT(Indx).HUBPART(k).XYZ(:,4) = 1;
            AUX1(i).WT(Indx).HUBPART(k+1).XYZ   = HUB_AERO(Indx).CONNECT(j).XYZ;
            AUX1(i).WT(Indx).HUBPART(k+1).XYZ(:,4) = 1;

            AUX1(i).WT(Indx).HUBPART(k).XYZCP        = HUB_AERO(Indx).PATCH(j).XYZCP;
            AUX1(i).WT(Indx).HUBPART(k).XYZCP(:,4)   = 1;
            AUX1(i).WT(Indx).HUBPART(k+1).XYZCP      = HUB_AERO(Indx).CONNECT(j).XYZCP;
            AUX1(i).WT(Indx).HUBPART(k+1).XYZCP(:,4) = 1;
            k = k + 2;
        end


        for j = 1:DATA(Indx).NumBld*2+1
            for k = 1:size(AUX1(i).WT(Indx).HUBPART(j).XYZ,1)
                AUX1(i).WT(Indx).HUBPART(j).XYZ(k,:) = transpose (SEM * ( transpose(AUX1(i).WT(Indx).HUBPART(j).XYZ(k,:))));
            end
            for k = 1:size(AUX1(i).WT(Indx).HUBPART(j).XYZCP,1)
                AUX1(i).WT(Indx).HUBPART(j).VCP(k,:)   = transpose (DSEM * (transpose(AUX1(i).WT(Indx).HUBPART(j).XYZCP(k,:))) );
                AUX1(i).WT(Indx).HUBPART(j).XYZCP(k,:) = transpose (SEM * (transpose(AUX1(i).WT(Indx).HUBPART(j).XYZCP(k,:))) );
            end

            AUXEr(i).WT(Indx).HUBPART(j).XYZ   = AUX1(i).WT(Indx).HUBPART(j).XYZ(:,1:3) - KINEMATICS(i).WT(Indx).HUBPART(j).XYZ;
            AUXEr(i).WT(Indx).HUBPART(j).XYZCP = AUX1(i).WT(Indx).HUBPART(j).XYZCP(:,1:3) - KINEMATICS(i).WT(Indx).HUBPART(j).XYZCP;
            AUXEr(i).WT(Indx).HUBPART(j).VCP   = (AUX1(i).WT(Indx).HUBPART(j).VCP(:,1:3) - KINEMATICS(i).WT(Indx).HUBPART(j).VCP)./KINEMATICS(i).WT(Indx).HUBPART(j).VCP;
            AUXEr(i).WT(Indx).HUBPART(j).ErrorNN  = max(max(abs(AUXEr(i).WT(Indx).HUBPART(j).XYZ)));
            AUXEr(i).WT(Indx).HUBPART(j).ErrorCP  = max(max(abs(AUXEr(i).WT(Indx).HUBPART(j).XYZCP)));
            AUXEr(i).WT(Indx).HUBPART(j).ErrorVCP = max(max(abs(AUXEr(i).WT(Indx).HUBPART(j).VCP)));

        end

    end

end

for i = 1:TimeSteps
    for j = 1:DATA(Indx).NumBld*2+1
        HUBCP{i,j}=AUX1(i).WT(Indx).HUBPART(j).XYZCP(:,1:3);
    end
end
for j = 1:DATA(Indx).NumBld*2+1
    [DXYZ] = FiniteDiference (HUBCP, DT, TimeSteps, j, 2);
    DHUBCP{j} = DXYZ;
end

for i = 1:TimeSteps
    for j = 1:DATA(Indx).NumBld*2+1
        AUXEr(i).WT(Indx).HUBPART(j).ErrorVCP1   = max(max(abs((DHUBCP{j}{i} - KINEMATICS(i).WT(Indx).HUBPART(j).VCP)./KINEMATICS(i).WT(Indx).HUBPART(j).VCP)));

    end
end

end

%% Finite difference 2nd Orden

function [DXYZ] = FiniteDiference (XYZ, DT, TimeSteps, j, Order)

switch Order

    case (2)

        for i = 1:TimeSteps
            if i == 1
                Mi  = XYZ{i,j};
                Mi1 = XYZ{i+1,j};
                Mi2 = XYZ{i+2,j};
                DXYZ{i} = (-3*Mi + 4*Mi1 - Mi2) ./ (2*DT);
            elseif i == TimeSteps
                Mi  = XYZ{i,j};
                Mi1 = XYZ{i-1,j};
                Mi2 = XYZ{i-2,j};
                DXYZ{i} = (3*Mi - 4*Mi1 + Mi2) ./ (2*DT);
            else
                Mi1 = XYZ{i-1,j};
                Mi2 = XYZ{i+1,j};
                DXYZ{i} = (Mi2 - Mi1) ./ (2*DT);
            end
        end
    
    otherwise


end

end
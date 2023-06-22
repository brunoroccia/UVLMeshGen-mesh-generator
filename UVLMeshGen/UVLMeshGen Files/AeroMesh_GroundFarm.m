function [GROUND_FARM, CONNECT] = AeroMesh_GroundFarm (DATA, Num, EQWT_FLAG, WTNames, GroundDivision)

NBoxX = size(GroundDivision{1,1},2) - 1;
NBoxY = size(GroundDivision{2,1},2) - 1;

NumBox = NBoxX * NBoxY;

WTCoord = zeros(Num,4);

for  i = 1:Num
    WTCoord(i,1) = WTNames{i,2};
    WTCoord(i,2) = WTNames{i,3};
    R0(i,:)      = [WTNames{i,2}, WTNames{i,3}, WTNames{i,4}];
end

Xmin = min(GroundDivision{1,1});
Xmax = max(GroundDivision{1,1});
Ymin = min(GroundDivision{2,1});
Ymax = max(GroundDivision{2,1});

A1 = (Xmax+Xmin)/2;
B1 = (Xmax-Xmin)/2;
A2 = (Ymax+Ymin)/2;
B2 = (Ymax-Ymin)/2;

for  i = 1:Num
    WTCoord(i,3) = (WTCoord(i,1)-A1)/B1;
    WTCoord(i,4) = (WTCoord(i,2)-A2)/B2;
end

%% Generation od the ID for the ground patches

DX = sort(GroundDivision{1,1}, 'ascend');
DY = sort(GroundDivision{2,1}, 'ascend');

k = 1;
for i = 1:NBoxY
    for j = 1:NBoxX
        BoxBoundary(k,1) = (DX(j) - A1) / B1;
        BoxBoundary(k,2) = (DX(j+1) - A1) / B1;
        BoxBoundary(k,3) = (DY(i) - A2) / B2;
        BoxBoundary(k,4) = (DY(i+1) - A2) / B2;
        k = k + 1;
    end
end

IDWT = zeros(1,NumBox);

k = 1;
for i = 1:Num
    if DATA(i).Tower == 1
        for j = 1:NumBox
            if (WTCoord(i,3)>BoxBoundary(j,1) && WTCoord(i,3)<=BoxBoundary(j,2)) && (WTCoord(i,4)>BoxBoundary(j,3) && WTCoord(i,4)<=BoxBoundary(j,4))
                IDWT(1,j) = k;
                k = k + 1;
            end
        end
    end
end

GROUND_FARM.IDWT = IDWT;
GROUND_FARM.NumBox = NumBox;

%%

Grd_NodR = DATA(1).NRadGround;       % Radial direction
Grd_NodC = DATA(1).NCircGround;      % Circunferential direction

NNSquare = (Grd_NodC + 3) / 4;

for i = 1:NumBox

    if IDWT(1,i) == 0

        DR = linspace (BoxBoundary(i,1), BoxBoundary(i,2), NNSquare);
        DS = linspace (BoxBoundary(i,3), BoxBoundary(i,4), NNSquare);
        kk = 1;
        for j = 1:length(DR)
            for k = 1:length(DS)
                GROUND_FARM.PATCH(i).XYZ(kk,1) = A1 + B1*DR(k);
                GROUND_FARM.PATCH(i).XYZ(kk,2) = A2 + B2*DS(j);
                GROUND_FARM.PATCH(i).XYZ(kk,3) = 0.0;
                kk = kk + 1;
            end
        end

        ICON = Connectivities (length(DS), length(DR));
        GROUND_FARM.PATCH(i).ICON = ICON;
        GROUND_FARM.PATCH(i).NNY       = length(DS);
        GROUND_FARM.PATCH(i).NNX       = length(DR);
        GROUND_FARM.PATCH(i).NN        = length(DR)*length(DS);
        GROUND_FARM.PATCH(i).NP        = (length(DR)-1)*(length(DS)-1);
        GROUND_FARM.PATCH(i).NNR       = [];
        GROUND_FARM.PATCH(i).NNC       = [];
        GROUND_FARM.PATCH(i).FLAG      = 0;

        CONNECT.GROUND_FARM.PATCH(i).Icon = ICON;

    else
        if strcmpi(EQWT_FLAG, 'Yes')
            Grd_DB   = DATA(1).RTowGround * 2;
            Grd_NodR = DATA(1).NRadGround;
            Grd_NodC = DATA(1).NCircGround;
        else
            Grd_DB   = (DATA(IDWT(1,i)).RTowGround * 2);
            Grd_NodR = DATA(IDWT(1,i)).NRadGround;
            Grd_NodC = DATA(IDWT(1,i)).NCircGround;
        end
        X1 = BoxBoundary(i,1)*B1 + A1;
        X2 = BoxBoundary(i,2)*B1 + A1;
        Y1 = BoxBoundary(i,3)*B2 + A2;
        Y2 = BoxBoundary(i,4)*B2 + A2;
        BoxXY = [X1 X2 Y1 Y2];

        [AUX] = GroundFarmDivision (BoxXY, WTCoord(IDWT(1,i),:), Grd_DB, Grd_NodR, Grd_NodC);

        GROUND_FARM.PATCH(i).ICON = AUX.ICONGround;
        GROUND_FARM.PATCH(i).XYZ  = AUX.XYZGround;
        GROUND_FARM.PATCH(i).NNY       = [];
        GROUND_FARM.PATCH(i).NNX       = [];
        GROUND_FARM.PATCH(i).NNR       = AUX.NNGroundR;
        GROUND_FARM.PATCH(i).NNC       = AUX.NNGroundC;
        GROUND_FARM.PATCH(i).NN        = AUX.NNGround;
        GROUND_FARM.PATCH(i).NP        = AUX.NPGround;
        GROUND_FARM.PATCH(i).FLAG      = 1;

        CONNECT.GROUND_FARM.PATCH(i).Icon = AUX.ICONGround;

        for  j = 1:size(GROUND_FARM.PATCH(i).XYZ,1)
            GROUND_FARM.PATCH(i).XYZ(j,:) = GROUND_FARM.PATCH(i).XYZ(j,:) + R0(IDWT(1,i),:);
        end


    end

end

%% Control points and normalls

for i = 1:NumBox

    XYZ_CP = ControlPoints (GROUND_FARM.PATCH(i).XYZ, GROUND_FARM.PATCH(i).ICON, GROUND_FARM.PATCH(i).NP);
    NOR    = NormallVector (GROUND_FARM.PATCH(i).XYZ, GROUND_FARM.PATCH(i).ICON, GROUND_FARM.PATCH(i).NP);
    GROUND_FARM.PATCH(i).XYZCP  = XYZ_CP;
    GROUND_FARM.PATCH(i).Normal = NOR;

end


end

%% CONTROL POINTS 

function XYZCP = ControlPoints (XYZ, ICON, NP)
XYZCP = zeros(NP,3);
for i = 1:NP
    XYZCP (i, 1:3) =  0.25D+0 * ( XYZ(ICON(i,1), 1:3) + ...
        XYZ(ICON(i,2), 1:3) + XYZ(ICON(i,3), 1:3) + ...
        XYZ(ICON(i,4), 1:3) );
end
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

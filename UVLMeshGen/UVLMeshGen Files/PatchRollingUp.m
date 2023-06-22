function PATCH = PatchRollingUp (Num_BD, COORD, D_Hub, H_Square)

AuxAngle = 2*pi / Num_BD / 2;

NN = size(COORD,1);

for i = 1:NN

    PATCH(1).XYZ(i,1) = COORD(i,1);
    PATCH(1).XYZ(i,2) = D_Hub/2 * sin (COORD(i,2)*AuxAngle*(1/H_Square));
    PATCH(1).XYZ(i,3) = D_Hub/2 * cos (COORD(i,2)*AuxAngle*(1/H_Square));

end

DGamma = 2 * pi / Num_BD;

for i = 2:Num_BD

    Gamma = DGamma * i;
    TNB   = [1 0 0; 0 cos(Gamma) -sin(Gamma); 0 sin(Gamma) cos(Gamma)];
    PATCH(i).XYZ(i,1:3) = transpose (TNB * transpose(PATCH(1).XYZ(i,1:3)));


end

end
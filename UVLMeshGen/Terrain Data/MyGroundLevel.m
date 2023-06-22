function XYZOutput = MyGroundLevel (XYZ, NN)

Ymin = min(XYZ(:,2));
Ymax = max(XYZ(:,2));
DD   = Ymax - Ymin;

XYZOutput = XYZ;

A = [Ymin^2 Ymin, 1; Ymax^2 Ymax 1; 2*Ymin 1 0];
F = [0; DD/4; 0];
Coef = A\F;

for i = 1:NN
    XYZOutput(i,3) = Coef(1)*XYZOutput(i,2)^2 + Coef(2)*XYZOutput(i,2) + Coef(3);
end

end
function XYZOutput = MyGroundLevel_1 (XYZ, NN)

Xmin = min(XYZ(:,1));
Xmax = max(XYZ(:,1));
Ymin = min(XYZ(:,2));
Ymax = max(XYZ(:,2));
DD   = Ymax - Ymin;

XYZOutput = XYZ;

WW1 = 4*pi / Xmax;
WW2 = 4*pi / Ymax;

for i = 1:NN
    XYZOutput(i,3) = DD/50 * sin(WW2 * XYZOutput(i,2));
end

end
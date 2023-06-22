function [DATA, CodError] = ReadWindTurbinesFiles(DATA, Indx, Folder, NameWT, CodError)

Name = fullfile (Folder, NameWT);

fid = fopen( [ Name ] );
if fid<0
    CodError.WTFile = 1;
    return
else
    CC = textscan( fid , '%s' ,  'commentStyle' , '%' );
    fclose(fid);
end

%% GENERAL PARAMETERS

DATA(Indx).NumBld       = str2double(CC{1,1}(1,1));
DATA(Indx).PreCone      = str2double(CC{1,1}(2,1));
DATA(Indx).HubRad       = str2double(CC{1,1}(3,1));
DATA(Indx).HubInnerR    = str2double(CC{1,1}(4,1));
DATA(Indx).Tilt         = str2double(CC{1,1}(5,1));
DATA(Indx).TowHeight    = str2double(CC{1,1}(6,1));

%% WIBD TURBINE COMPONENTS

DATA(Indx).Blade    = str2double(CC{1,1}(7,1));
DATA(Indx).Tower    = str2double(CC{1,1}(8,1));
DATA(Indx).Nacelle  = str2double(CC{1,1}(9,1));
DATA(Indx).Hub      = str2double(CC{1,1}(10,1));
DATA(Indx).Ground   = str2double(CC{1,1}(11,1));
DATA(Indx).Monopile = str2double(CC{1,1}(12,1));

%% BLADE PARAMETERS AND DISCRETIZATION (UVLM)

DATA(Indx).NameBld      = string(CC{1,1}(13,1));
DATA(Indx).LBld         = str2double(CC{1,1}(14,1));
DATA(Indx).LSLBLd       = str2double(CC{1,1}(15,1));
DATA(Indx).RLBld        = str2double(CC{1,1}(16,1));
DATA(Indx).NBldC        = str2double(CC{1,1}(17,1));
DATA(Indx).NBldS        = str2double(CC{1,1}(18,1));
DATA(Indx).ShedBld      = str2double(CC{1,1}(19,1));
DATA(Indx).GAPBld       = str2double(CC{1,1}(20,1));
DATA(Indx).aBld         = str2double(CC{1,1}(21,1));
DATA(Indx).bBld         = str2double(CC{1,1}(22,1));
DATA(Indx).r1Bld        = str2double(CC{1,1}(23,1));
DATA(Indx).r2Bld        = str2double(CC{1,1}(24,1));
DATA(Indx).X0Bld        = str2double(CC{1,1}(25,1));
DATA(Indx).QGaussBld    = str2double(CC{1,1}(26,1));
DATA(Indx).DXBld        = str2double(CC{1,1}(27,1));
DATA(Indx).Op1Bld       = str2double(CC{1,1}(28,1));
DATA(Indx).Op2Bld       = str2double(CC{1,1}(29,1));

%% HUB PARAMETERS AND DISCRETIZATION (UVLM)

DATA(Indx).LCylHub      = str2double(CC{1,1}(30,1));
DATA(Indx).LNoseHub     = str2double(CC{1,1}(31,1));
DATA(Indx).TrimNoseHub  = str2double(CC{1,1}(32,1));
DATA(Indx).RBladeHub    = str2double(CC{1,1}(33,1));
DATA(Indx).ShapeNosHub  = str2double(CC{1,1}(34,1));
DATA(Indx).NCircHub     = str2double(CC{1,1}(35,1));
DATA(Indx).NRadHub      = str2double(CC{1,1}(36,1));
DATA(Indx).NZcoupHub    = str2double(CC{1,1}(37,1));
DATA(Indx).NNoseHub     = str2double(CC{1,1}(38,1));

DATA(Indx).DBladeHub    = 2 * DATA(Indx).RBladeHub;

%% NACELLE PARAMETERS AND DISCRETIZATION (UVLM)

DATA(Indx).RadNac       = str2double(CC{1,1}(39,1));
DATA(Indx).RTailNac     = str2double(CC{1,1}(40,1));
DATA(Indx).LCylNac      = str2double(CC{1,1}(41,1));
DATA(Indx).LTailNac     = str2double(CC{1,1}(42,1));
DATA(Indx).LConTNac     = str2double(CC{1,1}(43,1));
DATA(Indx).RConTNac     = str2double(CC{1,1}(44,1));
DATA(Indx).ShapeTailNac = str2double(CC{1,1}(45,1));
DATA(Indx).NCircNac     = str2double(CC{1,1}(46,1));
DATA(Indx).NRadNac      = str2double(CC{1,1}(47,1));
DATA(Indx).NCircCylNac  = str2double(CC{1,1}(48,1));
DATA(Indx).NTailNac     = str2double(CC{1,1}(49,1));
DATA(Indx).NZCoupNac    = str2double(CC{1,1}(50,1));

%% TOWER PARAMETERS AND DISCRETIZATION (UVLM)

DATA(Indx).RConTTow     = str2double(CC{1,1}(51,1));
DATA(Indx).RGroundTow   = str2double(CC{1,1}(52,1));
DATA(Indx).NZTow        = str2double(CC{1,1}(53,1));
DATA(Indx).NCircTow     = str2double(CC{1,1}(54,1));

%% GROUND PARAMETERS AND DISCRETIZATION (UVLM)

DATA(Indx).RTowGround   = str2double(CC{1,1}(55,1));
DATA(Indx).LGround      = str2double(CC{1,1}(56,1));
DATA(Indx).NCircGround  = str2double(CC{1,1}(57,1));
DATA(Indx).NRadGround   = str2double(CC{1,1}(58,1));

%% MONOPILE PARAMETERS AND DISCRETIZATION (UVLM)

DATA(Indx).RWaterMon    = str2double(CC{1,1}(59,1));
DATA(Indx).RDeepMon     = str2double(CC{1,1}(60,1));
DATA(Indx).LMon         = str2double(CC{1,1}(61,1));
DATA(Indx).NCircMon     = str2double(CC{1,1}(62,1));
DATA(Indx).NZMon        = str2double(CC{1,1}(63,1));

%% ASSEMBLING PARAMETERS

DATA(Indx).Yaw0WT       = str2double(CC{1,1}(64,1));
DATA(Indx).Rot0WT       = str2double(CC{1,1}(65,1));
DATA(Indx).Pitch0WT     = str2double ( split (string(CC{1,1}(66,1)), ",") );

%% KINEMATIC PARAMETERS

DATA(Indx).NameRot      = string(CC{1,1}(67,1));
DATA(Indx).NameYaw      = string(CC{1,1}(68,1));
DATA(Indx).NamePitch    = string(CC{1,1}(69,1));
DATA(Indx).UVLMLC       = str2double(CC{1,1}(70,1));
DATA(Indx).UVLMVC       = str2double(CC{1,1}(71,1));
DATA(Indx).UVLMSteps    = str2double(CC{1,1}(72,1));



end
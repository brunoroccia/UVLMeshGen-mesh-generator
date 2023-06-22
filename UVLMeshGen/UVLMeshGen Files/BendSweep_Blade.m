function [DEFORM, LBlade, CodError] = BendSweep_Blade (DATA, BLADE, Zcoord, CodError, Indx)

% Data coming from the Datasheet of the blade and the wind turbine

Gamma1 = DATA(Indx).aBld;
Gamma2 = DATA(Indx).bBld;
Tip1   = DATA(Indx).r1Bld;
Tip2   = DATA(Indx).r2Bld;
ZStart = DATA(Indx).X0Bld * Zcoord(end);
NGauss = DATA(Indx).QGaussBld;
Inc    = DATA(Indx).DXBld;

ZZ     = Zcoord(1):Inc:Zcoord(end);
N      = length(ZZ);

%% Computation of Pre-bend and sweep according to input options

if (DATA(Indx).Op1Bld == 0 && DATA(Indx).Op2Bld == 0)
    
    Deflec_1(1:N)  = 0.0;
    Deflec_2(1:N)  = 0.0;
    
    DEFORM.Deflec_1(1:length(Zcoord)) = 0.0;
    DEFORM.Deflec_2(1:length(Zcoord)) = 0.0;
    
    DEFORM.Label1 = 'Disabled';
    DEFORM.Label2 = 'Disabled';
    
elseif (DATA(Indx).Op1Bld == 0 && DATA(Indx).Op2Bld == 1)
    
    Deflec_1(1:N)  = 0.0;
    Deflec_2       = ppval (BLADE(Indx).SSweep, ZZ);
    
    DEFORM.Deflec_1(1:length(Zcoord)) = 0.0;
    DEFORM.Deflec_2 = ppval (BLADE(Indx).SSweep, Zcoord);
    
    DEFORM.Label1 = 'Disabled';
    DEFORM.Label2 = 'read from DATASheet';
    
elseif (DATA(Indx).Op1Bld == 0 && DATA(Indx).Op2Bld == 2)
    
    Deflec_1(1:N)  = 0.0;
    Deflec_2(1:N)  = 0.0;
    
    for i = 1:N
        if ZZ(i)<=ZStart
            Deflec_2(i) =0.0;
        else            
            Deflec_2(i) = Tip2 * ((ZZ(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma2);
        end        
    end
    
    for i = 1:length(Zcoord)
        if Zcoord(i)<=ZStart
            DEFORM.Deflec_2(i) =0.0;
        else            
            DEFORM.Deflec_2(i) = Tip2 * ((Zcoord(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma2);
        end        
    end
    
    DEFORM.Deflec_1(1:length(Zcoord)) = 0.0;  
    
    DEFORM.Label1 = 'Disabled';
    DEFORM.Label2 = 'computed from Zutecks formula';    
    
elseif (DATA(Indx).Op1Bld == 1 && DATA(Indx).Op2Bld == 0)  
    
    Deflec_1       = ppval (BLADE(Indx).SPrebend, ZZ);
    Deflec_2(1:N)  = 0.0;
    
    DEFORM.Deflec_1  = ppval (BLADE(Indx).SPrebend, Zcoord);
    DEFORM.Deflec_2(1:length(Zcoord)) = 0.0;  
    
    DEFORM.Label1 = 'read from DATASheet';
    DEFORM.Label2 = 'Disabled';    
    
elseif (DATA.Op1Bld == 1 && DATA.Op2Bld == 1)  
    
    Deflec_1       = ppval (BLADE(Indx).SPrebend, ZZ);
    Deflec_2       = ppval (BLADE(Indx).SSweep, ZZ);
    
    DEFORM.Deflec_1  = ppval (BLADE(Indx).SPrebend, Zcoord);
    DEFORM.Deflec_2  = ppval (BLADE(Indx).SSweep, Zcoord); 
    
    DEFORM.Label1 = 'read from DATASheet';
    DEFORM.Label2 = 'read from DATASheet';     
    
elseif (DATA(Indx).Op1Bld == 1 && DATA(Indx).Op2Bld == 2)    
    
    Deflec_1       = ppval (BLADE(Indx).SPrebend, ZZ);
    Deflec_2(1:N)  = 0.0;

    for i = 1:N
        if ZZ(i)<=ZStart
            Deflec_2(i) =0.0;
        else            
            Deflec_2(i) = Tip2 * ((ZZ(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma2);
        end        
    end
    
    for i = 1:length(Zcoord)
        if Zcoord(i)<=ZStart
            DEFORM.Deflec_2(i) =0.0;
        else            
            DEFORM.Deflec_2(i) = Tip2 * ((Zcoord(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma2);
        end        
    end    

    DEFORM.Deflec_1  = ppval (BLADE(Indx).SPrebend, Zcoord);  
    
    DEFORM.Label1 = 'read from DATASheet';
    DEFORM.Label2 = 'computed from Zutecks formula';    
    
elseif (DATA(Indx).Op1Bld == 2 && DATA(Indx).Op2Bld == 0)   
    
    Deflec_1(1:N)  = 0.0;

    for i = 1:N
        if ZZ(i)<=ZStart
            Deflec_1(i) =0.0;
        else            
            Deflec_1(i) = Tip1 * ((ZZ(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma1);
        end        
    end 
    
    Deflec_2(1:N)  = 0.0;
    
    for i = 1:length(Zcoord)
        if Zcoord(i)<=ZStart
            DEFORM.Deflec_1(i) =0.0;
        else            
            DEFORM.Deflec_1(i) = Tip1 * ((Zcoord(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma1);
        end        
    end    

    DEFORM.Deflec_2(1:length(Zcoord)) = 0.0;  
    
    DEFORM.Label1 = 'computed from Zutecks formula';
    DEFORM.Label2 = 'Disabled';     
    
elseif (DATA(Indx).Op1Bld == 2 && DATA(Indx).Op2Bld == 1)
    
    Deflec_1(1:N)  = 0.0;

    for i = 1:N
        if ZZ(i)<=ZStart
            Deflec_1(i) =0.0;
        else            
            Deflec_1(i) = Tip1 * ((ZZ(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma1);
        end        
    end 
    
    Deflec_2       = ppval (BLADE(Indx).SSweep, ZZ);    
    
    for i = 1:length(Zcoord)
        if Zcoord(i)<=ZStart
            DEFORM.Deflec_1(i) =0.0;
        else            
            DEFORM.Deflec_1(i) = Tip1 * ((Zcoord(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma1);
        end        
    end    
   
    DEFORM.Deflec_2  = ppval (BLADE(Indx).SSweep, Zcoord);  
    
    DEFORM.Label1 = 'computed from Zutecks formula';
    DEFORM.Label2 = 'read from DATASheet';    
    
elseif (DATA(Indx).Op1Bld == 2 && DATA(Indx).Op2Bld == 2) 
    
    Deflec_1(1:N)  = 0.0;

    for i = 1:N
        if ZZ(i)<=ZStart
            Deflec_1(i) =0.0;
        else            
            Deflec_1(i) = Tip1 * ((ZZ(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma1);
        end        
    end  
    
    for i = 1:length(Zcoord)
        if Zcoord(i)<=ZStart
            DEFORM.Deflec_1(i) =0.0;
        else            
            DEFORM.Deflec_1(i) = Tip1 * ((Zcoord(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma1);
        end        
    end    

    Deflec_2(1:N)  = 0.0;
    
    for i = 1:N
        if ZZ(i)<=ZStart
            Deflec_2(i) =0.0;
        else            
            Deflec_2(i) = Tip2 * ((ZZ(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma2);
        end        
    end
    
    for i = 1:length(Zcoord)
        if Zcoord(i)<=ZStart
            DEFORM.Deflec_2(i) =0.0;
        else            
            DEFORM.Deflec_2(i) = Tip2 * ((Zcoord(i) - ZStart) / (ZZ(end) - ZStart) )^(Gamma2);
        end        
    end
    
    DEFORM.Label1 = 'computed from Zutecks formula';
    DEFORM.Label2 = 'computed from Zutecks formula';
    
end

%% Computation of the numerical derivative

Phi_1 = NumericalDifferentiation (Deflec_1, Inc);
Phi_2 = NumericalDifferentiation (Deflec_2, Inc);

SPhi_1 = spline (ZZ, Phi_1);
SPhi_2 = spline (ZZ, Phi_2);

%% Computation of axial displacement & new Z coordinate

uu(1:length(Zcoord))     = 0.0;
RC(1:length(Zcoord))     = 0.0;
Theta1(1:length(Zcoord)) = 0.0;
Theta2(1:length(Zcoord)) = 0.0;

for i = 1:length(Zcoord)
            
    uu(i)  = NumericalIntegration (SPhi_1, SPhi_2, 0, Zcoord(i), NGauss);
    RC(i) = Zcoord(i) - uu(i);
    
    Theta1(i) = ppval(SPhi_1, Zcoord(i));
    Theta2(i) = ppval(SPhi_2, Zcoord(i));
    
end

DEFORM.Theta1 = Theta1;
DEFORM.Theta2 = Theta2;
DEFORM.RC     = RC;
DEFORM.uu     = uu;

if ~isreal(DEFORM.RC)
    CodError.BldBend = 1;
    LBlade = [];
    return
end

%% Satinity procedure (LBlade_OLD = LBlade_NEW checking)

LBlade  = NumericalIntegration_1 (SPhi_1, SPhi_2, 0, RC(end), NGauss);

end


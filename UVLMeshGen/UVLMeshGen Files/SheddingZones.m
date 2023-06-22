function [Nod_Shed, Panel_Shed, NNShed, NPShed] = SheddingZones (DATA, LSNN, Indx)

NNShed       = LSNN - DATA(Indx).GAPBld;
NPShed       = LSNN - DATA(Indx).GAPBld - 1;
Nod_Shed     = zeros(1,NNShed);
Panel_Shed   = zeros(1,NPShed);

switch DATA(Indx).ShedBld
    
    case 1
        
        k = 1;
        
        for i = 1 : NNShed
            
            Nod_Shed(k) = DATA(Indx).NBldC*(i + DATA(Indx).GAPBld - 1) + DATA(Indx).NBldC;
            k           = k + 1;
            
        end
        
        k = 1;
        
        for i = 1:NPShed
            
            Panel_Shed(k) = (DATA(Indx).NBldC-1)*(i + DATA(Indx).GAPBld - 1) + (DATA(Indx).NBldC-1);
            k             = k + 1;
            
        end        
        
    case 2
        
        NNShed       = LSNN - DATA(Indx).GAPBld + (DATA(Indx).NBldC - 1);  
        NPShed       = LSNN - DATA(Indx).GAPBld - 1 + (DATA(Indx).NBldC - 1);
        
        for i = 1 : LSNN - DATA(Indx).GAPBld
            
            Nod_Shed(i) = DATA(Indx).NBldC*(i + DATA(Indx).GAPBld - 1) + DATA(Indx).NBldC;
            
        end
        
        Zone2 = (LSNN * DATA(Indx).NBldC) - 1:-1:LSNN * DATA(Indx).NBldC - (DATA(Indx).NBldC-1);        
        Nod_Shed = [Nod_Shed, Zone2];
        
        for i = 1:LSNN - DATA(Indx).GAPBld - 1
            
            Panel_Shed(i) = (DATA(Indx).NBldC-1)*(i + DATA(Indx).GAPBld - 1) + (DATA(Indx).NBldC-1);
            
        end 
        
        Zone2 = (LSNN-1) * (DATA(Indx).NBldC-1):-1:(LSNN-1) * (DATA(Indx).NBldC-1) - (DATA(Indx).NBldC-2);        
        Panel_Shed = [Panel_Shed, Zone2];
        
    otherwise
        
        fprintf('Warning: Shedding zones have not been defined\n')
        fprintf('Please, enter valid flags 1 or 2')
        Nod_Shed = [];
        Panel_Shed = [];
        
end

end


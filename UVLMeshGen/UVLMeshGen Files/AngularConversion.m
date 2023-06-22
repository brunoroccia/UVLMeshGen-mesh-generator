function [Ang] = AngularConversion (COORD)

N = size(COORD,1);

FLAG = 1;

for i = 1:N
    
    X = COORD(i,1);
    Y = COORD(i,2);
    
    if (X>=0) && (Y>=0)
        
        Ang(i) = acos (X / sqrt(X^2 + Y^2));     
        
    elseif (X<0) && (Y>=0)
        
        Ang(i) = (pi/2 - asin (Y / sqrt(X^2 + Y^2))) + pi/2;
        
    elseif (X<0) && (Y<0)
        
        Ang(i) = atan(Y/X) + pi;
        
    elseif (X>=0) && (Y<0)
        
        Ang(i) = (pi/2 - acos(X / sqrt(X^2 + Y^2))) + 3/2*pi;
        
    end               
    
end


end


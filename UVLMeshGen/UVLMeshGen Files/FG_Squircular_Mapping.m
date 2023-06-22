function [Y, Z] = FG_Squircular_Mapping (a, b, ss, Alpha, Type)

% Type: Squircular mapping type

if ss == 0
    
    for i = 1:length(Alpha)
    
    Y(i) = a * cos(Alpha(i));
    Z(i) = b * sin(Alpha(i));
    
    end
    
else
    
    for i = 1:length(Alpha)
        
        if Alpha(i)==0 || Alpha(i)==2*pi
            
            Y(i) = a;
            
        elseif Alpha(i)==pi
            
            Y(i) = -a;
            
        else
            
            Y(i) = a*sign(cos(Alpha(i)))/(ss*sqrt(2)*abs(sin(Alpha(i)))) * sqrt (1 - sqrt(1 - ss^2*sin(2*Alpha(i))^2));
            
        end
        
        if Alpha(i)==pi/2
            
            Z(i) = b;
            
        elseif Alpha(i)==3/2*pi
            
            Z(i) = -b;
            
        else
            
            Z(i) = b*sign(sin(Alpha(i)))/(ss*sqrt(2)*abs(cos(Alpha(i)))) * sqrt (1 - sqrt(1 - ss^2*sin(2*Alpha(i))^2));
            
        end
        
        
    end
    
end

end


function DY = NumericalDifferentiation (Y, DX)

for i = 1:length(Y)        
    
   if i == 1
       
       F1 = Y(i);
       F2 = Y(i+1);
       F3 = Y(i+2);
       
       DY(i) = (-3*F1 +4*F2 - F3) / (2*DX);
       
   elseif i == length(Y)
       
       F1 = Y(i);
       F2 = Y(i-1);
       F3 = Y(i-2);
       
       DY(i) = (3*F1 - 4*F2 + F3) / (2*DX);  
       
   else
       
       F1 = Y(i-1);
       F2 = Y(i+1);
       
       DY(i) = (F2 - F1) / (2*DX);
       
   end
  
end

end


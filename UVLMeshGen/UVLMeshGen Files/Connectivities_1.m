function ICON = Connectivities_1 (NNSpan, NNChord)

ICON = zeros( ( NNSpan - 1 ) * ( NNChord - 1 ), 4 );

k = 1;

for i = 1 : NNSpan-1
    
    for j = 1 : NNChord-1
        
        ICON (k,1) = j + (i - 1) * NNChord;
        ICON (k,2) = (j + 1) + (i - 1) * NNChord;
        ICON (k,3) = j + NNChord * i + 1;
        ICON (k,4) = j + NNChord * i;
        
        k = k + 1;
        
    end
    
end


end


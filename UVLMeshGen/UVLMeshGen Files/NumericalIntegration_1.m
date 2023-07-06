function [Int] = NumericalIntegration_1 (SPhi_1, SPhi_2, a, b, NGauss)

A = (a+b)/2;
B = (b-a)/2;
h = B;

switch NGauss

    case 2
        
        W(1) = 1.0;
        W(2) = 1.0;
        
        s(1) = -0.5773502691896257;
        s(2) =  0.5773502691896257;
        
    case 3
        
        W(1) = 0.8888888888888888;
        W(2) = 0.5555555555555556;
        W(3) = 0.5555555555555556; 
        
        s(1) = 0.0;
        s(2) = -0.7745966692414834;
        s(3) = 0.7745966692414834; 
        
    case 4        
        
        W(1) = 0.347854845137454;
        W(2) = 0.347854845137454;
        W(3) = 0.652145154862546;
        W(4) = 0.652145154862546;   
        
        s(1) = 0.861136311594053;
        s(2) = -0.861136311594053;
        s(3) = 0.339981043584856;
        s(4) = -0.339981043584856;

    case 5
        
        W(1) = 0.5688888888888889;
        W(2) = 0.4786286704993665;
        W(3) = 0.4786286704993665;
        W(4) = 0.2369268850561891;
        W(5) = 0.2369268850561891;
        
        s(1) =  0.0;
        s(2) = -0.5384693101056831;
        s(3) =  0.5384693101056831;
        s(4) = -0.9061798459386640;
        s(5) =  0.9061798459386640;        

    case 6
        
        W(1) = 0.171324492379170;
        W(2) = 0.171324492379170;
        W(3) = 0.360761573048139;
        W(4) = 0.360761573048139;
        W(5) = 0.467913934572691;
        W(6) = 0.467913934572691; 
        
        s(1) =  0.932469514203152;
        s(2) = -0.932469514203152;
        s(3) =  0.661209386466265;
        s(4) = -0.661209386466265;
        s(5) =  0.238619186083197;
        s(6) = -0.238619186083197;

end 

Int = 0.0;

for i = 1:NGauss
    
    SS  = A + B*s(i);
    
    Int = Int + W(i)*(sqrt(1 + ppval(SPhi_1, SS)^2 + ...
          ppval(SPhi_2, SS)^2) )*h;
    
end


end

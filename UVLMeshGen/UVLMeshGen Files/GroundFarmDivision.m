function [GROUND_AERO] = GroundFarmDivision (BoxXY, WTXY, Grd_DB, Grd_NodR, Grd_NodC)

%% Square coordinates (From circle to square) / FG-Squircular mapping

% x = a sgn(cost)/(s sqrt(2) abs(sint)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))
% y = a sgn(sint)/(s sqrt(2) abs(cost)) * sqrt (1 - sqrt(1 - s^2 sin(2t)^2))

% where s is a parameter which is used to blend the circle and the square smoothly
% Reference: Analytical Methods for Squaring the Disc (Chamberlain Fong)

Theta = linspace(0, 2*pi, Grd_NodC);                                       % Circunferential division along the hub-blade hole

XX = Grd_DB/2 * cos(Theta);                                                  % X-coordinate along the hub-blade hole
YY = Grd_DB/2 * sin(Theta);                                                  % Y-coordinate along the hub-blade hole

DAng = linspace (0, pi, Grd_NodR);                          % Incremental angle used to make a radial division (sine-shape) from ellipse to square

a1 = Grd_DB/2 + ((BoxXY(2)-WTXY(1)) - Grd_DB/2)*(1 + cos(DAng))/2;
a2 = Grd_DB/2 + ((WTXY(1)-BoxXY(1)) - Grd_DB/2)*(1 + cos(DAng))/2;
b1 = Grd_DB/2 + ((BoxXY(4)-WTXY(2)) - Grd_DB/2)*(1 + cos(DAng))/2;
b2 = Grd_DB/2 + ((WTXY(2)-BoxXY(3)) - Grd_DB/2)*(1 + cos(DAng))/2;

%a1 = Grd_DB/2 + (L1/2 - Grd_DB/2)*(1 + cos(DAng))/2;                          % Major semi-axis for different radial coordinates
%b1 = Grd_DB/2 + (L2/2 - Grd_DB/2)*(1 + cos(DAng))/2;                       % Minor semi-axis for different radial coordinates

a1 = sort(a1, 'ascend');
a2 = sort(a2, 'ascend');
b1 = sort(b1, 'ascend');
b2 = sort(b2, 'ascend');

ss1 = linspace (0, pi/2, Grd_NodR);                                        

ss1 = sin(ss1);                                                            % incremental FG-squircicurlar parameter (s = 0: circle/ellipse // s = 1: square)

for k = 1:Grd_NodR
    
    [AUX_X, AUX_Y] = FG_Squircular_Modified (a1(k), b1(k), a2(k), b2(k), ss1(k), Theta);
    
    XX2(k,:) = AUX_X;
    YY2(k,:) = AUX_Y;
    
end

%% Patch coordinates on the Ground

NN = length(Theta);

COORD(1:NN,1) = transpose(XX);
COORD(1:NN,2) = transpose(YY);
COORD(1:NN,3) = 0;

for i = 1:Grd_NodR - 4                                                      % Hub_NodR - 4 is to eliminate the second and second-to-last radial divisions
    
    I1 = NN*(i) + 1;
    I2 = NN*(i+1);
    
    COORD(I1:I2,1) = transpose(XX2(i+2,:));
    COORD(I1:I2,2) = transpose(YY2(i+2,:));
    COORD(I1:I2,3) = 0;
    
end

I1 = NN*(i+1) + 1;
I2 = NN*(i+2);

COORD(I1:I2,1) = transpose(XX2(end,:));
COORD(I1:I2,2) = transpose(YY2(end,:));
COORD(I1:I2,3) = 0;

%--------------------------------------------------------------------------
%                    Ground connectivities (ICON)
%--------------------------------------------------------------------------

ICON_Ground = Connectivities_1 (Grd_NodR-2, NN);

%%

GROUND_AERO.XYZGround  = COORD;

GROUND_AERO.ICONGround = ICON_Ground;

GROUND_AERO.NNGround   = size(COORD,1);
GROUND_AERO.NPGround   = size(ICON_Ground,1);
    
GROUND_AERO.NNGroundR  = Grd_NodR-2;
GROUND_AERO.NNGroundC  = NN;

    
end

%%

function [Y, Z] = FG_Squircular_Modified (a1, b1, a2, b2, ss, Alpha)

% Type: Squircular mapping type

if ss == 0
    for i = 1:length(Alpha)
        if Alpha(i)>=0 && Alpha(i)<pi/2
            Y(i) = a1 * cos(Alpha(i));
            Z(i) = b1 * sin(Alpha(i));
        elseif Alpha(i)>=pi/2 && Alpha(i)<pi
            Y(i) = a2 * cos(Alpha(i));
            Z(i) = b1 * sin(Alpha(i));
        elseif Alpha(i)>=pi && Alpha(i)<3*pi/2
            Y(i) = a2 * cos(Alpha(i));
            Z(i) = b2 * sin(Alpha(i));
        else
            Y(i) = a1 * cos(Alpha(i));
            Z(i) = b2 * sin(Alpha(i));
        end
    end    
else
    
    for i = 1:length(Alpha)
        
        if Alpha(i)==0 || Alpha(i)==2*pi
            
            Y(i) = a1;
            
        elseif Alpha(i)==pi
            
            Y(i) = -a2;
            
        else
            if (Alpha(i)>0 && Alpha(i)<=pi/2) || (Alpha(i)>=3*pi/2 && Alpha(i)<2*pi)
                Y(i) = a1*sign(cos(Alpha(i)))/(ss*sqrt(2)*abs(sin(Alpha(i)))) * sqrt (1 - sqrt(1 - ss^2*sin(2*Alpha(i))^2));
            elseif (Alpha(i)>=pi/2 && Alpha(i)<=3*pi/2)
                Y(i) = a2*sign(cos(Alpha(i)))/(ss*sqrt(2)*abs(sin(Alpha(i)))) * sqrt (1 - sqrt(1 - ss^2*sin(2*Alpha(i))^2));
            end
            
        end
        
        if Alpha(i)==pi/2
            
            Z(i) = b1;
            
        elseif Alpha(i)==3/2*pi
            
            Z(i) = -b2;
            
        else
            if (Alpha(i)>=0 && Alpha(i)<=pi)
            Z(i) = b1*sign(sin(Alpha(i)))/(ss*sqrt(2)*abs(cos(Alpha(i)))) * sqrt (1 - sqrt(1 - ss^2*sin(2*Alpha(i))^2));
            else
            Z(i) = b2*sign(sin(Alpha(i)))/(ss*sqrt(2)*abs(cos(Alpha(i)))) * sqrt (1 - sqrt(1 - ss^2*sin(2*Alpha(i))^2));
            end
        end
        
        
    end
    
end

end

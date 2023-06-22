function [ P_extr, P_intr, CodError ] = ReadAirfoils( CodError, Folder, NameA, Thick )

% VERSION 1.0

% Airfoil reafing (with unit chord)
% Thickness ratio adaptation (with respect to the reference chord, not the mean line)

%% Airfoil file reading

Name = fullfile (Folder, strcat(NameA,'.dat'));
Flag1 = exist (Name,'file');
if Flag1 == 0
    CodError.BldAirfoil = 1;
    CodError.AirfoilName = NameA;
    P_extr = [];
    P_intr = [];
    return
else
    fid = fopen( Name );
    c1 = textscan( fid , '%f'    , 1 , 'commentStyle' , '%' );
    c2 = textscan( fid , '%f %f' ,     'commentStyle' , '%' );
    fclose(fid);
end

%% Thickness ratio modification

Thick_0 = c1{1};      % Thickness ratio (definition in the DATA SHEET)
Y       = c2{:,1};    % Airfoil points defined w.r.t the Y chord coordinate 
X       = c2{:,2};    % Y coordinate (thickness)

X = X * ( Thick / Thick_0 );  % Modification of the thickness ratio

%% Elimination of repeated coordinates (First time)

i = 1;

while i < length(Y)
    if( Y(i) == Y(i+1) )
        Y(i) = [];
        X(i) = [];
    else
        i = i + 1;
    end
end

%% Separation of Extrados and Intrados points

% Derivative of Y w.r.t. the vector index indicates whether Y is increasing or
% decreasing as the vector is traversed

DY    = Y(2:length(Y))-Y(1:length(Y)-1);

for i = 1 : length(DY)-1
    AUX = sign( DY(i)*DY(i+1) );
    if ( AUX == -1 )
        
        IF = i;
        break
        
    end
end

Y1 = Y(1:IF);
Y2 = Y(IF+1:length(Y));
X1 = X(1:IF);
X2 = X(IF+1:length(X));

%% Elimination of repeated coordinates (Second time)

i = 1;
while i <= length(Y1)
    ind = find( Y1(i) == Y1 );
    if ( size(ind,1) ~= 1 )
        Y1( ind(2:length(ind)) ) = [];
        X1( ind(2:length(ind)) ) = [];
    end
    i = i + 1;
end
i = 1;
while i <= length(Y2)
    ind = find( Y2(i) == Y2 );
    if ( size(ind,1) ~= 1 )
        Y2( ind(2:length(ind)) ) = [];
        X2( ind(2:length(ind)) ) = [];
    end
    i = i + 1;
end

%% Arrays ordered from min to max

[Y1,nvoOrden] = sort(Y1);
X1 = X1(nvoOrden);
[Y2,nvoOrden] = sort(Y2);
X2 = X2(nvoOrden);

%% Addition of Y = [0 1] and X = [0 0] if they do not exist

if( isempty(find(Y1==0,1)) ), Y1=[0 ; Y1]; X1=[0 ; X1]; end
if( isempty(find(Y1==1,1)) ), Y1=[Y1; 1 ]; X1=[X1; 0 ]; end
if( isempty(find(Y2==0,1)) ), Y2=[0 ; Y2]; X2=[0 ; X2]; end
if( isempty(find(Y2==1,1)) ), Y2=[Y2; 1 ]; X2=[X2; 0 ]; end


%% Spline computation

max1 = max(X1);
max2 = max(X2);

if( max1 > max2 )
    P_extr = spline( Y1 , X1 );    % spline for the extrados
    P_intr = spline( Y2 , X2 );    % spline for the intrados
else
    P_extr = spline( Y2 , X2 );    % spline for the extrados
    P_intr = spline( Y1 , X1 );    % spline for the intrados
end


end


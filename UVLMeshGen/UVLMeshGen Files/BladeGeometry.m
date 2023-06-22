%**************************************************************************
% Function BladeGeometry is called from MainProgram.m
% This function reads all the parameters (Chord, Twist, Bend, Sweep, 
% Thickness ratio, etc) related to the blade and computes the splines to be
% used to interpolate these parameters.
% UNRC - UNC - CONICET (July 07, 2020)
% broccia@ing.unrc.edu.ar
%**************************************************************************

function [BLADE, CodError] = BladeGeometry (BLADE, Folders, Name_Blade, CodError, Indx)

Name = fullfile (Folders{2,1}, Name_Blade);

Flag1 = exist (Name,'file');
if Flag1 == 0
    CodError.BldBlade = 1;
    CodError.BladeName = Name_Blade;
    return
else
    fid = fopen( [ Name ] );
    c = textscan( fid , '%f %s %f %f %f %f %f %f' ,  'commentStyle' , '%' );
    fclose(fid);
end

Zcoord  = c{:,1}; % vector coordinate along the spanwise direction
Airfoil = c{:,2}; % Names of the airfoil files
Beta    = c{:,3}; % Geoemtric twits vector
Chord   = c{:,4}; % Chordwise distribution vector
Thick   = c{:,5}; % vector with thick-to-chord ratios
Offset  = c{:,6}; % vector with distances between leading-edge and twist-axis( offset )
Prebend = c{:,7}; % Pre-bend blade
Sweep   = c{:,8}; % Sweep blade

% Spline computation

Airfoil  = char( Airfoil{ : } );      % characters array
SBeta    = spline( Zcoord, Beta );    % spline for the geometric twist
SChord   = spline( Zcoord, Chord );   % spline for the chord
SThick   = spline( Zcoord, Thick );   % spline for the thick-to-chord ratio
SOffset  = spline( Zcoord, Offset );  % spline for the offset distance
SPrebend = spline( Zcoord, Prebend ); % spline for the pre-bend blade
SSweep   = spline( Zcoord, Sweep );   % spline for the sweep blade

NumSec  = length(Zcoord);

for i = 1:NumSec
   
    AUX     = strtrim( Airfoil(i,:) );
    
    [ P_extr, P_intr, CodError ] = ReadAirfoils( CodError, Folders{1,1}, AUX, Thick(i) );    % Read Airfoils and compute splines for extrados and intrados (w.r.t. a unit chord)
    
    if CodError.BldAirfoil == 1
        return
    end

    PP_Airfoil(i,1:2) = { P_extr , P_intr };               % Store splines in a cell array
    
end

%%

BLADE(Indx).Zcoord     = Zcoord;
BLADE(Indx).Airfoil    = Airfoil;
BLADE(Indx).SBeta      = SBeta;
BLADE(Indx).SChord     = SChord;
BLADE(Indx).SThick     = SThick;
BLADE(Indx).SOffset    = SOffset;
BLADE(Indx).SPrebend   = SPrebend;
BLADE(Indx).SSweep     = SSweep;
BLADE(Indx).PP_Airfoil = PP_Airfoil;

end


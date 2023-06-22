%***********************************************************************************************
% Wind Turbine Mesh generator intended for UVLM aerodynamic simulations
% Developed by Dr. Bruno Roccia
%       UNRC - CONICET, Argentina (2021)
%       Bergen Offshore Wind Centre, University of Bergen, Norway (2022)
% Contact: bruno.roccia@uib.no / broccia@ing.unrc.edu.ar
%***********************************************************************************************

clear all
clc
close all

%***********************************************************************************************
%                                           DATA INPUT
%***********************************************************************************************

% Script file containing geometric parameters to build the aerodynamic mesh
% of all the wind turbine

NumWT = 1;         % Number of wind turbines

EQWT_FLAG = 'OFF';  % Homogeneous wind turbine farm
                   % 'ON' means that all wind turbines are equal (kinematics is also the same)
                   % 'OFF'  means that all wind turbines are different 

WTNames    = {'DataSheet_DTU_10MW.DAT', 0.0, 0.0, 0.0
              'DataSheet_DTU_10MW.DAT',  0.0, 500.0, 0.0
              'DataSheet_Sandia01.DAT',  -500.0, 0.0, 0.0
              'DataSheet_siemens_2.3MW.DAT',  -500.0, 500.0, 0.0
              'DataSheet_IEA_15MW.DAT',  100.0,  100.0, 0.0};       % Array with the names and coordinates XYZ of each wind turbine setting


OutputName = 'DTU_10MW';

GroundDivision = {[-250, 250]
                 [-250, 250, 750]};

Ground_FLAG     = {'ON', 'ON', 'userfunction', 'MyGroundLevel_1', 'GroundData.DAT', 'poly23'};
                % Field 1: ON = Ground ON / OFF = Ground OFF
                % Field 2: ON = Ground level ON / OFF = Ground level OFF
                % Field 3: Options: userfunction or externaldata
                %          userfunction = the ground level is defined
                %          through a user defined function called
                %          "MyGroundLevel_1.m" 
                %          externaldata = the ground heigh is defined
                %          through a list of coordinates given in file
                %          "GroundData.DAT"


Kinematic_FLAG  = {'OFF', 'RotorON', 'YawOFF', 'PitchOFF', 'min'};
                % Field 1: ON = Kinematics ON / OFF = Kinematics OFF
                % Field 2: RotorON = Rotor kinematics ON / RotorOFF = Rotor kinematics off
                % Field 3: YawON = Yaw motion ON / YawOFF = Yaw motion OFF
                % Field 4: PitchON = Pitch motion ON / PitchOFF = Pitch motion OFF
                % Field 5: Options: Average or Max or Min
                %          Average = If several wind turbines are used,
                %          this option computes the simulation time step as
                %          the average of time steps associated with each
                %          wind turbine --> DT = mean ([DT1, DT2,...,DTn])
                %          Max = Uses the larger time step among all time steps
                %          Min = Uses the smaller time step among all time steps

OPT_FLAG    = {'ON','TecplotOutput'};
                % Field 1: ON = Exporting files ON using the script
                % provided in field 2
                % TecplotOutput = script included by default in UVLMeshGen
                % (write a Tecplot file containing the assembled wind farm)
                % TecplotKinematics = script included by default in UVLMeshGen
                % (write a Tecplot file containing the assembled wind farm including its kinematics


%***********************************************************************************************

MainProgram;
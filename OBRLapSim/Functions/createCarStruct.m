function [Car] = createCarStruct(chassisParameters,tyreCoefficients,powertrainParameters,aeroParameters,kinematicsData)
% *************************************************************************
% FUNCTION NAME:
%   createCarStruct
%
% DESCRIPTION:
%   Creates the struct that contains all the information about the car
%   Inputs are xls filesname strings
%
% INPUTS:
%   xls file name strings containing the desired data for the car
%
% OUTPUTS:
%   Car - struct contain all car data
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
% *************************************************************************


Car=struct();

% Create chassis struct
Car.Chassis = struct();
Car.Chassis = addParametersFromFile(Car.Chassis,chassisParameters);
Car.Chassis = calcChassisParameters(Car.Chassis);

% Create tyre struct
Car.Tyre = struct();
Car.Tyre = addParametersFromFile(Car.Tyre,tyreCoefficients);

% Create powertrain struct
Car.Powertrain = struct();
Car.Powertrain = createPowertrain(Car.Powertrain,powertrainParameters);
[Car.Powertrain.torqueVsSpeed,Car.Powertrain.speed,Car.Powertrain.torque] = torqueMapCalc(Car.Powertrain,Car.Chassis.tyreRadius,Car.Powertrain.efficiency);

% Create Aero struct
Car.Aero = struct();
Car.Aero = addParametersFromFile(Car.Aero,aeroParameters);
if isfield(Car.Aero,'aeroMap')
    tempStruct = load(char(Car.Aero.aeroMap));
    Car.Aero.dims = tempStruct.dims;
    Car.Aero.YRegCx = tempStruct.YRegCx;
    Car.Aero.YRegCzf = tempStruct.YRegCzf;
    Car.Aero.YRegCzr = tempStruct.YRegCzr;
    Car.Aero.map = true;
    clear tempStruct
else
    Car.Aero.map = false;
end

%% Rear kinematics

% Load Heave breakpoints
rear_heave_bp = xlsread(kinematicsData,2,'D13:D63') ;

% Load Camber Angle breakpoints
rear_camber_bp = xlsread(kinematicsData,2,'F13:F63') ;

% Load Toe Angle breakpoints
rear_toe_bp = xlsread(kinematicsData,2,'G13:G63') ;

% Fitting polynomial to characteristics

% Fit 2 dimensional poly to data table of heave BP vs. processed camber/toe
Car.Kin.camber_poly_r = polyfit(rear_heave_bp,rear_camber_bp,2) ;
Car.Kin.toe_poly_r = polyfit(rear_heave_bp,rear_toe_bp,2) ;

%Refit polynomial removing static camber and toe
Car.Kin.camber_poly_r = polyfit(rear_heave_bp,rear_camber_bp-polyval(Car.Kin.camber_poly_r,0),2) ;
Car.Kin.toe_poly_r = polyfit(rear_heave_bp,rear_toe_bp-polyval(Car.Kin.toe_poly_r,0),2) ;

% Front Kinematics

% Values are obtained from "standard" simulation in OptimumK, where mostly
% static heave values are sweeped for different steering angles to sample
% whole matrix of heave/steer and toe/camber combinations

% Load Heave breakpoints
front_heave_bp = xlsread(kinematicsData,1,'D13:D113') ;

% Load steering wheel angle breakpoints
front_stw_bp = xlsread(kinematicsData,1,'E13:E113') ;

% Load front left wheel toe angle breakpoints
front_toe_bp = xlsread(kinematicsData,1,'G13:G113') ;

% Load front left wheel camber angle breakpoints
front_camber_bp = xlsread(kinematicsData,1,'H13:H113') ;


FL_camber_bp = front_camber_bp;
FR_camber_bp = front_camber_bp;
FL_toe_bp = front_toe_bp;
FR_toe_bp = front_toe_bp;

% Concatenate independent variables

% to supply correct inputs for polyfitn, both independent variables need to
% be combined into a two column matrix

% as only left data has been imported, sign of steering angle needs to be
% mirrored to fit polynomial for RHS

heave_stw_bpl = [ front_heave_bp front_stw_bp ] ;

heave_stw_bpr = [ front_heave_bp -front_stw_bp ] ;

% Fit polynomials

% 4th order polynomial was chosen
Car.Kin.camber_poly_fl = polyfitn( heave_stw_bpl , FL_camber_bp , 4 ) ;
Car.Kin.camber_poly_fr = polyfitn( heave_stw_bpr , FR_camber_bp , 4 ) ;
Car.Kin.toe_poly_fl = polyfitn( heave_stw_bpl , FL_toe_bp , 4 ) ;
Car.Kin.toe_poly_fr = polyfitn( heave_stw_bpr , FR_toe_bp , 4 ) ;

% Refit to remove static camber and toe
Car.Kin.camber_poly_fl = polyfitn( heave_stw_bpl , FL_camber_bp - polyvaln( Car.Kin.camber_poly_fl , [0,0] ) , 4 ) ;
Car.Kin.camber_poly_fr = polyfitn( heave_stw_bpr , FR_camber_bp - polyvaln( Car.Kin.camber_poly_fr , [0,0] ) , 4 ) ;
Car.Kin.toe_poly_fl = polyfitn( heave_stw_bpl , FL_toe_bp - polyvaln( Car.Kin.toe_poly_fl , [0,0] ) , 4 ) ;
Car.Kin.toe_poly_fr = polyfitn( heave_stw_bpr , FR_toe_bp - polyvaln( Car.Kin.toe_poly_fr , [0,0] ) , 4 ) ;

%% Determine average steered angle per degree of steering

[~, ~, ~, ~, sigmaFL, sigmaFR, ~, ~] = kinematics(Car, 0, 0, 0, 0, 0, Car.Chassis.maxSteering);
Car.Kin.steeringRate = (sigmaFL+Car.Chassis.toeFL+sigmaFR-Car.Chassis.toeFR)/2/Car.Chassis.maxSteering;


end

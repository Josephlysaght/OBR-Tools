function [stateDeriv, accelX, outputData] = vehicleModel(Car, stateVar, speed, curvature, throttleOrBrake)
% *************************************************************************
% FUNCTION NAME:
%   vehicle model
%
% DESCRIPTION:
%   This function contains the vehicle model
%
% INPUTS:
%   Car - car data struct
%   stateVar - vehicle state variables vector
%   speed - (m/s)
%   curvature - track curvature (1/m) 
%   throttleOrBrake - throttle (%) or braking force (N)
%
% OUTPUTS:
%   stateDeriv - state derivatives vector
%   accelX - longitudinal acceleration (m/s^2)
%   outputData - vector of additional data
% 
% KNOW ISSUES:
%
% CHANGE LOG:
%   2018-04-02: Initial revision
%   2018-06-15: Removed geometric weight transfer since jacking forces
%   already account for that effect
%   2018-06-15: Changed Car.Chassis.H to Car.Chassis.SMCG to use susp mass
%   CG instead of total CG height
%   2018-06-15: Added front pitch moment to equation while accelerating
%   2018-06-16: Removed geometric weight transfer from wheel accel equations
% *************************************************************************



%Create struct for tyre information
FLTyre=struct();
FRTyre=struct();
RLTyre=struct();
RRTyre=struct();

% Assign state variables to appropriate variable name
steering = stateVar(1); %Steering wheel angle [deg]
yaw = stateVar(2); %yaw angle [deg]
roll = stateVar(3); %roll angle [deg]
heave = stateVar(4); %CG heave movement [m]
pitch = stateVar(5); %pitch angle [deg]
FLTyreZ = stateVar(6); %Tyre vertical displacement [m]
FRTyreZ = stateVar(7); %Tyre vertical displacement [m]
RLTyreZ = stateVar(8); %Tyre vertical displacement [m]
RRTyreZ = stateVar(9); %Tyre vertical displacement [m]
FLTyre.angVel = stateVar(10); %Wheel angular velocity [rad/s]
FRTyre.angVel = stateVar(11); %Wheel angular velocity [rad/s]
RLTyre.angVel = stateVar(12); %Wheel angular velocity [rad/s]
RRTyre.angVel = stateVar(13); %Wheel angular velocity [rad/s]
FLcamberDeflection = stateVar(14); %Camber deflection due to compliance [deg]
FRcamberDeflection = stateVar(15); %Camber deflection due to compliance [deg]
RLcamberDeflection = stateVar(16); %Camber deflection due to compliance [deg]
RRcamberDeflection = stateVar(17); %Camber deflection due to compliance [deg]

% Calculate ride heights
fRH = Car.Chassis.fRHStatic+heave-Car.Chassis.a*sind(pitch);
rRH = Car.Chassis.rRHStatic+heave+Car.Chassis.a*sind(pitch);

%calculate tyre normal loads
FLTyre.Fz = -FLTyreZ*Car.Chassis.tyreVertStiffness+Car.Chassis.staticFzFL;
if FLTyre.Fz <= 0
    FLTyre.Fz = 0.1;
end
FRTyre.Fz = -FRTyreZ*Car.Chassis.tyreVertStiffness+Car.Chassis.staticFzFR;
if FRTyre.Fz <= 0
    FRTyre.Fz = 0.1;
end
RLTyre.Fz = -RLTyreZ*Car.Chassis.tyreVertStiffness+Car.Chassis.staticFzRL;
if RLTyre.Fz <= 0
    RLTyre.Fz = 0.1;
end
RRTyre.Fz = -RRTyreZ*Car.Chassis.tyreVertStiffness+Car.Chassis.staticFzRR;
if RRTyre.Fz <= 0
    RRTyre.Fz = 0.1;
end

% Determine gear and rpm
gearPos = Car.Powertrain.torqueVsSpeed(find(Car.Powertrain.torqueVsSpeed(:,1)>=((RLTyre.angVel*Car.Chassis.tyreRadius+RRTyre.angVel*Car.Chassis.tyreRadius)/2),1),3);
if isempty(gearPos)
    gearPos = nan();
    engineRPM = nan();
else
    engineRPM = ((RLTyre.angVel+RRTyre.angVel)/2)*60/(2*pi)*Car.Powertrain.gearRatio(gearPos)*Car.Powertrain.finalDriveRatio*Car.Powertrain.primaryRatio;
end

%Determine engine or brake torque at each wheel
if throttleOrBrake > 0
    % Engine Torque
    totalTorque=throttleOrBrake*interp1(Car.Powertrain.torqueVsSpeed(:,1),Car.Powertrain.torqueVsSpeed(:,2),(RLTyre.angVel*Car.Chassis.tyreRadius+RRTyre.angVel*Car.Chassis.tyreRadius)/2);
    FLTyre.torque=0;
    FRTyre.torque=0;
    % Torque split evenly between rear wheels (open diff)
    RLTyre.torque=totalTorque/2;
    RRTyre.torque=totalTorque/2;
elseif throttleOrBrake < 0
    % Braking Torques
    totalTorque=throttleOrBrake;
    FLTyre.torque=totalTorque*Car.Chassis.brakeBias/2;
    FRTyre.torque=totalTorque*Car.Chassis.brakeBias/2;
    RLTyre.torque=totalTorque*(1-Car.Chassis.brakeBias)/2;
    RRTyre.torque=totalTorque*(1-Car.Chassis.brakeBias)/2;
else
    % Coasting (no applied torque)
    FLTyre.torque=0;
    FRTyre.torque=0;
    RLTyre.torque=0;
    RRTyre.torque=0;
end

%Calculate wheel displacement (positive is wheel extension)
FLWheelDisp = (fRH-Car.Chassis.fRHStatic)+Car.Chassis.fTrack/2*sind(roll)-FLTyreZ;
FRWheelDisp = (fRH-Car.Chassis.fRHStatic)-Car.Chassis.fTrack/2*sind(roll)-FRTyreZ;
RLWheelDisp = (rRH-Car.Chassis.rRHStatic)+Car.Chassis.rTrack/2*sind(roll)-RLTyreZ;
RRWheelDisp = (rRH-Car.Chassis.rRHStatic)-Car.Chassis.rTrack/2*sind(roll)-RRTyreZ;

%Calculate suspension force
FLSusForce = -FLWheelDisp*Car.Chassis.fWheelrate;
FRSusForce = -FRWheelDisp*Car.Chassis.fWheelrate;
RLSusForce = -RLWheelDisp*Car.Chassis.rWheelrate;
RRSusForce = -RRWheelDisp*Car.Chassis.rWheelrate;

%Calculate kinematics
[FLTyre.camber, FRTyre.camber, RLTyre.camber, RRTyre.camber, FLTyre.sigma, FRTyre.sigma, RLTyre.sigma, RRTyre.sigma] = kinematics(Car, FLWheelDisp, FRWheelDisp, RLWheelDisp, RRWheelDisp, roll, steering);

%Add camber compliances
FLTyre.camber = FLTyre.camber + FLcamberDeflection;
FRTyre.camber = FRTyre.camber + FRcamberDeflection;
RLTyre.camber = RLTyre.camber + RLcamberDeflection;
RRTyre.camber = RRTyre.camber + RRcamberDeflection;

%get Aero
averageSteeredAngle = (FLTyre.sigma+FRTyre.sigma)/2;
[fCl, rCl, Cd] = aeroCoefficients(Car, speed, yaw, roll, fRH, rRH, averageSteeredAngle);
fAeroLoad=-0.5*Car.Aero.airDensity*Car.Aero.frontalArea*fCl*speed^2;
rAeroLoad=-0.5*Car.Aero.airDensity*Car.Aero.frontalArea*rCl*speed^2;
drag=0.5*Car.Aero.airDensity*Car.Aero.frontalArea*Cd*speed^2;

%Calculate slip angles and ratios
[FLTyre,FRTyre,RLTyre,RRTyre] = calcTyreSlip(Car, speed, curvature, yaw, FLTyre,FRTyre,RLTyre,RRTyre);
    
%Get tire forces
[FLTyre,FRTyre,RLTyre,RRTyre] = getTireForces(Car,FLTyre,FRTyre,RLTyre,RRTyre);

%Calculate camber compliance deflection accelerations
FLcamberAccel = (-FLTyre.Fy*Car.Chassis.tyreRadius-FLcamberDeflection/Car.Chassis.fCamberCompliance)/Car.Chassis.wheelInertia; %Assume inertia of wheel assembly about x axis is similar to y
FRcamberAccel = (FRTyre.Fy*Car.Chassis.tyreRadius-FRcamberDeflection/Car.Chassis.fCamberCompliance)/Car.Chassis.wheelInertia; %Assume inertia of wheel assembly about x axis is similar to y
RLcamberAccel = (-RLTyre.Fy*Car.Chassis.tyreRadius-RLcamberDeflection/Car.Chassis.rCamberCompliance)/Car.Chassis.wheelInertia; %Assume inertia of wheel assembly about x axis is similar to y
RRcamberAccel = (RRTyre.Fy*Car.Chassis.tyreRadius-RRcamberDeflection/Car.Chassis.rCamberCompliance)/Car.Chassis.wheelInertia; %Assume inertia of wheel assembly about x axis is similar to y

%Calculate rolling resistance
FLrollingResistance = FLTyre.Fz*Car.Chassis.rollingResistanceCoefficient;
FRrollingResistance = FRTyre.Fz*Car.Chassis.rollingResistanceCoefficient;
RLrollingResistance = RLTyre.Fz*Car.Chassis.rollingResistanceCoefficient;
RRrollingResistance = RRTyre.Fz*Car.Chassis.rollingResistanceCoefficient;
    
%Calculate longitudinal acceleration
accelX = ((FLTyre.Fx*cosd(FLTyre.sigma)+FRTyre.Fx*cosd(FRTyre.sigma)+RLTyre.Fx*cosd(RLTyre.sigma)+RRTyre.Fx*cosd(RRTyre.sigma)-drag-FLrollingResistance-FRrollingResistance-RLrollingResistance-RRrollingResistance)+(FLTyre.Fy*(-sind(FLTyre.sigma))+FRTyre.Fy*(-sind(FRTyre.sigma))+RLTyre.Fy*(-sind(RLTyre.sigma))+RRTyre.Fy*(-sind(RRTyre.sigma))))*cosd(yaw)/Car.Chassis.totalMass;

%Calculate lateral acceleration
accelY = ((FLTyre.Fy*cosd(FLTyre.sigma)+FRTyre.Fy*cosd(FRTyre.sigma)+RLTyre.Fy*cosd(RLTyre.sigma)+RRTyre.Fy*cosd(RRTyre.sigma))+(FLTyre.Fx*sind(FLTyre.sigma)+FRTyre.Fx*sind(FRTyre.sigma)+RLTyre.Fx*sind(RLTyre.sigma)+RRTyre.Fx*sind(RRTyre.sigma)))*cosd(yaw)/Car.Chassis.totalMass;

%Calculate roll moment
fARBMoment= atand((FLWheelDisp-FRWheelDisp)/Car.Chassis.fTrack)*Car.Chassis.fARB;
rARBMoment= atand((RLWheelDisp-RRWheelDisp)/Car.Chassis.rTrack)*Car.Chassis.rARB;
rollAccel = (Car.Chassis.suspMass*accelY*Car.Chassis.h+FLSusForce*Car.Chassis.centerlineFL-FRSusForce*Car.Chassis.centerlineFR+RLSusForce*Car.Chassis.centerlineRL-RRSusForce*Car.Chassis.centerlineRR-fARBMoment-rARBMoment)/Car.Chassis.rollInertia;

%Calculate roll jacking
FLRollJackingForce = -FLTyre.Fy*sin(Car.Chassis.fRC/(Car.Chassis.fTrack/2));
FRRollJackingForce = FRTyre.Fy*sin(Car.Chassis.fRC/(Car.Chassis.fTrack/2));
RLRollJackingForce = -RLTyre.Fy*sin(Car.Chassis.rRC/(Car.Chassis.rTrack/2));
RRRollJackingForce = RRTyre.Fy*sin(Car.Chassis.rRC/(Car.Chassis.rTrack/2));

%Calculate pitch moment
%Pitch moment from tyre forces depend on whether brake or accel
fPitchJackingForce = -(FLTyre.Fx+FRTyre.Fx)*sind(Car.Chassis.fAntidiveAngle);
if RLTyre.Fx+RRTyre.Fx>0
    pitchMomentSus=-(FLTyre.Fx+FRTyre.Fx)*Car.Chassis.SMCG*(1-tan(Car.Chassis.fAntidiveAngle*pi/180)/(Car.Chassis.SMCG/Car.Chassis.wheelbase))-(RLTyre.Fx+RRTyre.Fx)*Car.Chassis.SMCG*(1-Car.Chassis.antisquat);

    %Calculate rear jacking force using antisquat angle
    rPitchJackingForce = (RLTyre.Fx+RRTyre.Fx)*sind(Car.Chassis.antisquatAngle);
else
    pitchMomentSus=-(FLTyre.Fx+FRTyre.Fx)*Car.Chassis.SMCG*(1-tan(Car.Chassis.fAntidiveAngle*pi/180)/(Car.Chassis.SMCG/Car.Chassis.wheelbase))-(RLTyre.Fx+RRTyre.Fx)*Car.Chassis.SMCG*(1-tan(Car.Chassis.rAntidiveAngle*pi/180)/(Car.Chassis.SMCG/Car.Chassis.wheelbase));
    
    %Calculate rear jacking force using rear antidive angle
    rPitchJackingForce = (RLTyre.Fx+RRTyre.Fx)*sind(Car.Chassis.rAntidiveAngle);
end

rollJackingPitchMoment = -(FLRollJackingForce+FRRollJackingForce)*Car.Chassis.a+(RLRollJackingForce+RRRollJackingForce)*Car.Chassis.b;

pitchMomentAero=fAeroLoad*Car.Chassis.a-rAeroLoad*Car.Chassis.b;
pitchAccel = (pitchMomentSus+pitchMomentAero+rollJackingPitchMoment-(FLSusForce+FRSusForce)*Car.Chassis.a+(RLSusForce+RRSusForce)*Car.Chassis.b)/Car.Chassis.pitchInertia;

%Calculate Yaw acceleration
yawAccel = 1/Car.Chassis.yawInertia*((FLTyre.Fy*cosd(FLTyre.sigma)+FLTyre.Fx*sind(FLTyre.sigma)+FRTyre.Fy*cosd(FRTyre.sigma)+FRTyre.Fx*sind(FRTyre.sigma))*Car.Chassis.a-(RLTyre.Fy*cosd(RLTyre.sigma)+RLTyre.Fx*sind(RLTyre.sigma)+RRTyre.Fy*cosd(RRTyre.sigma)+RRTyre.Fx*sind(RRTyre.sigma))*Car.Chassis.b...
    +(FLTyre.Fy*sind(FLTyre.sigma)-FLTyre.Fx*cosd(FLTyre.sigma))*Car.Chassis.centerlineFL+(-FRTyre.Fy*sind(FRTyre.sigma)+FRTyre.Fx*cosd(FRTyre.sigma))*Car.Chassis.centerlineFR+(RLTyre.Fy*sind(RLTyre.sigma)-RLTyre.Fx*cosd(RLTyre.sigma))*Car.Chassis.centerlineRL+(-RRTyre.Fy*sind(RRTyre.sigma)+RRTyre.Fx*cosd(RRTyre.sigma))*Car.Chassis.centerlineRR);

%Calculate Vertical Acceleration
accelZ = 1/Car.Chassis.suspMass*(FLSusForce+FRSusForce+RLSusForce+RRSusForce+fPitchJackingForce+rPitchJackingForce+FLRollJackingForce+FRRollJackingForce+RLRollJackingForce+RRRollJackingForce-fAeroLoad-rAeroLoad);

%Calculate weight transfer that effects unsprung masses
% Lateral Nonsuspended Mass Weight Transfer
fLatNSMWT = Car.Chassis.fUnsuspMass*2*accelY*Car.Chassis.fUMCG/Car.Chassis.fTrack;
rLatNSMWT = Car.Chassis.rUnsuspMass*2*accelY*Car.Chassis.rUMCG/Car.Chassis.rTrack;
% Longitudinal Nonsuspended Mass Weight Transfer
longNSMWT = (2*Car.Chassis.fUnsuspMass*(-accelX)*Car.Chassis.fUMCG+2*Car.Chassis.rUnsuspMass*(-accelX)*Car.Chassis.rUMCG)/Car.Chassis.wheelbase;

%Calculate wheel vertical accelerations
FLWheelAccelZ = 1/Car.Chassis.fUnsuspMass*(FLTyre.Fz-Car.Chassis.staticFzFL-FLSusForce+(fLatNSMWT)-(longNSMWT)/2-FLRollJackingForce-fPitchJackingForce/2);
FRWheelAccelZ = 1/Car.Chassis.fUnsuspMass*(FRTyre.Fz-Car.Chassis.staticFzFR-FRSusForce-(fLatNSMWT)-(longNSMWT)/2-FRRollJackingForce-fPitchJackingForce/2);
RLWheelAccelZ = 1/Car.Chassis.rUnsuspMass*(RLTyre.Fz-Car.Chassis.staticFzRL-RLSusForce+(rLatNSMWT)+(longNSMWT)/2-RLRollJackingForce-rPitchJackingForce/2);
RRWheelAccelZ = 1/Car.Chassis.rUnsuspMass*(RRTyre.Fz-Car.Chassis.staticFzRR-RRSusForce-(rLatNSMWT)+(longNSMWT)/2-RRRollJackingForce-rPitchJackingForce/2);

%Calculate wheel rotational accelerations
FLWheelRotAccel = (FLTyre.torque - FLTyre.Fx*Car.Chassis.tyreRadius)/Car.Chassis.wheelInertia;
FRWheelRotAccel = (FRTyre.torque - FRTyre.Fx*Car.Chassis.tyreRadius)/Car.Chassis.wheelInertia;
RLWheelRotAccel = (RLTyre.torque - RLTyre.Fx*Car.Chassis.tyreRadius)/Car.Chassis.wheelInertia;
RRWheelRotAccel = (RRTyre.torque - RRTyre.Fx*Car.Chassis.tyreRadius)/Car.Chassis.wheelInertia;

%State derivatives
stateDeriv=zeros(17,1);
stateDeriv(1) = accelY;
stateDeriv(2) = yawAccel;
stateDeriv(3) = accelZ;
stateDeriv(4) = rollAccel;
stateDeriv(5) = pitchAccel;
stateDeriv(6) = FLWheelAccelZ;
stateDeriv(7) = FRWheelAccelZ;
stateDeriv(8) = RLWheelAccelZ;
stateDeriv(9) = RRWheelAccelZ;
stateDeriv(10) = FLWheelRotAccel;
stateDeriv(11) = FRWheelRotAccel;
stateDeriv(12) = RLWheelRotAccel;
stateDeriv(13) = RRWheelRotAccel;
stateDeriv(14) = FLcamberAccel;
stateDeriv(15) = FRcamberAccel;
stateDeriv(16) = RLcamberAccel;
stateDeriv(17) = RRcamberAccel;

%Output useful data
outputData(1) = averageSteeredAngle;
outputData(2) = FLWheelDisp;
outputData(3) = FRWheelDisp;
outputData(4) = RLWheelDisp;
outputData(5) = RRWheelDisp;
outputData(6) = FLTyre.Fz;
outputData(7) = FRTyre.Fz;
outputData(8) = RLTyre.Fz;
outputData(9) = RRTyre.Fz;
outputData(10) = FLTyre.Fy;
outputData(11) = FRTyre.Fy;
outputData(12) = RLTyre.Fy;
outputData(13) = RRTyre.Fy;
outputData(14) = FLTyre.Fx;
outputData(15) = FRTyre.Fx;
outputData(16) = RLTyre.Fx;
outputData(17) = RRTyre.Fx;
outputData(18) = FLTyre.slipAngle;
outputData(19) = FRTyre.slipAngle;
outputData(20) = RLTyre.slipAngle;
outputData(21) = RRTyre.slipAngle;
outputData(22) = FLTyre.slipRatio;
outputData(23) = FRTyre.slipRatio;
outputData(24) = RLTyre.slipRatio;
outputData(25) = RRTyre.slipRatio;
outputData(26) = FLTyre.camber;
outputData(27) = FRTyre.camber;
outputData(28) = RLTyre.camber;
outputData(29) = RRTyre.camber;
outputData(30) = fRH;
outputData(31) = rRH;
outputData(32) = gearPos;
outputData(33) = engineRPM;
outputData(34) = fCl;
outputData(35) = rCl;
outputData(36) = Cd;
outputData(37) = fAeroLoad;
outputData(38) = rAeroLoad;
outputData(39) = drag;
outputData(40) = fCl/(fCl+rCl);

end
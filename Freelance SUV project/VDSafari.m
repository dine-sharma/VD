clearvars;
clc;
close all;

%% Engine Data
EngineData = load('SafariTorque.csv');
EngineRPM = EngineData(:,1);                      % Engine speed in RPM
EngineRadPs = EngineRPM *(pi/30);
EngineTorque = EngineData(:,2);                   % Engine speed in radian per second
EnginePower = (EngineTorque.*EngineRadPs)/1000;    %kW
% Define a second degree polynomial function establishing relationship
% between Engine speed in Radian per second and Engine Torque in Nm
% This relationship is of the form:
% EngineTorque = Equation_1(1)* (EngineRadPs)^2
%              + Equation_1(2)*(EngineRadPs) + Equation_1(3)
%
% We use this relationship to establish different engine torque for
% variable engine RPM values at different speeds

Equation = polyfit(EngineRadPs,EngineTorque,2);

%--------------------------------------------------------------------

%% Vehicle parameters

Wheelbase = 2.741;                         % meters
Height = 0.800;

AirDragCoefficient = 0.45;
FrontalArea = 3.138;                        % m^2
AirDensity = 1.2256;                        % kg/m^3

KerbWeight = 1675;                         % kg
DriverWeight = 70;                         % kg
CarWeight = KerbWeight + DriverWeight;
g = 9.8;

AllowedFrontLoad = 500 * g;                % Allowed load on FL and FR
% tires individually

AllowedRearLoad = 500 * g;                 % Allowed load on RL and RR
% tires individually

FrontWheelRadius = 0.369;                  % meters
RearWheelRadius = 0.369;                   % meters

% Weight Distribution

FrontWeight = 0.64;                        % 64 percent weight is in front
RearWeight = 0.36;                         % 36 perecnt weight is in rear

% Gears

MaximumGears = 6;

%Gear Number and Corresponding ratio

Gears = [1 3.96 ; 2 2.210 ; 3 1.422 ; 4 1 ; 5 0.779 ; 6 0.710];
FinalDrive = 3.75;

MinimumEngineRPM = 1200;
MaximumEngineRPM = 3600;

CoefficientOfFriction = 0.9;
CoefficientOfRollingResistance = 0.035;

%% Vehicle Dynamics
Counter=1;

Velocity = zeros(1,100);
Time = zeros(size(Velocity));
Torque=zeros(size(Velocity));
RPM=zeros(size(Velocity));
ForceLoad=zeros(size(Velocity));
MaxAcc=zeros(size(Velocity));
Acc =zeros(size(Velocity));
Power=zeros(size(Velocity));
G=zeros(size(Velocity));

dt = 0.05;
t = 0;

StaticFrontLoad = 0.64 *CarWeight*g;

% In Absence of ResistanceForces, Friction limited
MaximumTraction =((StaticFrontLoad*CoefficientOfFriction))...
    /(1+(Height*CoefficientOfFriction)/Wheelbase);

ViKph = 0;
ViMps = ViKph/3.6;

EngineRPM = MinimumEngineRPM;

GearNo = Gears(1,1);
GearRatio = Gears(GearNo,2);

while ViKph<=100
    
    AirResistance = (0.5* AirDensity * FrontalArea *...
        AirDragCoefficient*((ViMps)^2));
    RollingResistance = CoefficientOfRollingResistance * CarWeight*g;
    
    ResistanceForce = (AirResistance+RollingResistance);
    
    Fx = MaximumTraction-ResistanceForce;
    MaximumAcceleration=(Fx)/CarWeight;                                     % Theoretical Maximum
    
    EngineRadPs = EngineRPM *(pi/30);
    EngineTorque = Equation(1)*(pi/30)^2 * (EngineRPM)^2 +...
        Equation(2)* (pi/30)* EngineRPM + Equation(3);
    EnginePower = (EngineTorque*EngineRadPs)/1000;
    TotalEffect = GearRatio*FinalDrive;
    
    WheelTorque = EngineTorque * TotalEffect;
    EngineTractionForce = WheelTorque/FrontWheelRadius;
    
    Traction = min(Fx,EngineTractionForce);
    
    LoadTransfer = (Traction)*(Height/Wheelbase);
    
    Acceleration=Traction/CarWeight;
    VfMps = Acceleration*dt+ViMps;
    VfKph = VfMps*3.6;
    
    EngineFinalRPM = ((VfMps/FrontWheelRadius)*TotalEffect)*(30/pi);
    EngineFinalRPM = max(EngineFinalRPM,MinimumEngineRPM);
    
    if EngineFinalRPM > MaximumEngineRPM
        GearNo = GearNo +1;
        if (GearNo > MaximumGears)
            GearNo = MaximumGears;
        end
    end
    
    NewGearRatio = Gears(GearNo,2);
    EngineRPM = EngineFinalRPM* (NewGearRatio/GearRatio);
    
    GearRatio = NewGearRatio;
    
    ViMps = VfMps;
    ViKph = VfKph;
    
    FrontLoad = (StaticFrontLoad-LoadTransfer);
    
    if (FrontLoad/2 > AllowedFrontLoad)
        
        FrontLoad = AllowedFrontLoad*2;
    end
    
    MaximumTraction = (FrontLoad)*CoefficientOfFriction;
    
    t = t+dt;
    
    Velocity(Counter) = VfKph;
    Time(Counter) = t;
    Torque(Counter) = EngineTorque;
    RPM(Counter) = EngineRPM;
    ForceLoad(Counter) = ResistanceForce;
    MaxAcc(Counter) = MaximumAcceleration;
    Power(Counter) = EnginePower;
    G(Counter) = GearNo;
    Acc(Counter) = Acceleration;
    Counter = Counter+1;
end

figure(1)
plot(Time,Velocity);
xlabel("Time");
ylabel("Velocity");

figure(2)
plot(Time,RPM);
xlabel("Time");
ylabel("Engine RPM");

figure(3)
plot(Time,Torque);
xlabel("Time");
ylabel("Torque");

figure(4)
plot(Velocity,Torque);
xlabel("Velocity");
ylabel("Torque");






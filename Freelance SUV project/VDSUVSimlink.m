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

dt = 0.05;
StaticFrontLoad = 0.64 *CarWeight*g;

% Initial Hear
GearNo = Gears(1,1);
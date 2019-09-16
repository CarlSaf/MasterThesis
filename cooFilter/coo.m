tic, format long, clear variables
clc, close all
global lang global_filename trimstart trimend rt rx ry

%% Vechicle parameters
m = mean([1684 1774]);      % Mass [kg]
iz = 4.192e3;

rt = 33e-2;                 % Tyre radius [m]
L = 280.5e-2;               % Wheel base [m]
Ltot = (92+284+112)*1e-2;   % Total car lenght, end to end [m]
cog_x = 0.25;%0.25;         % Center of gravity ratio (from front) [-]
lf = cog_x*L;               % Distance from CoG to front axis [m]
lr = (1-cog_x)*L;           % D-istance from CoG to rear axis [m]
tw = 160e-2;                % Track width [m]
c = tw;

% IMU parameters
rt = 33e-2;                 % Tyre radius [m]
rx = 90e-2;%72/////30e-2;   % Distance from IMU to CoG x-axle (m)
ry = 0;                     % Distance from IMU to CoG y-axle (m)
rz = 60e-2;                 % Distance from IMU to CoG z-axle (m)

dataProcess = 1;
if dataProcess ~= 1
    disp('New vehicle data has not been loaded')
end   
if dataProcess == 1
    global_filename='1b-circle-medium-speed-diam-22,6m-low-mass.csv'; %1a-circle-low-speed-test-1-diam-22,6m.csv
    trimstart = 10000; trimend = 0; %tightcirc30
    DataTreatment
end

% 1a) 235
% 1b) 125
% 1c) 40.5 30.5;
% 2b) 117
% 2d) 107
% lt) 93.5

yawOffsetinit = 0;
lang.yaw = lang.yaw + yawOffsetinit;


vx=5;%8;
vy=0;%0.1;
w=0;

A = [
    0, w,  vy
    -w,   0, -vx
    0,   0,     0];

B = [
    1/m 1/m 0 0;
    0 0 1/m 1/m;
    c/(2*iz) -c/(2*iz) lf/iz -lr/iz];

C = [
    0 0 0;
    0 0 0;
    1 0 0;
    0 0 1];

D = [
    1/m 1/m 0 0;
    0 0 1/m 1/m;
    0 0 0 0;
    0 0 0 0];

Gpss = ss(A,B,C,D);

low = 2e-2;
high = 2e5;


Q = [
    low 0 0 0 0 0 0;
    0 low 0 0 0 0 0;
    0 0 low 0 0 0 0;
    0 0 0 high*5e0 0 0 0;
    0 0 0 0 high*1e-3 0 0;
    0 0 0 0 0 high*5e1 0;
    0 0 0 0 0 0 high*6e0]; 

R = [
    1e4 0 0 0;
    0 1*1e-2 0 0;
    0 0 0.9e-1 0;
    0 0 0 8]*1e-8;
% 
% Q = [
%     low 0 0 0 0 0 0;
%     0 low 0 0 0 0 0;
%     0 0 low 0 0 0 0;
%     0 0 0 high*1e-2 0 0 0;
%     0 0 0 0 high*1.5e-2 0 0;
%     0 0 0 0 0 high*5e1 0;
%     0 0 0 0 0 0 high*6e0]; 
% 
% R = [
%     1e1 0 0 0;
%     0 1*1e1 0 0;
%     0 0 0.9e-1 0;
%     0 0 0 8]*1e-8;

% Q = [
%     low 0 0 0 0 0 0;
%     0 low 0 0 0 0 0;
%     0 0 low 0 0 0 0;
%     0 0 0 high 0 0 0;
%     0 0 0 0 high*1.5e-6 0 0;
%     0 0 0 0 0 high*5e0 0;
%     0 0 0 0 0 0 high*6e0]; 
% 
% R = [
%     1 0 0 0;
%     0 1*1e-3 0 0;
%     0 0 0.9 0;
%     0 0 0 8]*1e-8;

% % Linjärinterpolera (bakåt) accelerationssignaler  


% Q = [
%     low 0 0 0 0 0 0;
%     0 low 0 0 0 0 0;
%     0 0 low 0 0 0 0;
%     0 0 0 high*1e0 0 0 0;
%     0 0 0 0 high*1.5e-6 0 0;
%     0 0 0 0 0 high*5e0 0;
%     0 0 0 0 0 0 high*6e0]; 
% 
% R = [
%     1 0 0 0;
%     0 1*1e-3 0 0;
%     0 0 0.9 0;
%     0 0 0 8]*1e-8;

% Q = [
%     low 0 0 0 0 0 0;
%     0 low 0 0 0 0 0;
%     0 0 low 0 0 0 0;
%     0 0 0 high 0 0 0;
%     0 0 0 0 high*5e-6 0 0;
%     0 0 0 0 0 high 0;
%     0 0 0 0 0 0 high*6e0]; 
% 
% R = [
%     1 0 0 0;
%     0 1*4e2 0 0;
%     0 0 1 0;
%     0 0 0 8]*1e-8;

Ki = lqi(Gpss,Q,R);
disp('LQI construrction complete')

%% RESET

swaThreshold = 5;
yawThreshold = 0.1;
vxThreshold = 1;
nrRange=200;

lang.steering.meanAngle = lang.steering.angle;
lang.yawrateMean = lang.yawrate;
for i=nrRange+1:length(lang.steering.angle)
    lang.steering.meanAngle(i) = mean(lang.steering.angle(i-nrRange:i));
    lang.yawrateMean(i) = mean(lang.yawrate(i-nrRange:i));
end
resetLogic = [lang.steering.meanAngle lang.yawrateMean];

%% GPS treatment
gpsTrim = 400;
lang.g.lat = lang.g.lat(gpsTrim:end);
lang.g.lon = lang.g.lon(gpsTrim:end);
lang.g.h = lang.g.h(gpsTrim:end);

lat0 = lang.g.lat(1);
lon0 = lang.g.lon(1);
h0 = lang.g.h(1);

[xEast,yNorth,zUp] = geodetic2enu(lang.g.lat,lang.g.lon,lang.g.h,lat0,lon0,h0,referenceEllipsoid('GRS 80'));

%% Simulation
TH = 9999;%0.1;

% lang.acceleration.y = lang.acceleration.y + 0.19;

Time = lang.time;
var.signals.values = [lang.acceleration.x lang.acceleration.y lang.wheelspeed.R lang.yawrate lang.yaw resetLogic];

simin = [Time(10:end) var.signals.values(10:end, :)];
disp('inizializing simulation')
sim('cooSim', Time)
disp('Simulation complete'), toc

%%
dxdy(1) = 1e3;
yawOffset = 0.5;
n = 1000;
yawOffsetinit = 22;
lang.yaw = lang.yaw + yawOffsetinit;
totaloff=yawOffsetinit;
figure
for i=2:1:n
    i
    totaloff = totaloff + yawOffset;
    lang.yaw = lang.yaw + yawOffset;
    
    var.signals.values = [lang.acceleration.x lang.acceleration.y lang.wheelspeed.R lang.yawrate lang.yaw resetLogic];
    simin = [Time(10:end) var.signals.values(10:end, :)];
    sim('cooSim', Time)
    
    xw = simOutXY.Data(:,1);
    yw = simOutXY.Data(:,2);

    a = xEast(2e3)-xw(2e3);
    b = yNorth(2e3)-yw(2e3);
    c = sqrt(a^2+b^2)
    dxdy(i) = c;
     
    plot(yNorth(1:2e3), xEast(1:2e3), yw(1:2e3), xw(1:2e3)), axis equal
    legend('ENU Measured', 'Washout Estimated')
    title('Position, Estimation vs Measured')

    if dxdy(i) > dxdy(i-1)
        
        disp('hej')
        totaloff = totaloff - yawOffset;
        lang.yaw = lang.yaw - yawOffset;
        
        var.signals.values = [lang.acceleration.x lang.acceleration.y lang.wheelspeed.R lang.yawrate lang.yaw resetLogic];
        simin = [Time(10:end) var.signals.values(10:end, :)];
        sim('cooSim', Time)
        break
    end
    

    
end

totaloff

vx = simOut.Data(:,1);
vy = simOut.Data(:,2);
w = simOut.Data(:,3);
vyFilt = simOut.Data(:,end);

ax = simOutY.Data(:,1);
ay = simOutY.Data(:,2);
vxx = simOutY.Data(:,3);
wy = simOutY.Data(:,4);

xm = simOutXY.Data(:,1);
ym = simOutXY.Data(:,2);

vyOlin = simOutVy.Data(:,1);
vyLin = simOutVy.Data(:,2);

vyOlinOfilt = simOutVyOfilt.Data(:,1);
vyLinOfilt = simOutVyOfilt.Data(:,2);

%% Plot
%Position, model VS GPS
figure, plot(yNorth, xEast, ym, xm), axis equal
legend('ENU Measured', 'COO Estimated')
title('Position, Estimation vs Measured')
% (gpsTrim:end)
% 
figure, plot(lang.time, [lang.wheelspeed.R, vx])
legend('CAN Measured', 'COO Estimated')
title('Longitudinal velocity, Estimation vs Measured')


% figure, plot(lang.time, [lang.localVel.y, vyLin, vyOlin])
% legend('meassured vy', 'filt Linear vy', 'filt Nonlinear vy')
% grid on
% figure, plot(lang.time, [lang.localVel.y, vyLinOfilt, vyOlinOfilt])
% legend('meassured vy', 'Linear vy', 'Nonlinear vy')
% grid on

figure, plot(lang.time, [lang.localVel.y, vyOlin])
legend('IMU Measured', 'COO Estimated')
title('Lateral velocity, Estimation vs Measured')

% figure, plot(lang.time, [lang.localVel.y, vyOlinOfilt])
% legend('IMU Measured', 'COO Estimated')
% title('Lateral velocity, Estimation vs Measured')

figure, plot(lang.time, [lang.yawrate, w])
legend('IMU Measured', 'COO Estimated')
title('Yawrate, Estimation vs Measured')

figure, plot(lang.time, [lang.acceleration.x, ax])
legend('IMU Measured', 'COO Estimated')
title('Acceleration, Estimation vs Measured')

figure, plot(lang.time, [lang.acceleration.y, ay])
legend('IMU Measured', 'COO Estimated')
title('Acceleration, Estimation vs Measured')

% sqrt((ym(end)-yNorth(end))^2+(xm(end)-xEast(end))^2)
a = xEast-xm(1:length(xEast));
b = yNorth-ym(1:length(xEast));
c = rms(sqrt(a.^2+b.^2))

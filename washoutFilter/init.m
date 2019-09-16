%% init file for washout filter based position estimation
% Mikael Nybacka 2013, Carl Säflund KTH 2019

tic, format long, clear variables
% close all
global lang global_filename trimstart trimend rt rx yawOffset

%% Vechicle parameters

rt = 33e-2;                 % Tyre radius [m]
L = 280.5e-2;               % Wheel base [m]
Ltot = (92+284+112)*1e-2;   % Total car lenght, end to end [m]
cog_x = 0.25;%0.25;         % Center of gravity ratio (from front) [-]
lf = cog_x*L;               % Distance from CoG to front axis [m]
lr = (1-cog_x)*L;           % D-istance from CoG to rear axis [m]
h = 60e-2;                  % Hight from ground to CoG [m]
mass = 1905;                % Mass [kg]
tw = 160e-2;                % Track width [m]

% SWA = mean(lang.steering.angle); %% for ackerman test run
% radius = 22.6/2;
% d = atand(L/(radius));
% Ratio = SWA/d;             % Steering gear ratio
Ratio = 10%12%18.2;%12%14.5;%14.73;

% Tire parameters
Cf = 80000;%90e3%31300;%80000;   %9600           % Lateral stiffness front axle (N/rad) [FREE TO TUNE]
Cr = 90000;%e3%41300;%90000; %9700%            % Lateral stiffness rear axle (N/rad) [FREE TO TUNE]
rollGrad=5*(pi/180);        % Rolling resistance of tyre

% IMU parameters
rx=30e-2;%72/////30e-2;     % Distance from IMU to CoG x-axle (m)
ry=0;                       % Distance from IMU to CoG y-axle (m)
rz=60e-2;                   % Distance from IMU to CoG z-axle (m)

% Misc

% dG = 0.0001:0.05:0.01
% xG = 0.01:0.01:0.1
% figure, surf(xG,dG,xG.*dG);

dG = 0.05;
xG = 0.01;
xC = 0;

T0 = 0.3;
TTh = 0.1%T0/2;

TH = 0.1;
Mu=0.95;                    % Coefficient of friction (Road)
g=9.81;                     % Gravity constantightcirc30t (m/s^2)

dataProcess = 1;
if dataProcess == 1
    global_filename='1b-circle-medium-speed-diam-22,6m-low-mass.csv'; %1a-circle-low-speed-test-1-diam-22,6m.csv
    trimstart = 11e3; trimend = 0; %tightcirc30
    DataTreatment % calls dataloging
end

% yawOffset = 1; %2d;
% yawOffset = 1.235e+02; %1b
% 
% lang.yaw = lang.yaw + yawOffset;

%% GPS treatment and plot
gpsTrim = 400;
lang.g.lat = lang.g.lat(gpsTrim:end);
lang.g.lon = lang.g.lon(gpsTrim:end);
lang.g.h = lang.g.h(gpsTrim:end);

lat0 = lang.g.lat(1);
lon0 = lang.g.lon(1);
h0 = lang.g.h(1);

[xEast,yNorth,zUp] = geodetic2enu(lang.g.lat,lang.g.lon,lang.g.h,lat0,lon0,h0,referenceEllipsoid('GRS 80'));

%% Ay filter design





% lang.acceleration.y = lowpass(lang.acceleration.y, 0.01);

% Wn = 0.5; %Of sample rate
% [butterB, butterA] = butter(1,Wn,'low','s');
% figure, freqs(butterB,butterA,100)
% [pxx,w] = periodogram(lang.acceleration.y - mean(lang.acceleration.y),[],[],400);
% figure, plot(w,10*log10(pxx)), grid on
for i = 1:length(lang.acceleration.y)
    if abs(lang.acceleration.y(i)) < 0.1
        lang.acceleration.y(i) = 0;
    end
end


%% Resetlogik
swaThreshold = 5;
yawThreshold = 0.1;
vxThreshold  = 0.5;
nrRange=200;

if strcmp(global_filename, 'new.csv') == 1
    lang.steering.meanAngle = lang.steering.angle;
    lang.yawrateMean = lang.canYawrate;
    for i=nrRange+1:length(lang.steering.angle)
        lang.steering.meanAngle(i) = mean(lang.steering.angle(i-nrRange:i));
        lang.yawrateMean(i) = mean(lang.canYawrate(i-nrRange:i));
    end
else
    lang.steering.meanAngle = lang.steering.angle;
    lang.yawrateMean = lang.yawrate;
    for i=nrRange+1:length(lang.steering.angle)
        lang.steering.meanAngle(i) = mean(lang.steering.angle(i-nrRange:i));
        lang.yawrateMean(i) = mean(lang.yawrate(i-nrRange:i));
    end
end

%% Simulation
Time = lang.time;
% A=lang.steering.angleTrue;
% B = A(~isnan(A));

% var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.canAcceleration.y lang.canYawrate rad2deg(lang.canRoll) lang.yaw lang.steering.meanAngle lang.yawrateMean];
var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.roll lang.yaw lang.steering.meanAngle lang.yawrateMean];
simin = [Time var.signals.values];
sim('washout', Time)

% Wn = 0.5; %Of sample rate
% [butterB, butterA] = butter(1,Wn,'low','s');
% figure, freqs(butterB,butterA,100)
% [pxx,w] = periodogram(lang.acceleration.y - mean(lang.acceleration.y),[],[],400);
% figure, plot(w,10*log10(pxx)), grid on
xm = xSim.Data(:,1); xi = xSim.Data(:,2); xw = xSim.Data(:,3);
ym = ySim.Data(:,1); yi = ySim.Data(:,2); yw = ySim.Data(:,3);

%% Yaw offset iteration
dxdy(1) = 1e3;

% 1a) 235
% 1b) 125
% 1c) 40.5
% 2b) 117
% 2d) 107
% lt) 93.5


yawOffset = 0.5;
n = 1000;
yawOffsetinit = 123;
lang.yaw = lang.yaw + yawOffsetinit;
totaloff=yawOffsetinit;
figure
for i=2:1:n
    i
    totaloff = totaloff + yawOffset;
    lang.yaw = lang.yaw + yawOffset;
    
    var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.roll lang.yaw lang.steering.meanAngle lang.yawrateMean];
    simin = [Time var.signals.values];
    sim('washout', Time)
    
    xw = xSim.Data(:,3); yw = ySim.Data(:,3);
    
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
        
        var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.roll lang.yaw lang.steering.meanAngle lang.yawrateMean];
        simin = [Time var.signals.values];
        sim('washout', Time)
        break
    end
    

    
end

totaloff


%% Plots
% 
% Meassured position
% figure, subplot(2,1,1)
% plot(yNorth,xEast)
% legend('Raw GPS'), axis equal, title('Position estimations vs meassured')
% % Position estimation
% 
% subplot(2,1,2), axis equal
% plot(ym,xm), hold on, plot(yw,xw), plot(yi,xi)
% legend('modelbased', 'washout filter')
% % 
%% Position, model VS GPS

% figure, title('Position, Estimation vs Measured')
% plot((ym(gpsTrim:end)-ym(gpsTrim)), xm(gpsTrim:end)-xm(gpsTrim), yNorth, xEast, yw(gpsTrim:end)-yw(gpsTrim), xw(gpsTrim:end)-xw(gpsTrim)), axis equal
% legend('Model','Measured ENU', 'Estimated Washout')

figure
plot(yNorth, xEast, yw, xw), axis equal
legend('ENU Measured', 'Washout Estimated')
title('Position, Estimation vs Measured')

% %% Velocity
% figure, plot(lang.localVel.y), hold on
% plot(vSim.Data)
% legend('Messured', 'model', 'integration', 'model + integration')
% title('Velocity estimations vs meassured')

figure, plot(Time, lang.localVel.y), hold on
plot(Time, vSim.Data(:,3))
legend('IMU Measured', 'Washout Estimated')
title('Velocity, Estimation vs Measured')
% 
% % % % % figure, plot(lang.localVel.y), hold on
% % % % % plot(vSim.Data(:,2))
% % % % % legend('Messured', 'Integration')
% % % % % title('Velocity integrate vs meassured')
%
% % figure, plot(lang.localVel.y), hold on
% plot(Time, v
% % plot(lang.wheelspeed.R)
% legend('Messured', 'model', 'washout')

% close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% END OF CODE
disp("simulation succesfully run"), toc
% close all

% c = b./a;
% figure, plot(c)

% rollgrad = t(deg2rad(lang.roll)./(lang.acceleration.y./g),100);
% figure, plot(rollgrad(1:2.4e3));
% rollgrad = mean(rollgrad(1:2.4e3))

a = xEast-xw(1:length(xEast));
b = yNorth-yw(1:length(xEast));
c = rms(sqrt(a.^2+b.^2))
% % % % a = rms(xEast-xw(1:length(xEast)));
% % % % b = rms(yNorth-yw(1:length(xEast)));
% % % % rmsPosError = rms(a+b)
% a = xEast-xw(2e3);
% b = yNorth(2e3)-yw(2e3);
% c = sqrt(a^2+b^2)

%%
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % close all
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % n=3000;
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % c = 1000*ones(n,1);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % figure, title('Position estimatied VS messured')
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % 
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % totaloff=0;
% % % % % % % % % % % % % % % % % % % % % % % % % % clo% % % % Cf=1+totaloff;                  % Lateral stiffness front axle (N/rad) [FREE TO TUNE]
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % Cr=1+totaloff;
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.yaw];
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     simin=[Time var.signals.values];
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     sim('washout', Time)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % for i=2:1:n
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     i
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.yaw];
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     simin=[Time var.signals.values];
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     sim('washout', Time)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     xm = xSim.Data(:,1); ym = ySim.Data(:,1);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     a = rms(xEast-xm(1:ll));
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     b = rms(yNorth-ym(1:ll));
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     c(i) = rms(a+b);
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     c(i)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     if c(i) > c(i-1)
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %         disp('hej')
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %         break
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     end
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     cOffset = 10;
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     totaloff = totaloff + cOffset;
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     Cf=Cf + cOffset;                   % Lateral stiffness front axle (N/rad) [FREE TO TUNE]
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     Cr=Cr + cOffset;       
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     plot(ym, xm, yNorth, xEast), axis equal
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     legend('Model', 'messurement ENU')
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % end




%% Map view
[LAT, LON, H] = enu2geodetic(xm(1:length(zUp)), ym(1:length(zUp)), zUp, lat0,lon0,h0,referenceEllipsoid('GRS 80'));

% wmline(lang.g.lat(400:end),lang.g.lon(400:end));
% wmline(LAT(400:end),LON(400:end))

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % % %
% % % close all
% % % n=3000;
% % % dxdy = 1000*ones(n,1);
% % % figure, title('Position estimatied VS messured')
% % % 
% % % totaloff=3.0886;
% % % lang.yaw = lang.yaw + totaloff;
% % % 
% % %     var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.yaw];
% % %     simin=[Time var.signals.values];
% % %     sim('washout', Time)
% % %     



% % % % close all
% % % % n=3000;
% % % % dxdy = 1000*ones(n,1);
% % % % figure, title('Position estimatied VS messured')
% % % % 
% % % % totaloff=0;
% % % % Cf=20e3;                  % Lateral stiffness front axle (N/rad) [FREE TO TUNE]
% % % % Cr=30e3;
% % % % 
% % % % 
% % % % for i=2:1:n
% % % %     i
% % % %     
% % % %     Time = lang.time;
% % % %     var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.yaw lang.roll];
% % % %     simin=[Time var.signals.values];
% % % %     sim('washout', Time)
% % % % 
% % % %     
% % % %     xm = xSim.Data(:,1); ym = ySim.Data(:,1);
% % % %     
% % % %     dxdy(i) = rms(lang.localVel.y - vSim.Data(:,1))
% % % %     if dxdy(i) > dxdy(i-1)
% % % %         disp('hej')
% % % %         break
% % % %     end
% % % %     
% % % %     cOffset = 100;
% % % %     totaloff = totaloff + cOffset;
% % % %     
% % % %     Cf=Cf + cOffset                   % Lateral stiffness front axle (N/rad) [FREE TO TUNE]
% % % %     Cr=Cr + cOffset;       
% % % %     
% % % %     
% % % %     plot(ym, xm, yNorth, xEast), axis equal
% % % %     legend('Model', 'messurement ENU')
% % % % end


%%
% real(sqrt((asind(lang.acceleration.y(1:2e3)./lang.acceleration.x(1:2e3))).^2))


%% Functions, no code below this!!!!!
% % % Removes 0 and values outside of a determined threshold
function v = t(v,t)
    first = true;
    threshold = t;
    meanV = abs(mean(v));
    for i = 1:length(v)
        if (abs(v(i) - meanV) > threshold)
            if first == true
                v(i) = 0;
%                 v(i) = rec(v)
%                 disp('found a value')
                first = false;
            else
                v(i) = v(i-1);
                
            end
        end
    end
end

% function first = rec(v)
%     for i = 1:length(v)
%         if v(i) == 0
%             rec(v(2:end));
%         elseif v(i) ~= 0
%             first = v(i);
%             break
%         end
%     end
% end
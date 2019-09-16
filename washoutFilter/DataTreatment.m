function DataTreatment
tic, clear variables

% OBS, kordinatsystemet för IMU X = sidan? y=fram?
global lang global_filename trimstart trimend rt rx yawOffset;
trimstart = trimstart + 1;

m = readtable(global_filename);%,'ReadVariableNames',false);
m = m(trimstart:end-trimend,:);

lang.time                       = table2array(m(:,1));
lang.lcmChannel                 = m(:,2);

fprintf('Processing CAN\n');

% CAN
lang.steering.angle             = f(table2array(m(:,3))); %[grader]
lang.steering.angleTrue         = table2array(m(:,3)); %[grader]
lang.steering.vel               = f(table2array(m(:,4))); %[kph]
lang.steering.cmd               = f(table2array(m(:,5))); %[grader]


% [rad/s] samtliga
lang.wheelspeed.FL              = f(table2array(m(:,6))); %[rad/sec?]
lang.wheelspeed.FR              = f(table2array(m(:,7)));
lang.wheelspeed.RL              = f(table2array(m(:,8)));
lang.wheelspeed.RR              = f(table2array(m(:,9)));
lang.wheelspeed.R               = (lang.wheelspeed.RR + lang.wheelspeed.RL)./2.*rt; %[m/s]

fprintf('Processing IMU\n')

% IMU
lang.roll                       = t(f(table2array(m(:,10))),999999999999);
lang.pitch                      = f(table2array(m(:,11)));
lang.yaw                        = f(table2array(m(:,12)));

lang.acceleration.x             = f(table2array(m(:,13))); % x initially
lang.acceleration.y             = f(table2array(m(:,14))); % y initially
lang.acceleration.z             = f(table2array(m(:,15)));

lang.deltaV.x                   = f(table2array(m(:,16)));
lang.deltaV.y                   = f(table2array(m(:,17)));
lang.deltaV.z                   = f(table2array(m(:,18)));

lang.rollrate                   = f(table2array(m(:,19))); %[rad/s]
lang.pitchrate                  = f(table2array(m(:,20))); %[rad/s]
lang.yawrate                    = f(table2array(m(:,21))); %[rad/s]

lang.deltaQ                     = f(table2array(m(:,22:25)));

gpsThreshold = 5;

lang.lat                        = t(f(table2array(m(:,26))), gpsThreshold);
lang.lon                        = t(f(table2array(m(:,27))), gpsThreshold);
lang.alt                        = t(f(table2array(m(:,28))), gpsThreshold);

lang.velocity.x                 = f(table2array(m(:,29)));
lang.velocity.y                 = f(table2array(m(:,30)));
lang.velocity.z                 = f(table2array(m(:,31)));

lang.g.lon                      = t(f(table2array(m(:,32)))* 1e-7, gpsThreshold);
lang.g.lat              	    = t(f(table2array(m(:,33)))* 1e-7, gpsThreshold);
lang.g.h                        = t(f(table2array(m(:,34)))* 1e-3, gpsThreshold);
lang.g.heading                  = t(f(table2array(m(:,35)))* 1e-5, 1e4); %(vehicle heading [circ])

%% Estimations and marhsalling

% Yawacceleration
lang.yawacceleration = zeros(length(lang.yawrate),1);
for i = 1:length(lang.yawrate)
    if i ~= 1
        dy = lang.yawrate(i) - lang.yawrate(i-1);
        dx = lang.time(i) - lang.time(i-1);
        lang.yawacceleration(i) = dy/dx;
    end
end

% IMU to CoG coordinate transform
lang.acceleration.y = lang.acceleration.y + lang.velocity.x.*lang.yawrate;% - ry*lang.yawrate;

% Removal of gravity contributions
lang.acceleration.x = lang.acceleration.x.*(1 - sind(lang.pitch));
lang.acceleration.y = lang.acceleration.y.*(1 - sind(lang.roll));

lang.localVel.x = ones(length(lang.velocity.x),1);
lang.localVel.y = ones(length(lang.velocity.x),1);
for i=1:length(lang.velocity.x)
    A = [lang.velocity.x(i) lang.velocity.y(i)]';
    R = [cosd(lang.yaw(i)) -1*sind(lang.yaw(i)); sind(lang.yaw(i)) cosd(lang.yaw(i))];
    C = (R^-1)*A;
    
    lang.localVel.x(i) = C(1,1);
    lang.localVel.y(i) = C(2,1) - rx * lang.yawrate(i);
end



%% Offset

% % SWA
swaOffset = 0%-45%4.943;      % swaOffset = mean(lang.steering.angle(8e3:10e3))
lang.steering.angle = lang.steering.angle - swaOffset;

% Wheelspeed rear
wheelspeedOffset = 0.0264;  % mean(lang.wheelspeed.R(3.5e4:end))
lang.wheelspeed.R = lang.wheelspeed.R - wheelspeedOffset;

% Roll
rollOffset = -1.798463525047371e+02; % mean(lang.roll(3.5e4:end));
lang.roll = lang.roll - rollOffset;% + 180;

% Pitch
pitchOffset = -1.198331421737713; % mean(lang.pitch(3.5e4:end))
lang.pitch = lang.pitch - pitchOffset;

% Yaw
yawOffset = 0;% 1.078724809284089e+02; % mean(lang.yaw(3.5e4:end))
lang.yaw = lang.yaw - mean(lang.yaw(1e2:2e2));% - lang.g.heading(100) + yawOffset;
lang.plotyaw = lang.yaw + yawOffset;

% n=3000;
% dxdy = 1000*ones(n,1);
% for i=2:1:n
%     i
%     var.signals.values = [lang.wheelspeed.R lang.steering.angle lang.acceleration.y lang.yawrate lang.yaw];
%     simin=[Time var.signals.values];
%     sim('washout', Time)
%     dxdy(i) = abs(xm(2000) - mean(xraw(1:4000)) + abs(ym(2000) - mean(yraw(1:4000))));
%     if dxdy(i) > dxdy(i-1)
%         disp('hej')
%         break
%     end
%     yawOffset = 0.3234800 + i*.00001; 
%     lang.yaw = lang.yaw + yawOffset;


ayOffset = -0.045659673441993; % mean(lang.acceleration.y(3.5e4:end));
% lang.acceleration.y = lang.acceleration.y - ayOffset; % - lang.acceleration.x*sind;

% lang.acceleration.y = lang.acceleration.y - lang.acceleration.x*cosd(
% ay - sind(yaw)*ax = 0
% asind(ay/ax)

%% Local vehicle speeds



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('Data processing done\n'), toc
end

% Removes NaN & 0 type values
function v = f(v)
    firstNaN = true;
    for i = 1:length(v)
        if isnan(v(i)) == true
            if firstNaN == true
                v(i) = 0;
                firstNaN = false;
            else
                v(i) = v(i-1);
            end
        end
        if v(i) == 0 && i ~= 1
            v(i) = v(i-1);
        end
    end
end

% Removes 0 and values outside of a determined threshold
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
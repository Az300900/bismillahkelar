clc;clear;close

% Track Generator 2 - Track Circuit
% Note
% total time used = 61.2
% Time
ts = 0.1; % Sampling Time
ttotal = 52; % Total Simulation Time
% ttotal = 52; % Total Simulation Time
nt = ttotal/ts; % Total sampling
n = 0:ts:ttotal-ts; % Time every sampling
% Road Gradient1
psi_R = 0;
for k=2:nt
    if (k*ts<1)
          psi_R(k)=psi_R(k-1)+0.04;
    elseif (k*ts > 28)&&(k*ts <= 32)
        psi_R(k) = psi_R(k-1)-0.05;
    elseif (k*ts > 32)&&(k*ts <= 36)
        psi_R(k) = psi_R(k-1)-0.04;
%     elseif (k*ts > 20)&&(k*ts <= 25)
%         psi_R(k) = psi_R(k-1)-0.02;
%     elseif (k*ts > 30)&&(k*ts <= 40)
%         psi_R(k) = psi_R(k-1)-0.03;
%     elseif (k*ts > 40)&&(k*ts <= 45)
%         psi_R(k) = psi_R(k-1)+0.05;
%     elseif (k*ts > 45)&&(k*ts <= 50)
%         psi_R(k) = psi_R(k-1)-0.037;
%     elseif (k*ts > 53)&&(k*ts <= 60)
%         psi_R(k) = psi_R(k-1)-0.05;
    else
        psi_R(k) = psi_R(k-1);
    end
end
plot(psi_R)
% %road curvature2
% vx = 30; % Velocity used
% v_x = vx*ones(1,nt);
% llane = 100; % Lane Width
% tturn = 20; % Time used to lane changing
% tturn2 = 40;
% 
% % Position Reference
% Ystr = llane*ones(1,nt);
% ysin = llane/2*(1+cos((0:ts:tturn)./tturn*pi));
% ysin2 = llane/2*(1+cos((tturn:ts:tturn2)./tturn*pi));
% 
% yp = Ystr -[llane*ones(1,(nt-length(ysin)-length(ysin2))/2) ysin ysin2 llane*ones(1,(nt-length(ysin)-length(ysin2))/2)];
% xp = vx*n;
% %road gradient
% psi_R = [diff(yp)./diff(xp) 0];
% plot(psi_R)

% Road Curvature
curvature = [diff(psi_R) 0];
plot(curvature)
% Velocity Used
vx = 20;
v_x = vx*ones(1,nt);

% Position Reference
xp = 0; yp = 0;
for k=2:nt
    yp(k) = yp(k-1) + v_x(k)*ts*sin(psi_R(k));
    xp(k) = xp(k-1) + v_x(k)*ts*cos(psi_R(k));
end         
%%
psi_Rm = 0;yp1=yp(1);xp1=xp(1);
for k=2:nt
    psi_Rm(k) = psi_Rm(k-1)+curvature(k);
    yp1(k) = yp1(k-1) + v_x(k)*ts*sin(psi_Rm(k));
    xp1(k) = xp1(k-1) + v_x(k)*ts*cos(psi_Rm(k));
end

figure
plot(xp1,yp1)

% Global Position
LateralDeviation=xlsread('HE25Np=10.xlsx','lateral');
YawDeviation=xlsread('HE25Np=10.xlsx','yaw');
LateralDeviation = LateralDeviation(2:521,2)';
YawDeviation = YawDeviation(2:521,2)';
%panggil nilai noise
lnoise=xlsread('HE25Np=10.xlsx','ldeviasi');
ynoise=xlsread('HE25Np=10.xlsx','ydeviasi');
lnoise = lnoise(2:521,2)';
ynoise = ynoise(2:521,2)';

xv=0;
yv=0;
psi_V = psi_R(1:end)+YawDeviation;

np = 10;

l_s = 5;
xv = xp1(1:end)-LateralDeviation.*sin(psi_Rm(1:end))-l_s*cos(psi_V);
yv = yp1(1:end)+LateralDeviation.*cos(psi_Rm(1:end))-l_s*sin(psi_V);
xn = xp1(1:end)-lnoise.*sin(psi_Rm(1:end))-l_s*cos(psi_V);
yn = yp1(1:end)+ynoise.*cos(psi_Rm(1:end))-l_s*sin(psi_V);
plot(xv,yv)
hold on
plot(xn,yn)
hold on
plot(xp1,yp1)
legend
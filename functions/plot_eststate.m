function plot_eststate(clkest, rpyest, imubiasest)
%% Plot estimated states
% Author: Taro Suzuki
arguments
    clkest 
    rpyest = []
    imubiasest = []
end

% Clock
clkesttmp = clkest;
clkesttmp(clkesttmp==0) = NaN;

%% Plot estimated receiver clock
figure;
nexttile
plot(clkesttmp(:,1),'.-'); grid on;
title('Receiver clock bias');
ylabel('Clock Bias [m]');
legend('GPS L1');
nexttile
plot(clkesttmp(:,2:end),'.-'); grid on;
ylabel('Clock Bias [m]');
legstr = ["GLO L1","GAL L1","BDS L1","GPS L5","GAL L5","BDS L5"];
legstr(all(isnan(clkesttmp(:,2:end)),1)) = "";
legend(legstr)

%% Plot estimated attitude
if ~isempty(rpyest)
    figure;
    tiledlayout(3,1);
    nexttile
    plot(wrapTo180(rad2deg(rpyest(:,1))));
    grid on; hold on;
    legend('Roll');
    title('Attitude')
    nexttile
    plot(wrapTo180(rad2deg(rpyest(:,2))));
    grid on; hold on;
    legend('Pitch');
    nexttile
    plot(wrapTo180(rad2deg(rpyest(:,3))));
    grid on; hold on;
    legend('Yaw');
end

%% Plot estimated IMU bias
if ~isempty(imubiasest)
    figure;
    tiledlayout(2,1);
    nexttile
    plot(imubiasest(:,1:3)); grid on; hold on;
    title('Acc')
    legend('x','y','z');
    nexttile
    plot(rad2deg(imubiasest(:,4:6))); grid on; hold on;
    title('Gyro')
    legend('x','y','z');
end

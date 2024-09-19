function [acc, gyro, idx_stop] = imuprocessing(obs, acc, gyro, vel, prm)
%% Preprocessing for IMU data
% Author: Taro Suzuki

%% Time synchronization
% Accelerometer
if any(~isnan(obs.elapsedns))
    acc.utcms = interp1(obs.elapsedns,obs.utcms,acc.elapsedns,"linear","extrap");
    acc.sync_coefficient = prm.imu_sync_coefficient;
else
    acc.utcms = acc.utcms+prm.acctimeoffset;
    acc.sync_coefficient = 1.0;
end
% Gyroscope
if any(~isnan(obs.elapsedns))
    gyro.utcms = interp1(obs.elapsedns,obs.utcms,gyro.elapsedns,"linear","extrap");
    gyro.sync_coefficient = prm.imu_sync_coefficient;
else
    gyro.utcms = gyro.utcms+prm.gyrotimeoffset;
    gyro.sync_coefficient = 1.0;
end

% Accelerometer and gyroscope synchronization
if prm.imu_sync == "acc"
    acc.utcmssync = acc.utcms;
    gyro.utcmssync = acc.utcms;
    acc.xyzsync = acc.xyz;
    gyro.xyzsync = interp1(gyro.utcms,gyro.xyz,acc.utcms,"linear","extrap");
elseif prm.imu_sync == "gyro"
    acc.utcmssync = gyro.utcms;
    gyro.utcmssync = gyro.utcms;
    acc.xyzsync = interp1(acc.utcms,acc.xyz,gyro.utcms,"linear","extrap");
    gyro.xyzsync = gyro.xyz;
else
    error("Wrong prm.imu_sync: %s",prm.imu_sync);
end
acc.dt = diff(acc.utcmssync)/1000;
acc.dt = [acc.dt; acc.dt(end)];
gyro.dt = diff(gyro.utcmssync)/1000;
gyro.dt = [gyro.dt; gyro.dt(end)];

%% Standard deviation
% Accelerometer
acc.d3 = vecnorm(acc.xyzsync,2,2);
acc.d3std = movstd(acc.d3,prm.stop_windowsize);
% Gyroscope
gyro.d3 = vecnorm(gyro.xyzsync,2,2);
gyro.d3std = movstd(gyro.d3,prm.stop_windowsize);

%% Stop detection
accminstd = min(acc.d3std);
gyrominstd = min(gyro.d3std);
acc.stop_std_th = accminstd+prm.stop_acc_std_offset;
gyro.stop_std_th = gyrominstd+prm.stop_gyro_std_offset;

idx_stop = acc.d3std <acc.stop_std_th & ...
           gyro.d3std<gyro.stop_std_th & ...
           gyro.d3   <prm.stop_gyro_max;

if prm.plotflag
    figure;
    plot(acc.utcmssync,acc.d3-prm.g,'b.'); hold on; grid on;
    plot(acc.utcmssync(idx_stop),acc.d3(idx_stop)-prm.g,'r.');
    plot(acc.utcmssync,acc.d3std,'g','LineWidth',2)
    plot(obs.utcms,vel.v3,'k','LineWidth',2)
    plot([acc.utcmssync(1) acc.utcmssync(end)],[acc.stop_std_th acc.stop_std_th ],'r','LineWidth',2)
    ylim([-0.2 0.5]);
    xlim([acc.utcmssync(1) acc.utcmssync(end)])
    legend("3D Acc","3D Acc(STOP)","3D acc STD","3D velocity","STD TH")
    title("Acc")

    figure;
    plot(gyro.utcmssync,gyro.d3,'b.'); hold on; grid on;
    plot(gyro.utcmssync(idx_stop),gyro.d3(idx_stop),'r.');
    plot(gyro.utcmssync,gyro.d3std,'g','LineWidth',2);
    plot(obs.utcms,vel.v3,'k','LineWidth',2)
    plot([gyro.utcmssync(1) gyro.utcmssync(end)],[gyro.stop_std_th gyro.stop_std_th],'r','LineWidth',2);
    ylim([-0.2 0.5]);
    xlim([gyro.utcmssync(1) gyro.utcmssync(end)])
    legend("3D angular rate","3D angular rate(STOP)","3D angular rate STD","3D velocity","STD TH")
    title("Gyro")
    drawnow;
end
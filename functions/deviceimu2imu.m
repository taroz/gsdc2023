function [acc ,gyro, mag] = deviceimu2imu(fname)
%% Read device_imu.csv
% Author: Taro Suzuki

%% Read data
dataS = readtable(fname,'NumHeaderLines',0,'ReadVariableNames',true);

% UncalAccel
accS = dataS(strcmp(dataS{:,1},'UncalAccel'),:);
[~,idxunique] = unique(accS.utcTimeMillis);
accS = accS(idxunique,:);
acc.n = size(accS,1);
acc.timems = utcms2gtime(accS.utcTimeMillis);
acc.utcms = accS.utcTimeMillis;
acc.elapsedns = accS.elapsedRealtimeNanos;
acc.xyz = [accS.MeasurementX accS.MeasurementY accS.MeasurementZ];
acc.bias = [accS.BiasX accS.BiasY accS.BiasZ];

% UncalGyro
gyroS = dataS(strcmp(dataS{:,1},'UncalGyro'),:);
[~,idxunique] = unique(gyroS.utcTimeMillis);
gyroS = gyroS(idxunique,:);
gyro.n = size(gyroS,1);
gyro.timems = utcms2gtime(gyroS.utcTimeMillis);
gyro.utcms = gyroS.utcTimeMillis;
gyro.elapsedns = gyroS.elapsedRealtimeNanos;
gyro.xyz = [gyroS.MeasurementX gyroS.MeasurementY gyroS.MeasurementZ];
gyro.bias = [gyroS.BiasX gyroS.BiasY gyroS.BiasZ];

% UncalMag
magS = dataS(strcmp(dataS{:,1},'UncalGyro'),:);
[~,idxunique] = unique(magS.utcTimeMillis);
magS = magS(idxunique,:);
mag.n = size(magS,1);
mag.timems = utcms2gtime(magS.utcTimeMillis);
mag.utcms = magS.utcTimeMillis;
mag.elapsedns = magS.elapsedRealtimeNanos;
mag.xyz = [magS.MeasurementX magS.MeasurementY magS.MeasurementZ];
mag.bias = [magS.BiasX magS.BiasY magS.BiasZ];

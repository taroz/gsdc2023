function prm = parameters(setting, initflag)
%% Preprocessing parameters
% Author: Taro Suzuki
arguments
    setting                  % Setting data from setting_train.csv or setting_test.csv
    initflag logical = false % Initialization flag, default = false
end

%% Flag
prm.plotflag = false; % plot flag

%% Copy setting
prm.Course = setting.Course;
prm.Phone = setting.Phone;
prm.Type = setting.Type;
prm.L5 = setting.L5;
prm.BDS = setting.BDS;
prm.RINEX = setting.RINEX;
prm.Base = setting.Base1;
prm.initflag = initflag;

%% Observation error model
% Elevation or SNR model
prm.P_model = "sn";
prm.D_model = "sn";
prm.L_model = "sn";

% Elavation model
prm.el_a = 0.003; % (m)
prm.el_b = 0.003; % (m)
prm.P_el_ratio = 1000;  % Factor of pseudorange
prm.D_el_ratio = 50;    % Factor of Doppler
prm.L_el_ratio = 1;     % Factor of carrier phase

% SNR model
prm.sn_ptile = 85;      % Percentile (%)
prm.sn_den = 20;        % Denominator
prm.P_sn_ratio = 1;     % Factor of pseudorange
prm.D_sn_ratio = 1/12;  % Factor of Doppler
prm.L_sn_ratio = 1/400; % Factor of carrier phase

% Weight for signal type
% L1, G1, E1, B1, L5, E5, B2a
prm.sigtype_factor = [0.8, 1.5, 0.8, 0.8, 0.5, 0.5, 0.5, NaN];

%% Time difference threshold
prm.time_diff_th = 1.5; % (s)

%% Velocity parameter for attitude estimation
prm.velocity_smooth_window = 20;
prm.velocity_threshold = 0.5; % Minimum 3D velocity (m/s)

%% Base station parameter
prm.base_movmean_n_15s = 11; % smoothing parameter for 15s interval observation
prm.base_movmean_n_1s = 151; % smoothing parameter for 1s interval observation

%% Robust kernels
huber = @gtsam.noiseModel.mEstimator.Huber.Create;
tukey = @gtsam.noiseModel.mEstimator.Tukey.Create;
cauchy = @gtsam.noiseModel.mEstimator.Cauchy.Create;

%% Pseudorange parameters
if setting.L5
    prm.Pmask_el = dictionary(["L1","L5"],[5 5]);    % Elevation mask (deg)
else
    prm.Pmask_el = dictionary(["L1","L5"],[10 10]);
end
prm.Pmask_sn = dictionary(["L1","L5"],[20 20]);      % SNR mask (dB-Hz)
prm.Pmask_dDP = dictionary(["L1","L5"],[40 20]);     % Doppler-pseudorange difference (m)

if initflag
    prm.Pmask_res = dictionary(["L1","L5"],[50 30]); % Pseudorange residuals (m)
else
    prm.Pmask_res = dictionary(["L1","L5"],[20 15]);
end

% Robust optimization
if setting.Type == "Street" || setting.Type == "Mix"
    prm.P_robust_prm = 0.1; % Huber kernel parameter
else
    prm.P_robust_prm = 0.2;
end
prm.P_kernel = huber(prm.P_robust_prm); % Huber kernel

%% Doppler parameters
if setting.L5
    prm.Dmask_el = dictionary(["L1","L5"],[5 5]);    % Elevation mask (deg)
else
    prm.Dmask_el = dictionary(["L1","L5"],[10 10]);
end
prm.Dmask_sn = dictionary(["L1","L5"],[20 20]);      % SNR mask (dB-Hz)

if initflag
    prm.Dmask_res = dictionary(["L1","L5"],[20 20]); % Doppler residuals (m/s)
else
    prm.Dmask_res = dictionary(["L1","L5"],[3 3]);
end

% Robust optimization
if setting.Type == "Street" || setting.Type == "Mix"
    prm.D_robust_prm = 0.4; % Huber kernel parameter
else
    prm.D_robust_prm = 0.8;
end
if contains(setting.Phone,"pixel4")
    prm.D_robust_prm =  0.2;
end
prm.D_kernel = huber(prm.D_robust_prm); % Huber kernel

%% Carrier phase parameters
if setting.L5
    prm.Lmask_el = dictionary(["L1","L5"],[5 5]);    % Elevation mask (deg)
else
    prm.Lmask_el = dictionary(["L1","L5"],[10 10]);
end
prm.Lmask_sn = dictionary(["L1","L5"],[20 20]);      % SNR mask (dB-Hz)
prm.Lmask_dDL = dictionary(["L1","L5"],[1.5 1.5]);   % Doppler-TDCP difference (m)

% Robust optimization
if setting.Type == "Street" || setting.Type == "Mix"
    prm.L_robust_prm = 0.2;
else
    prm.L_robust_prm = 0.5;
end
prm.L_kernel = huber(prm.L_robust_prm); % Huber kernel

% TDCP offset for some smartphones
prm.Loffset = 1.117; % m

%% Graph optimization
% Clock factor
prm.sigma_between_clk_gps = 1000;
prm.sigma_between_clk_others = 0.0001;

% Motion factor
prm.sigma_motion_clk = 0.1;
if setting.Type == "Street"
    prm.sigma_motion = 0.05;
else
    prm.sigma_motion = 0.01;
end
if ismember(setting.Phone,["mi8","xiaomimi8"])
    prm.sigma_motion = 0.1;
end

% Stop detection
prm.acctimeoffset = -20;
prm.gyrotimeoffset = -20;
prm.stop_windowsize = 500;
prm.stop_acc_std_offset = 0.08;
prm.stop_gyro_std_offset = 0.005;
prm.stop_gyro_max = 0.05;
prm.stop_v_th = 0.5;

% Stop factor
prm.sigma_stop_v = 0.01;
prm.stop_robust_prm = 0.5;
prm.stop_kernel = huber(prm.stop_robust_prm);

% Stop factor for pose
prm.sigma_stop_p  = [deg2rad(0.1)*ones(3,1); 0.02*ones(3,1)];
prm.stop_p_robust_prm = 0.5;
prm.stop_p_kernel = huber(prm.stop_p_robust_prm);

% Height factor
prm.hight_dist = 15; % m
prm.hight_cumdist = 100; % m
prm.hight_sigma = 0.1; % m
prm.hight_robust_prm = 0.5;
prm.hight_kernel = huber(prm.hight_robust_prm);

% Absolute height factor
prm.hight_abs_dist = 15; % m
prm.hight_abs_sigma = 0.1; % m
prm.hight_abs_robust_prm = 0.5;
prm.hight_abs_kernel = huber(prm.hight_abs_robust_prm);

% IMU factor
prm.g = 9.80665;
prm.imu_sync = "gyro";
if contains(setting.Phone,"pixel")
    prm.AccSigma = 0.05; % m/s²/√Hz
    prm.GyroSigma = 0.001; % rad/s/√Hz
elseif contains(setting.Phone,["sm-s908","sm-g988","sm-g325f","sm-g325f","samsun"])
    prm.AccSigma = 0.05; % m/s²/√Hz
    prm.GyroSigma = 0.001; % rad/s/√Hz
elseif contains(setting.Phone,"sm-a217m")
    prm.AccSigma = 0.1; % m/s²/√Hz
    prm.GyroSigma = 0.005; % rad/s/√Hz
elseif contains(setting.Phone,"sm")
    prm.AccSigma = 0.1; % m/s²/√Hz
    prm.GyroSigma = 0.001; % rad/s/√Hz
elseif contains(setting.Phone,"mi")
    prm.AccSigma = 0.05; % m/s²/√Hz
    prm.GyroSigma = 0.001; % rad/s/√Hz
else
    error("wrong phone");
end
prm.imu_sync_coefficient = 0.5;

prm.AccBiasSigma = 0.00025; % m √Hz/s²
prm.GyroBiasSigma = 0.0000005; % rad √Hz/s
prm.IntegrationSigma = 0.05;
prm.mountingAngle = deg2rad([-85 178 -94]');
prm.mountingPosition = [0 0 0]';

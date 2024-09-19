function optstatus = fgo_gnss_imu(datapath, setting, initflag)
%% Factor graph optimization using GNSS and IMU
% Author: Taro Suzuki
arguments
    datapath string  % Dataset path
    setting table    % Setting data from setting_train.csv or setting_test.csv
    initflag = false % Initialization flag, default = false
end

%% Path
addpath ./functions/
addpath ./gtsam_gnss/
addpath /usr/local/gtsam_toolbox/

%% Load data
course = setting.Course; phone = setting.Phone;
fprintf('Course: %s, Phone: %s\n', course, phone);

% Load preprocessed smartphone data
load(datapath+course+"/"+phone+"/"+"phone_data.mat");

% Load if the reference height is available
if exist(datapath+course+"/ref_hight.mat", "file")
    load(datapath+course+"/ref_hight.mat");
    posgt.setOrg(posbl.orgllh, 'llh'); % Set orgin and convert to ENU
end

%% Setting
is = setting.IdxStart; % Start index for optimization
ie = setting.IdxEnd;   % End index for optimization
n = obs.n;             % Number of total epochs
nsat = obs.nsat;       % Number of satellites
FTYPE = ["L1","L5"];   % Frequency type

prm = parameters(setting, initflag); % Processing parameter

%% Initial position/velocity/clk/dclk/rpy
if initflag
    % If this is the first run of fgo_gnss_imu, set the output of fgo_gnss to the initial value
    load(datapath+course+"/"+phone+"/"+"result_gnss.mat");
    posini = posest.copy();
    velini = velest.copy();
    clk = clkest;
    dclk = dclkest;
    rpy = vel2rpy(velini.enu, prm); % Estimate attitude from velocity
else
    % If this is not the first run, the previous estimate is used as the initial value
    load(datapath+course+"/"+phone+"/"+"result_gnss_imu.mat");
    posini = posest.copy();
    velini = velest.copy();
    clk = clkest;
    dclk = dclkest;
    if setting.RPYReset % Initial attitude reset flag
        rpy = vel2rpy(velini.enu, prm); % Estimate attitude from velocity
    else
        rpy = rpyest;
    end
end

%% Compute residuals
% Exclude outliers
obsr = exobs(obs, prm);

% Observation residuals
satr = gt.Gsat(obsr, nav);
satr.setRcvPosVel(posini, velini);
obsr = obsr.residuals(satr);

% Exclude outliers from residuals
obsr = exobs_residuals(obsr, satr, clk(:,1), dclk, prm);
obsr = obsr.residuals(satr);

% ECEF to ENU
for j=1:nsat
    exyz = [-satr.ex(:,j) -satr.ey(:,j) -satr.ez(:,j)]; % Line-of-sight vector in ECEF
    eenu = rtklib.ecef2enu(exyz, posini.orgllh); % Line-of-sight vector in ENU
    ee(:,j) = eenu(:,1);
    en(:,j) = eenu(:,2);
    eu(:,j) = eenu(:,3);
end

%% Pseudorange compensation using base observation
for f=FTYPE
    if ~isempty(obsr.(f))
        % Pseudorange correction
        pc = correct_pseudorange(datapath, obsr, obsb, nav, f, prm);
        obsr.(f).resPc = obsr.(f).resPc-pc;
    end
end

%% Observation error model
obserr = obserrmodel(obsr,satr,prm);

%% Parameters for graph optimization
noise_sigmas = @gtsam.noiseModel.Diagonal.Sigmas;
noise_robust = @gtsam.noiseModel.Robust.Create;
sym = @gtsam.symbol;

% Initial state
x_ini  = posini.enu'; % x (position) in ENU
v_ini  = velini.enu'; % v (velocity) in ENU
c_ini  = clk'; % c (clock)
d_ini  = dclk'; % d (clock drift)

% Motion factor
sigma_motion = prm.sigma_motion*ones(3,1);
noise_motion = noise_sigmas(sigma_motion);

% Clock factor
noise_clk = noise_sigmas(prm.sigma_motion_clk*ones(7,1));
noise_clkjump = noise_sigmas([Inf; prm.sigma_motion_clk*ones(6,1)]);

% Between clock factor
sigma_between_clk = [prm.sigma_between_clk_gps; prm.sigma_between_clk_others*ones(6,1)];
noise_between_clk = noise_sigmas(sigma_between_clk);
sigma_between_clkjump = [Inf; prm.sigma_between_clk_others*ones(6,1)];
noise_between_clkjump = noise_sigmas(sigma_between_clkjump);

% Stop factor
noise_stop_v = noise_sigmas(prm.sigma_stop_v*ones(3,1));
noise_stop_v_robust = noise_robust(prm.stop_kernel, noise_stop_v);

% Hight factor
cumdist = cumsum(velini.v3);
noise_equal_x_hight = noise_sigmas([Inf Inf prm.hight_sigma]');
noise_equal_x_hight_robust = noise_robust(prm.hight_kernel, noise_equal_x_hight);

% Absolute hight factor
noise_abs_hight = noise_sigmas([Inf Inf prm.hight_abs_sigma]');
noise_abs_hight_robust = noise_robust(prm.hight_abs_kernel, noise_abs_hight);

%% IMU
% Synchronization and stop detection
[acc, gyro, idx_stop] = imuprocessing(obs, acc ,gyro, velini, prm);

% Stop index
stop = logical(interp1(acc.utcmssync,double(idx_stop), obs.utcms, "nearest", "extrap"));

% Preintegration paramters
w_coriolis = [0;0;0];
IMU_params = gtsam.PreintegrationParams([0;0;-prm.g]);
IMU_params.setAccelerometerCovariance((acc.sync_coefficient*prm.AccSigma).^2*eye(3));
IMU_params.setGyroscopeCovariance((gyro.sync_coefficient*prm.GyroSigma).^2*eye(3));
IMU_params.setIntegrationCovariance(prm.IntegrationSigma.^2*eye(3));
IMU_params.setOmegaCoriolis(w_coriolis);
IMU_params.setBodyPSensor(gtsam.Pose3(gtsam.Rot3.RzRyRx(prm.mountingAngle), gtsam.Point3(prm.mountingPosition)));

% Initial pose in ENU
rotm = eul2rotm(rpy);
for i=1:n
    p_ini(i) = gtsam.Pose3(gtsam.Rot3(rotm(:,:,i)), gtsam.Point3(posini.enu(i,:)'));
end

% Initial IMU bias
imuBiasZero = gtsam.imuBias.ConstantBias(zeros(3,1), zeros(3,1));

% Between IMU bias factor
sigma_between_b = [prm.AccBiasSigma*ones(3,1); prm.GyroBiasSigma*ones(3,1)];

% Pose3 to Point3 factor
noise_pose3point3 = noise_sigmas([0 0 0]');

% Stop factor for pose
p0 = gtsam.Pose3();
noise_stop_p = noise_sigmas(prm.sigma_stop_p);
noise_stop_p_robust = noise_robust(prm.stop_p_kernel, noise_stop_p);

%% Graph Construction
% Create a factor graph container
graph = gtsam.NonlinearFactorGraph;

% Initial factor/state
initials = gtsam.Values;
for i=is:ie
    % Initial state
    initials.insert(sym('p',i), p_ini(i));
    initials.insert(sym('x',i), x_ini(:,i));
    initials.insert(sym('v',i), v_ini(:,i));
    initials.insert(sym('c',i), c_ini(:,i));
    initials.insert(sym('d',i), d_ini(:,i));
    initials.insert(sym('b',i), imuBiasZero)

    % Initial factor
    graph.add(gtsam.PriorFactorPose3(sym('p',i), p_ini(:,i), noise_sigmas(Inf*ones(6,1))));
    graph.add(gtsam.PriorFactorVector(sym('x',i), x_ini(:,i), noise_sigmas(Inf*ones(3,1))));
    graph.add(gtsam.PriorFactorVector(sym('v',i), v_ini(:,i), noise_sigmas(Inf*ones(3,1))));
    graph.add(gtsam.PriorFactorVector(sym('c',i), c_ini(:,i), noise_sigmas(Inf*ones(7,1))));
    graph.add(gtsam.PriorFactorVector(sym('d',i), d_ini(:,i), noise_sigmas(Inf*ones(1,1))));
    graph.add(gtsam.PriorFactorConstantBias(sym('b',i), imuBiasZero, noise_sigmas(Inf*ones(6,1))));
end

% Pseudorange/Doppler factor
for i=progress(is:ie)
    keyP = sym('p',i);
    keyX = sym('x',i);
    keyV = sym('v',i);
    keyC = sym('c',i);
    keyD = sym('d',i);
    orgx = posini.enu(i,:)';
    orgv = velini.enu(i,:)';

    % Pose3 to Point3 factor
    graph.add(gtsam_gnss.Pose3Point3Factor(keyP, keyX, noise_pose3point3));

    for j=1:nsat
        losvec = [ee(i,j) en(i,j) eu(i,j)]';
        for f=FTYPE
            if ~isempty(obsr.(f))
                sigtype = sysfreq2sigtype(obsr.sys,f);
                % Pseudorange factor
                if ~isnan(obsr.(f).resPc(i,j))
                    noise = noise_sigmas(obserr.(f).P(i,j));
                    noise_rubust = noise_robust(prm.P_kernel, noise);
                    graph.add(gtsam_gnss.PseudorangeFactorWithClock(keyX, keyC, losvec, obsr.(f).resPc(i,j), sigtype(j), orgx, noise_rubust));
                end
                % Doppler factor
                if ~isnan(obsr.(f).resD(i,j))
                    noise = noise_sigmas(obserr.(f).D(i,j));
                    noise_rubust = noise_robust(prm.D_kernel, noise);
                    graph.add(gtsam_gnss.DopplerFactorWithClockV(keyV, keyD, losvec, obsr.(f).resD(i,j), orgv, noise_rubust));
                end
            end
        end
    end

    if ~initflag
        % Stop factor
        if stop(i) && velini.v3(i)<prm.stop_v_th
            graph.add(gtsam.PriorFactorVector(keyV, zeros(3,1), noise_stop_v_robust));
        end

        % Absolute hight factor
        if exist("posgt","var")
            distdiff = vecnorm(posgt.enu(:,1:2)-posini.enu(i,1:2),2,2);
            [mindist,minidx] = min(distdiff);
            if mindist<prm.hight_abs_dist
                enu = [0 0 posgt.up(minidx)]';
                graph.add(gtsam.PriorFactorVector(keyX, enu, noise_abs_hight_robust));
            end
            % Hight factor
        else
            distdiff = vecnorm(posini.xyz-posini.xyz(i,:),2,2); % Difference of distance at current location
            cumdistdiff = cumdist-cumdist(i); % Difference of cummlative distance at current location
            idx_near = distdiff<prm.hight_dist & cumdistdiff>prm.hight_cumdist;
            for idx = find(idx_near)'
                if ~stop(i) && ~stop(idx)
                    keyX2 = sym('x',idx);
                    graph.add(gtsam.BetweenFactorVector(keyX, keyX2, zeros(3,1), noise_equal_x_hight_robust));
                end
            end
        end
    end
end

% Motion/Clock/IMU/TDCP factor
for i=progress(is:ie-1)
    keyP1 = sym('p',i); keyP2 = sym('p',i+1);
    keyX1 = sym('x',i); keyX2 = sym('x',i+1);
    keyV1 = sym('v',i); keyV2 = sym('v',i+1);
    keyC1 = sym('c',i); keyC2 = sym('c',i+1);
    keyD1 = sym('d',i); keyD2 = sym('d',i+1);
    keyB1 = sym('b',i); keyB2 = sym('b',i+1);
    orgx1 = posini.enu(i,:)';
    orgx2 = posini.enu(i+1,:)';

    % Time difference
    dtgps = (obs.utcms(i+1)-obs.utcms(i))/1000;

    if dtgps<prm.time_diff_th
        % Motion factor
        graph.add(gtsam_gnss.MotionFactor(keyX1, keyX2, keyV1, keyV2, dtgps, noise_motion));

        % Clock factor
        if ~ismember(phone,["sm-a205u","sm-a505u","samsunga325g"])
            if obs.clkjump(i+1)
                graph.add(gtsam_gnss.ClockFactor(keyC1, keyC2, keyD1, keyD2, dtgps, noise_clkjump));
            else
                graph.add(gtsam_gnss.ClockFactor(keyC1, keyC2, keyD1, keyD2, dtgps, noise_clk));
            end
            % Between clock factor
            if obs.clkjump(i+1)
                graph.add(gtsam.BetweenFactorVector(keyC1, keyC2, zeros(7,1), noise_between_clkjump));
            else
                graph.add(gtsam.BetweenFactorVector(keyC1, keyC2, zeros(7,1), noise_between_clk));
            end
        end
    end

    % IMU preintegration
    IMUindices = find(acc.utcmssync >= obs.utcms(i) & acc.utcmssync <= obs.utcms(i+1))';
    currentSummarizedMeasurement = gtsam.PreintegratedImuMeasurements(IMU_params,imuBiasZero);
    for imuIndex = IMUindices
        currentSummarizedMeasurement.integrateMeasurement(acc.xyzsync(imuIndex,:)', gyro.xyzsync(imuIndex,:)', acc.dt(imuIndex));
    end

    if dtgps<prm.time_diff_th
        % Stop factor for pose
        if stop(i) && stop(i+1) && velini.v3(i)<prm.stop_v_th
            graph.add(gtsam.BetweenFactorPose3(keyP1, keyP2, p0, noise_stop_p_robust));
        end
        % IMU factor
        graph.add(gtsam.ImuFactor(keyP1, keyV1, keyP2, keyV2, keyB2, currentSummarizedMeasurement));
    end

    % IMU Bias
    assert(numel(IMUindices)~=0)
    noise_btween_b = gtsam.noiseModel.Diagonal.Sigmas(sqrt(numel(IMUindices))*sigma_between_b);
    graph.add(gtsam.BetweenFactorConstantBias(keyB1, keyB2, imuBiasZero, noise_btween_b));

    if ~ismember(setting.Phone,["sm-a325f","samsunga32"])
        for j=1:nsat
            losvec = [ee(i,j),en(i,j),eu(i,j)]';
            for f=FTYPE
                if ~isempty(obsr.(f))
                    sigtype = sysfreq2sigtype(obsr.sys,f);
                    % TDCP factor
                    if ~isnan(obsr.(f).resL(i,j)) && ~isnan(obsr.(f).resL(i+1,j)) && ~obs.clkjump(i+1)
                        noise = noise_sigmas(obserr.(f).L(i,j));
                        noise_rubust = noise_robust(prm.L_kernel, noise);
                        if ismember(phone,["sm-a205u","sm-a217m","sm-a505g","sm-a600t","sm-a505u"])
                            graph.add(gtsam_gnss.TDCPFactorWithDClock(keyX1, keyX2, keyD1, keyD2, losvec, obsr.(f).resL(i,j)-prm.Loffset, obsr.(f).resL(i+1,j), orgx1, orgx2, noise_rubust));
                        elseif ismember(phone,"samsunga325g")
                            graph.add(gtsam_gnss.TDCPFactorWithDClock(keyX1, keyX2, keyD1, keyD2, losvec, obsr.(f).resL(i,j), obsr.(f).resL(i+1,j), orgx1, orgx2, noise_rubust));
                        else
                            graph.add(gtsam_gnss.TDCPFactorWithClock(keyX1, keyX2, keyC1, keyC2, losvec, obsr.(f).resL(i,j), obsr.(f).resL(i+1,j), sigtype(j), orgx1, orgx2, noise_rubust));
                        end
                    end
                end
            end
        end
    end
end

%% Optimization
optparameters = gtsam.LevenbergMarquardtParams;
optparameters.setVerbosity('TERMINATION');
optparameters.setMaxIterations(1000);
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initials, optparameters);

% Optimize!
disp('optimization... ');
fprintf('Initial Error: %.2f\n',optimizer.error);
tic;
results = optimizer.optimize();
fprintf('Error: %.2f Iter: %d\n',optimizer.error,optimizer.iterations);
toc;
optstatus.OptTime = toc;
optstatus.OptIter = optimizer.iterations;
optstatus.OptError = optimizer.error;

% Retrieving the estimated value
pest = NaN(n,6);
xest = NaN(n,3);
vest = NaN(n,3);
clkest = NaN(n,7);
dclkest = NaN(n,1);
imubiasest = NaN(n,6);
for i=is:ie
    pose = results.atPose3(sym('p',i));
    pest(i,:) = [pose.translation.vector' pose.rotation.rpy'];
    xest(i,:) = results.atVector(sym('x',i))';
    vest(i,:) = results.atVector(sym('v',i))';
    clkest(i,:) = results.atVector(sym('c',i))';
    dclkest(i,:) = results.atVector(sym('d',i))';
    imubiasest(i,:) = results.atConstantBias(gtsam.symbol('b',i)).vector';
end

% Estimated position/velocity
posest = gt.Gpos(pest(:,1:3),'enu',posini.orgllh,'llh');
velest = gt.Gvel(vest,'enu',posini.orgllh,'llh');
rpyest = pest(:,4:6);

%% Add position offset
posest = add_position_offset(posest, rpyest, phone);

%% Plot results
plot_eststate(clkest, rpyest, imubiasest);

% Plot score
if contains(datapath,'train')
    load(datapath+course+"/"+phone+"/"+"gt.mat");
    optstatus.Score =  plot_score(posest, posbl, posgt);
else
    optstatus.Score = NaN;
end

%% Save results
fname = datapath+course+"/"+phone+"/"+"result_gnss_imu.mat";
save(fname,"posest","clkest","velest","dclkest","imubiasest","rpyest");
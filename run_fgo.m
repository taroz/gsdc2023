%% Run factor graph optimization for smartphone's GNSS and IMU integration
% Author Taro Suzuki
clear; close all; clc;
addpath ./functions/

%% Setting
% Initialization flag for position estimation.
% If you want to estimate without using the results of the previous estimation,
% set this to true. In that case, the graph optimization will be executed three times.
initflag = false;

% Target dataset: "train" or "test"
dataset = "test";

%% Read setting.csv
datapath = "./dataset_2023/"+dataset+"/";

% Read setting.csv
settings = readtable("./dataset_2023/settings_"+dataset+".csv", "TextType","string");
n = height(settings);  % Number of trip

% Collecting optimization status
optStat = repmat(struct("OptTime",NaN,"OptIter",NaN,"OptError",NaN,"Score",NaN), n, 1);

%% Run FGO
tic;
% Use parfor to speed up. The figure will not be displayed
parfor i=1:n
    % Trip path
    setting = settings(i,:);
    trippath = datapath+setting.Course+"/"+setting.Phone+"/";

    %% FGO using GNSS only (first position/velocity estimation)
    % The results are saved in result_gnss.mat.
    if initflag
        fgo_gnss(datapath, setting, true);
    end

    %% FGO using GNSS+IMU (first attitude estimation)
    % The results are saved in result_gnss_imu.mat.
    if initflag
        if ~exist(trippath+"/result_gnss.mat", "file")
            error("Please execute fgo_gnss(datapath, setting, true) first!")
        end
        fgo_gnss_imu(datapath, setting, true);
    end

    %% FGO using GNSS+IMU (final estimation)
    % Estimate a more accurate position using the previous estimated result
    % (result_gnss_imu.mat) as input.
    if ~exist(trippath+"/result_gnss_imu.mat", "file")
        error("Please execute fgo_gnss_imu(datapath, setting, true) first!")
    end
    optStat(i) = fgo_gnss_imu(datapath, setting, false);
end
toc;

%% Write results to file
resultpath = "./results/"+dataset+"/";
[mScore, resultpath] = write_results(resultpath, settings, optStat);

% Show mean score (train dataset)
if contains(dataset, "train")
    fprintf("Mean score = %.4f m\n", mScore);
end

% Create submission file (test dataset)
if contains(dataset, "test")
    submission(resultpath, settings);
end
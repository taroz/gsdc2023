%% Run preprocessing of smartphone log
% Input log_gnss.txt and output phone_data.mat. 
% The dataset_2023 already contains the processed phone_data.mat, 
% so there is no need to run it again.
%
% Author Taro Suzuki
clear; close all; clc;

%% Dataset
dataset = "test"; % "train" or "test"
datasetpath = "./dataset_2023/"+dataset+"/";

%% Setting
settings = readtable("./dataset_2023/settings_"+dataset+".csv","TextType","string");
n = height(settings); % Number of trip

%% Preprocessing
for i=1:n
    preprocessing(datasetpath, settings(i,:));
end

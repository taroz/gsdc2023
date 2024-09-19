function [mScore, resultpath] = write_results(resultpath, settings, optStat)
%% Write optimization results to file
% Author: Taro Suzuki

% Optimization status
optStat = struct2table(optStat);
settings.OptTime = string(round(optStat.OptTime,1));   % Optimization time
settings.OptIterations = string(optStat.OptIter);      % Iterations
settings.OptError = string(round(optStat.OptError,2)); % Final error
settings.Score = string(round(optStat.Score,3));       % Score (for train)

% Mean score (for train)
mScore = mean(optStat.Score, "omitmissing");

% Make result directory
dstr = string(datetime("now"), "yyyyMMdd_HHmm");
if contains(resultpath, "train")
    sstr = "_score_"+num2str(mScore,"%.3f"); % Insert mean score in file name
else
    sstr = "";
end
resultpath = resultpath+dstr+sstr+"/";
mkdir(resultpath);

% Write optimization result
writetable(settings, resultpath+"results"+sstr+".csv");

% Copy files
copyfile("parameters.m", resultpath+"parameters.m");
copyfile("fgo_gnss_imu.m", resultpath+"fgo_gnss_imu.m");
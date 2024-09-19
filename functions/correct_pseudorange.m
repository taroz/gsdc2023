function pc = correct_pseudorange(datasetpath, obsr, obsb, nav, freq, prm)
%% Compute pseudorange compensation using base station
% Author: Taro Suzuki

% Extruct same satellite
obsb = obsb.sameSat(obsr);

% Read base station position list
basepos = readtable(datasetpath+"../base/base_position.csv","TextType","string");
baseoffset = readtable(datasetpath+"../base/base_offset.csv","TextType","string");

% Compute base station position
year = prm.Course.split("-"); year = year(1);
idx = contains(basepos.Base, prm.Base) & basepos.Year==double(year);
posbase = gt.Gpos(mean(basepos{idx,3:5}, 1, "omitmissing"),'xyz');

% Observation residuals at base station
satb = gt.Gsat(obsb, nav);
satb.setRcvPosVel(posbase, gt.Gvel([0 0 0], 'xyz'));
obsb = obsb.residuals(satb);

% Add antenna reference position offset
offset = baseoffset{baseoffset.Base==prm.Base, 2:4};
posbase.addOffset(offset,'enu');

% Smoothing base station pseudorange residuals 
if obsb.dt == 1.0
    pc_ = smoothdata(obsb.(freq).resPc, "movmean", prm.base_movmean_n_1s);
elseif obsb.dt == 15.0
    pc_ = smoothdata(obsb.(freq).resPc, "movmean", prm.base_movmean_n_15s);
end

% Interpolation at time.t
pc = interp1(obsb.time.t, pc_, obsr.time.t, "linear");

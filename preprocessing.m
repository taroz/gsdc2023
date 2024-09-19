function preprocessing(datapath, setting)
%% Preprocessing of smartphone log
% Author: Taro Suzuki
arguments
    datapath string  % Dataset path
    setting table    % Setting data from setting_train.csv or setting_test.csv
end

%% Settings
warning('off','backtrace')
addpath ./functions

phone = setting.Phone;
course = setting.Course;
fprintf('Course: %s, Phone: %s\n', course, phone);

path = datapath+course+"/";

%% Read GNSS/IMU data
% RINEX navigation
navfile = path+"brdc.*";
nav = gt.Gnav(navfile);

% GNSS observation from gnss_log.txt
logfile = path+phone+"/supplemental/gnss_log.txt";
obs = GobsPhone(logfile);

% IMU observation from device_imu.csv
imufile = path+phone+"/device_imu.csv";
[acc, gyro, mag] = deviceimu2imu(imufile);

%% Baseline position
data = readtable(path+phone+"/device_gnss.csv");
data(data.BiasUncertaintyNanos>1e4,:) = [];
[utcTimeMillis, idx] = unique(data.utcTimeMillis);
xyzbl = [data.WlsPositionXEcefMeters(idx),...
         data.WlsPositionYEcefMeters(idx),...
         data.WlsPositionZEcefMeters(idx)];
timebl = utcms2gtime(utcTimeMillis);
posbl = gt.Gpos(xyzbl, 'xyz');
orgllh = posbl.llh(find(~isnan(posbl.lat),1),:);
posbl.setOrg(orgllh,'llh');

% Check epoch size
if obs.n~=posbl.n
    warning('Observation size and baseline size do not match, nobs:%d nbl:%d',obs.n,posbl.n);
end

% Check NaN in baseline
if any(isnan(posbl.xyz),'all')
    nnan = nnz(isnan(posbl.xyz(:,1)));
    warning('Baseline has NaN and interpolated, nNaN:%d',nnan);
    xyz = fillmissing(posbl.xyz,'linear');
    posbl = gt.Gpos(xyz,'xyz',orgllh,'llh');
end

% Check baseline outlilers
tf = isoutlier(posbl.east,'movmedian',10,'ThresholdFactor',20);
tf = tf | isoutlier(posbl.north,'movmedian',10,'ThresholdFactor',20);
tf = tf | isoutlier(posbl.up,'movmedian',10,'ThresholdFactor',20);
noutlier = nnz(tf);
if any(tf)
    warning('Baseline has jump, removed and interpolated, noutlier:%d',noutlier);
    xyz = posbl.xyz;
    xyz(tf,:) = nan;
    xyz = fillmissing(xyz,'linear');
    posbl = gt.Gpos(xyz,'xyz',orgllh,'llh');
end

%% Base station RINEX data
% Time span for triming
ts = obs.timems.select(1);
te = obs.timems.select(obs.timems.n);
ts.addOffset(-180); % 180 sec margins for smoothing
te.addOffset(180);

% GNSS observation of base station
if setting.RINEX=="V2"
    filernx = path+setting.Base1+"_rnx2.obs";
elseif setting.RINEX=="V3"
    filernx = path+setting.Base1+"_rnx3.obs";
end
obsb = gt.Gobs(filernx);
obsb = obsb.selectTimeSpan(ts,te);

assert(obsb.n>0,'No common time for base station observations');

if obsb.n>0
    obsb = obsb.eliminateNaN();
end

% Check time span
if obsb.time.t(1)>obs.time.t(1) || obsb.time.t(end)<obs.time.t(end)
    error("Base station time span does not match rover observation");
end

%% Ground truth in train dataset
if contains(datapath,"train")
    gturth = readtable(path+phone+"/ground_truth.csv");
    llhgt = [gturth.LatitudeDegrees,...
        gturth.LongitudeDegrees,...
        gturth.AltitudeMeters];
    timegt = utcms2gtime(gturth.UnixTimeMillis);
    yawgt = -gturth.BearingDegrees+90;
    speedgt = gturth.SpeedMps;
    if timegt.n~=timebl.n
        warning('Ground truth epochs:%d, Baseline epochs:%d',timegt.n,timebl.n);
        llhgt = interp1(timegt.t,llhgt,timebl.t,'nearest','extrap');
        yawgt = interp1(timegt.t,yawgt,timebl.t,'nearest','extrap');
        speedgt = interp1(timegt.t,speedgt,timebl.t,'nearest','extrap');
        timegt = timebl;
    end
    posgt = gt.Gpos(llhgt,'llh',orgllh,'llh');
    velgt = posgt.gradient(obs.dt);

    % Check NaN in ground truth
    assert(~any(isnan(posgt.xyz),'all'),'Ground truth has NaN');

    % Check time in ground truth
    assert(all(timebl.t==timegt.t),'Ground truth time does not match baseline time');
end

%% Check GNSS data
% Check hardware clock discontinue
if any(diff(obs.hcdc)~=0)
    njump = nnz(diff(obs.hcdc)~=0);
    warning('Hardware clock discontinue, njump:%d',njump);
end

% mi8 do not contain clock drift
if ismember(phone,["mi8","xiaomimi8"])
    obs.dclk = gradient(obs.clk);
    obs.dclk(abs(obs.dclk)>1e3) = NaN;
    obs.dclk = fillmissing(obs.dclk,"linear");
end

% sm-xxx clock drift is wrong
% Estimate receiver clock drift
if ismember(phone,["sm-a226b","sm-a505g","sm-a600t","sm-a325f","sm-a217m","sm-a205u","samsunga325g","samsunga32","sm-a505u"])
    obsgps = obs.selectSat(obs.sys==gt.C.SYS_GPS);
    obsgps.L1.P(obsgps.L1.P<1e7|obsgps.L1.P>4e7|obsgps.L1.S<15) = NaN;
    sat = gt.Gsat(obsgps,nav);
    sat.setRcvPosVel(posbl,posbl.gradient(obs.dt));
    obsgps = obsgps.residuals(sat);
    obs.dclk = median(obsgps.L1.resD,2,"omitmissing");
    warning('obs.dclk is estimated. residuals=%.1f',rms(obsgps.L1.resD-obs.dclk,"all","omitmissing"));
end

% Estimate receiver clock
if ismember(phone,["sm-a226b","sm-a505g","sm-a600t","sm-a325f","sm-a217m","sm-a205u","samsunga325g","samsunga32","sm-a505u"])
    obsgps = obs.selectSat(obs.sys==gt.C.SYS_GPS);
    obsgps.L1.P(obsgps.L1.P<1e7|obsgps.L1.P>4e7|obsgps.L1.S<15) = NaN;
    sat = gt.Gsat(obsgps,nav);
    sat.setRcvPosVel(posbl,posbl.gradient(obs.dt));
    obsgps = obsgps.residuals(sat);
    obs.clk = median(obsgps.L1.resPc,2,"omitmissing");
    warning('obs.clk is estimated, estimated. residuals=%.1f',rms(obsgps.L1.resPc-obs.clk,"all","omitmissing"));
end

% Check receiver clock drift
idx = abs(diff(obs.dclk))>50;
if any(idx)
    idxjump = [find(idx); find(idx)+1];
    obs.dclk(idxjump) = NaN;
    warning('obs.dclk is not continued, njump: %d',length(idxjump));
    obs.dclk = fillmissing(obs.dclk,'linear');
end
if any(isnan(obs.dclk))
    nnan = nnz(isnan(obs.dclk));
    warning('obs.dclk has NaN, nNaN: %d',nnan);
    obs.dclk = fillmissing(obs.dclk,'nearest');
end

% Check receiver clock
if any(isnan(obs.clk))
    nnan = nnz(isnan(obs.clk));
    warning('obs.clk has NaN, nNaN: %d',nnan);
    obs.clk = fillmissing(obs.clk,'nearest');
end

% Check clock jump
if ismember(phone,["sm-s908b", "sm-a226b", "samsungs22ultra", "sm-a325f", "samsunga325g", "samsunga32", "pixel7pro","sm-a505u"])
    jumpth = 2000;
elseif ismember(phone,["sm-a205u", "sm-a217m","sm-a505g","sm-a600t","sm-g988b", "pixel6pro"])
    jumpth = 500;
elseif ismember(phone,["pixel4xl","pixel7"])
    jumpth = 100;
else % pixel5, pixel4, mi8
    jumpth = 50;
end
idxjump = find(abs(diff(obs.clk))>jumpth)+1;
if ~isempty(idxjump)
    warning('obs.clk has jump, index: %s',num2str(idxjump'));
    obs.clkjump(idxjump) = true;
end

% Check day cross over
if obs.time.day(1)~=obs.time.day(end)
    warning('Day cross over detected, Day %d-%d',obs.time.day(1),obs.time.day(end));
end

% Check week cross over
if obs.time.week(1)~=obs.time.week(end)
    warning('Week cross over detected, Week %d-%d',obs.time.week(1),obs.time.week(end));
end

%% Save data
save(path+phone+"/phone_data.mat","nav","obs","obsb","timebl","posbl","acc","gyro","mag");
if contains(datapath, "train")
    save(path+phone+"/"+"gt.mat",'timegt','posgt','velgt','yawgt','speedgt');
end
warning('on','backtrace')
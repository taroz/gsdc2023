function obs = gnsslog2obs(file)
%% Convert GNSS log data to GNSS observation object
% Input:  gnss_log.txt captured by GNSSLogger App
% Output: GobsPhone object
% Author: Taro Suzuki

%% Read gnss_log.txt
dataS = readtable(file,'NumHeaderLines',0,'ReadVariableNames',false,'FileType','text');
dataM = readmatrix(file,'NumHeaderLines',0);
dataMi = readmatrix(file,'NumHeaderLines',0,'OutputType','int64');

% Raw
nRaw = size(dataS,2)-1; % 36 or 29
hRaw = {'utcTimeMillis','TimeNanos','LeapSecond','TimeUncertaintyNanos','FullBiasNanos','BiasNanos','BiasUncertaintyNanos','DriftNanosPerSecond','DriftUncertaintyNanosPerSecond','HardwareClockDiscontinuityCount','Svid','TimeOffsetNanos','State','ReceivedSvTimeNanos','ReceivedSvTimeUncertaintyNanos','Cn0DbHz','PseudorangeRateMetersPerSecond','PseudorangeRateUncertaintyMetersPerSecond','AccumulatedDeltaRangeState','AccumulatedDeltaRangeMeters','AccumulatedDeltaRangeUncertaintyMeters','CarrierFrequencyHz','CarrierCycles','CarrierPhase','CarrierPhaseUncertainty','MultipathIndicator','SnrInDb','ConstellationType','AgcDb','BasebandCn0DbHz','FullInterSignalBiasNanos','FullInterSignalBiasUncertaintyNanos','SatelliteInterSignalBiasNanos','SatelliteInterSignalBiasUncertaintyNanos','CodeType','ChipsetElapsedRealtimeNanos'};
hRaw = hRaw(1:nRaw);
Raw = array2table(dataM(strcmp(dataS{:,1},'Raw'),2:nRaw+1),'VariableNames',hRaw);
Rawi = array2table(dataMi(strcmp(dataS{:,1},'Raw'),2:nRaw+1),'VariableNames',hRaw);

% Eliminate invalid rows
idx = Raw.TimeNanos~=0;
Raw = Raw(idx,:);
Rawi = Rawi(idx,:);

% sm-a205u/sm-a600t repeats the same observation twice
if contains(file,["sm-a205u","sm-a600t"])
    [~,idx] = unique(Raw.utcTimeMillis);
    idx = [idx; height(Raw)+1];
    didx = diff(idx)/2;
    halfi = [idx(1:end-1) idx(1:end-1)+didx-1];
    halfidx = false(size(Raw,1),1);
    for i=1:size(halfi,1)
        halfidx(halfi(i,1):halfi(i,2)) = true;
    end
    Raw = Raw(halfidx,:);
    Rawi = Rawi(halfidx,:);
end

%% Process Raw data
% Zero to NaN
idx = Raw.AccumulatedDeltaRangeMeters==0;
Raw.AccumulatedDeltaRangeMeters(idx) = NaN;

% Eliminate unknown SVID GLONASS data
idx = Raw.Svid>24 & Raw.ConstellationType==3;
if any(idx)
    warning("Eliminate unkown SVID GLONASS satellites, Svid=%s",num2str(unique(Raw.Svid(idx))'));
    Raw(idx,:) = [];
    Rawi(idx,:) = [];
end

% Eliminate QZSS data
idx = Raw.ConstellationType==4;
if any(idx)
    warning("Eliminate QZSS satellites, Svid=%s",num2str(unique(Raw.Svid(idx))'));
    Raw(idx,:) = [];
    Rawi(idx,:) = [];
end

% Eliminate SBAS data
idx = Raw.ConstellationType==2;
if any(idx)
    warning("Eliminate SBAS satellites, Svid=%s",num2str(unique(Raw.Svid(idx))'));
    Raw(idx,:) = [];
    Rawi(idx,:) = [];
end

% Eliminate IRNSS data
idx = Raw.ConstellationType==7;
if any(idx)
    warning("Eliminate IRNSS satellites, Svid=%s",num2str(unique(Raw.Svid(idx))'));
    Raw(idx,:) = [];
    Rawi(idx,:) = [];
end

% Eliminate invalid data
idx = Raw.BiasUncertaintyNanos>1e4;
if any(idx)
    warning("Eliminate invalid data from BiasUncertaintyNanos, ninvalid=%d",nnz(idx));
    Raw(idx,:) = [];
    Rawi(idx,:) = [];
end

% Eliminate invalid ReceivedSvTimeNanos
idx = Rawi.ReceivedSvTimeNanos<1e10;
if any(idx)
    warning("Remove invalid ReceivedSvTimeNanos, ninvalid=%d",nnz(idx));
    Raw(idx,:) = [];
    Rawi(idx,:) = [];
end

% Estimate carrier frequency
satall = svconst2sat(Raw.Svid, Raw.ConstellationType);
sysall = rtklib.satsys(satall);
[freqall, ftypeall] = estimatecarrfreq(Raw.CarrierFrequencyHz, sysall);

[sat,~,sind] = unique(satall);
utcmsall = Raw.utcTimeMillis;
[utcms,idx_unique] = unique(utcmsall);

n = size(utcms,1);
nsat = size(sat,2);

% Compute tow_rx
timenanos = Rawi.TimeNanos(idx_unique);
fullbiasnanos = Rawi.FullBiasNanos(idx_unique);
biasnanos = Raw.BiasNanos(idx_unique);
hcdc = Raw.HardwareClockDiscontinuityCount(idx_unique);

% Check time jump
idxjump = find(abs(diff(timenanos))>1e9)+1;
idxbase = ones(size(fullbiasnanos));
for i=1:length(idxjump)
    idxbase(idxjump(i):end) = idxjump(i);
end
% Base fullbiasnano
basebiasnanos = fullbiasnanos(idxbase);
clk = gt.C.CLIGHT*double(fullbiasnanos-basebiasnanos)/1e9;
dclk = gt.C.CLIGHT*Raw.DriftNanosPerSecond(idx_unique)/1e9;

week = floor(double(timenanos - basebiasnanos) / 1e9 / 604800);
tow_rxall = double(timenanos - basebiasnanos - int64(week)*604800*1e9)/1e9 - biasnanos/1e9;

%% Generate GNSS observation struct
obs.n = n;
obs.nsat = nsat;
obs.elapsedns = Raw.ChipsetElapsedRealtimeNanos(idx_unique);
obs.timems = utcms2gtime(utcms);
obs.utcms = utcms;
obs.time = gt.Gtime(tow_rxall,week);
obs.ep = obs.time.ep;
obs.sat = sat;
[obs.sys, obs.prn] = rtklib.satsys(obs.sat);
obs.satstr = rtklib.satno2id(obs.sat);
obs.hcdc = hcdc;
obs.clk = clk;
obs.dclk = dclk;
obs.clkjump = false(n,1);

FTYPE = {'L1','L2','L5','L6','L7','L8','L9'};
for f = FTYPE
    P = NaN(n,nsat);
    L = NaN(n,nsat);
    D = NaN(n,nsat);
    S = NaN(n,nsat);
    I = NaN(n,nsat);
    Perr = NaN(n,nsat);
    Lerr = NaN(n,nsat);
    Derr = NaN(n,nsat);
    Pstat = zeros(n,nsat);
    Lstat = zeros(n,nsat);
    ctype = cell(1,nsat); ctype(:) = {''};
    freq = NaN(1,nsat);
    lam = NaN(1,nsat);
    carrfreq = NaN(n,nsat);
    multipath = NaN(n,nsat);
    fexist = false;

    for i=1:nsat
        isat = sind==i & matches(ftypeall,f{:});
        if any(isat)
            freqsat = unique(freqall(isat));
            assert(isscalar(freqsat),'not same carrier frequency');
            freq(i) = freqsat;
            lam(i) = gt.C.CLIGHT./freq(i);
            code = sysfreq2code(obs.sys(i),freq(i));
            ctype(i) = rtklib.code2obs(code);

            [~,itime] = intersect(utcms,utcmsall(isat));

            timeoffsetnanos = Raw.TimeOffsetNanos(isat);
            tow_rx = tow_rxall(itime) - timeoffsetnanos/1e9;

            tow_tx = double(Rawi.ReceivedSvTimeNanos(isat))/1e9;
            switch obs.sys(i)
                case gt.C.SYS_GLO
                    tow_tx = glot2gpst(tow_tx,tow_rx);
                case gt.C.SYS_CMP
                    tow_tx = tow_tx + 14;
            end
            tow_rx = unwrap_data(tow_rx,3600*24*7);
            tow_tx = unwrap_data(tow_tx,3600*24*7);
            P(itime,i) = (tow_rx - tow_tx) * gt.C.CLIGHT;
            L(itime,i) = Raw.AccumulatedDeltaRangeMeters(isat)./lam(i);
            D(itime,i) = -Raw.PseudorangeRateMetersPerSecond(isat)./lam(i);
            S(itime,i) = Raw.Cn0DbHz(isat);
            Perr(itime,i) = Raw.ReceivedSvTimeUncertaintyNanos(isat) / 1e9 * gt.C.CLIGHT;
            Lerr(itime,i) = Raw.AccumulatedDeltaRangeUncertaintyMeters(isat)./lam(i);
            Derr(itime,i) = Raw.PseudorangeRateUncertaintyMetersPerSecond(isat)./lam(i);
            Pstat(itime,i) = Raw.State(isat);
            Lstat(itime,i) = Raw.AccumulatedDeltaRangeState(isat);
            carrfreq(itime,i) = Raw.CarrierFrequencyHz(isat);
            multipath(itime,i) = Raw.MultipathIndicator(isat);
            fexist = true;
        end
    end
    if fexist
        if contains(file,["sm-a205u","sm-a217m","sm-a505g","sm-a505u","sm-a600t"])
            L = -L;
        end
        if contains(file,"mi8")
            D = interp1(1:n,D,(1:n)+0.5,"linear","extrap");
        end
        obs.(f{:}).P = P;
        obs.(f{:}).L = L;
        obs.(f{:}).D = D;
        obs.(f{:}).S = S;
        obs.(f{:}).I = I;
        obs.(f{:}).ctype = ctype;
        obs.(f{:}).freq = freq;
        obs.(f{:}).lam = lam;
        obs.(f{:}).carrfreq = carrfreq;
        obs.(f{:}).Perr = Perr;
        obs.(f{:}).Lerr = Lerr;
        obs.(f{:}).Derr = Derr;
        obs.(f{:}).Pstat = Pstat;
        obs.(f{:}).Lstat = Lstat;
        obs.(f{:}).multipath = multipath;
    end
end
end

%% Functions
% Svid+ConstellationType to satno
function sat = svconst2sat(sv,const)
sys = const2sys(const)';
sv(sys==gt.C.SYS_QZS) = sv(sys==gt.C.SYS_QZS)-192;
prn = sv';
iqzss = sys==gt.C.SYS_QZS;
prn(iqzss) = prn(iqzss)+192;
sat = rtklib.satno(sys,prn);
end

% ConstellationType to satsys
function sys = const2sys(const)
C = gt.C;
sys = NaN(size(const));
sys(const==1) = C.SYS_GPS; % GPS
sys(const==2) = C.SYS_SBS; % SBAS
sys(const==3) = C.SYS_GLO; % GLONASS
sys(const==4) = C.SYS_QZS; % QZSS
sys(const==5) = C.SYS_CMP; % BeiDou
sys(const==6) = C.SYS_GAL; % Galileo
sys(const==7) = C.SYS_IRN; % IRNSS
end

% GLONASS time to GPS time
function gpst = glot2gpst(glot,gpstref)
dayofweek = floor(gpstref/(3600*24));
gpst = glot + dayofweek*3600*24 - 3600*3 + 18;
dayoffset = dayofweek - floor(gpst/(3600*24));
gpst = gpst + dayoffset*3600*24;
end

% Estimate carrier frequency
function [freq, ftype] = estimatecarrfreq(freq,sys)
C = gt.C;
fgps = [C.FREQ1 C.FREQ2 C.FREQ5];
fqzs = [C.FREQ1 C.FREQ2 C.FREQ5 C.FREQ6];
fgal = [C.FREQ1 C.FREQ5 C.FREQ6 C.FREQ7 C.FREQ8];
fbds = [C.FREQ1 C.FREQ1_CMP C.FREQ5 C.FREQ2_CMP C.FREQ3_CMP];
fglo = [C.FREQ1_GLO+C.DFRQ1_GLO*(-7:6) C.FREQ2_GLO+C.DFRQ2_GLO*(-7:6)];

freq(sys==C.SYS_GPS) = nearest(fgps,freq(sys==C.SYS_GPS));
freq(sys==C.SYS_QZS) = nearest(fqzs,freq(sys==C.SYS_QZS));
freq(sys==C.SYS_GAL) = nearest(fgal,freq(sys==C.SYS_GAL));
freq(sys==C.SYS_CMP) = nearest(fbds,freq(sys==C.SYS_CMP));
freq(sys==C.SYS_GLO) = nearest(fglo,freq(sys==C.SYS_GLO));
ftype = freq2ftype(freq);
end

% Nearest neighbor
function nearv = nearest(v,x)
if isempty(x)
    nearv = [];
    return;
end
[~,idx] = min(abs(v-x),[],2);
nearv = v(idx);
end

% Frequency to frequency type
function ftype = freq2ftype(freq)
C = gt.C;
ftype = cell(size(freq));
[ftype{freq==C.FREQ1}] = deal('L1'); % GPS/QZS/GAL L1
[ftype{freq==C.FREQ2}] = deal('L2'); % GPS/QZS L2
[ftype{freq==C.FREQ5}] = deal('L5'); % GPS/QZS/GAL/BDS L5
[ftype{freq==C.FREQ1_CMP}] = deal('L1'); % BDS L1
[ftype{freq>=C.FREQ1_GLO-7*C.DFRQ1_GLO & freq<=C.FREQ1_GLO+6*C.DFRQ1_GLO}] = deal('L1'); % GLONASS L1
end

% Unwrap data
function d = unwrap_data(d,period)
dd = diff(d);
ijump = -round(dd/period);
jump = period*cumsum(ijump);
d(2:end) = d(2:end)+jump;
end

% Satellite system and frequency to code
function code = sysfreq2code(sys,freq)
C = gt.C;
code = zeros(size(freq));
code(freq==C.FREQ1 & sys==C.SYS_GPS) = C.CODE_L1C; % L1C/A
code(freq==C.FREQ1 & sys==C.SYS_GAL) = C.CODE_L1C; % E1C
code(freq==C.FREQ1 & sys==C.SYS_QZS) = C.CODE_L1C; % L1C/A
code(freq==C.FREQ1 & sys==C.SYS_SBS) = C.CODE_L1C; % L1C/A
code(freq==C.FREQ1 & sys==C.SYS_CMP) = C.CODE_L1D; % B1D
code(freq==C.FREQ1_CMP & sys==C.SYS_CMP) = C.CODE_L2I; % B1_2I
code(freq==C.FREQ5 & sys==C.SYS_GPS) = C.CODE_L5I; % L5I
code(freq==C.FREQ5 & sys==C.SYS_GAL) = C.CODE_L5I; % E5I
code(freq==C.FREQ5 & sys==C.SYS_CMP) = C.CODE_L5P; % B2a for sm-908f
code(freq>=C.FREQ1_GLO-7*C.DFRQ1_GLO &...
    freq<=C.FREQ1_GLO+6*C.DFRQ1_GLO &...
    sys==C.SYS_GLO) = C.CODE_L1C; % G1C/A
end
function obsex = exobs(obs, prm)
%% Exclude observation using signal status
% Author: Taro Suzuki

warning('off','backtrace')
obsex = obs.copy();

% RTKLIB constant
C = gt.C;

% Frequency type
FTYPE = ["L1","L5"];

% Code/Carrier status
LSTATE_SLIP = bitor(2^1,2^2);
LSTATE_HALF = 2^4;
LSTATE_HALFOK = 2^3;
LSTATE_VALID = 2^0;
PSTATE_CODE_LOCK = bitor(2^0,2^10);
PSTATE_TOD_OK = bitor(2^7,2^15);
PSTATE_TOW_OK = bitor(2^3,2^14);

%% Doppler
for f = FTYPE
    if ~isempty(obs.(f))
        Dmask = obs.(f).S<prm.Dmask_sn(f)... % SNR mask
                      | obs.(f).multipath==1;             % Multipath flag from smartphone

        % Exclude Doppler observation
        obsex.(f).D(Dmask) = NaN;
    end
end

%% Pseudorange
for f = FTYPE
    if ~isempty(obs.(f))
        
        Pmask = obs.(f).S<prm.Pmask_sn(f)... % SNR mask     
                      | bitand(obs.(f).Pstat,PSTATE_CODE_LOCK)==0 ... % Tracking report
                      | obs.(f).multipath==1 ...          % Multipath flag from smartphone
                      | obs.(f).P<1e7 ...                 % Exclude abnormal pseudorange
                      | obs.(f).P>4e7;                    % Exclude GEO satellite

        iglo = obs.sys==C.SYS_GLO;  % Tracking report for GLONASS
        Pmask(:, iglo) = Pmask(:, iglo) | bitand(obs.(f).Pstat(:, iglo),PSTATE_TOD_OK)==0;
        Pmask(:,~iglo) = Pmask(:,~iglo) | bitand(obs.(f).Pstat(:,~iglo),PSTATE_TOW_OK)==0;

        % Exclude psedorange observation
        obsex.(f).P(Pmask) = NaN;
    end
end

%% Carrier phase
for f = FTYPE
    if ~isempty(obs.(f))
        Lmask = obs.(f).S<prm.Lmask_sn(f)... % SNR mask
                      | bitand(obs.(f).Lstat,LSTATE_SLIP)~=0 ...  % Tracking report
                      | bitand(obs.(f).Lstat,LSTATE_VALID)==0 ... % Tracking report
                      | obs.(f).multipath==1;                     % Multipath flag from smartphone
        
        % Exclude GLONASS carrier phase in some smartphone
        if ismember(prm.Phone,["sm-a205u","sm-a217m","samsungs22ultra","sm-s908b","sm-a505g","sm-a600t","sm-a505u"])
            Lmask(:,obsex.sys==C.SYS_GLO) = true;
        end

        % Exclude carrier phase observation
        obsex.(f).L(Lmask) = NaN;

        % Exclude abnormal TDCP
        tdcp = [zeros(1,obsex.nsat); diff(obsex.(f).L)];
        Lmask = abs(tdcp)>2e4; % cycle
        if nnz(Lmask)>0
            warning('Exclude carrier phase by TDCP jump n=%d',nnz(Lmask));
        end
        obsex.(f).L(Lmask) = NaN;
    end
end
warning('on','backtrace')
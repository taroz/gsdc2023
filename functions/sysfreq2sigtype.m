function sigtype = sysfreq2sigtype(sys,freq)
%% Satellite system/frequency to signal type for FGO
% Author: Taro Suzuki

sigtype = NaN(size(sys));
if freq=="L1"
    sigtype(sys==gt.C.SYS_GPS) = 0; % GPS L1
    sigtype(sys==gt.C.SYS_GLO) = 1; % GLO G1
    sigtype(sys==gt.C.SYS_GAL) = 2; % GAL E1
    sigtype(sys==gt.C.SYS_CMP) = 3; % BDS B1
end
if freq=="L5"
    sigtype(sys==gt.C.SYS_GPS) = 4; % GPS L5
    sigtype(sys==gt.C.SYS_GAL) = 5; % GAL E5
    sigtype(sys==gt.C.SYS_CMP) = 6; % BDS B2a
end
sigtype(isnan(sigtype)) = 7; % Others

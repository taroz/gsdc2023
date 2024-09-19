function obserr = obserrmodel(obs, sat, prm)
%% Observation error model
% Author: Taro Suzuki

FTYPE = ["L1","L5"];

% Elevation model
sel = sind(sat.el);
el_base = (prm.el_a+prm.el_b./sel);

for f=FTYPE
    if ~isempty(obs.(f))
        % SNR model
        sigtype = sysfreq2sigtype(sat.sys,f);
        sigfactor = prm.sigtype_factor(sigtype+1);
        sn_base = 10.^(-(obs.(f).S-prctile(obs.(f).S,prm.sn_ptile,"all"))/prm.sn_den);
        
        % Pseudorange
        if prm.P_model == "el" % Elevation base
            obserr.(f).P = prm.P_el_ratio*el_base.*sigfactor;
        else % SNR base
            obserr.(f).P = prm.P_sn_ratio*sn_base.*sigfactor;
        end
        
        % Doppler
        if prm.D_model == "el"
            obserr.(f).D = prm.D_el_ratio*el_base;
        else
            obserr.(f).D = prm.D_sn_ratio*sn_base;
        end
        
        % Carrier phase
        if prm.L_model == "el"
            obserr.(f).L = prm.L_el_ratio*el_base.*sigfactor;
        else
            obserr.(f).L = prm.L_sn_ratio*sn_base.*sigfactor;
        end
    end
end

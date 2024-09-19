function obsex = exobs_residuals(obs, sat, clk, dclk, prm)
%% Exclude observation using residuals
% Author: Taro Suzuki

warning('off','backtrace')
obsex = obs.copy();

% frequency type
FTYPE = ["L1","L5"];

medianall = @(x) median(x,'all','omitmissing');
g = findgroups(obs.sys);

%% Doppler
for f = FTYPE
    if ~isempty(obs.(f))
        % Elevation mask
        Dmask = sat.el<prm.Dmask_el(f);
        obsex.(f).D(Dmask) = NaN;
        
        % Doppler residuals
        Dres = obsex.(f).resD-dclk/obsex.dt; % Residuals
        Dmask = abs(Dres)>prm.Dmask_res(f);
        obsex.(f).D(Dmask) = NaN;

        % Plot Doppler residuals
        if prm.plotflag
            figure;
            plot(Dres,'.');
            grid on; hold on;
            plot([1 obsex.n], [prm.Dmask_res(f) prm.Dmask_res(f)],'r-','LineWidth',2);
            plot([1 obsex.n],-[prm.Dmask_res(f) prm.Dmask_res(f)],'r-','LineWidth',2);
            title('Doppler residuals')
        end
    end
end

%% Pseudorange
for f = FTYPE
    if ~isempty(obs.(f))
        % Elevation mask
        Pmask = sat.el<prm.Pmask_el(f);
        obsex.(f).P(Pmask) = NaN;

        % Pseudorange-Doppler difference
        if ~ismember(prm.Phone,["sm-a205u","sm-a505u"])
            % (-lam*(D2-D1)/2*dt)-(P2-P1)
            dDP = -(obsex.(f).D(1:end-1,:)+obsex.(f).D(2:end,:)).*obsex.(f).lam/2*obsex.dt-diff(obsex.(f).P);
            Pmask = abs(dDP)>prm.Pmask_dDP(f);
            Pmask = [Pmask; false(1,obsex.nsat)] | [false(1,obsex.nsat); Pmask];
            obsex.(f).P(Pmask) = NaN;

            % Plot Pseudorange/Doppler difference
            if prm.plotflag
                figure;
                plot(dDP,'.');
                grid on; hold on;
                plot([1 obsex.n], [prm.Pmask_dDP(f) prm.Pmask_dDP(f)],'r-','LineWidth',2);
                plot([1 obsex.n],-[prm.Pmask_dDP(f) prm.Pmask_dDP(f)],'r-','LineWidth',2);
                title('Pseudorange-Doppler difference')
            end
        end
        
        % Pseudorange residuals
        isb = splitapply(medianall,obsex.(f).resPc-clk,g); % Inter system bias
        Pres = obsex.(f).resPc-clk-isb(g); % Residuals
        Pmask = abs(Pres)>prm.Pmask_res(f);
        obsex.(f).P(Pmask) = NaN;

        % Plot pseudorange residuals
        if prm.plotflag
            figure;
            plot(Pres);
            grid on; hold on;
            plot([1 obsex.n], [prm.Pmask_res(f) prm.Pmask_res(f)],'r-','LineWidth',2);
            plot([1 obsex.n],-[prm.Pmask_res(f) prm.Pmask_res(f)],'r-','LineWidth',2);
            title('Pseudorange residuals')
        end
    end
end

%% Carrier phase
for f = FTYPE
    if ~isempty(obs.(f))
        % Elevation mask
        Lmask = sat.el<prm.Lmask_el(f);
        obsex.(f).L(Lmask) = NaN;
        
        % Carrier phase-Doppler difference
        % (-lam*(D2-D1)/2*dt)-lam*(L2-L1)
        dDL = -(obsex.(f).D(1:end-1,:)+obsex.(f).D(2:end,:)).*obsex.(f).lam/2*obsex.dt-diff(obsex.(f).L).*obsex.(f).lam;
        
        % Carrier phase offset in some smartphone
        if ismember(prm.Phone,["sm-a205u","sm-a217m","sm-a505g","sm-a600t","sm-a505u"])
            dDL = dDL-prm.Loffset;
        end
        Lmask = abs(dDL)>prm.Lmask_dDL(f);
        Lmask = [Lmask; false(1,obsex.nsat)] | [false(1,obsex.nsat); Lmask];
        obsex.(f).L(Lmask) = NaN;

        % Plot Carrier phase/Doppler difference
        if prm.plotflag
            figure;
            plot(dDL,'.');
            grid on; hold on;
            plot([1 obsex.n], [prm.Lmask_dDL(f) prm.Lmask_dDL(f)],'r-','LineWidth',2);
            plot([1 obsex.n],-[prm.Lmask_dDL(f) prm.Lmask_dDL(f)],'r-','LineWidth',2);
            title('Carrier phase-Doppler difference')
        end
    end
end
warning('on','backtrace')
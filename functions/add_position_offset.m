function pos = add_position_offset(posest, rpyest, phone)
%% Add position offset to smartphone position
% Author: Taro Suzuki
arguments
    posest
    rpyest
    phone
end

% Small position offset for each smartphone
if contains(phone, "mi8")
    offsetRL = 0.25;
    offsetUD = -0.35;
elseif ismember(phone, "sm-g988")
    offsetRL = 0.20;
    offsetUD = -0.05;
elseif contains(phone, ["sm","samsung"])
    offsetRL = 0.30;
    offsetUD = -0.25;
elseif ismember(phone, "pixel6pro")
    offsetRL = -0.20;
    offsetUD = -0.15;
elseif contains(phone, "pixel7")
    offsetRL = -0.10;
    offsetUD = -0.20;
elseif contains(phone,"pixel4")
    offsetRL = -0.00;
    offsetUD = -0.15;
elseif contains(phone,"pixel5")
    offsetRL = -0.10;
    offsetUD = -0.30;
else
    error("phone:%d\n", phone)
end

% Convert offset to smartphone coordinate
R = eul2rotm(rpyest-[0 0 pi]);
enu  = posest.enu + squeeze(pagemtimes(R, repmat([offsetUD offsetRL 0]', [1 1 posest.n])))';
pos = gt.Gpos(enu, 'enu', posest.orgllh, 'llh');
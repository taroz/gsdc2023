function ango = wrapTo180(angi)
%% Implementation of wrapTo180 without the Mapping Toolbox
% Author: Taro Suzuki

ango = mod(angi+180,360)-180;
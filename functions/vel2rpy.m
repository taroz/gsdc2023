function rpy = vel2rpy(venu, prm)
%% Calculate the initial value of attitude from velocity
% Author: Taro Suzuki

% Smoothing velocity
venu = smoothdata(venu, 1, "movmean", prm.velocity_smooth_window);

% Compute heading
head = atan2d(venu(:,2),venu(:,1));
head(vecnorm(venu,2,2)<prm.velocity_threshold) = NaN; % NaN for low speed and then interpolate
head = fillmissing(head,'nearest','EndValues','nearest'); % Interpolation

% Roll, Pitch, Yaw in radians
rpy = deg2rad(wrapTo180([zeros(size(venu,1),2) head+180]));
function [score, dist] = score(ll1,ll2)
%% Compute Haversine distance and score
% Author: Taro Suzuki

ll1r = [deg2rad(ll1(:,1)) deg2rad(ll1(:,2))];
ll2r = [deg2rad(ll2(:,1)) deg2rad(ll2(:,2))];
dll = ll2r-ll1r;

% Haversine distance
% https://en.wikipedia.org/wiki/Haversine_formula
dist = 2*gt.C.RE_WGS84*asin(sqrt(sin(dll(:,1)/2).^2 + cos(ll1r(:,1)).*cos(ll2r(:,1)).*sin(dll(:,2)/2).^2));

% Mean of 50% and 95% error
score = (prctile(dist,50)+prctile(dist,95))/2;


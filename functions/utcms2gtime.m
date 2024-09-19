function gtime = utcms2gtime(utcms)
%% UTC mili second to gt.Gtime
% Author: Taro Suzuki

d = datetime(utcms/1000,'ConvertFrom','posixtime');
epoch_utc = [d.Year d.Month d.Day d.Hour d.Minute d.Second];
gtime = gt.Gtime(epoch_utc, 1);
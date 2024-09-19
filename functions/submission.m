function submission(result_path, settings)
%% Generate submission file
% Author: Taro Suzuki

%% Setting
n = height(settings);

%% Sample submission
sub = readtable('./results/sample_submission.csv');
sub.LatitudeDegrees(:) = NaN;
sub.LongitudeDegrees(:) = NaN;

%% Read optimization results
for i=1:n
    course = settings.Course(i);
    phone = settings.Phone(i);
    fprintf('Course: %s, Phone: %s\n', course, phone);
    load("./dataset_2023/test/"+course+"/"+phone+"/result_gnss_imu.mat");
    load("./dataset_2023/test/"+course+"/"+phone+"/phone_data.mat");

    idx = sub.tripId==course+"/"+phone;
    utc = sub.UnixTimeMillis(idx);
    n = length(utc);
    if n~=posest.n
        warning("number of epochs: sub=%d, posest=%d", n, posest.n);
        [~,idxsel] = intersect(obs.utcms, utc);
        posest = posest.select(idxsel);

        if any(isnan(posest.llh), "all")
            warning("NaN!!! number of NaNs: %d", nnz(isnan(posest.llh(:,1))));
            llh = fillmissing(posest.llh, 'nearest');
            posest = gt.Gpos(llh, 'llh');
        end

        if n~=posest.n
            warning("extrapolation!!! number of epochs: sub=%d, posest=%d", n, posest.n);
            llh = interp1(obs.utcms, posest.llh,utc, "nearest", "extrap");
            posest = gt.Gpos(llh, 'llh');
        end
    end

    sub.LatitudeDegrees(idx) = posest.lat;
    sub.LongitudeDegrees(idx) = posest.lon;
end

% Assertion
assert(~any(isnan(sub.LatitudeDegrees)), 'Submission has NaN!');
assert(~any(isnan(sub.LongitudeDegrees)), 'Submission has NaN!');

%% Write submission
dstrs = split(result_path, "/");
writetable(sub, result_path+"/submission_"+dstrs(end-1)+".csv");

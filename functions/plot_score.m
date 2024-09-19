function score_est = plot_score(posest, posbl, posgt)
%% Plot score for tarining data
% Author: Taro Suzuki

% Compute score
[score_bl, dist_bl] = score(posbl.llh, posgt.llh);
[score_est, dist_est] = score(posest.llh, posgt.llh);
fprintf('Score Baseline %.4f [m]\n',score_bl)
fprintf('Score FGO      %.4f [m]\n',score_est)

% Plot
figure;
plot(dist_bl,'b.-'); grid on; hold on;
plot(dist_est,'r.-');
legstr = {['Baseline, Score: ' num2str(score_bl,'%.4f') ' m'],...
          ['FGO     , Score: ' num2str(score_est,'%.4f') ' m']};
legend(legstr);
title('Distance error');
ylabel('Distance error [m]');
ylim([0 30]);
drawnow;
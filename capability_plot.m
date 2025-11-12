%Capability plot- Simulation 5
n_dirs = 36;                        % number of directions
directions = linspace(0, 360 - 360/n_dirs, n_dirs);
TU_avg_all = zeros(size(directions));

for k = 1:length(directions)
    direction = directions(k);

    % Kjør simulering, og overfør 'direction' som parameter
    simOut = sim("part2_ny.slx", ...
        'ReturnWorkspaceOutputs', 'on', ...
        'SrcWorkspace', 'current', ...
        'SaveOutput', 'on', ...
        'SaveFormat', 'StructureWithTime');

    % Hent TU direkte fra simOut-struktur 
    TU = simOut.get('TU').signals.values;

    % Beregn snittverdien av thrust utilization
    TU_avg_all(k) = mean(TU);
end
directions_rad = deg2rad([directions directions(1)]);
TU_plot = TU_avg_all;
TU_plot = [TU_plot TU_plot(1)];
figure;

%plotter polarplott 
polarplot(directions_rad, TU_plot, 'LineWidth', 2, 'Color', [0 0.447 0.741]);
hold on;
polarplot(directions_rad, TU_plot, 'o', 'MarkerFaceColor', 'w', 'MarkerEdgeColor', [0 0.447 0.741]);
title('Average Thrust Utilization vs Environment Direction');
rlim([0 100]);                      
thetaticks(0:30:330);
ax = gca;
ax.ThetaZeroLocation = 'top';      
ax.ThetaDir = 'clockwise';        
grid on;

%plott for å ha limitazions for 3 m og 3 grader. 
% direction = 10;
% 
% n_dirs = 36;
% directions = linspace(0, 360 - 360/n_dirs, n_dirs);
% 
% TU_avg_all = zeros(1, n_dirs);
% pos_err_max = zeros(1, n_dirs);
% yaw_err_max = zeros(1, n_dirs);
% 
% pos_tolerance = 3; % m
% yaw_tolerance = 3*pi/180; % rad
% 
% for k = 1:length(directions)
%     direction = directions(k);
%     assignin('base','direction',direction);
%     sim('part2_ny');
% 
% 
%     eta_data = simOut.actual_values;
%     eta_data = reshape(permute(eta_data, [3 1 2]), [], 6);
%     eta_data = eta_data(:, 1:3);
%     eta_ref_data = simOut.reference_eta;
%     t_eta = simOut.tout;
%     t_ref = simOut.tout;
% 
%     % Interpolation to same time base
%     t_common = linspace(min([t_eta; t_ref]), max([t_eta; t_ref]), 1001);
%     eta_i = interp1(t_eta, eta_data, t_common, 'linear', 'extrap');
%     eta_ref_i = interp1(t_ref, eta_ref_data, t_common, 'linear', 'extrap');
% 
% 
%     pos_error = vecnorm(eta_ref_i(:,1:2) - eta_i(:,1:2), 2, 2);
%     yaw_error = abs(wrapToPi(eta_ref_i(:,3) - eta_i(:,3)));
% 
%     TU_avg_all(k)  = mean(TU.signals.values(:));
%     pos_err_max(k) = mean(pos_error);
%     yaw_err_max(k) = mean(yaw_error);
% end
% 
% % tolerance-based filtering
% valid = (pos_err_max <= pos_tolerance) & (yaw_err_max <= yaw_tolerance);
% TU_plot = TU_avg_all;
% TU_plot(~valid) = NaN;  % removing points for which deviation exceeds tolerance
% 
% directions_rad = deg2rad([directions directions(1)]);
% TU_plot = [TU_plot TU_plot(1)];
% 
% 
% figure;
% polarplot(directions_rad, TU_plot, 'LineWidth', 2, 'Color', [0 0.447 0.741]);
% hold on;
% polarplot(directions_rad, TU_plot, 'o', 'MarkerFaceColor', 'w', 'MarkerEdgeColor', [0 0.447 0.741]);
% title('Average Thrust Utilization (within 3m & 3° constraints) vs Environment Direction');
% rlim([0 100]);
% thetaticks(0:30:330);
% ax = gca;
% ax.ThetaZeroLocation = 'top';
% ax.ThetaDir = 'clockwise';
% grid on;
function plotSimulationResults(simType, simOut, xd_setpoint, t, dpMode, varargin)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% plotSimulationResults()                                                 %
%                                                                         %
% Plots results for different simulation scenarios in Project Part 2      %
%                                                                         %
% Inputs:                                                                 %
%   simType      - Simulation type (1-7)                                  %
%   simOut       - Simulink simulation output structure                   %
%   xd_setpoint  - Reference trajectory [6 x N]                           %
%   t            - Time vector for reference                              %
%   dpMode       - DP mode (1-4)                                          %
%   varargin     - Additional inputs depending on simulation type         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Extract simulation outputs
t_ship = simOut.tout;
X_ship = reshape(simOut.actual_values, 6, [])';

% Extract vessel states (positions & heading)
x_ship   = X_ship(:,1);   % surge position (x)
y_ship   = X_ship(:,2);   % sway position (y)
psi_ship = X_ship(:,3);   % heading (yaw)

% Extract vessel states (velocities)
u_ship   = X_ship(:,4);   % surge rate (u)
v_ship   = X_ship(:,5);   % sway rate (v)
r_ship   = X_ship(:,6);   % yaw rate (r)

% Extract reference from xd_setpoint
x_ref_full   = xd_setpoint(1,:).';
y_ref_full   = xd_setpoint(2,:).';
psi_ref_full = xd_setpoint(3,:).';
u_ref_full   = xd_setpoint(4,:).';
v_ref_full   = xd_setpoint(5,:).';
r_ref_full   = xd_setpoint(6,:).';

% Interpolate references onto t_ship
x_ref   = interp1(t, x_ref_full,   t_ship, 'linear', 'extrap');
y_ref   = interp1(t, y_ref_full,   t_ship, 'linear', 'extrap');
psi_ref = interp1(t, psi_ref_full, t_ship, 'linear', 'extrap');
u_ref   = interp1(t, u_ref_full,   t_ship, 'linear', 'extrap');
v_ref   = interp1(t, v_ref_full,   t_ship, 'linear', 'extrap');
r_ref   = interp1(t, r_ref_full,   t_ship, 'linear', 'extrap');

% Unwrap headings and convert to degrees
psi_ship_u   = unwrap(psi_ship);
psi_ref_u    = unwrap(psi_ref);
psi_ship_deg = rad2deg(psi_ship_u);
psi_ref_deg  = rad2deg(psi_ref_u);
r_ship_deg   = rad2deg(r_ship);
r_ref_deg    = rad2deg(r_ref);

switch simType
    case 1
        % Simulation 1 - Environmental Loads
        plotEnvironmentalLoads(t_ship, x_ship, y_ship, psi_ship_deg);
        
    case 2
        % Simulation 2 - DP and Thrust Allocation
        thrusterFault = varargin{1}; % Get thruster fault flag
        plotDPThrust(t_ship, x_ship, y_ship, psi_ship_deg, ...
                     x_ref, y_ref, psi_ref_deg, ...
                     u_ship, v_ship, r_ship_deg, ...
                     u_ref, v_ref, r_ref_deg, ...
                     simOut, dpMode, thrusterFault);
        
    case 3
        % Simulation 3 - DP and Environmental Forces
        plotDPEnvironmental(t_ship, x_ship, y_ship, psi_ship_deg, ...
                           x_ref, y_ref, psi_ref_deg, ...
                           u_ship, v_ship, r_ship_deg, ...
                           u_ref, v_ref, r_ref_deg, dpMode);
        
    case 4
        % Simulation 4 - Observer Selection
        observerData = varargin{1}; % Structure with observer comparison data
        plotObserverComparison(t_ship, X_ship, observerData);
        
    case 5
        % Simulation 5 - Capability Plot
        capabilityData = varargin{1}; % Structure with capability plot data
        plotCapability(capabilityData);
        
    case 6
        % Simulation 6 - Observer Robustness
        observerType = varargin{1}; % Observer type name
        plotObserverRobustness(t_ship, x_ship, y_ship, psi_ship_deg, ...
                              x_ref, y_ref, psi_ref_deg, observerType);
        
    case 7
        % Simulation 7 - Custom DP System Functionality
        plotCustomSimulation(t_ship, x_ship, y_ship, psi_ship_deg, ...
                            x_ref, y_ref, psi_ref_deg, ...
                            u_ship, v_ship, r_ship_deg, ...
                            u_ref, v_ref, r_ref_deg, simOut);
        
    otherwise
        error('Invalid simulation type. Must be 1-7.');
end

end

%% Simulation 1 - Environmental Loads
function plotEnvironmentalLoads(t, x, y, psi)
    figure('Name','Simulation 1 - Environmental Loads','Position',[100 100 1200 800]);
    
    % Position and heading time plots
    subplot(3,1,1)
    plot(t, x, 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title('Surge Position - Environmental Drift');
    xlabel('Time [s]');
    
    subplot(3,1,2)
    plot(t, y, 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title('Sway Position - Environmental Drift');
    xlabel('Time [s]');
    
    subplot(3,1,3)
    plot(t, psi, 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); 
    title('Heading - Environmental Drift');
    
    % XY trajectory plot
    figure('Name','Simulation 1 - XY Trajectory','Position',[150 150 800 800]);
    plot(y, x, 'LineWidth', 2); hold on;
    plot(y(1), x(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(y(end), x(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    grid on; axis equal;
    xlabel('y [m]'); ylabel('x [m]');
    title('Vessel Drift Trajectory (XY-Plot)');
    legend('Trajectory', 'Start', 'End', 'Location', 'best');
end

%% Simulation 2 - DP and Thrust Allocation
function plotDPThrust(t, x, y, psi, x_ref, y_ref, psi_ref, ...
                      u, v, r, u_ref, v_ref, r_ref, simOut, dpMode, thrusterFault)
    
    faultStr = '';
    if thrusterFault
        faultStr = ' (Thrusters 2&4 Failed)';
    end
    
    % Position and heading
    figure('Name',['Simulation 2 - Position & Heading' faultStr],'Position',[100 100 1200 800]);
    
    subplot(3,1,1)
    plot(t, x, 'LineWidth', 1.5); hold on;
    plot(t, x_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title(['Surge Position' faultStr]);
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,2)
    plot(t, y, 'LineWidth', 1.5); hold on;
    plot(t, y_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title(['Sway Position' faultStr]);
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,3)
    plot(t, psi, 'LineWidth', 1.5); hold on;
    plot(t, psi_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); 
    title(['Heading' faultStr]);
    legend('Ship','Reference','Location','best');
    
    % Velocities
    figure('Name',['Simulation 2 - Velocities' faultStr],'Position',[150 150 1200 800]);
    
    subplot(3,1,1)
    plot(t, u, 'LineWidth', 1.5); hold on;
    plot(t, u_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('u [m/s]'); title(['Surge Rate' faultStr]);
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,2)
    plot(t, v, 'LineWidth', 1.5); hold on;
    plot(t, v_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('v [m/s]'); title(['Sway Rate' faultStr]);
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,3)
    plot(t, r, 'LineWidth', 1.5); hold on;
    plot(t, r_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('r [deg/s]'); xlabel('Time [s]'); 
    title(['Yaw Rate' faultStr]);
    legend('Ship','Reference','Location','best');
    
    % XY trajectory
    figure('Name',['Simulation 2 - XY Trajectory' faultStr],'Position',[200 200 800 800]);
    plot(y, x, 'LineWidth', 2); hold on;
    plot(y_ref, x_ref, '--', 'LineWidth', 2);
    plot(y(1), x(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot(y_ref(1), x_ref(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    grid on; axis equal;
    xlabel('y [m]'); ylabel('x [m]');
    title(['XY Trajectory' faultStr]);
    legend('Ship', 'Reference', 'Start', 'Location', 'best');
    
    % Plot forces if available
    if isfield(simOut, 'tau_d') && isfield(simOut, 'tau_act')
        plotForces(simOut, thrusterFault);
    end
end

%% Plot Forces (for Simulation 2)
function plotForces(simOut, thrusterFault)
    faultStr = '';
    if thrusterFault
        faultStr = ' (Thrusters 2&4 Failed)';
    end
    
    % Extract force data
    t = simOut.tout;
    tau_d = simOut.tau_d;     % Desired force from controller [N x 3]
    tau_act = simOut.tau_act; % Actual force from thrusters [N x 3]
    
    % Reshape if needed
    if size(tau_d, 2) ~= 3
        tau_d = reshape(tau_d, [], 3);
        tau_act = reshape(tau_act, [], 3);
    end
    
    figure('Name',['Simulation 2 - Force Comparison' faultStr],'Position',[250 250 1200 800]);
    
    subplot(3,1,1)
    plot(t, tau_d(:,1)/1e3, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, tau_act(:,1)/1e3, 'r--', 'LineWidth', 1.5);
    grid on; ylabel('Force [kN]'); 
    title(['Surge Force' faultStr]);
    legend('Desired', 'Actual', 'Location', 'best');
    
    subplot(3,1,2)
    plot(t, tau_d(:,2)/1e3, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, tau_act(:,2)/1e3, 'r--', 'LineWidth', 1.5);
    grid on; ylabel('Force [kN]'); 
    title(['Sway Force' faultStr]);
    legend('Desired', 'Actual', 'Location', 'best');
    
    subplot(3,1,3)
    plot(t, tau_d(:,3)/1e6, 'b-', 'LineWidth', 1.5); hold on;
    plot(t, tau_act(:,3)/1e6, 'r--', 'LineWidth', 1.5);
    grid on; ylabel('Moment [MNm]'); xlabel('Time [s]');
    title(['Yaw Moment' faultStr]);
    legend('Desired', 'Actual', 'Location', 'best');
    
    % Plot individual thruster forces if available
    if isfield(simOut, 'f_thrusters')
        plotThrusterForces(simOut, thrusterFault);
    end
end

%% Plot Individual Thruster Forces
function plotThrusterForces(simOut, thrusterFault)
    faultStr = '';
    if thrusterFault
        faultStr = ' (Thrusters 2&4 Failed)';
    end
    
    t = simOut.tout;
    f_thrust = simOut.f_thrusters; % [N x 5] thruster forces
    
    if size(f_thrust, 2) ~= 5
        f_thrust = reshape(f_thrust, [], 5);
    end
    
    figure('Name',['Simulation 2 - Individual Thruster Forces' faultStr],'Position',[300 300 1200 900]);
    
    thrusterNames = {'Thruster 1 (Bow)', 'Thruster 2 (Azimuth)', ...
                     'Thruster 3 (Bow)', 'Thruster 4 (Stern Azimuth)', ...
                     'Thruster 5 (Stern Azimuth)'};
    
    for i = 1:5
        subplot(3,2,i)
        plot(t, f_thrust(:,i)/1e3, 'LineWidth', 1.5);
        grid on; ylabel('Force [kN]');
        title(thrusterNames{i});
        if i >= 4
            xlabel('Time [s]');
        end
        
        % Add horizontal line at max thrust
        maxThrust = [125, 150, 125, 300, 300];
        yline(maxThrust(i), 'r--', 'Max', 'LineWidth', 1.5);
        yline(-maxThrust(i), 'r--', 'LineWidth', 1.5);
    end
end

%% Simulation 3 - DP and Environmental Forces
function plotDPEnvironmental(t, x, y, psi, x_ref, y_ref, psi_ref, ...
                             u, v, r, u_ref, v_ref, r_ref, dpMode)
    
    % Position and heading
    figure('Name','Simulation 3 - Position & Heading (With Environment)','Position',[100 100 1200 800]);
    
    subplot(3,1,1)
    plot(t, x, 'LineWidth', 1.5); hold on;
    plot(t, x_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title('Surge Position (With Environment)');
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,2)
    plot(t, y, 'LineWidth', 1.5); hold on;
    plot(t, y_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title('Sway Position (With Environment)');
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,3)
    plot(t, psi, 'LineWidth', 1.5); hold on;
    plot(t, psi_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); 
    title('Heading (With Environment)');
    legend('Ship','Reference','Location','best');
    
    % Velocities
    figure('Name','Simulation 3 - Velocities (With Environment)','Position',[150 150 1200 800]);
    
    subplot(3,1,1)
    plot(t, u, 'LineWidth', 1.5); hold on;
    plot(t, u_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('u [m/s]'); title('Surge Rate (With Environment)');
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,2)
    plot(t, v, 'LineWidth', 1.5); hold on;
    plot(t, v_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('v [m/s]'); title('Sway Rate (With Environment)');
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,3)
    plot(t, r, 'LineWidth', 1.5); hold on;
    plot(t, r_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('r [deg/s]'); xlabel('Time [s]'); 
    title('Yaw Rate (With Environment)');
    legend('Ship','Reference','Location','best');
    
    % XY trajectory
    figure('Name','Simulation 3 - XY Trajectory (With Environment)','Position',[200 200 800 800]);
    plot(y, x, 'LineWidth', 2); hold on;
    plot(y_ref, x_ref, '--', 'LineWidth', 2);
    plot(y(1), x(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    grid on; axis equal;
    xlabel('y [m]'); ylabel('x [m]');
    title('XY Trajectory (With Environment)');
    legend('Ship', 'Reference', 'Start', 'Location', 'best');
    
    % Compute tracking errors
    plotTrackingErrors(t, x, y, psi, x_ref, y_ref, psi_ref);
end

%% Plot Tracking Errors
function plotTrackingErrors(t, x, y, psi, x_ref, y_ref, psi_ref)
    figure('Name','Tracking Errors','Position',[250 250 1200 800]);
    
    err_x = x - x_ref;
    err_y = y - y_ref;
    err_psi = angdiff(deg2rad(psi), deg2rad(psi_ref));
    err_psi = rad2deg(err_psi);
    
    subplot(3,1,1)
    plot(t, err_x, 'LineWidth', 1.5);
    grid on; ylabel('Error [m]'); title('Surge Position Error');
    
    subplot(3,1,2)
    plot(t, err_y, 'LineWidth', 1.5);
    grid on; ylabel('Error [m]'); title('Sway Position Error');
    
    subplot(3,1,3)
    plot(t, err_psi, 'LineWidth', 1.5);
    grid on; ylabel('Error [deg]'); xlabel('Time [s]');
    title('Heading Error');
end

%% Simulation 4 - Observer Comparison
function plotObserverComparison(t, X_ship, observerData)
    % observerData should contain:
    %   - obs1_states: Observer 1 estimates
    %   - obs2_states: Observer 2 estimates
    %   - obs1_name: Observer 1 name
    %   - obs2_name: Observer 2 name
    %   - with_waves: boolean
    
    x_true = X_ship(:,1);
    y_true = X_ship(:,2);
    psi_true = X_ship(:,3);
    u_true = X_ship(:,4);
    v_true = X_ship(:,5);
    r_true = X_ship(:,6);
    
    waveStr = '';
    if observerData.with_waves
        waveStr = ' (With Waves)';
    else
        waveStr = ' (No Waves)';
    end
    
    % Observer 1 comparison
    figure('Name',['Simulation 4 - ' observerData.obs1_name waveStr],'Position',[100 100 1400 900]);
    
    subplot(3,2,1)
    plot(t, x_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs1_states(:,1), 'b--', 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title('Surge Position');
    legend('True', observerData.obs1_name, 'Location', 'best');
    
    subplot(3,2,3)
    plot(t, y_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs1_states(:,2), 'b--', 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title('Sway Position');
    legend('True', observerData.obs1_name, 'Location', 'best');
    
    subplot(3,2,5)
    plot(t, rad2deg(psi_true), 'k-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(observerData.obs1_states(:,3)), 'b--', 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); title('Heading');
    legend('True', observerData.obs1_name, 'Location', 'best');
    
    subplot(3,2,2)
    plot(t, u_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs1_states(:,4), 'b--', 'LineWidth', 1.5);
    grid on; ylabel('u [m/s]'); title('Surge Velocity');
    legend('True', observerData.obs1_name, 'Location', 'best');
    
    subplot(3,2,4)
    plot(t, v_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs1_states(:,5), 'b--', 'LineWidth', 1.5);
    grid on; ylabel('v [m/s]'); title('Sway Velocity');
    legend('True', observerData.obs1_name, 'Location', 'best');
    
    subplot(3,2,6)
    plot(t, rad2deg(r_true), 'k-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(observerData.obs1_states(:,6)), 'b--', 'LineWidth', 1.5);
    grid on; ylabel('r [deg/s]'); xlabel('Time [s]'); title('Yaw Rate');
    legend('True', observerData.obs1_name, 'Location', 'best');
    
    % Observer 2 comparison
    figure('Name',['Simulation 4 - ' observerData.obs2_name waveStr],'Position',[150 150 1400 900]);
    
    subplot(3,2,1)
    plot(t, x_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs2_states(:,1), 'r--', 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title('Surge Position');
    legend('True', observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,3)
    plot(t, y_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs2_states(:,2), 'r--', 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title('Sway Position');
    legend('True', observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,5)
    plot(t, rad2deg(psi_true), 'k-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(observerData.obs2_states(:,3)), 'r--', 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]'); title('Heading');
    legend('True', observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,2)
    plot(t, u_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs2_states(:,4), 'r--', 'LineWidth', 1.5);
    grid on; ylabel('u [m/s]'); title('Surge Velocity');
    legend('True', observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,4)
    plot(t, v_true, 'k-', 'LineWidth', 1.5); hold on;
    plot(t, observerData.obs2_states(:,5), 'r--', 'LineWidth', 1.5);
    grid on; ylabel('v [m/s]'); title('Sway Velocity');
    legend('True', observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,6)
    plot(t, rad2deg(r_true), 'k-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(observerData.obs2_states(:,6)), 'r--', 'LineWidth', 1.5);
    grid on; ylabel('r [deg/s]'); xlabel('Time [s]'); title('Yaw Rate');
    legend('True', observerData.obs2_name, 'Location', 'best');
    
    % Direct comparison
    plotObserverErrors(t, X_ship, observerData);
end

%% Plot Observer Errors
function plotObserverErrors(t, X_ship, observerData)
    figure('Name','Simulation 4 - Observer Error Comparison','Position',[200 200 1400 900]);
    
    err1 = X_ship - observerData.obs1_states;
    err2 = X_ship - observerData.obs2_states;
    
    subplot(3,2,1)
    plot(t, err1(:,1), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, err2(:,1), 'r-', 'LineWidth', 1.5);
    grid on; ylabel('Error [m]'); title('Surge Position Error');
    legend(observerData.obs1_name, observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,3)
    plot(t, err1(:,2), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, err2(:,2), 'r-', 'LineWidth', 1.5);
    grid on; ylabel('Error [m]'); title('Sway Position Error');
    legend(observerData.obs1_name, observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,5)
    plot(t, rad2deg(err1(:,3)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(err2(:,3)), 'r-', 'LineWidth', 1.5);
    grid on; ylabel('Error [deg]'); xlabel('Time [s]'); title('Heading Error');
    legend(observerData.obs1_name, observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,2)
    plot(t, err1(:,4), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, err2(:,4), 'r-', 'LineWidth', 1.5);
    grid on; ylabel('Error [m/s]'); title('Surge Velocity Error');
    legend(observerData.obs1_name, observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,4)
    plot(t, err1(:,5), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, err2(:,5), 'r-', 'LineWidth', 1.5);
    grid on; ylabel('Error [m/s]'); title('Sway Velocity Error');
    legend(observerData.obs1_name, observerData.obs2_name, 'Location', 'best');
    
    subplot(3,2,6)
    plot(t, rad2deg(err1(:,6)), 'b-', 'LineWidth', 1.5); hold on;
    plot(t, rad2deg(err2(:,6)), 'r-', 'LineWidth', 1.5);
    grid on; ylabel('Error [deg/s]'); xlabel('Time [s]'); title('Yaw Rate Error');
    legend(observerData.obs1_name, observerData.obs2_name, 'Location', 'best');
    
    % Compute RMS errors
    rmse1 = sqrt(mean(err1.^2));
    rmse2 = sqrt(mean(err2.^2));
    
    fprintf('\n========== Observer RMSE Comparison ==========\n');
    fprintf('%s RMSE: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
            observerData.obs1_name, rmse1);
    fprintf('%s RMSE: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n', ...
            observerData.obs2_name, rmse2);
    fprintf('==============================================\n\n');
end

%% Simulation 5 - Capability Plot
function plotCapability(capData)
    % capData should contain:
    %   - angles: environmental directions [deg]
    %   - thrust_util: average thrust utilization [%]
    %   - thrust_util_constrained: with position/heading constraints
    %   - max_pos_error: maximum position errors [m]
    %   - max_heading_error: maximum heading errors [deg]
    
    % Plot 1: Full capability plot
    figure('Name','Simulation 5 - Capability Plot','Position',[100 100 800 800]);
    polarplot(deg2rad(capData.angles), capData.thrust_util, '-o', 'LineWidth', 2, 'MarkerSize', 6);
    hold on;
    
    % Add constraint line at 100%
    rlim([0 110]);
    rticks(0:10:110);
    thetaticks(0:30:330);
    title('Average Thrust Utilization vs Environmental Direction');
    grid on;
    
    % Plot 2: Capability plot with constraints
    figure('Name','Simulation 5 - Capability Plot (With Constraints)','Position',[150 150 800 800]);
    polarplot(deg2rad(capData.angles), capData.thrust_util_constrained, '-o', ...
              'LineWidth', 2, 'MarkerSize', 6);
    hold on;
    
    % Add constraint line at 100%
    rlim([0 110]);
    rticks(0:10:110);
    thetaticks(0:30:330);
    title({'Average Thrust Utilization vs Environmental Direction'; ...
           '(Position error < 3m, Heading error < 3°)'});
    grid on;
    
    % Plot 3: Position and heading errors
    figure('Name','Simulation 5 - Position and Heading Errors','Position',[200 200 1200 600]);
    
    subplot(1,2,1)
    polarplot(deg2rad(capData.angles), capData.max_pos_error, '-o', ...
              'LineWidth', 2, 'MarkerSize', 6);
    hold on;
    % Add 3m constraint circle
    theta_circle = linspace(0, 2*pi, 100);
    r_circle = 3 * ones(size(theta_circle));
    polarplot(theta_circle, r_circle, 'r--', 'LineWidth', 2);
    title('Maximum Position Error [m]');
    legend('Position Error', '3m Limit', 'Location', 'best');
    grid on;
    
    subplot(1,2,2)
    polarplot(deg2rad(capData.angles), capData.max_heading_error, '-o', ...
              'LineWidth', 2, 'MarkerSize', 6);
    hold on;
    % Add 3 deg constraint circle
    polarplot(theta_circle, 3 * ones(size(theta_circle)), 'r--', 'LineWidth', 2);
    title('Maximum Heading Error [deg]');
    legend('Heading Error', '3° Limit', 'Location', 'best');
    grid on;
    
    % Print summary
    fprintf('\n========== Capability Plot Summary ==========\n');
    fprintf('Max thrust utilization: %.2f%% at %.0f deg\n', ...
            max(capData.thrust_util), capData.angles(capData.thrust_util == max(capData.thrust_util)));
    fprintf('Min thrust utilization: %.2f%% at %.0f deg\n', ...
            min(capData.thrust_util), capData.angles(capData.thrust_util == min(capData.thrust_util)));
    
    % Find angles where constraints are violated
    constraint_violations = (capData.max_pos_error > 3) | (capData.max_heading_error > 3);
    if any(constraint_violations)
        fprintf('\nConstraint violations at angles: ');
        fprintf('%.0f ', capData.angles(constraint_violations));
        fprintf('deg\n');
    else
        fprintf('\nNo constraint violations - safe operation in all directions\n');
    end
    fprintf('============================================\n\n');
end

%% Simulation 6 - Observer Robustness
function plotObserverRobustness(t, x, y, psi, x_ref, y_ref, psi_ref, obsType)
    figure('Name',['Simulation 6 - Observer Robustness (' obsType ')'],'Position',[100 100 1200 800]);
    
    subplot(3,1,1)
    plot(t, x, 'LineWidth', 1.5); hold on;
    plot(t, x_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title(['Surge Position - ' obsType ' (High Waves)']);
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,2)
    plot(t, y, 'LineWidth', 1.5); hold on;
    plot(t, y_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title(['Sway Position - ' obsType ' (High Waves)']);
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,3)
    plot(t, psi, 'LineWidth', 1.5); hold on;
    plot(t, psi_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]');
    title(['Heading - ' obsType ' (High Waves)']);
    legend('Ship','Reference','Location','best');
    
    % XY plot
    figure('Name',['Simulation 6 - XY Trajectory (' obsType ')'],'Position',[150 150 800 800]);
    plot(y, x, 'LineWidth', 2); hold on;
    plot(y_ref, x_ref, 'k--', 'LineWidth', 2);
    plot(y(1), x(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Draw tolerance circle
    theta = linspace(0, 2*pi, 100);
    x_circle = 3*cos(theta);
    y_circle = 3*sin(theta);
    plot(y_circle, x_circle, 'r--', 'LineWidth', 1.5);
    
    grid on; axis equal;
    xlabel('y [m]'); ylabel('x [m]');
    title(['Station Keeping - ' obsType ' (H_s = 8m, T_p = 13s)']);
    legend('Ship', 'Reference', 'Start', '3m Tolerance', 'Location', 'best');
    
    % Compute statistics
    err_x = x - x_ref;
    err_y = y - y_ref;
    pos_error = sqrt(err_x.^2 + err_y.^2);
    
    fprintf('\n========== Observer Robustness Statistics ==========\n');
    fprintf('Observer type: %s\n', obsType);
    fprintf('Max position error: %.3f m\n', max(pos_error));
    fprintf('Mean position error: %.3f m\n', mean(pos_error));
    fprintf('Std position error: %.3f m\n', std(pos_error));
    fprintf('====================================================\n\n');
end

%% Simulation 7 - Custom DP System Functionality
function plotCustomSimulation(t, x, y, psi, x_ref, y_ref, psi_ref, ...
                               u, v, r, u_ref, v_ref, r_ref, simOut)
    
    % Position and heading
    figure('Name','Simulation 7 - Custom Test','Position',[100 100 1200 800]);
    
    subplot(3,1,1)
    plot(t, x, 'LineWidth', 1.5); hold on;
    plot(t, x_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('x [m]'); title('Surge Position - Custom Test');
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,2)
    plot(t, y, 'LineWidth', 1.5); hold on;
    plot(t, y_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('y [m]'); title('Sway Position - Custom Test');
    legend('Ship','Reference','Location','best');
    
    subplot(3,1,3)
    plot(t, psi, 'LineWidth', 1.5); hold on;
    plot(t, psi_ref, '--', 'LineWidth', 1.5);
    grid on; ylabel('\psi [deg]'); xlabel('Time [s]');
    title('Heading - Custom Test');
    legend('Ship','Reference','Location','best');
    
    % XY trajectory
    figure('Name','Simulation 7 - XY Trajectory','Position',[150 150 800 800]);
    plot(y, x, 'LineWidth', 2); hold on;
    plot(y_ref, x_ref, '--', 'LineWidth', 2);
    plot(y(1), x(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    grid on; axis equal;
    xlabel('y [m]'); ylabel('x [m]');
    title('Custom Test - XY Trajectory');
    legend('Ship', 'Reference', 'Start', 'Location', 'best');
    
    % Plot any available additional data
    if isfield(simOut, 'tau_d') && isfield(simOut, 'tau_act')
        plotForces(simOut, false);
    end
end
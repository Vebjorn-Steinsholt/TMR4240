function compute_integral_dp_gain_schedule()
% COMPUTE_INTEGRAL_DP_GAIN_SCHEDULE - Computes LQR gains with integral
% action for dynamic positioning of offshore supply ship. 
%
% Detailed explanation goes here

% System matrices
    M = [7.0184e6       0           0;
            0       8.5464e6    -4.4678e7;
            0       -4.5028e7   4.0504e9];
    Minv = M\eye(3);
    
    D = [2.6486e5       0           0;
            0       8.8164e5        0;
            0           0       3.3774e8];
    
    T_surge = 1; T_sway = 1; T_yaw = 1;
    A_thr = -diag([1/T_surge 1/T_sway 1/T_yaw]);

    % Weighting matrices
    Q = diag([ 10000 10000 100000,  0.1 0.1 0.1,  0.01 0.01 0.01 ]);
    Q_I = diag([100 100 100]);

    Qa = blkdiag(Q_I, Q);

    R = diag([1, 1, 1]);
    
    % Pre-compute gains for different headings
    psi_grid = linspace(-pi, pi, 72); % Every 5 degrees 
    
    % Storage for gain matrices
    G_p_table = zeros(3, 9, length(psi_grid));
    G_i_table = zeros(3, 3, length(psi_grid));
    Ga_table = zeros(3, 12, length(psi_grid));

    
    for i = 1:length(psi_grid)
        psi = psi_grid(i);
        
        % Time-varying rotation matrix
        J = [cos(psi) -sin(psi) 0;
             sin(psi)  cos(psi) 0;
             0         0        1];
        
         % Original state-space system (9 states)
        Ac = [zeros(3)  J         zeros(3);
              zeros(3) -Minv*D    Minv;
              zeros(3)  zeros(3)  A_thr];

        Bc = [zeros(3); zeros(3); -A_thr];

        Cc = [eye(3), zeros(3,6)];

        % Augmented system (12 states) - equation (16.95)
        
        Aa = [zeros(3), Cc;
               zeros(9,3), Ac];
        
        Ba = [zeros(3,3);
               Bc];

        % Compute LQR gain
        [Ka, ~, ~] = lqr(Aa,Ba , Qa, R);

        G_i = -Ka(:, 1:3);           % Integral gains
        G_p = -Ka(:, 4:12);          % Proportional gains on state
        
        % Store results
        Ga_table(:, :, i) = -Ka;
        G_i_table(:, :, i) = G_i;
        G_p_table(:, :, i) = G_p;

    end
    
    % Save for use in Simulink
    save('dp_gain_schedule_integral.mat', ...
         'G_p_table', 'G_i_table', 'Ga_table', 'psi_grid', ...
         'Q_I', 'Q', 'R');
    
    fprintf('Gain schedule computed for %d heading angles\n', length(psi_grid));

end
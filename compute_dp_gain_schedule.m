function compute_dp_gain_schedule()
    % load supply;
    
    % Your system matrices
    M = [7.0184e6       0           0;
            0       8.5464e6    -4.4678e7;
            0       -4.5028e5   4.0504e9];
    Minv = M\eye(3);
    
    D = [2.6486e5       0           0;
            0       8.8164e5        0;
            0           0       3.3774e8];
    
    T_surge = 1; T_sway = 1; T_yaw = 1;
    A_thr = -diag([1/T_surge 1/T_sway 1/T_yaw]);
    
    % Weighting matrices
    Q = diag([1, 1, 1, 0, 0, 0, 0, 0, 0]);
    R = diag([1, 1, 1]);
    
    % Pre-compute gains for different headings
    psi_grid = linspace(-pi, pi, 72); % Every 5 degrees 
    K_table = zeros(3, 9, length(psi_grid));
    
    for i = 1:length(psi_grid)
        psi = psi_grid(i);
        
        % Time-varying rotation matrix
        J = [cos(psi) -sin(psi) 0;
             sin(psi)  cos(psi) 0;
             0         0        1];
        
        % Build state-space for this heading
        Ac = [zeros(3)  J         zeros(3);
              zeros(3) -Minv*D   -Minv;
              zeros(3)  zeros(3)  A_thr];
        Bc = [zeros(3); zeros(3); -A_thr];
        
        % Compute LQR gain
        [K, ~, ~] = lqr(Ac, Bc, Q, R);
        K_table(:, :, i) = K;
    end
    
    G_table = -K_table;
    
    % Save for use in Simulink
    save('dp_gain_schedule.mat', 'G_table', 'psi_grid');
    
    fprintf('Gain schedule computed for %d heading angles\n', length(psi_grid));
end
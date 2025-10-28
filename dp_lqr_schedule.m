clear;

%Heading intervals
psi_deg = -180:5:180;
psi_rad = deg2rad(psi_deg);
n_angles = length(psi_deg);

% Pre allocate gain look-up
G_lookup = zeros(3,9,n_angles);


M = [7.0184e6       0           0;
        0       8.5464e6    -4.4678e7;
        0       -4.5028e5   4.0504e9];
Minv = M\eye(3);


D = [2.6486e5       0           0; 
        0       8.8164e5        0;
        0           0       3.3774e8];

T_surge = 1.5; T_sway = 2.5; T_yaw = 3.5;
Athr = -diag([1/T_surge 1/T_sway 1/T_yaw]);

Q1 = 1; Q2 = 1; Q6 = 820;
Q = diag([Q1 Q2 Q6]);

R1 = 1.56e-12; R2 = 1.56e-12; R3 = 2.5e-17;
R = diag([R1 R2 R3]);

for i = 1:n_angles

    [J, ~, ~] = eulerang(0,0 ,psi_rad(i));
    
    idx = [1 2 6];
    J_reduced = J(idx, idx);
    
    Ac = [zeros(3) J_reduced zeros(3);
        zeros(3) -Minv*D Minv;
        zeros(3) zeros(3) Athr];
    
    Bc = [zeros(3) zeros(3) -Athr]';
    
    Cc = [eye(3) zeros(3,6)];
    
    [K, ~, ~] = lqr(Ac, Bc, Cc'*Q*Cc ,R);
    
    G_lookup(:,:,i) = -K;


end

% Save lookup table data
save('DP_gain_lookup.mat', 'G_lookup', 'psi_deg', 'psi_rad');

% Display sample gains
fprintf('\nSample gains:\n');
fprintf('At 0 deg:\n');
disp(G_lookup(:,:,1));
fprintf('At 90 deg:\n');
disp(G_lookup(:,:,19));
fprintf('At 180 deg:\n');
disp(G_lookup(:,:,37));

% Optional: Visualize how gains vary with heading
figure;
for row = 1:3
    for col = 1:3
        subplot(3, 3, (row-1)*3 + col);
        gain_values = squeeze(G_lookup(row, col, :));
        plot(psi_deg, gain_values, 'LineWidth', 1.5);
        grid on;
        xlabel('Heading (deg)');
        ylabel(sprintf('G(%d,%d)', row, col));
        title(sprintf('Gain element G(%d,%d)', row, col));
    end
end
sgtitle('Gain Matrix Elements vs Heading Angle');
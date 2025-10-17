eta = [0 0 0 0 0 0]';

M = [7.0184e6       0           0;
        0       8.5464e6    -4.4678e7;
        0       -4.5028e5   4.0504e9];
Minv = M\eye(3);


D = [2.6486e5       0           0; 
        0       8.8164e5        0;
        0           0       3.3774e8];

T_surge = 1; T_sway = 1; T_yaw = 1;
Athr = -diag([1/T_surge 1/T_sway 1/T_yaw]);

Q1 = 1; Q2 = 1; Q3 = 1;
Q = diag([Q1 Q2 Q3]);

R1 = 1; R2 = 1; R3 = 1;
R = diag([R1 R2 R3]);

[J, ~, ~] = eulerang(eta(4),eta(5),eta(6));

idx = [1 2 6];
J_reduced = J(idx, idx);

Ac = [zeros(3) J_reduced zeros(3);
    zeros(3) -Minv*D Minv;
    zeros(3) zeros(3) Athr];

Bc = [zeros(3) zeros(3) -Athr]';

Cc = [eye(3) zeros(3,6)];

[K, ~, ~] = lqr(Ac, Bc, Cc'*Q*Cc,R);

G = -K;

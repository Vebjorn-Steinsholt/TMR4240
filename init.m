%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init()                                                                  %
%                                                                         %              
% Set initial parameters for part1.slx and part2.slx                      %
%                                                                         %
% Created:      2018.07.12	Jon Bjørnø                                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

load('thrusters_sup.mat')
load('supply.mat');
load('supplyABC.mat');

% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';


% LQR Controller

MRB = vessel.MRB;
MA = vessel.A;


% M = MRB + MA;

M = [7.0184e6       0           0;
        0       8.5464e6    -4.4678e7;
        0       -4.5028e5   4.0504e9];

D = [2.6486e5       0           0; 
        0       8.8164e5        0;
        0           0       3.3774e8];



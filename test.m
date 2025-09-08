% Parameters
dt = 0.1;                  % Time step [s]
T_end = 600;               % Total simulation time [s]
t = 0:dt:T_end;            % Time vector
v_mag = 0.5;               % Constant current speed [m/s]

% Initialize current vector storage
v_north = zeros(size(t));
v_east = zeros(size(t));

for i = 1:length(t)
    % Rotate from North to East over 300 seconds
    theta = min(pi/2, (pi/2)*(t(i)/300));  % angle in radians

    % Current vector in NED (North-East)
    v_north(i) = v_mag * cos(theta);  % North component
    v_east(i)  = v_mag * sin(theta);  % East component
end

% 📈 Plot current components over time
figure;
subplot(2,1,1);
plot(t, v_north, 'b'); xlabel('Time [s]'); ylabel('North [m/s]');
title('Current Component - North');

subplot(2,1,2);
plot(t, v_east, 'r'); xlabel('Time [s]'); ylabel('East [m/s]');
title('Current Component - East');

% 🧭 Plot current vector trajectory
figure;
plot(v_east, v_north, 'k');
xlabel('East [m/s]'); ylabel('North [m/s]');
title('Current Direction in NED Frame');
axis equal; grid on;

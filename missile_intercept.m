% Missile intercept simulation using proportional navigation
% Tristan Burnett

clear; clc; close all;

rng(1);

scen = makeScenario();
fprintf('Target range: %.0f m, speed %.0f m/s, %.1fg turn\n', ...
    norm(scen.r_t), scen.spd, scen.gT/9.81);

% single run
N = 4;
[Rm, Rt, t, range, closing, miss] = intercept(N, scen);
fprintf('Miss distance: %.2f m\n', miss);

figure;
plot3(Rm(1,:), Rm(2,:), Rm(3,:), 'b', 'LineWidth', 2); hold on;
plot3(Rt(1,:), Rt(2,:), Rt(3,:), 'r--', 'LineWidth', 2);
plot3(0,0,0, 'go', 'MarkerFaceColor','g');
grid on; axis equal; view(45,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Missile','Target','Launch');
title(sprintf('Miss = %.2f m, N = %d', miss, N));

figure;
subplot(2,1,1); plot(t, range); grid on;
ylabel('Range [m]');
subplot(2,1,2); plot(t, closing); grid on;
xlabel('Time [s]'); ylabel('Closing velocity [m/s]');

% compare a few values of N
Nlist = [2 3 4 6];
figure; hold on;
for i = 1:length(Nlist)
    [Rm, Rt, ~, ~, ~, m] = intercept(Nlist(i), scen);
    plot3(Rm(1,:), Rm(2,:), Rm(3,:), 'LineWidth', 2, ...
        'DisplayName', sprintf('N = %d (%.1f m)', Nlist(i), m));
end
plot3(Rt(1,:), Rt(2,:), Rt(3,:), 'k--', 'LineWidth', 2, 'DisplayName','Target');
grid on; axis equal; view(45,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend; title('Trajectories for different N');

% sweep N
Nvals = 2:0.5:6;
miss = zeros(size(Nvals));
for i = 1:length(Nvals)
    [~,~,~,~,~,miss(i)] = intercept(Nvals(i), scen);
end

figure;
plot(Nvals, miss, 'o-', 'LineWidth', 1.5); grid on;
xlabel('N'); ylabel('Miss distance [m]');
title('Miss distance vs N');

disp('N sweep:');
for i = 1:length(Nvals)
    fprintf('N = %.1f, miss = %.2f m\n', Nvals(i), miss(i));
end


function scen = makeScenario()
% random target start, heading, and evasive turn

r = 3000 + 3000*rand;
az = deg2rad(-60 + 120*rand);
el = deg2rad(-10 + 30*rand);
scen.r_t = r * [cos(el)*cos(az); cos(el)*sin(az); sin(el)];

% bias heading toward the missile so the intercept is possible
u = -scen.r_t / norm(scen.r_t);
p = randn(3,1);
p = p - dot(p,u)*u;
p = p / norm(p);
mix = 0.55 + 0.35*rand;
dir = mix*u + sqrt(1-mix^2)*p;
dir = dir / norm(dir);

scen.spd = 180 + 120*rand;
scen.v_t = scen.spd * dir;

% turn axis perpendicular to velocity
q = randn(3,1);
q = q - dot(q,dir)*dir;
scen.axis = q / norm(q);
scen.gT = (1 + 3*rand) * 9.81;
scen.turnRate = scen.gT / scen.spd;
end


function [Rm, Rt, t, range, closing, miss] = intercept(N, scen)

dt = 0.001;
tmax = 40;
Vm = 400;

r_m = [0;0;0];
r_t = scen.r_t;
v_t = scen.v_t;

% start pointed at the target
v_m = Vm * (r_t - r_m) / norm(r_t - r_m);

n = round(tmax/dt);
Rm = zeros(3,n); Rt = zeros(3,n);
range = zeros(1,n); closing = zeros(1,n); t = zeros(1,n);

lastR = norm(r_t - r_m);
miss = lastR;

for k = 1:n
    rel = r_t - r_m;
    vrel = v_t - v_m;
    R = norm(rel);

    Vc = -dot(rel, vrel) / R;              % closing velocity
    Omega = cross(rel, vrel) / dot(rel,rel);   % LOS rotation rate

    a = N * cross(Omega, v_m);             % PN command

    v_m = v_m + a*dt;
    v_m = Vm * v_m / norm(v_m);            % speed stays constant
    r_m = r_m + v_m*dt;

    % target turns at constant rate
    v_t = v_t + cross(scen.turnRate*scen.axis, v_t)*dt;
    v_t = scen.spd * v_t / norm(v_t);
    r_t = r_t + v_t*dt;

    Rm(:,k) = r_m; Rt(:,k) = r_t;
    range(k) = R; closing(k) = Vc; t(k) = k*dt;

    if R > lastR
        miss = lastR;
        Rm = Rm(:,1:k); Rt = Rt(:,1:k);
        range = range(1:k); closing = closing(1:k); t = t(1:k);
        return
    end
    lastR = R;
end
end

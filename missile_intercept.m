%% =====================================================================
%  3-DOF Missile Intercept Simulation — Proportional Navigation
%  Tristan Burnett
%
%  Simulates a missile intercepting a maneuvering target in 3D using a
%  proportional navigation (PN) guidance law. Engagement geometry is
%  randomized each run.
% =====================================================================
clear; clc; close all;

% rng(42);   % <-- uncomment (any number) to replay one exact engagement

%% ---- Generate one random engagement ----------------------------------
scen = randomScenario();

fprintf('=== Random engagement ===\n');
fprintf('Target start    : [%6.0f %6.0f %6.0f] m   (range %.0f m)\n', scen.r_t, norm(scen.r_t));
fprintf('Target velocity : [%6.0f %6.0f %6.0f] m/s (speed %.0f m/s)\n', scen.v_t, scen.spd);
fprintf('Evasive turn    : %.1f g\n\n', scen.gT/9.81);

%% ---- Baseline run ----------------------------------------------------
N_base = 4;
[Rm,Rt,tHist,rangeHist,closeHist,missDist] = runIntercept(N_base, scen);
fprintf('Miss distance at N = %.1f : %.2f m\n', N_base, missDist);

figure('Color','w');
plot3(Rm(1,:),Rm(2,:),Rm(3,:),'b-','LineWidth',2); hold on;
plot3(Rt(1,:),Rt(2,:),Rt(3,:),'r--','LineWidth',2);
plot3(0,0,0,'go','MarkerSize',10,'MarkerFaceColor','g');
plot3(Rm(1,end),Rm(2,end),Rm(3,end),'kp','MarkerSize',14,'MarkerFaceColor','y');
grid on; axis equal; view(45,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Missile','Target','Launch','Intercept','Location','best');
title(sprintf('PN Intercept | Miss = %.2f m | N = %.1f | %.1fg target', ...
      missDist, N_base, scen.gT/9.81));

%% ---- Range and closing velocity --------------------------------------
figure('Color','w');
subplot(2,1,1);
plot(tHist,rangeHist,'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('Range [m]'); title('Range to Target');
subplot(2,1,2);
plot(tHist,closeHist,'LineWidth',1.5); grid on;
xlabel('Time [s]'); ylabel('Closing Velocity [m/s]'); title('Closing Velocity');

%% ---- Trajectory comparison across N (same engagement) ----------------
Ncompare = [2 3 4 6];
colors   = lines(numel(Ncompare));

figure('Color','w'); hold on;
RtLongest = [];
for i = 1:numel(Ncompare)
    [Rm_i,Rt_i,~,~,~,md] = runIntercept(Ncompare(i), scen);
    plot3(Rm_i(1,:),Rm_i(2,:),Rm_i(3,:),'-','LineWidth',2,'Color',colors(i,:), ...
        'DisplayName',sprintf('N = %.1f  (miss %.1f m)',Ncompare(i),md));
    if size(Rt_i,2) > size(RtLongest,2), RtLongest = Rt_i; end
end
plot3(RtLongest(1,:),RtLongest(2,:),RtLongest(3,:),'k--','LineWidth',2.5,'DisplayName','Target');
plot3(0,0,0,'go','MarkerSize',10,'MarkerFaceColor','g','DisplayName','Launch');
grid on; axis equal; view(45,25);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Location','best');
title(sprintf('Trajectories vs. Navigation Constant | %.1fg evasive target', scen.gT/9.81));

%% ---- N sweep (same engagement) ---------------------------------------
Nvals = 2:0.5:6;
miss  = zeros(size(Nvals));
for i = 1:numel(Nvals)
    [~,~,~,~,~,miss(i)] = runIntercept(Nvals(i), scen);
end

figure('Color','w');
plot(Nvals,miss,'o-','LineWidth',1.8,'MarkerFaceColor','b'); grid on;
xlabel('Navigation Constant N'); ylabel('Miss Distance [m]');
title('Miss Distance vs. Navigation Constant');

fprintf('\nN sweep:\n');
for i = 1:numel(Nvals)
    fprintf('  N = %.1f  ->  miss = %8.2f m\n', Nvals(i), miss(i));
end


%% =====================================================================
%  FUNCTIONS  (must remain at the end of the file)
% =====================================================================

function scen = randomScenario()
% Builds a random but physically sensible engagement.

    % --- Target start position: 3-6 km out, random bearing/elevation ---
    range0 = 3000 + 3000*rand;
    az = deg2rad(-60 + 120*rand);
    el = deg2rad(-10 +  30*rand);
    scen.r_t = range0 * [cos(el)*cos(az); cos(el)*sin(az); sin(el)];

    % --- Target heading: biased toward a closeable geometry ---
    u = -scen.r_t/norm(scen.r_t);                        % straight at the missile
    p = randn(3,1); p = p - dot(p,u)*u; p = p/norm(p);   % perpendicular to that
    mix = 0.55 + 0.35*rand;                              % head-on vs crossing
    dir = mix*u + sqrt(1-mix^2)*p;  dir = dir/norm(dir);

    scen.spd = 180 + 120*rand;                           % 180-300 m/s
    scen.v_t = scen.spd * dir;

    % --- Evasive turn: constant-rate rotation about a random axis ---
    q = randn(3,1); q = q - dot(q,dir)*dir;              % axis perpendicular to velocity
    scen.axis     = q/norm(q);
    scen.gT       = (1 + 3*rand)*9.81;                   % 1-4 g turn
    scen.turnRate = scen.gT/scen.spd;                    % omega = a/v  [rad/s]
end


function [Rm,Rt,tHist,rangeHist,closeHist,missDist] = runIntercept(N, scen)
% Flies one engagement with navigation constant N.

    dt = 0.001; tmax = 40; Vm = 400;      % step, max time, missile speed

    r_m = [0;0;0];                        % missile starts at origin
    r_t = scen.r_t;
    v_t = scen.v_t;

    los0 = r_t - r_m;                     % aim the missile at the target
    v_m  = Vm * los0/norm(los0);

    nsteps = round(tmax/dt);
    Rm = zeros(3,nsteps);  Rt = zeros(3,nsteps);
    rangeHist = zeros(1,nsteps); closeHist = zeros(1,nsteps); tHist = zeros(1,nsteps);

    prevRange = norm(r_t - r_m);
    missDist  = prevRange;

    for k = 1:nsteps
        % ---- Relative geometry ----
        r_rel = r_t - r_m;                          % line-of-sight vector
        v_rel = v_t - v_m;                          % relative velocity
        R     = norm(r_rel);                        % range
        Vc    = -dot(r_rel,v_rel)/R;                % closing velocity  (= -dR/dt)
        Omega = cross(r_rel,v_rel)/dot(r_rel,r_rel);% LOS rotation rate

        % ---- Proportional navigation guidance law ----
        a_cmd = N * cross(Omega, v_m);              % accel perpendicular to velocity

        % ---- Missile update (Euler) ----
        v_m = v_m + a_cmd*dt;
        v_m = Vm * v_m/norm(v_m);                   % constant speed, direction only
        r_m = r_m + v_m*dt;

        % ---- Target update: constant-rate evasive turn ----
        v_t = v_t + cross(scen.turnRate*scen.axis, v_t)*dt;
        v_t = scen.spd * v_t/norm(v_t);             % hold speed, only turn
        r_t = r_t + v_t*dt;

        % ---- Record ----
        Rm(:,k)=r_m;  Rt(:,k)=r_t;
        rangeHist(k)=R;  closeHist(k)=Vc;  tHist(k)=k*dt;

        % ---- Closest approach = miss distance ----
        if R > prevRange
            missDist = prevRange;
            Rm=Rm(:,1:k); Rt=Rt(:,1:k);
            rangeHist=rangeHist(1:k); closeHist=closeHist(1:k); tHist=tHist(1:k);
            return;
        end
        prevRange = R;
    end
    missDist = prevRange;
end
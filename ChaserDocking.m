clc; clear;

function angle = wrapToPi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end


% ---- Parameters ----
m = 10;         % Mass of chaser (kg)
I = 0.1;        % Moment of inertia (kg*m^2)
t_end = 10;     % Total simulation time (s)
dt = 0.01;      % Time step (s)
N = t_end/dt;   % Number of simulation steps
time = 0:dt:t_end;

% ---- PID Gains ----
Kp = [5, 5, 3];     % Proportional gains: [x, y, theta]
Ki = [0.1, 0.1, 0.05];
Kd = [2, 2, 0.5];

% ---- Target State ----
target = [0, 0, 0];  % Target: [x, y, theta]

% ---- Monte Carlo Simulation ----
num_trials = 10;
initial_variation = [2, 2, 20, 0.5, 0.5, 0.1];  % Larger noise

figure('Name','Monte Carlo with PID Control');
hold on; grid on;

for trial = 1:num_trials
    x = zeros(N,6);
    % Random perturbation
    perturb = initial_variation .* randn(1,6);
    perturb(3) = deg2rad(perturb(3));  % degrees to radians
    x(1,:) = [0, 0, 0, 0, 0, 0] + perturb;

    % PID states
    integral_error = zeros(1,3);
    prev_error = zeros(1,3);

    for k = 1:N-1
        % Current state
        pos = x(k,1:2);
        theta = x(k,3);
        vel = x(k,4:5);
        omega = x(k,6);

        % Compute errors
        e_pos = target(1:2) - pos;
        e_theta = wrapToPi(target(3) - theta);
        error = [e_pos, e_theta];

        % PID terms
        integral_error = integral_error + error * dt;
        derivative_error = (error - prev_error) / dt;
        prev_error = error;

        % PID control output (in inertial frame)
        u = Kp .* error + Ki .* integral_error + Kd .* derivative_error;
        Fx_world = u(1);
        Fy_world = u(2);
        Tau = u(3);

        % Convert world-frame force to body-frame (for thrust)
        Fx_body = Fx_world * cos(theta) + Fy_world * sin(theta);
        Fy_body = -Fx_world * sin(theta) + Fy_world * cos(theta);

        % Dynamics
        dxdt = zeros(6,1);
        dxdt(1) = x(k,4);
        dxdt(2) = x(k,5);
        dxdt(3) = x(k,6);
        dxdt(4) = Fx_world / m;
        dxdt(5) = Fy_world / m;
        dxdt(6) = Tau / I;

        x(k+1,:) = x(k,:) + dxdt' * dt;
    end

    % Plot
    plot(x(:,1), x(:,2), 'Color', [0.5 0.5 1 trial/num_trials]); % Blue-ish
    plot(x(1,1), x(1,2), 'go', 'MarkerFaceColor','g');           % Start
    plot(x(end,1), x(end,2), 'rx', 'MarkerSize', 8, 'LineWidth',1.5); % End
end

xlabel('x (m)'); ylabel('y (m)');
title('Monte Carlo with PID-Controlled Chaser');
legend('Monte Carlo Runs', 'Start', 'End');

% --- Setup Animation ---
figure('Name','Chaser Docking Animation');
hold on; grid on; axis equal;
xlim([-5 5]); ylim([-5 5]);
xlabel('x (m)'); ylabel('y (m)');
title('2D Docking Animation');

% Target indicator
plot(target(1), target(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2); 

% Animation objects
traj = animatedline('Color','b','LineWidth',1.5);                  % Path
chaser_dot = plot(0, 0, 'ko', 'MarkerFaceColor','k');              % Chaser
heading = plot([0 0],[0 0],'r','LineWidth',2);                     % Orientation
thrust_vec = quiver(0,0,0,0,0,'Color','g','LineWidth',2,'MaxHeadSize',2); % Thruster

len_heading = 0.4;   % Length of orientation arrow
scale_thrust = 0.5;  % Scale factor for thrust arrows

% Animate
for k = 1:20:N
    pos = x(k,1:2);
    theta = x(k,3);

    % Calculate thrust force from PID at that point
    e_pos = target(1:2) - pos;
    e_theta = wrapToPi(target(3) - theta);
    error = [e_pos, e_theta];

    Fx_world = Kp(1)*error(1);
    Fy_world = Kp(2)*error(2);

    % Update path
    addpoints(traj, pos(1), pos(2));

    % Update chaser position marker
    set(chaser_dot, 'XData', pos(1), 'YData', pos(2));

    % Orientation line (heading)
    hx = [pos(1), pos(1) + len_heading*cos(theta)];
    hy = [pos(2), pos(2) + len_heading*sin(theta)];
    set(heading, 'XData', hx, 'YData', hy);

    % Thruster force vector (in world frame)
    set(thrust_vec, 'XData', pos(1), 'YData', pos(2), ...
                    'UData', scale_thrust*Fx_world, ...
                    'VData', scale_thrust*Fy_world);

    drawnow;
    pause(0.01);  % slow down
end

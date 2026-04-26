clc; clear; close all;

%% parameters
m = 1000;
c = 0.3;
g = 9.81;

theta = 5*pi/180;

dt = 0.1;
T = 20;
N = T/dt;

F = 2000;

%% dynamics: x(1)=pos, x(2)=vel
%f_dyn = [x(1)_dot ; X(2)_dot] == [x(2) ; a]
f_dyn = @(x) [ x(2); (F - c*(x(2)^2) - m*g*sin(theta)) / m];

%% true system (RK4)

x = [0; 0];
X_true = zeros(2, N);

for k = 1:N
    
    k1 = f_dyn(x);
    k2 = f_dyn(x + 0.5*dt*k1);
    k3 = f_dyn(x + 0.5*dt*k2);
    k4 = f_dyn(x + dt*k3);
    
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    X_true(1,k) = x(1);
    X_true(2,k) = x(2);
end

%% EKF init

x_est = [10; 1];
P = eye(2);

Q = 0.01 * eye(2);
R = 10;

H = [1 0];

X_est = zeros(2, N);
Z = zeros(1, N);

x_sim = [0; 0];

%% main loop

for k = 1:N
    
    % simulate system (RK4 + noise)
    k1 = f_dyn(x_sim);
    k2 = f_dyn(x_sim + 0.5*dt*k1);
    k3 = f_dyn(x_sim + 0.5*dt*k2);
    k4 = f_dyn(x_sim + dt*k3);
    
    x_rk4 = x_sim + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
    
    w_k = mvnrnd([0 0], Q)';   
    x_sim = x_rk4 + w_k;
    
    % measurement
    v_k = sqrt(R) * randn;
    z = x_sim(1) + v_k;
    
    Z(k) = z;
    
    % prediction (Euler)
    x_dot_est = f_dyn(x_est);
    x_pred = x_est + dt * x_dot_est;
    
    % Jacobian
    v = x_est(2);
    
    Fk = [1 dt; 0 1 - (2*c*v/m)*dt];
    
    % covariance prediction
    P_pred = Fk * P * Fk' + Q;
    
    % Kalman gain
    S = H * P_pred * H' + R;
    K = P_pred * H' / S;
    
    % update
    y_tilde = z - H * x_pred;
    x_est = x_pred + K * y_tilde;
    
    % covariance update
    P = (eye(2) - K * H) * P_pred;
    
    % store
    X_est(:, k) = x_est;
    
end

%% plotting

t = (1:N) * dt;

figure;

% position
subplot(2,1,1)

plot(t, X_true(1,:), 'b', 'LineWidth', 2); hold on;
plot(t, X_est(1,:), 'r--', 'LineWidth', 2);
plot(t, Z, 'c.', 'MarkerSize', 8);

legend('True','EKF','Measurements');
title('Position');
xlabel('Time (s)');
ylabel('Position');
grid on;

% velocity
subplot(2,1,2)

plot(t, X_true(2,:), 'b', 'LineWidth', 2); hold on;
plot(t, X_est(2,:), 'r--', 'LineWidth', 2);

legend('True','EKF');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity');
grid on;
clc;clear all; close all;
k = 1000;
a = 1000000;
F = 500;
e = 1e-6;
x_n = 0.5;


%mx_doubledot + cx_dot(if considering viscous damping) + kx + ax^3 = F
%For equilibrium points, v and a = 0. Hence: kx + ax^3 = F.
%Hence creating a function where to find points where force balance.
%i.e internal force of springs = external force F. 
%therefore, in function form: f(x) = kx + ax^3 - F.
%Now we find the points where this function equals to 0 i.e the roots.

function x_eq = n_raphson(k, a, F, x_n, e)
    x_n1 = x_n - (a*x_n^3 + k*x_n - F) / (3*a*x_n^2 + k);

    while abs(x_n1 - x_n)>e
        x_n = x_n1;
        x_n1 = x_n - (a*x_n^3 + k*x_n - F) / (3*a*x_n^2 + k);
    end
    
    x_eq = x_n1;
end

function adj_points = find_adj_point(start, step, stop, k, a, F)     
    adj_point_arr = [];
    for i = start:step:stop
        i1 = i+step;
        f_i0 = k*i + a*(i^3) - F;
        f_i1 = k*i1 + a*(i1^3) - F;
        if f_i0 * f_i1 < 0
            adj_point_arr = [adj_point_arr i];
        elseif f_i0 * f_i1 == 0
            adj_point_arr = [adj_point_arr i];
        end
    end

    adj_points = adj_point_arr;
end


% Find adjacent points for root-finding
start = -10; 
step = 0.01; 
stop = 10; 
adj_points = find_adj_point(start, step, stop, k, a, F);

x_eq = [];
for i = 1:length(adj_points)
    x = n_raphson(k, a, F, adj_points(i), e);
    x_eq = [x_eq x];
end

xeq = unique(round(x_eq, 6));
fprintf("The points of equilibrium are: %f",x_eq)

x = linspace(-0.5,0.5,1000);
f_nonlinear = k*x + a*x.^3 - F;
f_linear = k*x - F;
plot(x,f_nonlinear,'LineWidth',2)
grid on
xlabel('x'), ylabel('f(x)')
title('Equilibrium Points (f(x)=0)')
%ylim([-10 10])

hold on;
xline(0, 'g');
yline(0, 'g');
plot(x,f_linear,'LineWidth',2)
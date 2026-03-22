k = 1000;
a = 1000000;
F = 500;
e = 0.0001;
x_n = 0.5;
x2 = x_n;
x_n1 = x_n - (a*x_n^3 + k*x_n - F) / (3*a*x_n^2 + k);

while abs(x_n1 - x_n)>e
    x_n = x2;
    x_n1 = x_n - (a*x_n^3 + k*x_n - F) / (3*a*x_n^2 + k);
    x2 = x_n1;
end

display('Resuls of Newton Raphson:');
display(x_n1)
display(a*x_n1^3 + k*x_n1 - F)


x3 = -0.5;
x4 = 0.5;

while abs(x3 - x4)>e
    xr = (x3+x4)/2;
    yr = a*xr^3 + k*xr - F;
    y3 = a*x3^3 + k*x3 - F;
    y4 = a*x4^3 + k*x4 - F;
    if yr*y3 > 0
        x3 = xr;
    else
        x4 = xr;
    end
end

disp('Results of Bisection:')
disp(xr)



x5 = -0.5;
x6 = 0.5;

iter = 0;
max_iter = 100;

while abs(x5 - x6)>e && iter<max_iter
    iter = iter + 1;
    y5 = a*x5^3 + k*x5 - F;
    y6 = a*x6^3 + k*x6 - F;
    xrf = x5 - (y5*(x6-x5))/(y6-y5);
    yrf = a*xrf^3 + k*xrf - F;

    if yrf*y5 > 0
        x5 = xrf;
    else
        x6 = xrf;
    end
end

disp('Results of Regula Falsi/False Position:')
disp(xrf)

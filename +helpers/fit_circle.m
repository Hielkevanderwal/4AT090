function [a,b,r] = fit_circle(x_data,y_data)
% fit a circle over the rear axle data pose

% Exercise 1.2: Circle fit implementation
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

% Construct the design matrix A and vector b
A = [x_data, y_data, ones(size(x_data))];
b = -x_data.^2 - y_data.^2;
% Recover a b coeffs and radius

p = A \ b;

D = p(1);
E = p(2);
F = p(3);

a = -D / 2;
b = -E / 2;
r = sqrt((D/2)^2 + (E/2)^2 - F);


%%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%

end


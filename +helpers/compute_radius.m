function mean_radius = compute_radius(a,b,x,y)
% Function computes the radius of the circle
% mean distance between the origin of the circle and the current pose of
% the tractor's rear axle throughout the simulation
% inputs: (a,b) origin of the circle corrdinates; x and y vector of data point coordinates

% Exercise 1.3: Average radius implementation
%%%%%%%%%%%% YOUR CODE: START %%%%%%%%%%%%%

R2 = (x - a).^2 + (y - b).^2;
mean_radius = sqrt(mean(R2));

%%%%%%%%%%%%  YOUR CODE: END  %%%%%%%%%%%%%
end


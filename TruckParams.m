classdef TruckParams
    properties (Constant)
        % Tractor geometry
        L1 = 0.263;                  % wheelbase of the tractor [m]
        L1c = 0.051;                 % distance from the rear axle to hitch/king pin position [m]
        L2 = 0.572;                  % wheelbase of the trailer [m]
        
        % Limits
        delta_max = 38*pi/180;      % max steering angle [rad]
        gamma_max = 110*pi/180;     % max articulation angle [rad]
    end
end

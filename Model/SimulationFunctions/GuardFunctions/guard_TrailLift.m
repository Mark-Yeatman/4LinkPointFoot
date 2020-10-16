function [value, isterminal, direction] = guard_TrailLift(t,x)
%GUARD_TRAILLIFT Summary of this function goes here
%   Detailed explanation goes here
    lambda = Lambda(t,x');
	value = lambda(4);
    isterminal = 1;
    direction = 0;
end


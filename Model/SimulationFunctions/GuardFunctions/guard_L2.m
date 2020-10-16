function [value, isterminal, direction] = guard_L2(t,x)
%GUARD_TRAILLIFT Summary of this function goes here
%   Detailed explanation goes here
    global flowdata
    L0 = flowdata.Parameters.SLIP.L0;
    params = flowdata.Parameters.Biped.asvec;
    p2 = Foot_Sw_pos_func(x,params);
    p2 = x(1:2)-p2(1:2,4);
    L2 = norm(p2);
	value = L2-L0;
    isterminal = 1;
    direction = 0;
end


function E = SLIP_E_func(x)
%SLIP_E_FUNC Summary of this function goes here
%   Detailed explanation goes here
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    k = flowdata.Parameters.SLIP.k;
    L0 = flowdata.Parameters.SLIP.L0;
    Md = flowdata.Parameters.SLIP.Md;
    g = flowdata.Parameters.Environment.g;
    
    if strcmp(flowdata.State.c_phase,"DSupp")
        p1 = Foot_St_pos_func(x,params);
        p1 = x(1:2)-p1(1:2,4);
        L1 = norm(p1);
        p2 = Foot_Sw_pos_func(x,params);
        p2 = x(1:2)-p2(1:2,4);
        L2 = norm(p2);
        E = 1/2*Md*norm(x(8:9))^2 + Md*g*x(2) + 1/2*k*(L1-L0)^2 + 1/2*k*(L2-L0)^2; 
    elseif strcmp(flowdata.State.c_phase,"SSupp")
        p1 = Foot_St_pos_func(x,params);
        p1 = x(1:2)-p1(1:2,4);
        L1 = norm(p1);
        E = 1/2*Md*norm(x(8:9))^2 + Md*g*x(2) + 1/2*k*(L1-L0)^2;
    end
end


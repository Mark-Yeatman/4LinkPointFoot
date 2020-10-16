function cost = IK_SLIP_vel_cost(x)
    %IK_COST Summary of this function goes here
    %   Detailed explanation goes here
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    x= x(:);
    
    toe_p_vel2= Foot_St_vel_func(x,params);
    toe_vel2 = toe_p_vel2(1:2,4);
    e_toe_vel2 = toe_vel2(:);
    
    toe_p_vel= Foot_Sw_vel_func(x,params);
    toe_vel = toe_p_vel(1:2,4);
    e_toe_vel = toe_vel(:);
    
    e_vel = [e_toe_vel;e_toe_vel2];
    cost =e_vel;
end


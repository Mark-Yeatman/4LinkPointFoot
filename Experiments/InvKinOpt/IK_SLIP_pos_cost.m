function [cost] = IK_SLIP_pos_cost(x,pf1,pf2)
    %IK_COST Summary of this function goes here
    %   Detailed explanation goes here
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    x= x(:);
    
    toe_st_pose = Foot_St_pos_func(x,params);
    toe_xy = toe_st_pose(1:2,4);
    pf1 = pf1(:);
    e_toe_pos1 = toe_xy-pf1;
    e_toe_pos1 = e_toe_pos1(:);

    toe_sw_pose = Foot_Sw_pos_func(x,params);
    toe_xy = toe_sw_pose(1:2,4);
    pf2 = pf2(:);
    e_toe_pos2 = toe_xy-pf2;
    e_toe_pos2 = e_toe_pos2(:);
    
    e_pos = [e_toe_pos1;e_toe_pos2];
        
    cost = norm(e_pos,2);
end


function xout = map_L2(xprev,xnext)
    %MAP_FOOTSTRIKE Summary of this function goes here
%     %   Detailed explanation goes here
%     global flowdata
%     params = flowdata.Parameters.Biped.asvec;
%     if flowdata.Flags.step_done
%         nextpose = Foot_Sw_pos_func(xprev',params);            
%         xnext(1:2) = nextpose(1:2,4)';
%     end
    global flowdata
    p2 = Foot_Sw_pos_func(xnext',flowdata.Parameters.Biped.asvec);
    p2 = xnext(1:2)'-p2(1:2,4);
    theta2 = atan2(p2(2),p2(1));
    
    [px,py,s,f] = genSwingTraj(0,xnext');
    flowdata.Parameters.SLIP.poly_x = px;
    flowdata.Parameters.SLIP.poly_y = py;
    flowdata.Parameters.SLIP.xs = s;
    flowdata.Parameters.SLIP.xf = f;
    xout = xnext;
    flowdata.Parameters.Biped.alpha = flowdata.Parameters.Biped.alpha*0.2+ theta2*0.8;
    %flowdata.State.Eref = 0.8*SLIP_E_func(xnext') + 0.2*flowdata.State.Eref;
end


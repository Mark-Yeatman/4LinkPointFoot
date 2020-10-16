function xout = map_TrailLift(xprev,xnext)
    %MAP_FOOTSTRIKE Summary of this function goes here
%     %   Detailed explanation goes here
%     global flowdata
%     params = flowdata.Parameters.Biped.asvec;
%     if flowdata.Flags.step_done
%         nextpose = Foot_Sw_pos_func(xprev',params);            
%         xnext(1:2) = nextpose(1:2,4)';
%     end
    global flowdata
    [px,py,s,f] = genSwingTraj(0,xnext');
    flowdata.Parameters.SLIP.poly_x = px;
    flowdata.Parameters.SLIP.poly_y = py;
    flowdata.Parameters.SLIP.xs = s;
    flowdata.Parameters.SLIP.xf = f;
    xout = xnext;
    %flowdata.State.Eref = 0.8*SLIP_E_func(xnext') + 0.2*flowdata.State.Eref;
end


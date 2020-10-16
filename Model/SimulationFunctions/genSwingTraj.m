function [poly_x,poly_y,hip_xs,hip_xf] = genSwingTraj(t,x)
    %GENSWINGTRAJ Summary of this function goes here
    %   This function assumes x gives a configuation at the end of double
    %   support and also has some symmetry assumptions about the single
    %   support trajectory of the hip.
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    alpha = flowdata.Parameters.Biped.alpha;
    
    hip_start = Hip_pos_func(x,params);
    foot_start = Foot_Sw_pos_func(x,params);
    pf1 = Foot_St_pos_func(x,params);
    
    xdist = abs(hip_start(1,4) - pf1(1,4));
    
    hip_end = hip_start;
    hip_end(1,4) = hip_end(1,4) + 2*xdist;
    
    foot_end = hip_end;
    foot_end(1:2,4) = foot_end(1:2,4)+ flowdata.Parameters.SLIP.L0*[cos(alpha);-sin(alpha)];
    
    y0 = 0.1; %meters
    midpoint = (foot_start(1:2,4)+foot_end(1:2,4))/2;
    midpoint(2) = y0;
    
    poly_x = spline([0,0.5,1],[0,foot_start(1,4),midpoint(1),foot_end(1,4),0.0]);
    poly_y = spline([0,0.5,1],[0,foot_start(2,4),midpoint(2),foot_end(2,4)-0.0,-0.0]);
    hip_xs = hip_start(1,4);
    hip_xf = hip_end(1,4);
    
    flowdata.Parameters.SLIP.foot_start = foot_start(1:2,4);
    flowdata.Parameters.SLIP.midpoint = midpoint;
    flowdata.Parameters.SLIP.foot_end = foot_end(1:2,4);
    
    [R,xcyc] = fit_circle_through_3_points([foot_start(1:2,4),midpoint, foot_end(1:2,4)]');
    
    flowdata.Parameters.Foot.point = xcyc;
    flowdata.Parameters.Foot.R = R;
end


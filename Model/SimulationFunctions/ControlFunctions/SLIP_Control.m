function u = SLIP_Control(t,x)
    %SLIP_CONTROL Summary of this function goes here
    %   Detailed explanation goes here
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    k = flowdata.Parameters.SLIP.k;
    L0 = flowdata.Parameters.SLIP.L0;
    Md = flowdata.Parameters.SLIP.Md;
    d = flowdata.Parameters.SLIP.d;
    tor0 = flowdata.Parameters.SLIP.tor0;
    Eref = flowdata.State.Eref;
    kappa = flowdata.Parameters.SLIP.kappa;
    Omega = flowdata.Parameters.SLIP.Omega;
    if strcmp(flowdata.State.c_phase,"DSupp")
        p1 = Foot_St_pos_func(x,params);
        p1 = x(1:2)-p1(1:2,4);
        L1 = norm(p1);
        theta1 = atan2(p1(2),p1(1));
        Fs1 = -k*(L1-L0)*[cos(theta1);sin(theta1)];

        p2 = Foot_Sw_pos_func(x,params);
        p2 = x(1:2)-p2(1:2,4);
        L2 = norm(p2);
        theta2 = atan2(p2(2),p2(1));
        Fs2 = -k*(L2-L0)*[cos(theta2);sin(theta2)];
        
        E = SLIP_E_func(x);
        
        KPBC = -kappa*Omega*x(8:9)*(E-Eref);
        
        u_torso = -100*(x(7)- tor0) -100*x(14);
   
        F = [-[0;9.81*Md]+Fs1+Fs2+KPBC; u_torso];

        J = [Hip_Jacobian_func(x,params);
            0, 0, 0, 0, 0, 0, 1];
        Jdot = [Hip_Jacobian_Dot_func(x,params);
            0, 0, 0, 0, 0, 0, 0];
        MD = diag([Md,Md,1]);
    elseif strcmp(flowdata.State.c_phase,"SSupp")
        p1 = Foot_St_pos_func(x,params);
        p1 = x(1:2)-p1(1:2,4);
        L1 = norm(p1);
        theta1 = atan2(p1(2),p1(1));
        Fs1 = -k*(L1-L0)*[cos(theta1);sin(theta1)];
        
        u_torso = -100*(x(7)- tor0) -100*x(14);
        
        u_foot = zeros(2,1);
        
        pf=Foot_Sw_pos_func(x,params);
        vf=Foot_Sw_vel_func(x,params);
        
        poly_x = flowdata.Parameters.SLIP.poly_x;
        poly_y = flowdata.Parameters.SLIP.poly_y;
        
        dpoly_x = fnder(poly_x,1);
        dpoly_y = fnder(poly_y,1);
        
        ddpoly_x = fnder(dpoly_x,1);
        ddpoly_y = fnder(dpoly_y,1);
        
        xs = flowdata.Parameters.SLIP.xs;
        xf = flowdata.Parameters.SLIP.xf;
        pv = (x(1)-xs)/(xf-xs); 
        v_pv = x(8)/(xf-xs);
        if pv<0
            pv=0;
            v_pv = 0;
            e_x = pf(1,4)-ppval(poly_x,pv);
            e_y = pf(2,4)-ppval(poly_y,pv);

            de_x = vf(1,4) - v_pv*ppval(dpoly_x,pv);
            de_y = vf(2,4) - v_pv*ppval(dpoly_y,pv);

            u_foot(1) = -10000*(e_x) - 10000*(de_x);
            u_foot(2) = -10000*(e_y) - 10000*(de_y);

            Avhc = zeros(2,7);
            Avhc(1,1) = -ppval(dpoly_x,pv);
            Avhc(2,1) = -ppval(dpoly_y,pv);
            Avhc = Foot_Sw_Jacobian_func(x,params) + Avhc;

            Avhcdot = zeros(2,7);
            Avhcdot(1,1) = -v_pv*ppval(ddpoly_x,pv);
            Avhcdot(2,1) = -v_pv*ppval(ddpoly_y,pv);
            Avhcdot = Foot_Sw_Jacobian_Dot_func(x,params) + Avhcdot;
            MD = diag([Md,Md,1,1,1]);
        elseif pv>1
            u_foot(1) = 0;
            u_foot(2) = -100;

            Avhc = Foot_Sw_Jacobian_func(x,params);

            Avhcdot = Foot_Sw_Jacobian_Dot_func(x,params);
            MD = diag([Md,Md,1,1,1]);
        else
            e_x = pf(1,4)-ppval(poly_x,pv);
            e_y = pf(2,4)-ppval(poly_y,pv);

            de_x = vf(1,4) - v_pv*ppval(dpoly_x,pv);
            de_y = vf(2,4) - v_pv*ppval(dpoly_y,pv);

            u_foot(1) = -10000*(e_x) - 10000*(de_x);
            u_foot(2) = -10000*(e_y) - 10000*(de_y);

            Avhc = zeros(2,7);
            Avhc(1,1) = -ppval(dpoly_x,pv);
            Avhc(2,1) = -ppval(dpoly_y,pv);
            Avhc = Foot_Sw_Jacobian_func(x,params) + Avhc;

            Avhcdot = zeros(2,7);
            Avhcdot(1,1) = -v_pv*ppval(ddpoly_x,pv);
            Avhcdot(2,1) = -v_pv*ppval(ddpoly_y,pv);
            Avhcdot = Foot_Sw_Jacobian_Dot_func(x,params) + Avhcdot;
            MD = diag([Md,Md,1,1,1]);
        end
                     
        E = SLIP_E_func(x);        
        KPBC = -kappa*Omega*x(8:9)*(E-Eref);
        
        F = [-[0;9.81*Md]+Fs1 + KPBC; u_torso;u_foot];
        
        J = [Hip_Jacobian_func(x,params);
            0, 0, 0, 0, 0, 0, 1;
            Avhc];
        Jdot = [Hip_Jacobian_Dot_func(x,params);
            0, 0, 0, 0, 0, 0, 0;
            Avhcdot];
        
    end
       
    dim = flowdata.Parameters.dim;
    q = x(1:dim/2);         
    qdot = x(dim/2+1:dim);  
    
    [A,Adot] = flowdata.getConstraintMtxs(q,qdot);
    
    M = M_func(x,params);
    C = C_func(x,params);
    G = G_func(x,params); 
    B = flowdata.Parameters.B;
      
    Q = A'*inv(A*inv(M)*A')*( A*inv(M)*(-C*qdot -G) + Adot*qdot);
    Bt = J*inv(M)*(eye(7,7) - A'*inv(A*inv(M)*A')*A*inv(M))*B;
    
    tau = mldivide(Bt, inv(MD)*F -Jdot*qdot + J*inv(M)*(C*qdot+G+Q));
    u = B*tau;
end


function u = SLIP_Control2(t,x)
    %SLIP_CONTROL Summary of this function goes here
    %   Detailed explanation goes here
    global flowdata
    params = flowdata.Parameters.Biped.asvec;
    
    k = flowdata.Parameters.SLIP.k;
    L0 = flowdata.Parameters.SLIP.L0;
    Md = flowdata.Parameters.SLIP.Md;
    
    tor0 = flowdata.Parameters.SLIP.tor0;
    
    Eref = flowdata.State.Eref;
    kappa = flowdata.Parameters.SLIP.kappa;
    Omega = flowdata.Parameters.SLIP.Omega;
    
    dim = flowdata.Parameters.dim;
    q = x(1:dim/2);         
    qdot = x(dim/2+1:dim);          
    [A,Adot] = flowdata.getConstraintMtxs(q,qdot);
    
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
        MD = diag([Md,Md,1]);
        
        J = [Hip_Jacobian_func(x,params);
            0, 0, 0, 0, 0, 0, 1];
        Jdot = [Hip_Jacobian_Dot_func(x,params);
            0, 0, 0, 0, 0, 0, 0];
        
        Ao = A(1:3,:);
        Ad = A(4,:); %trailing foot y vector
        lambda_d = Fs2(2);        
    elseif strcmp(flowdata.State.c_phase,"SSupp")
        p1 = Foot_St_pos_func(x,params);
        p1 = x(1:2)-p1(1:2,4);
        L1 = norm(p1);
        theta1 = atan2(p1(2),p1(1));
        Fs1 = -k*(L1-L0)*[cos(theta1);sin(theta1)];
        
        E = SLIP_E_func(x);        
        KPBC = -kappa*Omega*x(8:9)*(E-Eref);
        
        u_torso = -100*(x(7)- tor0) -100*x(14);
        
        pf_sw = Foot_Sw_pos_func(x,params);
        vf_sw = Foot_Sw_vel_func(x,params);
        Lf = pf_sw(1:2,4)-flowdata.Parameters.Foot.point;
        R = flowdata.Parameters.Foot.R;
        g_star=flowdata.Parameters.SLIP.g_foot;
        p1 = Foot_St_pos_func(x,params);
        if vf_sw(1,4)>0 || pf_sw(1,4) < p1(1,4)
            u_foot =[0;g_star]-1/R^2*(g_star*Lf(2)+vf_sw(1,4)^2+vf_sw(2,4)^2)*[Lf(1);Lf(2)];
        else
            u_foot = [0;-10];
        end
                   
        F = [-[0;9.81*Md]+Fs1+KPBC; u_torso; u_foot];
        MD = diag([Md,Md,1,1,1]);
        
        J = [Hip_Jacobian_func(x,params);
            0, 0, 0, 0, 0, 0, 1;
            Foot_Sw_Jacobian_func(x,params)];
        Jdot = [Hip_Jacobian_Dot_func(x,params);
            0, 0, 0, 0, 0, 0, 0;
            Foot_Sw_Jacobian_Dot_func(x,params)];
        
        Ao = A;
        Ad = 0; %trailing foot y vector
        lambda_d = 0;        
        
    end
         
    M = M_func(x,params);
    C = C_func(x,params);
    G = G_func(x,params); 
    B = flowdata.Parameters.B;
      
    Q = A'*inv(A*inv(M)*A')*( A*inv(M)*(-C*qdot -G) + Adot*qdot);
    Bt = J*inv(M)*(eye(7,7) - A'*inv(A*inv(M)*A')*A*inv(M))*B;
    
    tau = mldivide([A;J]*inv(M)*[B,-Ao'], [A;J]*inv(M)*(C*qdot+G+Ad'*lambda_d) + [-Adot*qdot; (inv(MD)*F -Jdot*qdot)]);
    u = B*tau(1:rank(B));
end


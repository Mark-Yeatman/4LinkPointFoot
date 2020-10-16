figure;
slip_mat = load('slip_walk.mat');
pf1 = slip_mat.out_extra.steps{1}.phases{2}.foot_pos_1;
pf2 = slip_mat.out_extra.steps{1}.phases{2}.foot_pos_2;
indexi = find(slip_mat.tout==slip_mat.out_extra.steps{1}.phases{2}.tstart,1);
indexf = find(slip_mat.tout==slip_mat.out_extra.steps{1}.phases{2}.tend,1);
zs_dsupp = slip_mat.xout(indexi,:);
zf_dsupp = slip_mat.xout(indexf,:);
hold on
plot(slip_mat.xout(:,1),slip_mat.xout(:,2))
plot(zs_dsupp(1),zs_dsupp(2),'g*','MarkerSize',10)
plot(zf_dsupp(1),zf_dsupp(2),'b*','MarkerSize',10)
plot(pf1(1),pf1(2),'m+','MarkerSize',10)
plot(pf2(1),pf2(2),'mo','MarkerSize',10)
plot(slip_mat.xout(end,1)+slip_mat.xout(:,1),slip_mat.xout(:,2))

plot(xout(:,1),xout(:,2));
plot(out_extra.FootStPos(:,1),out_extra.FootStPos(:,2),'r*');
plot(out_extra.FootSwPos(:,1),out_extra.FootSwPos(:,2),'g+');

figure;
for i=1:length(out_extra.steps)
    for j=1:length(out_extra.steps{i}.phases)
        is = find(tout==out_extra.steps{i}.phases{j}.t_start,1,'last');
        ie = find(tout==out_extra.steps{i}.phases{j}.t_end,1);
        plot(tout(is:ie), out_extra.steps{i}.phases{j}.SLIPEnergy)
        hold on
        
        xs = flowdata.Parameters.SLIP.xs;
        xf = flowdata.Parameters.SLIP.xf;
        pv = (xout(ie:is,1)-xs)/(xf-xs);
        pv(pv<0) = 0;
        pv(pv>1) = 1;
        poly_x = flowdata.Parameters.SLIP.poly_x;
        poly_y = flowdata.Parameters.SLIP.poly_y;
        plot(ppval(poly_x,pv),ppval(poly_y,pv))
        
    end
end
hold off
M=[Mth.*cos(x(3)).^2+Ms.*cos(x(3)+x(4)).^2+Mth.*cos(x(5)).^2+Ms.*cos(x(5)+x(6)).^2+Mtor.*cos(x(3)+x(7)).^2+Mth.*sin(x(3)).^2+Ms.*sin(x(3)+x(4)).^2+Mth.*sin(x(5)).^2+Ms.*sin(x(5)+x(6)).^2+Mtor.*sin(x(3)+x(7)).^2,0,Mth.*rth.*cos(x(3))+Ms.*(rs+lth.*cos(x(4))).*cos(x(3)+x(4))+(-1).*Mtor.*rtor.*cos(x(3)+x(7))+lth.*Ms.*sin(x(4)).*sin(x(3)+x(4)),Ms.*rs.*cos(x(3)+x(4)),Mth.*rth.*cos(x(5))+Ms.*(rs+lth.*cos(x(6))).*cos(x(5)+x(6))+lth.*Ms.*sin(x(6)).*sin(x(5)+x(6)),Ms.*rs.*cos(x(5)+x(6)),(-1).*Mtor.*rtor.*cos(x(3)+x(7));0,Mth.*cos(x(3)).^2+Ms.*cos(x(3)+x(4)).^2+Mth.*cos(x(5)).^2+Ms.*cos(x(5)+x(6)).^2+Mtor.*cos(x(3)+x(7)).^2+Mth.*sin(x(3)).^2+Ms.*sin(x(3)+x(4)).^2+Mth.*sin(x(5)).^2+Ms.*sin(x(5)+x(6)).^2+Mtor.*sin(x(3)+x(7)).^2,Mth.*rth.*sin(x(3))+(-1).*lth.*Ms.*cos(x(3)+x(4)).*sin(x(4))+Ms.*(rs+lth.*cos(x(4))).*sin(x(3)+x(4))+(-1).*Mtor.*rtor.*sin(x(3)+x(7)),Ms.*rs.*sin(x(3)+x(4)),Mth.*rth.*sin(x(5))+(-1).*lth.*Ms.*cos(x(5)+x(6)).*sin(x(6))+Ms.*(rs+lth.*cos(x(6))).*sin(x(5)+x(6)),Ms.*rs.*sin(x(5)+x(6)),(-1).*Mtor.*rtor.*sin(x(3)+x(7));Mth.*rth.*cos(x(3))+Ms.*(rs+lth.*cos(x(4))).*cos(x(3)+x(4))+(-1).*Mtor.*rtor.*cos(x(3)+x(7))+lth.*Ms.*sin(x(4)).*sin(x(3)+x(4)),Mth.*rth.*sin(x(3))+(-1).*lth.*Ms.*cos(x(3)+x(4)).*sin(x(4))+Ms.*(rs+lth.*cos(x(4))).*sin(x(3)+x(4))+(-1).*Mtor.*rtor.*sin(x(3)+x(7)),Isz+Ithz+Itorz+Mth.*rth.^2+Mtor.*rtor.^2+Ms.*(rs+lth.*cos(x(4))).^2+lth.^2.*Ms.*sin(x(4)).^2,Isz+Ms.*rs.*(rs+lth.*cos(x(4))),0,0,Itorz+Mtor.*rtor.^2;Ms.*rs.*cos(x(3)+x(4)),Ms.*rs.*sin(x(3)+x(4)),Isz+Ms.*rs.*(rs+lth.*cos(x(4))),Isz+Ms.*rs.^2,0,0,0;Mth.*rth.*cos(x(5))+Ms.*(rs+lth.*cos(x(6))).*cos(x(5)+x(6))+lth.*Ms.*sin(x(6)).*sin(x(5)+x(6)),Mth.*rth.*sin(x(5))+(-1).*lth.*Ms.*cos(x(5)+x(6)).*sin(x(6))+Ms.*(rs+lth.*cos(x(6))).*sin(x(5)+x(6)),0,0,Isz+Ithz+Mth.*rth.^2+Ms.*(rs+lth.*cos(x(6))).^2+lth.^2.*Ms.*sin(x(6)).^2,Isz+Ms.*rs.*(rs+lth.*cos(x(6))),0;Ms.*rs.*cos(x(5)+x(6)),Ms.*rs.*sin(x(5)+x(6)),0,0,Isz+Ms.*rs.*(rs+lth.*cos(x(6))),Isz+Ms.*rs.^2,0;(-1).*Mtor.*rtor.*cos(x(3)+x(7)),(-1).*Mtor.*rtor.*sin(x(3)+x(7)),Itorz+Mtor.*rtor.^2,0,0,0,Itorz+Mtor.*rtor.^2];
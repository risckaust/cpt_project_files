function [f,a,b,c,d,e] = cpt_gatekeeper(cable_len,L,R2,tension,mass,theta1,theta1dot,theta2,xc,y1,y1dot,y2,yc,phi,x1dotdot,x2dotdot,xcdotdot,radial_os,Iz,intx2,x2,x2dot,t,v_max,ki,kp,kd,intx1,x1,x1dot,T_switch,zm)
% x2:
% USE EITHER:
% cpt_sat_o(a(4),a(5),a(6),tension,v_max(3),ki(3),kp(3),kd(3),a(1),a(2),a(3),cable_len,L)
% OR
% cpt_sat_i(a(4),a(5),a(6),tension,T_switch,0.*A_des(1,:),v_max(3),ki(3),kp(3),kd(3));

% xc,yc,theta1,theta2,phi
% (tension/mass)*(sin(theta1)-sin(theta2))____xc
% y1dot+L*theta1dot*sin(theta1) ____ yc
% (dadt(3)-dadt(9))/cable_len ____ tet1
% (dadt(9)-dadt(6))/cable_len ____ tet2
% (tension*radial_os/Iz)*(cos(a(20)-a(26))-cos(a(23)-a(26))) ___ phi

%cable_len is the non-extended length of each cable
c1 = sqrt((x1-xc-0.5*L*cos(phi)+0.5*R2*sin(phi))^2+(y1-yc-0.5*L*sin(phi)-0.5*R2*cos(phi))^2); %Length of cable 1
c2 = sqrt((x2-xc+0.5*L*cos(phi)+0.5*R2*sin(phi))^2+(y2-yc+0.5*L*sin(phi)-0.5*R2*cos(phi))^2); %Length of cable 2
if c1>=cable_len
    if c2>=cable_len
        a = (tension/mass)*(sin(theta1)-sin(theta2));
        b = y1dot+L*theta1dot*sin(theta1);
        c = (x1dotdot-xcdotdot)/cable_len;
        d = (xcdotdot-x2dotdot)/cable_len;
        e = (tension*radial_os/Iz)*(cos(theta1-phi)-cos(theta2-phi));
        f = cpt_sat_o(intx2,x2,x2dot,t,v_max,ki,kp,kd,intx1,x1,x1dot,cable_len,L);
    else
        a = (tension/mass)*(sin(theta1));
        b = 0;
        c = (x1dotdot-xcdotdot)/cable_len;
        d = 0;
        e = (tension*radial_os/Iz)*(cos(theta1-phi));
        f = cpt_sat_i(intx2,x2,x2dot,t,T_switch,zm,v_max,ki,kp,kd);
    end
else
    if c2>=cable_len
        a = (tension/mass)*(-sin(theta2));
        b = 0;
        c = 0;
        d = (xcdotdot-x2dotdot)/cable_len;
        e = (tension*radial_os/Iz)*(-cos(theta2-phi));
        f = cpt_sat_i(intx2,x2,x2dot,t,T_switch,zm,v_max,ki,kp,kd);
    else
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
        f = cpt_sat_i(intx2,x2,x2dot,t,T_switch,zm,v_max,ki,kp,kd);
    end
end

end
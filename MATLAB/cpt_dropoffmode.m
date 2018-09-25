function dadt = cpt_dropoffmode(t,a,ki,kp,kd,T_switch,A_des,v_max,T,cable_len,L,radial_os,mass,Iz)
%% System of equations for double integrator system 

%% ODEs
%Each line corresponds to a variable, given in the comment
dadt = zeros(27,1);
%c1 = sqrt((a(1)-a(8)-0.5*L*cos(a(26))+0.5*R2*sin(a(26)))^2+(a(11)-a(18)-0.5*L*sin(a(26))-0.5*R2*cos(a(26)))^2);
%c2 = sqrt((a(5)-a(8)+0.5*L*cos(a(26))+0.5*R2*sin(a(26)))^2+(a(14)-a(18)+0.5*L*sin(a(26))-0.5*R2*cos(a(26)))^2);

dadt(1) = a(2);%x1
dadt(2) = a(3);%x1'
dadt(3) = cpt_sat_i(a(1),a(2),a(3),t,T_switch,A_des(1,:),v_max(1),ki(1),kp(1),kd(1));%x1''
dadt(4) = a(5);%x2 
dadt(5) = a(6);%x2'
dadt(6) = cpt_sat_i(a(4),a(5),a(6),t,T_switch,A_des(1,:)-L-cable_len,v_max(3),ki(3),kp(3),kd(3));%x2''
dadt(7) = a(8);%xc
dadt(8) = a(9);%xc'
dadt(9) = (T/mass)*(sin(a(20))-sin(a(23)));%xc'' 
dadt(10) = a(11);%y1
dadt(11) = a(12);%y1'
dadt(12) = cpt_sat_i(a(10),a(11),a(12),t,T_switch,A_des(2,:),v_max(2),ki(2),kp(2),kd(2));%y1''
dadt(13) = a(14);%y2
dadt(14) = a(15);%y2'
dadt(15) = cpt_sat_i(a(13),a(14),a(15),t,T_switch,A_des(3,:),v_max(4),ki(4),kp(4),kd(4));%y2''
dadt(16) = a(17);%int(yc)
dadt(17) = a(18);%yc
if a(18)<=0
    dadt(18) = 0;
    dadt(19) = 0;%theta1
    dadt(20) = 0;%theta1'
    dadt(21) = 0;%theta1''
    dadt(22) = 0;%theta2
    dadt(23) = 0;%theta2'
    dadt(24) = 0;%theta2''
    dadt(25) = 0;%phi
    dadt(26) = 0;%phi'
    dadt(27) = 0;%phi''
else
    dadt(18) = a(12)+L*a(21)*sin(a(20)); %yc' approximation using phi = phi' = 0 due to bug with phi
    dadt(19) = a(20);%theta1
    dadt(20) = a(21);%theta1'
    dadt(21) = (dadt(3)-dadt(9))/cable_len;%theta1''
    dadt(22) = a(23);%theta2
    dadt(23) = a(24);%theta2'
    dadt(24) = (dadt(9)-dadt(6))/cable_len;%theta2''
    dadt(25) = a(26);%phi
    dadt(26) = a(27);%phi'
    dadt(27) = (T*radial_os/Iz)*(cos(a(20)-a(26))-cos(a(23)-a(26)));%phi''

end
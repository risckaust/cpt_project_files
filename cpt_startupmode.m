function dadt = cpt_startupmode(t,a,ki,kp,kd,T_switch,A_des,v_max,T,cable_len,L,R2,radial_os,mass,Iz)
%% System of equations for double integrator system 

%% ODEs
%Each line corresponds to a variable, given in the comment
dadt = zeros(27,1);

dadt(1) = a(2);%x1
dadt(2) = a(3);%x1'
dadt(3) = cpt_sat_i(a(1),a(2),a(3),t,T_switch,A_des(1,:),v_max(1),ki(1),kp(1),kd(1));%x1''
dadt(4) = a(5);%x2 
dadt(5) = a(6);%x2'
dadt(7) = a(8);%xc
dadt(8) = a(9);%xc'
[dadt(6),dadt(9),dadt(18),dadt(21),dadt(24),dadt(27)] =  cpt_gatekeeper(cable_len,L,R2,T,mass,a(20),a(21),a(23),...
    a(8),a(11),a(12),a(14),a(18),a(26),dadt(3),dadt(6),dadt(9),...
    radial_os,Iz,a(4),a(5),a(6),t,v_max(3),ki(3),kp(3),kd(3),a(1),a(2),a(3),T_switch,0.*A_des(1,:));%x2'',xc'',yc',theta1'',theta2'',phi''

dadt(10) = a(11);%y1
dadt(11) = a(12);%y1'
dadt(12) = cpt_sat_i(a(10),a(11),a(12),t,T_switch,A_des(2,:),v_max(2),ki(2),kp(2),kd(2));%y1''
dadt(13) = a(14);%y2
dadt(14) = a(15);%y2'
dadt(15) = cpt_sat_i(a(13),a(14),a(15),t,T_switch,A_des(3,:),v_max(4),ki(4),kp(4),kd(4));%y2''
dadt(16) = a(17);%int(yc)
dadt(17) = a(18);%yc
dadt(19) = a(20);%tet1
dadt(20) = a(21);%tet1'
dadt(22) = a(23);%tet2
dadt(23) = a(24);%tet2'
dadt(25) = a(26);%phi
dadt(26) = a(27);%phi'
end
function y = cpt_sat_i(a,a_dot,a_dotdot,t,T_switch,A_des,v_max,Ki,Kp,Kd)
%%  Velocity limiter (with Interpolation)
%   Impose limit on the velocity of the system
%
%   From unsaturated system:
%   x(1) = int(0->t) x(t)
%   x(2) = x(t)
%   x(3) = [d/dtau x(tau)]|tau=t
%   dadt = [a(2); a(3); Ki*(Ades*t - a(1)) + Kp*(Ades - a(2)) + Kd*(0-a(3))];
% 
%   New:
%   y = acceleration as input
%   a = x(1)
%   a_dot = x(2)
%   a_dotdot = x(3)
%
% y = Ki*(x_des*t - a) + Kp*(x_des - a_dot) + Kd*(0-a_dotdot);
b_des = interp1(T_switch,A_des,t,'previous');

u = Ki.*(b_des*t-a) + Kp.*(b_des-a_dot) + Kd.*(0-a_dotdot);
if (a_dotdot >= v_max) && (u>=0)
    y = 0;
elseif (a_dotdot >= v_max) && (u<0)
    y = u;
elseif (a_dotdot < -v_max) && (u>=0)
    y = u;
elseif (a_dotdot < -v_max) && (u<0)
    y = 0;
else
    y = u;
end
end
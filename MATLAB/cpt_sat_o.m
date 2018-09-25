function y = cpt_sat_o(a,a_dot,a_dotdot,t,v_max,Ki,Kp,Kd,b,b_dot,b_dotdot,cable_len,obj_len)
%%  Velocity Limiter
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

u = (Ki.*(b-a-obj_len*t) + Kp.*(b_dot-a_dot-obj_len) + Kd.*(b_dotdot-a_dotdot))*(1/2/cable_len);

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